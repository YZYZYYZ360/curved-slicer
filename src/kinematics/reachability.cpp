#include "kinematics/reachability.h"
#include "kinematics/ik_solver.h"
#include "path/pose_from_path.h"

#include <cmath>
#include <Eigen/Dense>

namespace cslc {

static double sq(double x) { return x * x; }

static double angle_diff_deg(double a, double b)
{
    double d = a - b;
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

ReachabilityResult checkReachability(
    const CartPose& pose,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params)
{
    ReachabilityResult result;

    // 1. Workspace check: TCP radial distance
    double r_tcp = std::sqrt(sq(pose.X) + sq(pose.Y));
    if (r_tcp < params.workspace_r_min_mm || r_tcp > params.workspace_r_max_mm) {
        result.status = ReachStatus::OutOfWorkspace;
        return result;
    }

    // 2. Solve IK
    auto sols = solveAnalyticalIK(pose, dh, &reference);
    result.num_valid = static_cast<int>(sols.size());

    if (sols.empty()) {
        result.status = ReachStatus::NoSolution;
        return result;
    }

    // 3. Find best solution (closest to reference)
    int best_idx = 0;
    double best_cost = 1e18;
    for (int i = 0; i < static_cast<int>(sols.size()); ++i) {
        if (sols[i].status != IKStatus::OK) continue;
        double cost = 0.0;
        for (int j = 0; j < 6; ++j) {
            double d = angle_diff_deg(sols[i].solution.q_deg[j], reference.q_deg[j]);
            cost += d * d;
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_idx = i;
        }
    }

    if (sols[best_idx].status != IKStatus::OK) {
        result.status = ReachStatus::JointLimit;
        return result;
    }

    result.solution = sols[best_idx].solution;

    // 4. Wrist singularity: |J5| < threshold
    double abs_j5 = std::abs(result.solution.q_deg[4]);
    result.wrist_sing = (abs_j5 < params.singularity_a5_deg ||
                         std::abs(abs_j5 - 180.0) < params.singularity_a5_deg);

    // 5. Shoulder singularity: |J2+J3 ∓ 90°| < threshold
    double j2pj3 = result.solution.q_deg[1] + result.solution.q_deg[2];
    double sh_mod = std::fmod(j2pj3, 360.0);
    result.shoulder_sing = (std::abs(sh_mod - 90.0) < params.singularity_shoulder_deg ||
                            std::abs(sh_mod + 90.0) < params.singularity_shoulder_deg ||
                            std::abs(sh_mod - 270.0) < params.singularity_shoulder_deg ||
                            std::abs(sh_mod + 270.0) < params.singularity_shoulder_deg);

    if (result.wrist_sing || result.shoulder_sing) {
        result.status = ReachStatus::NearSingularity;
        return result;
    }

    // 6. Arm-flip detection
    double max_delta = 0.0;
    for (int j = 0; j < 6; ++j) {
        double d = std::abs(angle_diff_deg(result.solution.q_deg[j], reference.q_deg[j]));
        if (d > max_delta) max_delta = d;
    }
    result.arm_flip = (max_delta > params.arm_flip_threshold_deg);

    result.status = ReachStatus::OK;
    return result;
}

std::vector<VoxelIndex> filterReachableVoxels(
    const VoxelGrid& grid,
    const Vec3& G_direction,
    const KR4DHParams& dh,
    const ReachabilityParams& params,
    const JointConfig& reference)
{
    Eigen::Vector3d G(G_direction.x, G_direction.y, G_direction.z);
    Eigen::Matrix3d R_tool = constructToolFrame(Eigen::Vector3d(1, 0, 0), G);

    auto occupied = grid.occupiedVoxels();
    std::vector<VoxelIndex> reachable;
    reachable.reserve(occupied.size());

    for (const auto& vi : occupied) {
        Vec3 c = grid.center(vi.x, vi.y, vi.z);
        CartPose pose = toolFrameToCartPose(Eigen::Vector3d(c.x, c.y, c.z), R_tool);
        auto r = checkReachability(pose, reference, dh, params);
        if (r.status == ReachStatus::OK) {
            reachable.push_back(vi);
        }
    }
    return reachable;
}

PathReachabilityResult filterReachablePathPoints(
    const std::vector<Vec3>& points,
    const std::vector<Vec3>& tangents,
    const Vec3& G_direction,
    const KR4DHParams& dh,
    const ReachabilityParams& params)
{
    PathReachabilityResult result;
    int n = static_cast<int>(points.size());
    result.statuses.resize(n);
    result.solutions.resize(n);

    Eigen::Vector3d G(G_direction.x, G_direction.y, G_direction.z);
    JointConfig ref{};

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d t(tangents[i].x, tangents[i].y, tangents[i].z);
        Eigen::Matrix3d R_tool = constructToolFrame(t, G);
        CartPose pose = toolFrameToCartPose(
            Eigen::Vector3d(points[i].x, points[i].y, points[i].z), R_tool);

        auto r = checkReachability(pose, ref, dh, params);
        result.statuses[i] = r.status;
        if (r.status == ReachStatus::OK) {
            result.solutions[i] = r.solution;
            ref = r.solution;
            ++result.num_reachable;
        }
    }
    return result;
}

}  // namespace cslc
