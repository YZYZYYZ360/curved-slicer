#pragma once

#include "kinematics/cart_pose.h"
#include "kinematics/dh_params.h"
#include "geometry/voxel_grid.h"
#include "field/kuka_projection.h"

#include <array>
#include <vector>

namespace cslc {

// ReachabilityParams is defined in field/kuka_projection.h (shared struct).

enum class ReachStatus : uint8_t {
    OK              = 0,
    OutOfWorkspace  = 1,
    JointLimit      = 2,
    NearSingularity = 3,
    NoSolution      = 4,
};

struct ReachabilityResult {
    ReachStatus status     = ReachStatus::NoSolution;
    bool        arm_flip   = false;
    bool        wrist_sing = false;
    bool        shoulder_sing = false;
    JointConfig solution{};
    int         num_valid  = 0;
};

// Check if a Cartesian pose is reachable by the IK solver.
// Performs workspace check, IK solve, singularity detection, and arm-flip detection.
ReachabilityResult checkReachability(
    const CartPose& pose,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params = {});

// Per-path-point reachability result.
struct PathReachabilityResult {
    std::vector<ReachStatus>  statuses;
    std::vector<JointConfig>  solutions;
    int                       num_reachable = 0;
};

// Filter occupied voxels by reachability.
// G_direction is the outward surface normal used to orient the tool (tool_z = -G).
// Returns indices of reachable voxels.
std::vector<VoxelIndex> filterReachableVoxels(
    const VoxelGrid& grid,
    const Vec3& G_direction,
    const KR4DHParams& dh,
    const ReachabilityParams& params = {},
    const JointConfig& reference = JointConfig{});

// Filter a sequence of path points by reachability with reference chaining.
// Each point's IK solution becomes the reference for the next point.
PathReachabilityResult filterReachablePathPoints(
    const std::vector<Vec3>& points,
    const std::vector<Vec3>& tangents,
    const Vec3& G_direction,
    const KR4DHParams& dh,
    const ReachabilityParams& params = {});

}  // namespace cslc
