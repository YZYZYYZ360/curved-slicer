#include "kinematics/ik_solver.h"
#include "kinematics/forward_kin.h"

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cslc {

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kSingularityDeg = 5.0;

double sq(double x) { return x * x; }

}  // namespace

Eigen::Vector3d tcp_to_wrist_center(const CartPose& tcp)
{
    Eigen::Matrix3d R = kukaABCToRotMat(tcp.A, tcp.B, tcp.C);
    constexpr double tcp_offset = 12.0 + 0.28;  // flange + tool
    return Eigen::Vector3d(tcp.X - tcp_offset * R(0, 2),
                           tcp.Y - tcp_offset * R(1, 2),
                           tcp.Z - tcp_offset * R(2, 2));
}

Eigen::Vector2d compute_wrist_local(double r_w, double z_w, double a1)
{
    return Eigen::Vector2d(r_w - a1, z_w);
}

bool joints_in_limits(const double q_rad[6], const std::array<std::array<double, 2>, 6>& qlim_deg)
{
    for (int i = 0; i < 6; ++i) {
        double deg = q_rad[i] * kRad2Deg;
        if (deg < qlim_deg[i][0] - 1e-3 || deg > qlim_deg[i][1] + 1e-3) {
            return false;
        }
    }
    return true;
}

double normalize_angle(double rad)
{
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

// Solve J2 and J3 using the correct derivation for KR4 DH (α2=π).
//
// The FK gives:
//   x_wc = c1 * (a2*c2 + a3*cos(θ2-θ3) + d4*sin(θ2-θ3))
//   y_wc = s1 * (a2*c2 + a3*cos(θ2-θ3) + d4*sin(θ2-θ3))
//   z_wc = d1 + a2*s2 + a3*sin(θ2-θ3) - d4*cos(θ2-θ3)
//
// With u = θ2-θ3 and α = u+δ (δ = atan2(a3,d4)):
//   R - L3_eff*sin(α) = a2*cos(θ2)
//   Z + L3_eff*cos(α) = a2*sin(θ2)
//
// Solving: R*sin(α) - Z*cos(α) = W/(2*L3_eff)
// where W = R² + Z² + L3_eff² - a2²
//
struct J2J3Solution {
    double theta2, theta3;
    bool valid;
};

J2J3Solution solve_single_j2j3(double R, double Z, double a2,
                                double L3_eff, double delta, int alpha_sign)
{
    J2J3Solution sol = {0, 0, false};

    double D = std::sqrt(R * R + Z * Z);
    if (D < 1e-9) return sol;

    double W = R * R + Z * Z + L3_eff * L3_eff - a2 * a2;
    double rhs = W / (2.0 * L3_eff * D);
    rhs = std::clamp(rhs, -1.0, 1.0);

    double gamma = std::atan2(-Z, R);
    double asin_val = std::asin(rhs);

    double alpha;
    if (alpha_sign == 0) {
        alpha = asin_val - gamma;
    } else {
        alpha = M_PI - asin_val - gamma;
    }

    // Compute θ2
    double A = Z + L3_eff * std::cos(alpha);
    double B = R - L3_eff * std::sin(alpha);

    if (std::abs(A) < 1e-9 && std::abs(B) < 1e-9) return sol;

    sol.theta2 = std::atan2(A, B);

    // θ3 = θ2 - u = θ2 - (α - δ)
    double u = alpha - delta;
    sol.theta3 = sol.theta2 - u;
    sol.valid = true;
    return sol;
}

bool decompose_j456(const Eigen::Matrix3d& R_456, bool wrist_flip,
                    double& j4, double& j5, double& j6)
{
    double cos_j5 = std::clamp(R_456(2, 2), -1.0, 1.0);
    double sin_j5_sq = 1.0 - sq(cos_j5);

    if (sin_j5_sq < 1e-10) {
        j5 = std::atan2(0.0, cos_j5);
        if (wrist_flip) j5 = -j5;

        double abs_j5_deg = std::abs(j5 * kRad2Deg);
        if (abs_j5_deg < kSingularityDeg || std::abs(abs_j5_deg - 180.0) < kSingularityDeg) {
            if (std::abs(cos_j5) > 0.0) {
                j4 = std::atan2(-R_456(0, 1), R_456(0, 0));
            } else {
                j4 = std::atan2(R_456(0, 1), -R_456(0, 0));
            }
            j6 = 0.0;
        } else {
            return false;
        }
        return true;
    }

    double sin_j5 = std::sqrt(sin_j5_sq);
    if (wrist_flip) sin_j5 = -sin_j5;
    j5 = std::atan2(sin_j5, cos_j5);

    j4 = std::atan2(R_456(1, 2), R_456(0, 2));
    j6 = std::atan2(R_456(2, 1), -R_456(2, 0));
    return true;
}

std::vector<IKResult> solveAnalyticalIK(
    const CartPose& tcp,
    const KR4DHParams& dh,
    const JointConfig* reference)
{
    auto sols = solveIKAll(tcp, dh, reference);
    std::vector<IKResult> results;
    results.reserve(sols.size());
    for (auto& s : sols) {
        IKResult r;
        r.status = IKStatus::OK;
        r.solution = s.config;
        results.push_back(r);
    }
    return results;
}

std::vector<IKSolution> solveIKAll(
    const CartPose& tcp,
    const KR4DHParams& dh,
    const JointConfig* reference)
{
    // Wrist center
    Eigen::Vector3d P_w = tcp_to_wrist_center(tcp);
    double wx = P_w.x(), wy = P_w.y(), wz = P_w.z();

    // DH parameters
    const double a2 = dh.a_mm[1];   // 290.0
    const double a3 = dh.a_mm[2];   // 20.0
    const double d1 = dh.d_mm[0];   // 330.0
    const double d4 = dh.d_mm[3];   // 310.0

    // Pre-fold a3=20mm offset
    double L3_eff = std::sqrt(a3 * a3 + d4 * d4);
    double delta = std::atan2(a3, d4);

    // J1 from wrist center
    // When wrist center is near the A1 rotation axis (r_xy small), atan2 is
    // numerically unstable — any A1 angle is geometrically valid.  Fall back
    // to the reference J1 in that case (standard textbook treatment).
    constexpr double kBaseAxisThresholdMm = 30.0;
    double r_xy = std::sqrt(wx * wx + wy * wy);
    double j1_base;
    if (r_xy < kBaseAxisThresholdMm && reference) {
        j1_base = reference->q_deg[0] * kDeg2Rad;
    } else {
        j1_base = std::atan2(wy, wx);
    }
    double j1_front = j1_base;
    double j1_back = normalize_angle(j1_front + M_PI);

    // Collect J1 candidates: front, back, and optionally reference
    std::vector<double> j1_candidates = {j1_front, j1_back};
    if (reference) {
        double j1_ref = reference->q_deg[0] * kDeg2Rad;
        // Add reference J1 if it differs from front/back
        bool dup = false;
        for (double j : j1_candidates) {
            if (std::abs(normalize_angle(j1_ref - j)) < 0.01) { dup = true; break; }
        }
        if (!dup) j1_candidates.push_back(j1_ref);
    }

    // Up to 6 position solutions: j1_candidates × alpha_sign(0/1)
    // Each has 2 wrist flip variants
    constexpr int kMaxPos = 6;
    struct PosSol { double j1, theta2, theta3; bool valid; };
    PosSol pos_sols[kMaxPos] = {};
    int num_pos = 0;

    for (size_t jc = 0; jc < j1_candidates.size() && num_pos < kMaxPos; ++jc) {
        double j1 = j1_candidates[jc];
        double c1 = std::cos(j1), s1 = std::sin(j1);

        // R = radial distance from z-axis (in J1 frame)
        double R = c1 * wx + s1 * wy;
        double Z = wz - d1;

        for (int as = 0; as < 2 && num_pos < kMaxPos; ++as) {
            auto sol = solve_single_j2j3(R, Z, a2, L3_eff, delta, as);
            if (sol.valid) {
                pos_sols[num_pos++] = {j1, sol.theta2, sol.theta3, true};
            }
        }
    }

    std::vector<IKSolution> solutions;
    solutions.reserve(12);

    for (int pi = 0; pi < num_pos; ++pi) {
        if (!pos_sols[pi].valid) continue;

        double j1 = pos_sols[pi].j1;
        double theta2 = pos_sols[pi].theta2;
        double theta3 = pos_sols[pi].theta3;

        // Build R03 using DH convention (matches forward_kin.cpp)
        double c1 = std::cos(j1), s1 = std::sin(j1);
        double c2 = std::cos(theta2), s2 = std::sin(theta2);
        double c3 = std::cos(theta3), s3 = std::sin(theta3);

        Eigen::Matrix3d R01;
        R01 << c1,  0, s1,
               s1,  0, -c1,
               0,   1, 0;

        Eigen::Matrix3d R12;
        R12 << c2,  s2, 0,
               s2, -c2, 0,
               0,   0, -1;

        Eigen::Matrix3d R23;
        R23 << c3,  0, -s3,
               s3,  0,  c3,
               0,  -1,  0;

        Eigen::Matrix3d R03 = R01 * R12 * R23;
        Eigen::Matrix3d R_tcp = kukaABCToRotMat(tcp.A, tcp.B, tcp.C);
        Eigen::Matrix3d R_456 = R03.transpose() * R_tcp;

        // Two wrist flip variants
        for (int wf = 0; wf < 2; ++wf) {
            double j4, j5, j6;
            if (!decompose_j456(R_456, wf == 1, j4, j5, j6)) continue;

            // Normalize θ₂,θ₃ to [-π,π] before limit check.
            // α₂=π DH convention can push θ₃ outside [-π,π] via θ₂-θ₃ coupling.
            double q_rad[6] = {j1, normalize_angle(theta2), normalize_angle(theta3),
                               j4, j5, j6};
            if (!joints_in_limits(q_rad, dh.qlim_deg)) continue;

            // Singularity check
            double abs_j5_deg = std::abs(j5 * kRad2Deg);
            if (abs_j5_deg < kSingularityDeg ||
                std::abs(abs_j5_deg - 180.0) < kSingularityDeg) continue;

            // FK position verification
            JointConfig cfg_result;
            for (int j = 0; j < 6; ++j) cfg_result.q_deg[j] = q_rad[j] * kRad2Deg;
            CartPose fk = forwardKin(cfg_result, dh);
            double pos_err = std::sqrt(sq(fk.X - tcp.X) + sq(fk.Y - tcp.Y) + sq(fk.Z - tcp.Z));
            if (pos_err > 1.0) continue;

            // Cost vs reference
            double cost = 0.0;
            if (reference) {
                for (int j = 0; j < 6; ++j) {
                    double diff = cfg_result.q_deg[j] - reference->q_deg[j];
                    cost += diff * diff;
                }
            }

            IKSolution sol;
            sol.config = cfg_result;
            sol.cost = cost;
            sol.valid = true;
            solutions.push_back(sol);
        }
    }

    std::sort(solutions.begin(), solutions.end(),
              [](const IKSolution& a, const IKSolution& b) { return a.cost < b.cost; });

    return solutions;
}

}  // namespace cslc
