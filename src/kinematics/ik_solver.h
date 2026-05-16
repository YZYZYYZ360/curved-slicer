#pragma once

#include "kinematics/cart_pose.h"
#include "kinematics/dh_params.h"

#include <Eigen/Dense>
#include <vector>

namespace cslc {

// Wrist center position (subtracts TCP tool offset)
Eigen::Vector3d tcp_to_wrist_center(const CartPose& tcp);

// Compute r_w - a1 and z_w for IK (legacy, not used in main solver)
Eigen::Vector2d compute_wrist_local(double r_w, double z_w, double a1);

// Check if joint angles are within limits
bool joints_in_limits(const double q_rad[6], const std::array<std::array<double, 2>, 6>& qlim_deg);

// Normalize angle to [-pi, pi]
double normalize_angle(double rad);

// All valid IK solutions with metadata
struct IKSolution {
    JointConfig config;
    double cost;
    bool valid;
};

// Analytical 6-DOF IK solver for KR4 R600
// Accounts for α2=π twist (θ2-θ3 coupling, not θ2+θ3)
// Returns all valid solutions (0-8) sorted by distance from reference.
std::vector<IKResult> solveAnalyticalIK(
    const CartPose& tcp,
    const KR4DHParams& dh,
    const JointConfig* reference = nullptr);

// Return all valid solutions with metadata
std::vector<IKSolution> solveIKAll(
    const CartPose& tcp,
    const KR4DHParams& dh,
    const JointConfig* reference = nullptr);

}  // namespace cslc
