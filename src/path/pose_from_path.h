#pragma once

#include "kinematics/cart_pose.h"

#include <Eigen/Dense>

namespace cslc {

// Construct tool frame rotation matrix from path tangent and outward normal.
// Convention: tool_z = -G_outward (tool points opposite to gravity/surface normal).
// Returns identity if tangent is near-parallel to G_outward (degenerate).
Eigen::Matrix3d constructToolFrame(const Eigen::Vector3d& path_tangent,
                                   const Eigen::Vector3d& G_outward);

// Convert a 3D point + tool rotation to CartPose (KUKA ZYX Euler).
CartPose toolFrameToCartPose(const Eigen::Vector3d& point,
                             const Eigen::Matrix3d& R_tool);

}  // namespace cslc
