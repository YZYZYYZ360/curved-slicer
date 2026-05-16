#pragma once

#include "kinematics/cart_pose.h"
#include "kinematics/dh_params.h"

#include <Eigen/Dense>

namespace cslc {

// Standard DH single-joint transformation matrix
// A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
Eigen::Matrix4d dhMatrix(double theta_deg, double d, double a, double alpha_rad);

// Full 6-DOF forward kinematics: returns 4x4 homogeneous transform (base to TCP)
Eigen::Matrix4d forwardKinMatrix(const JointConfig& q, const KR4DHParams& dh);

// Partial forward kinematics: first N joints only (N=1..6)
Eigen::Matrix4d forwardKinMatrix_partial(const JointConfig& q, const KR4DHParams& dh, int N);

// Forward kinematics: returns CartPose (X,Y,Z in mm, A,B,C in KUKA ZYX Euler deg)
CartPose forwardKin(const JointConfig& q, const KR4DHParams& dh);

// Rotation matrix to KUKA ZYX Euler angles (A=gamma/Z, B=beta/Y, C=alpha/X)
void rotMatToKukaABC(const Eigen::Matrix3d& R, double& A_deg, double& B_deg, double& C_deg);

// KUKA ZYX Euler angles to rotation matrix
Eigen::Matrix3d kukaABCToRotMat(double A_deg, double B_deg, double C_deg);

}  // namespace cslc
