#include "kinematics/forward_kin.h"

#include <cmath>

namespace cslc {

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

}  // namespace

Eigen::Matrix4d dhMatrix(double theta_deg, double d, double a, double alpha_rad)
{
    const double th = theta_deg * kDeg2Rad;
    const double ct = std::cos(th);
    const double st = std::sin(th);
    const double ca = std::cos(alpha_rad);
    const double sa = std::sin(alpha_rad);

    Eigen::Matrix4d T;
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0.0,      sa,       ca,      d,
         0.0,     0.0,      0.0,    1.0;
    return T;
}

Eigen::Matrix4d forwardKinMatrix_partial(const JointConfig& q, const KR4DHParams& dh, int N)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < N; ++i) {
        T = T * dhMatrix(q.q_deg[i], dh.d_mm[i], dh.a_mm[i], dh.alpha_rad[i]);
    }
    return T;
}

Eigen::Matrix4d forwardKinMatrix(const JointConfig& q, const KR4DHParams& dh)
{
    Eigen::Matrix4d T = forwardKinMatrix_partial(q, dh, 6);
    // Flange + tool z offset along the last z-axis
    Eigen::Matrix4d T_flange = Eigen::Matrix4d::Identity();
    T_flange(2, 3) = dh.flange_z_mm + dh.tool_z_mm;
    return T * T_flange;
}

void rotMatToKukaABC(const Eigen::Matrix3d& R, double& A_deg, double& B_deg, double& C_deg)
{
    // KUKA ZYX intrinsic Euler: yaw(A/Z) -> pitch(B/Y') -> roll(C/X'')
    // Standard decomposition:
    //   B = atan2(-R(2,0), sqrt(R(0,0)^2 + R(1,0)^2))
    //   if cos(B) != 0:
    //     A = atan2(R(1,0), R(0,0))
    //     C = atan2(R(2,1), R(2,2))
    //   else (gimbal lock at B = +/-90):
    //     A = atan2(-R(0,1), R(1,1))
    //     C = 0

    const double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    B_deg = std::atan2(-R(2, 0), sy) * kRad2Deg;

    if (sy > 1e-9) {
        A_deg = std::atan2(R(1, 0), R(0, 0)) * kRad2Deg;
        C_deg = std::atan2(R(2, 1), R(2, 2)) * kRad2Deg;
    } else {
        // Gimbal lock
        A_deg = std::atan2(-R(0, 1), R(1, 1)) * kRad2Deg;
        C_deg = 0.0;
    }
}

Eigen::Matrix3d kukaABCToRotMat(double A_deg, double B_deg, double C_deg)
{
    const double A = A_deg * kDeg2Rad;
    const double B = B_deg * kDeg2Rad;
    const double C = C_deg * kDeg2Rad;

    const double ca = std::cos(A), sa = std::sin(A);
    const double cb = std::cos(B), sb = std::sin(B);
    const double cc = std::cos(C), sc = std::sin(C);

    // R = Rz(A) * Ry(B) * Rx(C)
    Eigen::Matrix3d R;
    R << ca * cb,  ca * sb * sc - sa * cc,  ca * sb * cc + sa * sc,
         sa * cb,  sa * sb * sc + ca * cc,  sa * sb * cc - ca * sc,
         -sb,              cb * sc,                  cb * cc;
    return R;
}

CartPose forwardKin(const JointConfig& q, const KR4DHParams& dh)
{
    const Eigen::Matrix4d T = forwardKinMatrix(q, dh);
    CartPose pose;
    pose.X = T(0, 3);
    pose.Y = T(1, 3);
    pose.Z = T(2, 3);
    rotMatToKukaABC(T.block<3, 3>(0, 0), pose.A, pose.B, pose.C);
    return pose;
}

}  // namespace cslc
