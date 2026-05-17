#include "kinematics/KR4Model.h"

#include "kinematics/forward_kin.h"

#include <cmath>

namespace cslc {
namespace {

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

KR4Model::JointLimits limitsFromDh(const KR4DHParams& dh)
{
    KR4Model::JointLimits limits;
    for (int i = 0; i < 6; ++i) {
        limits(i, 0) = dh.qlim_deg[i][0] * kDeg2Rad;
        limits(i, 1) = dh.qlim_deg[i][1] * kDeg2Rad;
    }
    return limits;
}

}  // namespace

KR4Model::KR4Model()
    : KR4Model(KR4DHParams{})
{
}

KR4Model::KR4Model(const KR4DHParams& dh)
    : dh_(dh),
      joint_limits_rad_(limitsFromDh(dh))
{
}

Eigen::Matrix4d KR4Model::forwardKinematics(const JointVector& q_rad) const
{
    return forwardKinMatrix(toJointConfigDeg(q_rad), dh_);
}

JointConfig KR4Model::toJointConfigDeg(const JointVector& q_rad)
{
    JointConfig q;
    for (int i = 0; i < 6; ++i) {
        q.q_deg[i] = q_rad(i) * kRad2Deg;
    }
    return q;
}

KR4Model::JointVector KR4Model::fromJointConfigDeg(const JointConfig& q_deg)
{
    JointVector q_rad;
    for (int i = 0; i < 6; ++i) {
        q_rad(i) = q_deg.q_deg[i] * kDeg2Rad;
    }
    return q_rad;
}

}  // namespace cslc
