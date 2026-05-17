#pragma once

#include "kinematics/cart_pose.h"
#include "kinematics/dh_params.h"

#include <Eigen/Dense>

namespace cslc {

class KR4Model {
public:
    using JointVector = Eigen::Matrix<double, 6, 1>;
    using JointLimits = Eigen::Matrix<double, 6, 2>;

    KR4Model();
    explicit KR4Model(const KR4DHParams& dh);

    // Frame contract: target poses are in KUKA robot-base coordinates after
    // applying the configured kuka.world_to_base model placement transform.
    Eigen::Matrix4d forwardKinematics(const JointVector& q_rad) const;

    const KR4DHParams& dhParams() const { return dh_; }
    const JointLimits& jointLimits() const { return joint_limits_rad_; }
    double toolOffset() const { return dh_.flange_z_mm + dh_.tool_z_mm; }

    static JointConfig toJointConfigDeg(const JointVector& q_rad);
    static JointVector fromJointConfigDeg(const JointConfig& q_deg);

private:
    KR4DHParams dh_;
    JointLimits joint_limits_rad_;
};

}  // namespace cslc
