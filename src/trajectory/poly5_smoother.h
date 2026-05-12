#pragma once

#include "kinematics/cart_pose.h"

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace cslc {

struct TrajectoryParams {
    double target_line_speed_mm_per_s    = 5.0;
    double sample_period_ms              = 4.0;
    double max_joint_velocity_deg_per_s  = 200.0;
    double max_joint_accel_deg_per_s2    = 500.0;
    double max_joint_delta_per_cycle_deg = 3.0;
    bool   zero_endpoint_velocity        = true;
};

struct PathPointWithJoints {
    Eigen::Vector3d cart_pos;
    JointConfig     joint;
    bool            wire_on = true;
};

struct TrajectoryPoint {
    double timestamp_ms = 0.0;
    std::array<double, 6> joint_deg{};
    std::array<double, 6> joint_velocity_deg_per_s{};
    std::array<double, 6> joint_accel_deg_per_s2{};
    bool   wire_on   = true;
    double plc_mode  = 1.0;
    double plc_speed = 5.0;
    double plc_ratio = 1000.0;
};

// 5th-order polynomial coefficients: q(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
struct Poly5Coeffs {
    double c0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0;

    double eval(double t) const {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
        return c0 + c1*t + c2*t2 + c3*t3 + c4*t4 + c5*t5;
    }
    double evalDot(double t) const {
        double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
        return c1 + 2*c2*t + 3*c3*t2 + 4*c4*t3 + 5*c5*t4;
    }
    double evalDotDot(double t) const {
        double t2 = t * t, t3 = t2 * t;
        return 2*c2 + 6*c3*t + 12*c4*t2 + 20*c5*t3;
    }
};

// Compute poly5 coefficients for zero-velocity boundary conditions.
// q(0)=q0, q(T)=qT, q'(0)=q'(T)=0, q''(0)=q''(T)=0.
Poly5Coeffs computePoly5(double q0, double qT, double T);

// Compute segment duration from dual time constraint.
double computeSegmentDuration(const PathPointWithJoints& a,
                              const PathPointWithJoints& b,
                              const TrajectoryParams& params);

}  // namespace cslc
