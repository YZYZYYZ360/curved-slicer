#include "trajectory/poly5_smoother.h"

#include <algorithm>
#include <cmath>

namespace cslc {

Poly5Coeffs computePoly5(double q0, double qT, double T)
{
    // Boundary conditions: q(0)=q0, q(T)=qT, q'(0)=q'(T)=0, q''(0)=q''(T)=0
    // Closed-form: c0=q0, c1=c2=0, c3=10D/T^3, c4=-15D/T^4, c5=6D/T^5
    Poly5Coeffs c;
    double D = qT - q0;
    double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    c.c0 = q0;
    c.c1 = 0.0;
    c.c2 = 0.0;
    c.c3 = 10.0 * D / T3;
    c.c4 = -15.0 * D / T4;
    c.c5 = 6.0 * D / T5;
    return c;
}

double computeSegmentDuration(const PathPointWithJoints& a,
                              const PathPointWithJoints& b,
                              const TrajectoryParams& params)
{
    double d_cart = (b.cart_pos - a.cart_pos).norm();
    double dt_cart = d_cart / params.target_line_speed_mm_per_s;

    double max_dq = 0;
    for (int i = 0; i < 6; ++i) {
        max_dq = std::max(max_dq, std::abs(b.joint.q_deg[i] - a.joint.q_deg[i]));
    }
    double dt_joint = max_dq / params.max_joint_velocity_deg_per_s;

    return std::max(dt_cart, dt_joint);
}

}  // namespace cslc
