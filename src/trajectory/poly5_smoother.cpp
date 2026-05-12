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

std::vector<TrajectoryPoint> smoothTrajectoryPoly5(
    const std::vector<PathPointWithJoints>& path,
    const TrajectoryParams& params)
{
    std::vector<TrajectoryPoint> out;
    if (path.size() < 2) return out;

    const double sample_dt_ms = params.sample_period_ms;
    double timestamp_ms = 0.0;

    // First frame
    {
        TrajectoryPoint p0;
        p0.timestamp_ms = 0;
        p0.joint_deg = path[0].joint.q_deg;
        std::fill(p0.joint_velocity_deg_per_s.begin(),
                  p0.joint_velocity_deg_per_s.end(), 0.0);
        std::fill(p0.joint_accel_deg_per_s2.begin(),
                  p0.joint_accel_deg_per_s2.end(), 0.0);
        p0.wire_on = path[0].wire_on;
        out.push_back(p0);
        timestamp_ms = sample_dt_ms;
    }

    // Process each segment
    for (size_t k = 0; k + 1 < path.size(); ++k) {
        double T_s = computeSegmentDuration(path[k], path[k + 1], params);
        if (T_s < 1e-9) T_s = 1e-9;

        std::array<Poly5Coeffs, 6> coefs;
        for (int i = 0; i < 6; ++i) {
            coefs[i] = computePoly5(path[k].joint.q_deg[i],
                                    path[k + 1].joint.q_deg[i], T_s);
        }

        double T_ms = T_s * 1000.0;

        for (double tau_ms = sample_dt_ms; tau_ms <= T_ms + 1e-6; tau_ms += sample_dt_ms) {
            double tau_s = tau_ms / 1000.0;

            TrajectoryPoint pt;
            pt.timestamp_ms = timestamp_ms + tau_ms - sample_dt_ms;

            for (int i = 0; i < 6; ++i) {
                pt.joint_deg[i] = coefs[i].eval(tau_s);
                pt.joint_velocity_deg_per_s[i] = coefs[i].evalDot(tau_s);
                pt.joint_accel_deg_per_s2[i] = coefs[i].evalDotDot(tau_s);
            }

            pt.wire_on = path[k + 1].wire_on;
            out.push_back(pt);
        }

        timestamp_ms = out.back().timestamp_ms + sample_dt_ms;
    }

    return out;
}

}  // namespace cslc
