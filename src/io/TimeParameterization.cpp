#include "io/TimeParameterization.h"

#include "trajectory/poly5_smoother.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cslc {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double radToDeg(double rad)
{
    return rad * 180.0 / kPi;
}

double degToRad(double deg)
{
    return deg * kPi / 180.0;
}

JointConfig toJointConfigDeg(const Eigen::Matrix<double, 6, 1>& q_rad)
{
    JointConfig q_deg;
    for (int i = 0; i < 6; ++i) {
        q_deg.q_deg[static_cast<size_t>(i)] = radToDeg(q_rad(i));
    }
    return q_deg;
}

double gridAlignedSegmentDurationSeconds(
    const Eigen::Matrix<double, 6, 1>& a_rad,
    const Eigen::Matrix<double, 6, 1>& b_rad,
    const TrajectoryParams& params)
{
    double max_dq_deg = 0.0;
    for (int i = 0; i < 6; ++i) {
        max_dq_deg = std::max(max_dq_deg, std::abs(radToDeg(b_rad(i) - a_rad(i))));
    }

    const double joint_dt_s = max_dq_deg / params.max_joint_velocity_deg_per_s;
    const double sample_dt_s = params.sample_period_ms / 1000.0;
    const double min_dt_s = sample_dt_s * 2.0;
    const double raw_dt_s = std::max(joint_dt_s, min_dt_s);
    const double samples = std::ceil(raw_dt_s / sample_dt_s);
    return samples * sample_dt_s;
}

std::vector<PathPointWithJoints> buildV4Path(
    const std::vector<Eigen::Matrix<double, 6, 1>>& joint_path_rad,
    const TrajectoryParams& params)
{
    std::vector<PathPointWithJoints> path;
    path.reserve(joint_path_rad.size());

    double cumulative_x_mm = 0.0;
    for (size_t i = 0; i < joint_path_rad.size(); ++i) {
        if (i > 0) {
            const double duration_s = gridAlignedSegmentDurationSeconds(
                joint_path_rad[i - 1], joint_path_rad[i], params);
            cumulative_x_mm += duration_s * params.target_line_speed_mm_per_s;
        }

        PathPointWithJoints point;
        point.cart_pos = Eigen::Vector3d(cumulative_x_mm, 0.0, 0.0);
        point.joint = toJointConfigDeg(joint_path_rad[i]);
        point.wire_on = true;
        path.push_back(point);
    }

    return path;
}

}  // namespace

std::vector<TimeParameterization::TimedSample> TimeParameterization::parameterize(
    const std::vector<Eigen::Matrix<double, 6, 1>>& joint_path_rad,
    double target_speed_mm_s,
    BoundaryVelMode mode)
{
    if (mode == BoundaryVelMode::ENDPOINTS_ONLY_ZERO) {
        throw std::runtime_error("Not implemented until W6");
    }

    if (target_speed_mm_s <= 0.0) {
        throw std::invalid_argument(
            "TimeParameterization: target_speed_mm_s must be positive");
    }

    if (joint_path_rad.size() < 2) return {};

    TrajectoryParams params;
    params.target_line_speed_mm_per_s = target_speed_mm_s;
    params.sample_period_ms = 4.0;
    params.zero_endpoint_velocity = true;

    const auto v4_path = buildV4Path(joint_path_rad, params);
    const auto v4_samples = smoothTrajectoryPoly5(v4_path, params);

    std::vector<TimedSample> samples;
    samples.reserve(v4_samples.size());
    for (const auto& v4 : v4_samples) {
        TimedSample sample;
        sample.t = v4.timestamp_ms / 1000.0;
        for (int i = 0; i < 6; ++i) {
            const size_t idx = static_cast<size_t>(i);
            sample.q_rad(i) = degToRad(v4.joint_deg[idx]);
            sample.qd_rad_s(i) = degToRad(v4.joint_velocity_deg_per_s[idx]);
            sample.qdd_rad_s2(i) = degToRad(v4.joint_accel_deg_per_s2[idx]);
        }
        samples.push_back(sample);
    }

    return samples;
}

}  // namespace cslc
