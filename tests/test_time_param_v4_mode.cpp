#include "io/TimeParameterization.h"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

static Eigen::Matrix<double, 6, 1> makeWaypoint(double scale)
{
    Eigen::Matrix<double, 6, 1> q;
    q << scale, 2.0 * scale, 3.0 * scale,
         4.0 * scale, 5.0 * scale, 6.0 * scale;
    return q;
}

static void test_all_waypoints_zero_mode()
{
    using cslc::TimeParameterization;

    std::vector<Eigen::Matrix<double, 6, 1>> joint_path;
    for (int i = 0; i < 5; ++i) {
        joint_path.push_back(makeWaypoint(0.02 * static_cast<double>(i)));
    }

    const auto samples = TimeParameterization::parameterize(
        joint_path, 25.0, TimeParameterization::BoundaryVelMode::ALL_WAYPOINTS_ZERO);

    std::cout << "  time_param samples=" << samples.size()
              << " t_last=" << samples.back().t << "\n";

    require(samples.size() > joint_path.size(),
            "ALL_WAYPOINTS_ZERO: quintic interpolation adds samples");
    require(std::abs(samples.front().t) < 1e-12,
            "ALL_WAYPOINTS_ZERO: first sample starts at t=0");
    require(samples.back().t > 0.0,
            "ALL_WAYPOINTS_ZERO: last sample has positive time");

    for (size_t i = 1; i < samples.size(); ++i) {
        require(samples[i].t > samples[i - 1].t,
                "ALL_WAYPOINTS_ZERO: timestamps are strictly monotonic");
    }

    for (const auto& waypoint : joint_path) {
        const TimeParameterization::TimedSample* best = nullptr;
        double best_err = std::numeric_limits<double>::infinity();
        for (const auto& sample : samples) {
            const double err = (sample.q_rad - waypoint).lpNorm<Eigen::Infinity>();
            if (err < best_err) {
                best_err = err;
                best = &sample;
            }
        }

        require(best != nullptr, "ALL_WAYPOINTS_ZERO: found nearest waypoint sample");
        require(best_err < 1e-8,
                "ALL_WAYPOINTS_ZERO: original waypoint appears in samples");
        require(best->qd_rad_s.lpNorm<Eigen::Infinity>() < 1e-3,
                "ALL_WAYPOINTS_ZERO: every waypoint has near-zero velocity");
    }
}

static void test_endpoints_only_not_implemented()
{
    using cslc::TimeParameterization;

    std::vector<Eigen::Matrix<double, 6, 1>> joint_path = {
        makeWaypoint(0.0),
        makeWaypoint(0.1),
    };

    bool threw = false;
    try {
        (void)TimeParameterization::parameterize(
            joint_path, 25.0,
            TimeParameterization::BoundaryVelMode::ENDPOINTS_ONLY_ZERO);
    } catch (const std::runtime_error& e) {
        threw = true;
        std::cout << "  endpoints_only error=\"" << e.what() << "\"\n";
    }

    require(threw, "ENDPOINTS_ONLY_ZERO: throws until W6 implementation");
}

int main()
{
    try {
        test_all_waypoints_zero_mode();
        test_endpoints_only_not_implemented();
        std::cout << "test_time_param_v4_mode PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_time_param_v4_mode FAILED: " << e.what() << '\n';
        return 1;
    }
}
