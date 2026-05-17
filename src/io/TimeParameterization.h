#pragma once

#include <Eigen/Dense>

#include <vector>

namespace cslc {

class TimeParameterization {
public:
    enum class BoundaryVelMode {
        ALL_WAYPOINTS_ZERO,
        ENDPOINTS_ONLY_ZERO,
    };

    struct TimedSample {
        double t = 0.0;
        Eigen::Matrix<double, 6, 1> q_rad = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> qd_rad_s = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> qdd_rad_s2 = Eigen::Matrix<double, 6, 1>::Zero();
    };

    static std::vector<TimedSample> parameterize(
        const std::vector<Eigen::Matrix<double, 6, 1>>& joint_path_rad,
        double target_speed_mm_s,
        BoundaryVelMode mode = BoundaryVelMode::ALL_WAYPOINTS_ZERO);
};

}  // namespace cslc
