#pragma once

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace cslc {

class KRLWriter {
public:
    struct LinSegment {
        Eigen::Matrix<double, 6, 1> q_rad = Eigen::Matrix<double, 6, 1>::Zero();
        double velocity_mm_s = 0.0;
        // T4 callers must pass only OK executable segments. W8 will extend
        // this interface with status-bearing rows and enforce R4 in writer.
    };

    static void write(
        const std::string& path,
        const std::string& program_name,
        const std::vector<LinSegment>& segments);
};

}  // namespace cslc
