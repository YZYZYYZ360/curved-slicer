#pragma once

#include <array>
#include <cstdint>

namespace cslc {

struct CartPose {
    double X = 0.0;  // mm
    double Y = 0.0;  // mm
    double Z = 0.0;  // mm
    double A = 0.0;  // deg, KUKA ZYX Euler: yaw around Z
    double B = 0.0;  // deg, pitch around Y'
    double C = 0.0;  // deg, roll around X''
};

struct JointConfig {
    std::array<double, 6> q_deg{};
};

enum class IKStatus : uint8_t {
    OK = 0,
    OutOfWorkspace = 1,
    JointLimit = 2,
    NearSingularity = 3,
};

struct IKResult {
    IKStatus status = IKStatus::OutOfWorkspace;
    JointConfig solution{};
    std::array<JointConfig, 8> all_solutions{};
    int num_valid = 0;
};

}  // namespace cslc
