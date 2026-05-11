#pragma once

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cslc {

// KR4 R600 DH parameters (standard DH convention)
// Source: mstraj0110.m + KUKA KR4 R600 datasheet
// Convention: A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
struct KR4DHParams {
    // Link lengths (mm)
    std::array<double, 6> a_mm = {0.0, 290.0, 20.0, 0.0, 0.0, 0.0};
    // Link twists (rad)
    std::array<double, 6> alpha_rad = {
        M_PI / 2.0,     // J1: alpha=90
        M_PI,            // J2: alpha=180
        -M_PI / 2.0,    // J3: alpha=-90
        M_PI / 2.0,     // J4: alpha=90
        -M_PI / 2.0,    // J5: alpha=-90
        0.0              // J6: alpha=0
    };
    // Link offsets (mm)
    std::array<double, 6> d_mm = {330.0, 0.0, 0.0, 310.0, 0.0, 0.0};
    // Flange z offset (mm) - from mstraj0110.m Tz(0.012)
    double flange_z_mm = 12.0;
    // Tool z offset (mm) - user calibrated, from [kuka.robot] z_offset
    double tool_z_mm = 0.28;
    // Joint limits (deg) [min, max]
    std::array<std::array<double, 2>, 6> qlim_deg = {{
        {-170.0, 170.0},   // J1
        {-195.0, 40.0},    // J2
        {-115.0, 150.0},   // J3
        {-185.0, 185.0},   // J4
        {-120.0, 120.0},   // J5
        {-350.0, 350.0},   // J6
    }};
    // Max joint speeds (deg/s)
    std::array<double, 6> qmax_speed_deg_per_s = {336.0, 336.0, 488.0, 600.0, 529.0, 800.0};
};

}  // namespace cslc
