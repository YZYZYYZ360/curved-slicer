#pragma once

#include "field/poisson.h"
#include "geometry/voxel_grid.h"

#include <string>

namespace cslc {

struct WavefrontParams {
    std::string seed_strategy = "bottom";
    double isovalue_mode = 0.5;
    double height_interval_mm = 1.0;
    int serial_threshold = 1000;
    int parallel_thread_num = 24;
};

ScalarField solveWavefront(const VoxelGrid& grid, const WavefrontParams& params = {});

}  // namespace cslc
