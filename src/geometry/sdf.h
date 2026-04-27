#pragma once

#include "geometry/voxel_grid.h"

#include <vector>

namespace cslc {

struct SDF {
    AABB bbox;
    double spacing = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<double> values;
    double narrow_band_mm = 0.0;
};

SDF buildSDF(const TriangleMesh& mesh, const VoxelParams& params);

}  // namespace cslc
