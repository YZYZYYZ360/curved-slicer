#pragma once

#include "core/types.h"
#include "geometry/voxel_grid.h"

#include <stdexcept>
#include <vector>

namespace cslc {

struct VectorField;

struct PoissonParams {
    int max_iterations = 500;
    double tolerance = 1e-6;
    bool use_precondition = true;
    VoxelIndex anchor_voxel{0, 0, 0};
};

struct ScalarField {
    AABB bbox;
    double spacing = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<double> values;
};

inline ScalarField solvePoisson(const VoxelGrid&, const VectorField&, const PoissonParams&)
{
    throw std::runtime_error("not implemented until Phase 3");
}

}  // namespace cslc
