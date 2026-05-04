#pragma once

#include "core/types.h"
#include "geometry/voxel_grid.h"

#include <string>
#include <vector>

namespace cslc {

struct VectorField;

struct PoissonParams {
    int max_iterations = 500;
    double tolerance = 1e-6;
    bool use_precondition = true;
    VoxelIndex anchor_voxel{0, 0, 0};
    bool log_iterations = false;
    std::string progress_label;
    int* out_iterations = nullptr;
    double* out_error = nullptr;
};

struct ScalarField {
    AABB bbox;
    double spacing = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<double> values;
};

ScalarField solvePoisson(const VoxelGrid& grid, const VectorField& field, const PoissonParams& params);

}  // namespace cslc
