#pragma once

#include "core/types.h"
#include "field/poisson.h"
#include "geometry/sdf.h"

#include <array>
#include <string>
#include <vector>

namespace cslc {

struct BCParams {
    std::string strategy = "bottom_up";
    std::array<double, 3> print_direction{0.0, 0.0, 1.0};
    double bottom_dot_threshold = -0.5;
    double bottom_sdf_band = 1.0;
};

struct LaplacianBC {
    std::vector<VoxelIndex> fixed_indices;
    std::vector<double> fixed_values;
};

struct LaplacianParams {
    int max_iterations = 2000;
    double tolerance = 1e-6;
};

struct VectorField {
    AABB bbox;
    double spacing = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<Vec3> values;
};

LaplacianBC generateBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params);
ScalarField solveLaplacian(const VoxelGrid& grid, const LaplacianBC& bc, const LaplacianParams& params);

}  // namespace cslc
