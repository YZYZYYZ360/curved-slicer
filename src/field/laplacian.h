#pragma once

#include "core/types.h"
#include "geometry/voxel_grid.h"

#include <stdexcept>
#include <vector>

namespace cslc {

struct LaplacianBC {
    std::vector<VoxelIndex> fixed_indices;
    std::vector<Vec3> fixed_vectors;
};

struct LaplacianParams {
    int max_iterations = 2000;
    double tolerance = 1e-6;
    bool normalize_each_iter = false;
};

struct VectorField {
    AABB bbox;
    double spacing = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<Vec3> values;
};

inline LaplacianBC generateBC(const VoxelGrid&, const LaplacianBC&)
{
    throw std::logic_error("not_implemented: generateBC is deferred to Phase 2");
}

inline VectorField solveLaplacian(const VoxelGrid&, const LaplacianBC&, const LaplacianParams&)
{
    throw std::logic_error("not_implemented: solveLaplacian is deferred to Phase 2");
}

}  // namespace cslc
