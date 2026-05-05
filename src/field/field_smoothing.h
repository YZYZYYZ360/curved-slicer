#pragma once

#include "field/laplacian.h"

namespace cslc {

struct SmoothingParams {
    int passes = 1;
    double center_weight = 1.0;
    double neighbor_weight = 1.0;
    bool preserve_bc = true;
};

VectorField smoothVectorField(const VoxelGrid& grid,
                              const VectorField& field,
                              const LaplacianVectorBC& bc,
                              const SmoothingParams& params);

}  // namespace cslc
