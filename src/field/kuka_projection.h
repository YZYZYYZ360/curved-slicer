#pragma once

#include "field/laplacian.h"

namespace cslc {

struct ReachabilityParams {
    Vec3 workpiece_up{0.0, 0.0, 1.0};
    double min_dot_threshold = 0.05;
};

VectorField projectToHemisphere(const VoxelGrid& grid,
                                const VectorField& field,
                                const ReachabilityParams& params);

}  // namespace cslc
