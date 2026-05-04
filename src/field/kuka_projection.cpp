#include "field/kuka_projection.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace cslc {
namespace {

std::size_t denseIndex(int nx, int ny, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(z));
}

}  // namespace

VectorField projectToHemisphere(const VoxelGrid& grid,
                                const VectorField& field,
                                const ReachabilityParams& params)
{
    if (grid.nx() != field.nx || grid.ny() != field.ny || grid.nz() != field.nz) {
        throw std::runtime_error("projectToHemisphere requires grid and VectorField dimensions to match");
    }
    if (params.min_dot_threshold < -1.0 || params.min_dot_threshold > 1.0) {
        throw std::runtime_error("reachability.min_dot_threshold must be in [-1, 1]");
    }
    const Vec3 up = normalized(params.workpiece_up);
    if (norm(up) == 0.0) {
        throw std::runtime_error("reachability.workpiece_up must be non-zero");
    }

    VectorField projected = field;
    for (const VoxelIndex& voxel : grid.occupiedVoxels()) {
        const std::size_t index = denseIndex(field.nx, field.ny, voxel.x, voxel.y, voxel.z);
        Vec3 value = field.values[index];
        if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
            continue;
        }
        value = normalized(value);
        if (norm(value) == 0.0) {
            value = up;
        }

        const double projection = dot(value, up);
        if (projection < params.min_dot_threshold) {
            value = normalized(value + up * (params.min_dot_threshold - projection));
        }
        if (norm(value) == 0.0) {
            value = up;
        }
        projected.values[index] = value;
    }
    return projected;
}

}  // namespace cslc
