#include "field/field_smoothing.h"

#include <cmath>
#include <unordered_set>

namespace cslc {
namespace {

std::size_t denseIndex(int nx, int ny, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(z));
}

std::unordered_set<std::size_t> buildBcSet(const VoxelGrid& grid, const LaplacianVectorBC& bc)
{
    std::unordered_set<std::size_t> result;
    result.reserve(bc.fixed_indices.size());
    for (const VoxelIndex& vi : bc.fixed_indices) {
        result.insert(grid.index(vi.x, vi.y, vi.z));
    }
    return result;
}

}  // namespace

VectorField smoothVectorField(const VoxelGrid& grid,
                              const VectorField& field,
                              const LaplacianVectorBC& bc,
                              const SmoothingParams& params)
{
    if (grid.nx() != field.nx || grid.ny() != field.ny || grid.nz() != field.nz) {
        throw std::runtime_error("smoothVectorField requires grid and VectorField dimensions to match");
    }

    const std::unordered_set<std::size_t> bc_set = params.preserve_bc ? buildBcSet(grid, bc)
                                                                      : std::unordered_set<std::size_t>{};

    const int nx = field.nx;
    const int ny = field.ny;
    const int nz = field.nz;
    const double wc = params.center_weight;
    const double wn = params.neighbor_weight;

    const int dx[6] = {1, -1, 0, 0, 0, 0};
    const int dy[6] = {0, 0, 1, -1, 0, 0};
    const int dz[6] = {0, 0, 0, 0, 1, -1};

    VectorField current = field;

    for (int pass = 0; pass < params.passes; ++pass) {
        VectorField next = current;

        for (const VoxelIndex& voxel : grid.occupiedVoxels()) {
            const std::size_t idx = denseIndex(nx, ny, voxel.x, voxel.y, voxel.z);

            if (bc_set.count(idx)) {
                continue;
            }

            Vec3 sum = current.values[idx] * wc;
            double count = wc;

            for (int d = 0; d < 6; ++d) {
                const int nx_ = voxel.x + dx[d];
                const int ny_ = voxel.y + dy[d];
                const int nz_ = voxel.z + dz[d];

                if (!grid.inBounds(nx_, ny_, nz_)) {
                    continue;
                }
                if (!grid.occupied(nx_, ny_, nz_)) {
                    continue;
                }

                const std::size_t nidx = denseIndex(nx, ny, nx_, ny_, nz_);
                if (bc_set.count(nidx)) {
                    continue;
                }

                sum = sum + current.values[nidx] * wn;
                count += wn;
            }

            if (count > 0.0) {
                Vec3 avg = sum * (1.0 / count);
                const double len = norm(avg);
                if (len > 0.0) {
                    next.values[idx] = avg * (1.0 / len);
                }
            }
        }

        current = next;
    }

    return current;
}

}  // namespace cslc
