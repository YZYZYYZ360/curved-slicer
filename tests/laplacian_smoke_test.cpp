#include "field/laplacian.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

std::size_t denseIndex(int nx, int ny, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(z));
}

int fail(const std::string& message)
{
    std::cerr << "laplacian_smoke_test failed: " << message << '\n';
    return 1;
}

cslc::VoxelGrid makeBlockGrid(int n)
{
    cslc::VoxelGrid grid({{0.0, 0.0, 0.0}, {static_cast<double>(n), static_cast<double>(n), static_cast<double>(n)}},
                         1.0,
                         n,
                         n,
                         n);
    for (int z = 0; z < n; ++z) {
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                grid.setOccupied(x, y, z);
            }
        }
    }
    return grid;
}

cslc::SDF makeBlockSdf(int n)
{
    cslc::SDF sdf;
    sdf.bbox = {{0.0, 0.0, 0.0}, {static_cast<double>(n), static_cast<double>(n), static_cast<double>(n)}};
    sdf.spacing = 1.0;
    sdf.nx = n;
    sdf.ny = n;
    sdf.nz = n;
    sdf.values.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(n) * static_cast<std::size_t>(n));

    for (int z = 0; z < n; ++z) {
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const double dx = std::min(static_cast<double>(x) + 0.5, static_cast<double>(n) - x - 0.5);
                const double dy = std::min(static_cast<double>(y) + 0.5, static_cast<double>(n) - y - 0.5);
                const double dz = std::min(static_cast<double>(z) + 0.5, static_cast<double>(n) - z - 0.5);
                sdf.values[denseIndex(n, n, x, y, z)] = -std::min({dx, dy, dz});
            }
        }
    }
    return sdf;
}

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

int main()
{
    try {
        constexpr int n = 10;
        const cslc::VoxelGrid grid = makeBlockGrid(n);
        const cslc::SDF sdf = makeBlockSdf(n);

        cslc::BCParams bc_params;
        bc_params.bottom_sdf_band = 1.1;
        const cslc::LaplacianBC bc = cslc::generateBC(grid, sdf, bc_params);
        require(!bc.fixed_indices.empty(), "bottom_up BC should find bottom voxels");
        require(std::find(bc.fixed_values.begin(), bc.fixed_values.end(), 0.0) != bc.fixed_values.end(),
                "BC should include bottom phi=0");
        require(std::find(bc.fixed_values.begin(), bc.fixed_values.end(), 1.0) != bc.fixed_values.end(),
                "BC should include top phi=1");

        cslc::LaplacianParams params;
        params.tolerance = 1e-8;
        params.max_iterations = 1000;
        const cslc::ScalarField phi = cslc::solveLaplacian(grid, bc, params);

        const double bottom = phi.values[denseIndex(n, n, n / 2, n / 2, 0)];
        const double middle = phi.values[denseIndex(n, n, n / 2, n / 2, n / 2)];
        const double top = phi.values[denseIndex(n, n, n / 2, n / 2, n - 1)];
        require(std::abs(bottom) < 1e-8, "bottom phi should be 0");
        require(std::abs(top - 1.0) < 1e-8, "top phi should be 1");
        require(middle > bottom && middle < top, "middle phi should be between bottom and top");
        for (int z = 1; z < n; ++z) {
            const double previous = phi.values[denseIndex(n, n, n / 2, n / 2, z - 1)];
            const double current = phi.values[denseIndex(n, n, n / 2, n / 2, z)];
            require(current + 1e-8 >= previous, "centerline phi should be monotonic");
        }

        std::cout << "laplacian_smoke fixed=" << bc.fixed_indices.size()
                  << " bottom=" << bottom
                  << " middle=" << middle
                  << " top=" << top << '\n';
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }
    return 0;
}
