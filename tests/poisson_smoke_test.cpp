#include "field/laplacian.h"
#include "field/poisson.h"

#include <cmath>
#include <iostream>
#include <limits>
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
    std::cerr << "poisson_smoke_test failed: " << message << '\n';
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

cslc::VectorField makeConstantUpField(const cslc::VoxelGrid& grid)
{
    cslc::VectorField field;
    field.bbox = grid.bbox();
    field.spacing = grid.spacing();
    field.nx = grid.nx();
    field.ny = grid.ny();
    field.nz = grid.nz();
    field.values.assign(static_cast<std::size_t>(field.nx) *
                            static_cast<std::size_t>(field.ny) *
                            static_cast<std::size_t>(field.nz),
                        cslc::Vec3{0.0, 0.0, 1.0});
    return field;
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
        const cslc::VectorField field = makeConstantUpField(grid);

        cslc::PoissonParams params;
        params.anchor_voxel = {n / 2, n / 2, 0};
        params.max_iterations = 1000;
        params.tolerance = 1e-8;
        params.use_precondition = false;

        const cslc::ScalarField phi = cslc::solvePoisson(grid, field, params);
        const double bottom = phi.values[denseIndex(n, n, n / 2, n / 2, 0)];
        const double middle = phi.values[denseIndex(n, n, n / 2, n / 2, n / 2)];
        const double top = phi.values[denseIndex(n, n, n / 2, n / 2, n - 1)];

        require(std::isfinite(bottom) && std::isfinite(middle) && std::isfinite(top),
                "phi centerline should be finite");
        require(std::abs(bottom) < 1e-8, "anchor bottom phi should be 0");
        require(middle > bottom, "middle phi should be above bottom");
        require(top > middle, "top phi should be above middle");
        require(std::abs((top - bottom) - static_cast<double>(n - 1)) < 1e-6,
                "constant up vector should reconstruct z height");

        for (int z = 1; z < n; ++z) {
            const double previous = phi.values[denseIndex(n, n, n / 2, n / 2, z - 1)];
            const double current = phi.values[denseIndex(n, n, n / 2, n / 2, z)];
            require(current + 1e-8 >= previous, "centerline phi should be monotonic");
        }

        std::cout << "poisson_smoke bottom=" << bottom
                  << " middle=" << middle
                  << " top=" << top << '\n';
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }
    return 0;
}
