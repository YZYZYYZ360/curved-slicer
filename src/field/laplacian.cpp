#include "field/laplacian.h"

#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace cslc {
namespace {

using SparseMatrix = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;

constexpr std::array<std::array<int, 3>, 6> kNeighbors{{
    {{-1, 0, 0}},
    {{1, 0, 0}},
    {{0, -1, 0}},
    {{0, 1, 0}},
    {{0, 0, -1}},
    {{0, 0, 1}},
}};

std::size_t denseIndex(int nx, int ny, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(z));
}

std::size_t sdfIndex(const SDF& sdf, int x, int y, int z)
{
    return denseIndex(sdf.nx, sdf.ny, x, y, z);
}

double sdfValue(const SDF& sdf, int x, int y, int z)
{
    x = std::max(0, std::min(x, sdf.nx - 1));
    y = std::max(0, std::min(y, sdf.ny - 1));
    z = std::max(0, std::min(z, sdf.nz - 1));
    return sdf.values[sdfIndex(sdf, x, y, z)];
}

Vec3 sdfGradient(const SDF& sdf, int x, int y, int z)
{
    const auto derivative = [&sdf](int x0, int y0, int z0, int axis) {
        int minus_x = x0;
        int minus_y = y0;
        int minus_z = z0;
        int plus_x = x0;
        int plus_y = y0;
        int plus_z = z0;
        if (axis == 0) {
            minus_x = std::max(0, x0 - 1);
            plus_x = std::min(sdf.nx - 1, x0 + 1);
        } else if (axis == 1) {
            minus_y = std::max(0, y0 - 1);
            plus_y = std::min(sdf.ny - 1, y0 + 1);
        } else {
            minus_z = std::max(0, z0 - 1);
            plus_z = std::min(sdf.nz - 1, z0 + 1);
        }

        const int delta = std::abs(plus_x - minus_x) +
            std::abs(plus_y - minus_y) +
            std::abs(plus_z - minus_z);
        if (delta == 0) {
            return 0.0;
        }
        return (sdfValue(sdf, plus_x, plus_y, plus_z) -
                sdfValue(sdf, minus_x, minus_y, minus_z)) /
            (static_cast<double>(delta) * sdf.spacing);
    };

    return {derivative(x, y, z, 0), derivative(x, y, z, 1), derivative(x, y, z, 2)};
}

Vec3 directionFromParams(const BCParams& params)
{
    const Vec3 direction{
        params.print_direction[0],
        params.print_direction[1],
        params.print_direction[2],
    };
    const Vec3 unit = normalized(direction);
    if (norm(unit) == 0.0) {
        throw std::runtime_error("field.boundary.print_direction must be non-zero");
    }
    return unit;
}

std::unordered_map<std::size_t, int> buildRowMap(const VoxelGrid& grid,
                                                 const std::vector<VoxelIndex>& voxels)
{
    std::unordered_map<std::size_t, int> rows;
    rows.reserve(voxels.size());
    for (int row = 0; row < static_cast<int>(voxels.size()); ++row) {
        const VoxelIndex& voxel = voxels[static_cast<std::size_t>(row)];
        rows.emplace(grid.index(voxel.x, voxel.y, voxel.z), row);
    }
    return rows;
}

std::unordered_map<std::size_t, double> buildFixedMap(const VoxelGrid& grid, const LaplacianBC& bc)
{
    if (bc.fixed_indices.size() != bc.fixed_values.size()) {
        throw std::runtime_error("LaplacianBC fixed index/value count mismatch");
    }

    std::unordered_map<std::size_t, double> fixed;
    fixed.reserve(bc.fixed_indices.size());
    for (std::size_t i = 0; i < bc.fixed_indices.size(); ++i) {
        const VoxelIndex& voxel = bc.fixed_indices[i];
        if (grid.occupied(voxel.x, voxel.y, voxel.z)) {
            fixed[grid.index(voxel.x, voxel.y, voxel.z)] = bc.fixed_values[i];
        }
    }
    if (fixed.empty()) {
        throw std::runtime_error("solveLaplacian requires Dirichlet voxels");
    }
    return fixed;
}

ScalarField makeEmptyScalarField(const VoxelGrid& grid)
{
    ScalarField field;
    field.bbox = grid.bbox();
    field.spacing = grid.spacing();
    field.nx = grid.nx();
    field.ny = grid.ny();
    field.nz = grid.nz();
    field.values.assign(
        static_cast<std::size_t>(field.nx) * static_cast<std::size_t>(field.ny) * static_cast<std::size_t>(field.nz),
        std::numeric_limits<double>::quiet_NaN());
    return field;
}

}  // namespace

LaplacianBC generateBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params)
{
    if (params.strategy != "bottom_up") {
        throw std::runtime_error("not implemented in Phase 2");
    }
    if (grid.nx() != sdf.nx || grid.ny() != sdf.ny || grid.nz() != sdf.nz) {
        throw std::runtime_error("generateBC requires grid and SDF dimensions to match");
    }

    const Vec3 print_direction = directionFromParams(params);
    const double sdf_band = params.bottom_sdf_band * grid.spacing();
    std::size_t bottom_count = 0;
    std::size_t top_count = 0;

    LaplacianBC bc;
    const std::vector<VoxelIndex> voxels = grid.occupiedVoxels();
    bc.fixed_indices.reserve(voxels.size() / 10 + 1);
    bc.fixed_values.reserve(voxels.size() / 10 + 1);

    for (const VoxelIndex& voxel : voxels) {
        if (std::abs(sdfValue(sdf, voxel.x, voxel.y, voxel.z)) > sdf_band) {
            continue;
        }

        const Vec3 normal = normalized(sdfGradient(sdf, voxel.x, voxel.y, voxel.z));
        if (norm(normal) == 0.0) {
            continue;
        }

        const double alignment = dot(normal, print_direction);
        if (alignment < params.bottom_dot_threshold) {
            bc.fixed_indices.push_back(voxel);
            bc.fixed_values.push_back(0.0);
            ++bottom_count;
        } else if (alignment > -params.bottom_dot_threshold) {
            bc.fixed_indices.push_back(voxel);
            bc.fixed_values.push_back(1.0);
            ++top_count;
        }
    }

    if (bottom_count == 0) {
        throw std::runtime_error("generateBC bottom_up found no bottom boundary voxels");
    }
    if (top_count == 0) {
        throw std::runtime_error("generateBC bottom_up found no top boundary voxels");
    }
    return bc;
}

ScalarField solveLaplacian(const VoxelGrid& grid, const LaplacianBC& bc, const LaplacianParams& params)
{
    if (params.max_iterations <= 0) {
        throw std::runtime_error("laplacian.max_iterations must be positive");
    }
    if (params.tolerance <= 0.0) {
        throw std::runtime_error("laplacian.tolerance must be positive");
    }

    const std::vector<VoxelIndex> voxels = grid.occupiedVoxels();
    if (voxels.empty()) {
        throw std::runtime_error("solveLaplacian requires a non-empty occupied grid");
    }

    const auto rows = buildRowMap(grid, voxels);
    const auto fixed = buildFixedMap(grid, bc);
    const int n = static_cast<int>(voxels.size());

    std::vector<Triplet> triplets;
    triplets.reserve(static_cast<std::size_t>(n) * 7);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);

    for (int row = 0; row < n; ++row) {
        const VoxelIndex& voxel = voxels[static_cast<std::size_t>(row)];
        const std::size_t key = grid.index(voxel.x, voxel.y, voxel.z);
        const auto fixed_it = fixed.find(key);
        if (fixed_it != fixed.end()) {
            triplets.emplace_back(row, row, 1.0);
            rhs[row] = fixed_it->second;
            continue;
        }

        double diagonal = 0.0;
        for (const auto& offset : kNeighbors) {
            const int nx = voxel.x + offset[0];
            const int ny = voxel.y + offset[1];
            const int nz = voxel.z + offset[2];
            if (!grid.occupied(nx, ny, nz)) {
                continue;
            }

            ++diagonal;
            const std::size_t neighbor_key = grid.index(nx, ny, nz);
            const auto neighbor_fixed = fixed.find(neighbor_key);
            if (neighbor_fixed != fixed.end()) {
                rhs[row] += neighbor_fixed->second;
            } else {
                const auto neighbor_row = rows.find(neighbor_key);
                if (neighbor_row != rows.end()) {
                    triplets.emplace_back(row, neighbor_row->second, -1.0);
                }
            }
        }

        triplets.emplace_back(row, row, diagonal > 0.0 ? diagonal : 1.0);
    }

    SparseMatrix matrix(n, n);
    matrix.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::ConjugateGradient<SparseMatrix, Eigen::Lower | Eigen::Upper> solver;
    solver.setMaxIterations(params.max_iterations);
    solver.setTolerance(params.tolerance);
    solver.compute(matrix);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("solveLaplacian ConjugateGradient setup failed");
    }

    const Eigen::VectorXd solution = solver.solve(rhs);
    if (solver.info() != Eigen::Success || !std::isfinite(solver.error()) || solver.error() > params.tolerance) {
        throw std::runtime_error("solveLaplacian ConjugateGradient did not converge");
    }

    ScalarField phi = makeEmptyScalarField(grid);
    for (int row = 0; row < n; ++row) {
        const VoxelIndex& voxel = voxels[static_cast<std::size_t>(row)];
        phi.values[denseIndex(phi.nx, phi.ny, voxel.x, voxel.y, voxel.z)] = solution[row];
    }
    return phi;
}

}  // namespace cslc
