#include "field/poisson.h"

#include "field/laplacian.h"

#include <Eigen/Cholesky>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
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

Vec3 vectorAt(const VectorField& field, int x, int y, int z)
{
    return field.values[denseIndex(field.nx, field.ny, x, y, z)];
}

void ensureFiniteVector(const Vec3& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
        throw std::runtime_error("solvePoisson received non-finite VectorField values");
    }
}

}  // namespace

ScalarField solvePoisson(const VoxelGrid& grid, const VectorField& field, const PoissonParams& params)
{
    if (params.max_iterations <= 0) {
        throw std::runtime_error("poisson.max_iterations must be positive");
    }
    if (params.tolerance <= 0.0) {
        throw std::runtime_error("poisson.tolerance must be positive");
    }
    if (grid.nx() != field.nx || grid.ny() != field.ny || grid.nz() != field.nz) {
        throw std::runtime_error("solvePoisson requires grid and VectorField dimensions to match");
    }

    const std::vector<VoxelIndex> voxels = grid.occupiedVoxels();
    if (voxels.empty()) {
        throw std::runtime_error("solvePoisson requires a non-empty occupied grid");
    }
    const auto rows = buildRowMap(grid, voxels);
    const int n = static_cast<int>(voxels.size());

    // v4 §9: 构造 anchor 行集合
    std::unordered_map<int, double> anchor_rows;  // row -> phi value
    if (!params.anchor_voxels.empty()) {
        for (std::size_t i = 0; i < params.anchor_voxels.size(); ++i) {
            const VoxelIndex& voxel = params.anchor_voxels[i];
            const std::size_t key = grid.index(voxel.x, voxel.y, voxel.z);
            if (auto it = rows.find(key); it != rows.end()) {
                anchor_rows[it->second] = params.anchor_values[i];
            }
        }
    } else {
        // 回退到单 anchor 模式
        const std::size_t key = grid.index(params.anchor_voxel.x, params.anchor_voxel.y, params.anchor_voxel.z);
        if (auto it = rows.find(key); it != rows.end()) {
            anchor_rows[it->second] = 0.0;
        }
    }
    if (anchor_rows.empty()) {
        throw std::runtime_error("solvePoisson requires at least one anchor voxel");
    }

    std::vector<Triplet> triplets;
    triplets.reserve(static_cast<std::size_t>(n) * 7);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);

    for (int row = 0; row < n; ++row) {
        // v4 §9: anchor 行用 identity
        if (auto it = anchor_rows.find(row); it != anchor_rows.end()) {
            triplets.emplace_back(row, row, 1.0);
            rhs[row] = it->second;
            continue;
        }

        const VoxelIndex& voxel = voxels[static_cast<std::size_t>(row)];
        const Vec3 center_vector = vectorAt(field, voxel.x, voxel.y, voxel.z);
        ensureFiniteVector(center_vector);

        double diagonal = 0.0;
        double target_sum = 0.0;
        double anchor_rhs = 0.0;
        for (const auto& offset : kNeighbors) {
            const int nx = voxel.x + offset[0];
            const int ny = voxel.y + offset[1];
            const int nz = voxel.z + offset[2];
            if (!grid.occupied(nx, ny, nz)) {
                continue;
            }

            const std::size_t neighbor_key = grid.index(nx, ny, nz);
            const auto neighbor_row = rows.find(neighbor_key);
            if (neighbor_row == rows.end()) {
                continue;
            }

            const Vec3 neighbor_vector = vectorAt(field, nx, ny, nz);
            ensureFiniteVector(neighbor_vector);
            const Vec3 edge_vector = (center_vector + neighbor_vector) * 0.5;
            const Vec3 delta{static_cast<double>(offset[0]) * grid.spacing(),
                             static_cast<double>(offset[1]) * grid.spacing(),
                             static_cast<double>(offset[2]) * grid.spacing()};
            target_sum += dot(edge_vector, delta);

            ++diagonal;
            // v4 §9: 邻居是 anchor 时，把贡献移到 RHS
            if (auto anchor_it = anchor_rows.find(neighbor_row->second); anchor_it != anchor_rows.end()) {
                anchor_rhs += anchor_it->second;
            } else {
                triplets.emplace_back(row, neighbor_row->second, -1.0);
            }
        }

        triplets.emplace_back(row, row, diagonal > 0.0 ? diagonal : 1.0);
        rhs[row] = -target_sum + anchor_rhs;
    }

    SparseMatrix matrix(n, n);
    matrix.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXd solution;
    int total_iterations = 0;
    double final_error = std::numeric_limits<double>::infinity();
    if (params.use_precondition) {
        Eigen::ConjugateGradient<SparseMatrix, Eigen::Lower | Eigen::Upper, Eigen::IncompleteCholesky<double>> solver;
        solver.setTolerance(params.tolerance);
        solver.compute(matrix);
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("solvePoisson IncompleteCholesky ConjugateGradient setup failed");
        }

        solution = Eigen::VectorXd::Zero(n);
        for (int iter = 0; iter < params.max_iterations;) {
            const int step_count = std::min(50, params.max_iterations - iter);
            solver.setMaxIterations(step_count);
            solution = solver.solveWithGuess(rhs, solution);
            iter += static_cast<int>(solver.iterations());
            if (solver.iterations() == 0) {
                iter += step_count;
            }
            total_iterations = iter;
            final_error = solver.error();
            if (params.log_iterations) {
                const std::string label = params.progress_label.empty() ? "poisson" : params.progress_label + " poisson";
                std::cout << "[" << label << "] iter=" << total_iterations
                          << " error=" << final_error << "\n" << std::flush;
            }
            if (std::isfinite(final_error) && final_error <= params.tolerance) {
                break;
            }
            if (iter >= params.max_iterations) {
                break;
            }
        }

        if (!std::isfinite(final_error) || final_error > params.tolerance) {
            throw std::runtime_error("solvePoisson IncompleteCholesky ConjugateGradient did not converge"
                " iterations=" + std::to_string(total_iterations) +
                " error=" + std::to_string(final_error));
        }
    } else {
        Eigen::ConjugateGradient<SparseMatrix, Eigen::Lower | Eigen::Upper> solver;
        solver.setMaxIterations(params.max_iterations);
        solver.setTolerance(params.tolerance);
        solver.compute(matrix);
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("solvePoisson ConjugateGradient setup failed");
        }
        solution = solver.solve(rhs);
        if (solver.info() != Eigen::Success ||
            !std::isfinite(solver.error()) ||
            solver.error() > params.tolerance) {
            throw std::runtime_error("solvePoisson ConjugateGradient did not converge");
        }
        total_iterations = static_cast<int>(solver.iterations());
        final_error = solver.error();
    }

    if (params.out_iterations != nullptr) {
        *params.out_iterations = total_iterations;
    }
    if (params.out_error != nullptr) {
        *params.out_error = final_error;
    }

    const double residual = (matrix * solution - rhs).norm() / std::max(1.0, rhs.norm());
    if (!std::isfinite(residual) || residual > std::max(params.tolerance, 1e-10)) {
        throw std::runtime_error("solvePoisson residual exceeds tolerance");
    }

    ScalarField phi = makeEmptyScalarField(grid);
    for (int row = 0; row < n; ++row) {
        const VoxelIndex& voxel = voxels[static_cast<std::size_t>(row)];
        phi.values[denseIndex(phi.nx, phi.ny, voxel.x, voxel.y, voxel.z)] = solution[row];
    }
    return phi;
}

}  // namespace cslc
