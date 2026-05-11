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

std::unordered_map<std::size_t, Vec3> buildFixedVectorMap(const VoxelGrid& grid,
                                                          const LaplacianVectorBC& bc)
{
    if (bc.fixed_indices.size() != bc.fixed_vectors.size()) {
        throw std::runtime_error("LaplacianVectorBC fixed index/vector count mismatch");
    }

    std::unordered_map<std::size_t, Vec3> fixed;
    fixed.reserve(bc.fixed_indices.size());
    for (std::size_t i = 0; i < bc.fixed_indices.size(); ++i) {
        const VoxelIndex& voxel = bc.fixed_indices[i];
        if (grid.occupied(voxel.x, voxel.y, voxel.z)) {
            fixed[grid.index(voxel.x, voxel.y, voxel.z)] = normalized(bc.fixed_vectors[i]);
        }
    }
    if (fixed.empty()) {
        throw std::runtime_error("solveLaplacianVector requires Dirichlet voxels");
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

VectorField makeEmptyVectorField(const VoxelGrid& grid)
{
    VectorField field;
    field.bbox = grid.bbox();
    field.spacing = grid.spacing();
    field.nx = grid.nx();
    field.ny = grid.ny();
    field.nz = grid.nz();
    field.values.assign(
        static_cast<std::size_t>(field.nx) * static_cast<std::size_t>(field.ny) * static_cast<std::size_t>(field.nz),
        Vec3{std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN()});
    return field;
}

// v4 §10: geometric_z band 自动推算
double computeBand(const BCParams& params, double zspan)
{
    if (params.bottom_band_mm > 0.0) return params.bottom_band_mm;
    return std::clamp(zspan * 0.05, params.band_min_mm, params.band_max_mm);
}

LaplacianBC generateBC_geometric_z(const VoxelGrid& grid, const BCParams& params)
{
    const AABB& bbox = grid.bbox();
    const double zspan = bbox.max.z - bbox.min.z;
    const double bot_band = computeBand(params, zspan);
    const double top_band = params.top_band_mm > 0.0
        ? params.top_band_mm
        : std::clamp(zspan * 0.05, params.band_min_mm, params.band_max_mm);

    LaplacianBC bc;
    const std::vector<VoxelIndex> voxels = grid.occupiedVoxels();
    bc.fixed_indices.reserve(voxels.size() / 5 + 1);
    bc.fixed_values.reserve(voxels.size() / 5 + 1);

    for (const VoxelIndex& voxel : voxels) {
        const double z_world = bbox.min.z + (voxel.z + 0.5) * grid.spacing();
        if (z_world - bbox.min.z < bot_band) {
            bc.fixed_indices.push_back(voxel);
            bc.fixed_values.push_back(0.0);
        } else if (bbox.max.z - z_world < top_band) {
            bc.fixed_indices.push_back(voxel);
            bc.fixed_values.push_back(1.0);
        }
    }

    if (bc.fixed_indices.empty()) {
        throw std::runtime_error("generateBC geometric_z found no boundary voxels");
    }
    return bc;
}

LaplacianVectorBC generateVectorBC_geometric_z(const VoxelGrid& grid, const BCParams& params)
{
    const Vec3 print_direction = directionFromParams(params);
    const AABB& bbox = grid.bbox();
    const double zspan = bbox.max.z - bbox.min.z;
    const double bot_band = computeBand(params, zspan);
    const double top_band = params.top_band_mm > 0.0
        ? params.top_band_mm
        : std::clamp(zspan * 0.05, params.band_min_mm, params.band_max_mm);

    LaplacianVectorBC bc;
    const std::vector<VoxelIndex> voxels = grid.occupiedVoxels();
    bc.fixed_indices.reserve(voxels.size() / 5 + 1);
    bc.fixed_vectors.reserve(voxels.size() / 5 + 1);
    bc.bc_zones.reserve(voxels.size() / 5 + 1);

    for (const VoxelIndex& voxel : voxels) {
        const double z_world = bbox.min.z + (voxel.z + 0.5) * grid.spacing();
        if (z_world - bbox.min.z < bot_band) {
            bc.fixed_indices.push_back(voxel);
            bc.fixed_vectors.push_back(print_direction);
            bc.bc_zones.push_back(BCZone::Bottom);
        } else if (bbox.max.z - z_world < top_band) {
            bc.fixed_indices.push_back(voxel);
            bc.fixed_vectors.push_back(print_direction);
            bc.bc_zones.push_back(BCZone::Top);
        }
    }

    if (bc.fixed_indices.empty()) {
        throw std::runtime_error("generateVectorBC geometric_z found no boundary voxels");
    }
    return bc;
}

}  // namespace

LaplacianBC generateBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params)
{
    if (params.strategy == "geometric_z") {
        return generateBC_geometric_z(grid, params);
    }
    if (params.strategy != "bottom_up") {
        throw std::runtime_error("generateBC: unknown strategy '" + params.strategy + "'");
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

LaplacianVectorBC generateVectorBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params)
{
    if (params.strategy == "geometric_z") {
        return generateVectorBC_geometric_z(grid, params);
    }
    if (params.strategy != "bottom_up") {
        throw std::runtime_error("generateVectorBC: unknown strategy '" + params.strategy + "'");
    }
    if (grid.nx() != sdf.nx || grid.ny() != sdf.ny || grid.nz() != sdf.nz) {
        throw std::runtime_error("generateVectorBC requires grid and SDF dimensions to match");
    }

    const Vec3 print_direction = directionFromParams(params);
    const double sdf_band = params.bottom_sdf_band * grid.spacing();
    std::size_t bottom_count = 0;
    std::size_t top_count = 0;

    LaplacianVectorBC bc;
    const std::vector<VoxelIndex> voxels = grid.occupiedVoxels();
    bc.fixed_indices.reserve(voxels.size() / 10 + 1);
    bc.fixed_vectors.reserve(voxels.size() / 10 + 1);

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
            bc.fixed_vectors.push_back(print_direction);
            ++bottom_count;
        } else if (alignment > -params.bottom_dot_threshold) {
            bc.fixed_indices.push_back(voxel);
            bc.fixed_vectors.push_back(normal);
            ++top_count;
        }
    }

    if (bottom_count == 0) {
        throw std::runtime_error("generateVectorBC bottom_up found no bottom boundary voxels");
    }
    if (top_count == 0) {
        throw std::runtime_error("generateVectorBC bottom_up found no top boundary voxels");
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

VectorField solveLaplacianVector(const VoxelGrid& grid,
                                 const LaplacianVectorBC& bc,
                                 const LaplacianParams& params)
{
    if (params.max_iterations <= 0) {
        throw std::runtime_error("laplacian.max_iterations must be positive");
    }
    if (params.tolerance <= 0.0) {
        throw std::runtime_error("laplacian.tolerance must be positive");
    }

    const std::vector<VoxelIndex> voxels = grid.occupiedVoxels();
    if (voxels.empty()) {
        throw std::runtime_error("solveLaplacianVector requires a non-empty occupied grid");
    }

    const auto rows = buildRowMap(grid, voxels);
    const auto fixed = buildFixedVectorMap(grid, bc);
    const int n = static_cast<int>(voxels.size());

    std::vector<Triplet> triplets;
    triplets.reserve(static_cast<std::size_t>(n) * 7);
    std::array<Eigen::VectorXd, 3> rhs{
        Eigen::VectorXd::Zero(n),
        Eigen::VectorXd::Zero(n),
        Eigen::VectorXd::Zero(n),
    };

    for (int row = 0; row < n; ++row) {
        const VoxelIndex& voxel = voxels[static_cast<std::size_t>(row)];
        const std::size_t key = grid.index(voxel.x, voxel.y, voxel.z);
        const auto fixed_it = fixed.find(key);
        if (fixed_it != fixed.end()) {
            triplets.emplace_back(row, row, 1.0);
            rhs[0][row] = fixed_it->second.x;
            rhs[1][row] = fixed_it->second.y;
            rhs[2][row] = fixed_it->second.z;
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
                rhs[0][row] += neighbor_fixed->second.x;
                rhs[1][row] += neighbor_fixed->second.y;
                rhs[2][row] += neighbor_fixed->second.z;
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
        throw std::runtime_error("solveLaplacianVector ConjugateGradient setup failed");
    }

    std::array<Eigen::VectorXd, 3> solution{
        Eigen::VectorXd::Zero(n),
        Eigen::VectorXd::Zero(n),
        Eigen::VectorXd::Zero(n),
    };
    for (int axis = 0; axis < 3; ++axis) {
        solution[axis] = solver.solve(rhs[axis]);
        if (solver.info() != Eigen::Success ||
            !std::isfinite(solver.error()) ||
            solver.error() > params.tolerance) {
            throw std::runtime_error("solveLaplacianVector ConjugateGradient did not converge");
        }
    }

    VectorField field = makeEmptyVectorField(grid);
    for (int row = 0; row < n; ++row) {
        const VoxelIndex& voxel = voxels[static_cast<std::size_t>(row)];
        Vec3 value{solution[0][row], solution[1][row], solution[2][row]};
        if (norm(value) > 0.0) {
            value = normalized(value);
        }
        field.values[denseIndex(field.nx, field.ny, voxel.x, voxel.y, voxel.z)] = value;
    }
    return field;
}

}  // namespace cslc
