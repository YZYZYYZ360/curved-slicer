#include "voxel/VoxelizationV6.h"

#include "geometry/voxel_grid.h"
#include "io/stl_reader.h"

#include <igl/winding_number.h>

#include <omp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace cslc {
namespace {

constexpr long long kMaxVoxels = 200'000'000LL;
constexpr int kWindingChunkSize = 100'000;

void validateInput(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    if (V.cols() != 3 || V.rows() == 0) {
        throw std::invalid_argument("VoxelizationV6: V must be N x 3 and non-empty");
    }
    if (F.cols() != 3 || F.rows() == 0) {
        throw std::invalid_argument("VoxelizationV6: F must be M x 3 and non-empty");
    }
    for (int f = 0; f < F.rows(); ++f) {
        for (int c = 0; c < 3; ++c) {
            if (F(f, c) < 0 || F(f, c) >= V.rows()) {
                throw std::invalid_argument("VoxelizationV6: F contains out-of-range index");
            }
        }
    }
}

int axisCount(double extent, double spacing)
{
    const double raw = std::ceil(extent / spacing);
    if (!std::isfinite(raw) || raw <= 0.0 ||
        raw > static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("VoxelizationV6 grid dimension is invalid or too large");
    }
    return static_cast<int>(raw);
}

long long totalVoxelCount(const Eigen::Vector3i& dims)
{
    return static_cast<long long>(dims.x()) *
        static_cast<long long>(dims.y()) *
        static_cast<long long>(dims.z());
}

VoxelizationV6::Grid makeEmptyGrid(
    const Eigen::MatrixXd& V,
    double spacing,
    double padding)
{
    if (spacing <= 0.0) {
        throw std::runtime_error("VoxelizationV6 spacing_mm must be positive");
    }
    if (padding < 0.0) {
        throw std::runtime_error("VoxelizationV6 bbox_padding_mm must be non-negative");
    }

    const Eigen::Vector3d bbox_min = V.colwise().minCoeff();
    const Eigen::Vector3d bbox_max = V.colwise().maxCoeff();

    VoxelizationV6::Grid grid;
    grid.origin_mm = bbox_min - Eigen::Vector3d::Constant(padding);
    grid.spacing_mm = spacing;

    const Eigen::Vector3d padded_max = bbox_max + Eigen::Vector3d::Constant(padding);
    const Eigen::Vector3d extent = padded_max - grid.origin_mm;
    grid.dims = Eigen::Vector3i(
        axisCount(extent.x(), spacing),
        axisCount(extent.y(), spacing),
        axisCount(extent.z(), spacing));

    const long long total = totalVoxelCount(grid.dims);
    if (total > kMaxVoxels) {
        throw std::runtime_error("VoxelizationV6 grid exceeds 200000000 voxels");
    }

    grid.occupancy.assign(static_cast<size_t>(total), VoxelizationV6::Occ::OUTSIDE);
    return grid;
}

void classifyByWindingNumber(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    VoxelizationV6::Grid& grid)
{
    const int total = static_cast<int>(grid.occupancy.size());
    const int n_chunks = (total + kWindingChunkSize - 1) / kWindingChunkSize;

#pragma omp parallel for schedule(static)
    for (int chunk = 0; chunk < n_chunks; ++chunk) {
        const int offset = chunk * kWindingChunkSize;
        const int count = std::min(kWindingChunkSize, total - offset);
        Eigen::MatrixXd centers(count, 3);

        for (int local = 0; local < count; ++local) {
            const int flat = offset + local;
            const int nx = grid.dims.x();
            const int ny = grid.dims.y();
            const int i = flat % nx;
            const int yz = flat / nx;
            const int j = yz % ny;
            const int k = yz / ny;
            centers.row(local) = grid.center(i, j, k).transpose();
        }

        Eigen::VectorXd winding;
        igl::winding_number(V, F, centers, winding);
        for (int local = 0; local < count; ++local) {
            if (winding(local) > 0.5) {
                grid.occupancy[static_cast<size_t>(offset + local)] =
                    VoxelizationV6::Occ::INSIDE;
            }
        }
    }
}

Vec3 toVec3(const Eigen::Vector3d& p)
{
    return {p.x(), p.y(), p.z()};
}

TriangleMesh toTriangleMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    TriangleMesh mesh;
    mesh.format = StlFormat::ascii;
    mesh.triangles.reserve(static_cast<size_t>(F.rows()));

    for (int f = 0; f < F.rows(); ++f) {
        StlTriangle tri;
        for (int c = 0; c < 3; ++c) {
            const Eigen::Vector3d p = V.row(F(f, c));
            tri.vertices[static_cast<size_t>(c)] = toVec3(p);
            mesh.bbox.expand(tri.vertices[static_cast<size_t>(c)]);
        }
        const Vec3 ab = tri.vertices[1] - tri.vertices[0];
        const Vec3 ac = tri.vertices[2] - tri.vertices[0];
        tri.normal = normalized(cross(ab, ac));
        mesh.triangles.push_back(tri);
    }
    return mesh;
}

void classifyByV4RayParity(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    VoxelizationV6::Grid& grid,
    const VoxelizationV6::Options& opt)
{
    const TriangleMesh mesh = toTriangleMesh(V, F);
    VoxelParams params;
    params.spacing_mm = opt.spacing_mm;
    params.padding_mm = opt.bbox_padding_mm;

    const VoxelGrid v4_grid = voxelizeMesh(mesh, params);
    if (v4_grid.nx() != grid.dims.x() ||
        v4_grid.ny() != grid.dims.y() ||
        v4_grid.nz() != grid.dims.z()) {
        throw std::runtime_error("VoxelizationV6: v4 voxel grid dimensions mismatch");
    }

    for (const VoxelIndex& voxel : v4_grid.occupiedVoxels()) {
        grid.occupancy[static_cast<size_t>(grid.idx(voxel.x, voxel.y, voxel.z))] =
            VoxelizationV6::Occ::INSIDE;
    }
}

bool inBounds(const VoxelizationV6::Grid& grid, int i, int j, int k)
{
    return i >= 0 && i < grid.dims.x() &&
        j >= 0 && j < grid.dims.y() &&
        k >= 0 && k < grid.dims.z();
}

bool isOutsideNeighbor(const VoxelizationV6::Grid& grid, int i, int j, int k)
{
    if (!inBounds(grid, i, j, k)) return true;
    return grid.occupancy[static_cast<size_t>(grid.idx(i, j, k))] ==
        VoxelizationV6::Occ::OUTSIDE;
}

bool usesNeighborOffset(
    int di,
    int dj,
    int dk,
    VoxelizationV6::BoundaryConn conn)
{
    const int manhattan = std::abs(di) + std::abs(dj) + std::abs(dk);
    if (manhattan == 0) return false;
    if (conn == VoxelizationV6::BoundaryConn::CONN_6) return manhattan == 1;
    return manhattan <= 2;
}

void markBoundary(VoxelizationV6::Grid& grid, VoxelizationV6::BoundaryConn conn)
{
    const int total = static_cast<int>(grid.occupancy.size());
    std::vector<std::vector<int>> tls_boundary(static_cast<size_t>(omp_get_max_threads()));

#pragma omp parallel
    {
        const int tid = omp_get_thread_num();
        std::vector<int>& local = tls_boundary[static_cast<size_t>(tid)];
        local.reserve(grid.occupancy.size() / (8U * static_cast<size_t>(omp_get_max_threads())));

#pragma omp for schedule(static) nowait
        for (int flat = 0; flat < total; ++flat) {
            if (grid.occupancy[static_cast<size_t>(flat)] != VoxelizationV6::Occ::INSIDE) {
                continue;
            }

            const int nx = grid.dims.x();
            const int ny = grid.dims.y();
            const int i = flat % nx;
            const int yz = flat / nx;
            const int j = yz % ny;
            const int k = yz / ny;

            bool touches_outside = false;
            for (int dk = -1; dk <= 1 && !touches_outside; ++dk) {
                for (int dj = -1; dj <= 1 && !touches_outside; ++dj) {
                    for (int di = -1; di <= 1 && !touches_outside; ++di) {
                        if (!usesNeighborOffset(di, dj, dk, conn)) continue;
                        touches_outside = isOutsideNeighbor(grid, i + di, j + dj, k + dk);
                    }
                }
            }

            if (touches_outside) local.push_back(flat);
        }
    }

    size_t n_boundary = 0;
    for (const auto& local : tls_boundary) n_boundary += local.size();

    std::vector<int> boundary_indices;
    boundary_indices.reserve(n_boundary);
    for (const auto& local : tls_boundary) {
        boundary_indices.insert(boundary_indices.end(), local.begin(), local.end());
    }

    grid.boundary_voxels.reserve(boundary_indices.size());
    for (int flat : boundary_indices) {
        grid.occupancy[static_cast<size_t>(flat)] = VoxelizationV6::Occ::BOUNDARY;
        const int nx = grid.dims.x();
        const int ny = grid.dims.y();
        const int i = flat % nx;
        const int yz = flat / nx;
        const int j = yz % ny;
        const int k = yz / ny;
        grid.boundary_voxels.emplace_back(i, j, k);
    }
}

}  // namespace

VoxelizationV6::Grid VoxelizationV6::voxelize(
    const Eigen::MatrixXd& V_repaired,
    const Eigen::MatrixXi& F_repaired,
    const Options& opt)
{
    validateInput(V_repaired, F_repaired);

    Grid grid = makeEmptyGrid(V_repaired, opt.spacing_mm, opt.bbox_padding_mm);
    if (opt.inside_test == Options::InsideTest::WINDING_NUMBER) {
        classifyByWindingNumber(V_repaired, F_repaired, grid);
    } else if (opt.inside_test == Options::InsideTest::RAY_PARITY) {
        classifyByV4RayParity(V_repaired, F_repaired, grid, opt);
    } else {
        throw std::runtime_error("VoxelizationV6: unknown inside test");
    }

    markBoundary(grid, opt.boundary_conn);
    return grid;
}

}  // namespace cslc
