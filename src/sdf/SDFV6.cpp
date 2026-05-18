#include "sdf/SDFV6.h"

#include "geometry/bvh.h"

#include <igl/signed_distance.h>
#include <omp.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cslc {
namespace {

constexpr int kSignedDistanceChunkSize = 100'000;
constexpr int kMaxOpenMpThreads = 24;

void validateInput(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const VoxelizationV6::Grid& grid)
{
    if (V.cols() != 3 || V.rows() == 0) {
        throw std::invalid_argument("SDFV6: V must be N x 3 and non-empty");
    }
    if (F.cols() != 3 || F.rows() == 0) {
        throw std::invalid_argument("SDFV6: F must be M x 3 and non-empty");
    }
    const long long expected =
        static_cast<long long>(grid.dims.x()) *
        static_cast<long long>(grid.dims.y()) *
        static_cast<long long>(grid.dims.z());
    if (expected <= 0 || static_cast<long long>(grid.occupancy.size()) != expected) {
        throw std::invalid_argument("SDFV6: grid occupancy size does not match dims");
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

void capOpenMpThreads()
{
#ifdef _OPENMP
    omp_set_num_threads(std::min(omp_get_max_threads(), kMaxOpenMpThreads));
#endif
}

void computeLibiglSignedDistance(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const VoxelizationV6::Grid& grid,
    bool negative_inside,
    std::vector<float>& distance_mm)
{
    const int total = static_cast<int>(grid.occupancy.size());
    const int n_chunks = (total + kSignedDistanceChunkSize - 1) / kSignedDistanceChunkSize;

#pragma omp parallel for schedule(static)
    for (int chunk = 0; chunk < n_chunks; ++chunk) {
        const int offset = chunk * kSignedDistanceChunkSize;
        const int count = std::min(kSignedDistanceChunkSize, total - offset);
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

        Eigen::VectorXd S;
        Eigen::VectorXi I;
        Eigen::MatrixXd C;
        Eigen::MatrixXd N;
        igl::signed_distance(
            centers,
            V,
            F,
            igl::SIGNED_DISTANCE_TYPE_DEFAULT,
            S,
            I,
            C,
            N);

        for (int local = 0; local < count; ++local) {
            const double signed_distance = negative_inside ? S(local) : -S(local);
            distance_mm[static_cast<size_t>(offset + local)] =
                static_cast<float>(signed_distance);
        }
    }
}

void computeBvhClosestPlusOccupancy(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const VoxelizationV6::Grid& grid,
    bool negative_inside,
    std::vector<float>& distance_mm)
{
    const TriangleMesh mesh = toTriangleMesh(V, F);
    const TriangleBvh bvh(mesh);
    const int total = static_cast<int>(grid.occupancy.size());

#pragma omp parallel for schedule(static)
    for (int flat = 0; flat < total; ++flat) {
        const int nx = grid.dims.x();
        const int ny = grid.dims.y();
        const int i = flat % nx;
        const int yz = flat / nx;
        const int j = yz % ny;
        const int k = yz / ny;
        const Eigen::Vector3d center = grid.center(i, j, k);
        const double dist = bvh.nearestDistance(toVec3(center));
        const auto occ = grid.occupancy[static_cast<size_t>(flat)];
        const bool inside_or_boundary =
            occ == VoxelizationV6::Occ::INSIDE || occ == VoxelizationV6::Occ::BOUNDARY;
        double signed_distance = inside_or_boundary ? -dist : dist;
        if (!negative_inside) signed_distance = -signed_distance;
        distance_mm[static_cast<size_t>(flat)] = static_cast<float>(signed_distance);
    }
}

}  // namespace

SDFV6::Field SDFV6::compute(
    const Eigen::MatrixXd& V_repaired,
    const Eigen::MatrixXi& F_repaired,
    const VoxelizationV6::Grid& grid,
    const Options& opt)
{
    validateInput(V_repaired, F_repaired, grid);
    if (opt.narrow_band_mm > 0.0) {
        throw std::runtime_error("SDFV6 narrow_band_mm is not implemented until W4-W5");
    }

    capOpenMpThreads();

    Field field;
    field.grid = grid;
    field.distance_mm.assign(grid.occupancy.size(), 0.0f);

    if (opt.method == Options::Method::LIBIGL_SIGNED_DISTANCE) {
        computeLibiglSignedDistance(
            V_repaired, F_repaired, grid, opt.negative_inside, field.distance_mm);
    } else if (opt.method == Options::Method::BVH_CLOSEST_PLUS_OCCUPANCY) {
        computeBvhClosestPlusOccupancy(
            V_repaired, F_repaired, grid, opt.negative_inside, field.distance_mm);
    } else {
        throw std::runtime_error("SDFV6: unknown method");
    }

    return field;
}

}  // namespace cslc
