#include "geometry/sdf.h"

#include "geometry/bvh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace cslc {
namespace {

constexpr std::size_t kMaxPhase0SdfVoxels = 50'000'000;

bool intersectLineXWithTriangleYZ(const StlTriangle& triangle,
                                  double y,
                                  double z,
                                  double* out_x)
{
    const Vec3& a = triangle.vertices[0];
    const Vec3& b = triangle.vertices[1];
    const Vec3& c = triangle.vertices[2];

    const double denom = (b.y - c.y) * (a.z - c.z) + (c.z - b.z) * (a.y - c.y);
    if (std::abs(denom) < 1e-12) {
        return false;
    }

    const double u = ((b.y - c.y) * (z - c.z) + (c.z - b.z) * (y - c.y)) / denom;
    const double v = ((c.y - a.y) * (z - c.z) + (a.z - c.z) * (y - c.y)) / denom;
    const double w = 1.0 - u - v;

    constexpr double eps = 1e-10;
    if (u < -eps || v < -eps || w < -eps) {
        return false;
    }

    *out_x = u * a.x + v * b.x + w * c.x;
    return true;
}

std::vector<double> sortedUniqueIntersectionsX(const TriangleMesh& mesh,
                                               const std::vector<std::size_t>& candidates,
                                               double y,
                                               double z)
{
    std::vector<double> intersections;
    intersections.reserve(candidates.size() / 2);

    for (std::size_t tri_index : candidates) {
        double x = 0.0;
        if (intersectLineXWithTriangleYZ(mesh.triangles[tri_index], y, z, &x)) {
            intersections.push_back(x);
        }
    }

    std::sort(intersections.begin(), intersections.end());
    intersections.erase(std::unique(intersections.begin(), intersections.end(), [](double lhs, double rhs) {
        return std::abs(lhs - rhs) < 1e-8;
    }), intersections.end());

    return intersections;
}

bool isInsideClosedMeshRayX(const std::vector<double>& xs, double x)
{
    const auto first_positive = std::upper_bound(xs.begin(), xs.end(), x + 1e-9);
    const auto intersections_to_positive_x = std::distance(first_positive, xs.end());
    return (intersections_to_positive_x % 2) == 1;
}

double signedDistanceToMesh(const TriangleBvh& bvh, const std::vector<double>& xs, const Vec3& point)
{
    const double min_distance = bvh.nearestDistance(point);
    return isInsideClosedMeshRayX(xs, point.x) ? -min_distance : min_distance;
}

int axisCount(double extent, double spacing)
{
    return std::max(1, static_cast<int>(std::ceil(extent / spacing)));
}

std::size_t sdfIndex(const SDF& sdf, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(sdf.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(sdf.ny) * static_cast<std::size_t>(z));
}

Vec3 voxelCenter(const SDF& sdf, int x, int y, int z)
{
    return {
        sdf.bbox.min.x + (static_cast<double>(x) + 0.5) * sdf.spacing,
        sdf.bbox.min.y + (static_cast<double>(y) + 0.5) * sdf.spacing,
        sdf.bbox.min.z + (static_cast<double>(z) + 0.5) * sdf.spacing,
    };
}

struct RayXBins {
    int ny = 0;
    int nz = 0;
    std::vector<std::vector<std::size_t>> bins;

    const std::vector<std::size_t>& at(int y, int z) const
    {
        return bins[static_cast<std::size_t>(y) + static_cast<std::size_t>(ny) * static_cast<std::size_t>(z)];
    }
};

int lowerCenterIndex(double coord, double min_coord, double spacing, int upper)
{
    return std::max(0, std::min(static_cast<int>(std::floor((coord - min_coord) / spacing - 0.5)), upper - 1));
}

int upperCenterIndex(double coord, double min_coord, double spacing, int upper)
{
    return std::max(0, std::min(static_cast<int>(std::ceil((coord - min_coord) / spacing - 0.5)), upper - 1));
}

RayXBins buildRayXBins(const TriangleMesh& mesh, const SDF& sdf)
{
    RayXBins result;
    result.ny = sdf.ny;
    result.nz = sdf.nz;
    result.bins.resize(static_cast<std::size_t>(sdf.ny) * static_cast<std::size_t>(sdf.nz));

    for (std::size_t tri_index = 0; tri_index < mesh.triangles.size(); ++tri_index) {
        AABB tri_box;
        for (const Vec3& vertex : mesh.triangles[tri_index].vertices) {
            tri_box.expand(vertex);
        }
        const int y0 = lowerCenterIndex(tri_box.min.y, sdf.bbox.min.y, sdf.spacing, sdf.ny);
        const int y1 = upperCenterIndex(tri_box.max.y, sdf.bbox.min.y, sdf.spacing, sdf.ny);
        const int z0 = lowerCenterIndex(tri_box.min.z, sdf.bbox.min.z, sdf.spacing, sdf.nz);
        const int z1 = upperCenterIndex(tri_box.max.z, sdf.bbox.min.z, sdf.spacing, sdf.nz);

        for (int z = z0; z <= z1; ++z) {
            for (int y = y0; y <= y1; ++y) {
                result.bins[static_cast<std::size_t>(y) +
                    static_cast<std::size_t>(sdf.ny) * static_cast<std::size_t>(z)]
                    .push_back(tri_index);
            }
        }
    }
    return result;
}

}  // namespace

SDF buildSDF(const TriangleMesh& mesh, const VoxelParams& params)
{
    if (params.spacing_mm <= 0.0) {
        throw std::runtime_error("voxel.spacing_mm must be positive");
    }
    if (params.padding_mm < 0.0) {
        throw std::runtime_error("voxel.padding_mm must be non-negative");
    }
    if (!mesh.bbox.valid()) {
        throw std::runtime_error("Cannot build SDF for a mesh with invalid bounds");
    }

    SDF sdf;
    sdf.bbox = mesh.bbox.padded(params.padding_mm);
    sdf.spacing = params.spacing_mm;
    sdf.narrow_band_mm = params.sdf_band_mm;

    const Vec3 size = sdf.bbox.size();
    sdf.nx = axisCount(size.x, sdf.spacing);
    sdf.ny = axisCount(size.y, sdf.spacing);
    sdf.nz = axisCount(size.z, sdf.spacing);

    const auto total_voxels = static_cast<std::size_t>(sdf.nx) *
        static_cast<std::size_t>(sdf.ny) * static_cast<std::size_t>(sdf.nz);
    if (total_voxels > kMaxPhase0SdfVoxels) {
        throw std::runtime_error("Phase 0 SDF grid is too large for the simple CPU builder");
    }

    sdf.values.assign(total_voxels, 0.0);
    const TriangleBvh bvh(mesh);
    const RayXBins ray_bins = buildRayXBins(mesh, sdf);
    for (int z = 0; z < sdf.nz; ++z) {
        for (int y = 0; y < sdf.ny; ++y) {
            const Vec3 yz_center = voxelCenter(sdf, 0, y, z);
            const std::vector<double> xs =
                sortedUniqueIntersectionsX(mesh, ray_bins.at(y, z), yz_center.y, yz_center.z);
            for (int x = 0; x < sdf.nx; ++x) {
                sdf.values[sdfIndex(sdf, x, y, z)] = signedDistanceToMesh(bvh, xs, voxelCenter(sdf, x, y, z));
            }
        }
    }

    return sdf;
}

}  // namespace cslc
