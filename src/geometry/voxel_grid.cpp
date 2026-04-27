#include "geometry/voxel_grid.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace cslc {
namespace {

constexpr std::size_t kMaxPhase0Voxels = 50'000'000;

int axisCount(double extent, double spacing)
{
    return std::max(1, static_cast<int>(std::ceil(extent / spacing)));
}

int clampIndex(int value, int upper)
{
    return std::max(0, std::min(value, upper - 1));
}

int lowerCenterIndex(double coord, double min_coord, double spacing, int upper)
{
    return clampIndex(static_cast<int>(std::floor((coord - min_coord) / spacing - 0.5)), upper);
}

int upperCenterIndex(double coord, double min_coord, double spacing, int upper)
{
    return clampIndex(static_cast<int>(std::ceil((coord - min_coord) / spacing - 0.5)), upper);
}

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

double segmentDistance(const Vec3& point, const Vec3& a, const Vec3& b)
{
    const Vec3 ab = b - a;
    const double length_sq = squaredNorm(ab);
    if (length_sq == 0.0) {
        return norm(point - a);
    }

    const double t = clamp01(dot(point - a, ab) / length_sq);
    return norm(point - (a + ab * t));
}

double pointTriangleDistance(const Vec3& point, const StlTriangle& triangle)
{
    const Vec3& a = triangle.vertices[0];
    const Vec3& b = triangle.vertices[1];
    const Vec3& c = triangle.vertices[2];

    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 normal = cross(ab, ac);
    const double normal_length_sq = squaredNorm(normal);

    if (normal_length_sq > 0.0) {
        const double signed_plane_distance = dot(point - a, normal) / std::sqrt(normal_length_sq);
        const Vec3 projected = point - normalized(normal) * signed_plane_distance;

        const Vec3 ap = projected - a;
        const double d00 = dot(ab, ab);
        const double d01 = dot(ab, ac);
        const double d11 = dot(ac, ac);
        const double d20 = dot(ap, ab);
        const double d21 = dot(ap, ac);
        const double denom = d00 * d11 - d01 * d01;

        if (std::abs(denom) > 1e-12) {
            const double v = (d11 * d20 - d01 * d21) / denom;
            const double w = (d00 * d21 - d01 * d20) / denom;
            const double u = 1.0 - v - w;
            if (u >= 0.0 && v >= 0.0 && w >= 0.0) {
                return std::abs(signed_plane_distance);
            }
        }
    }

    return std::min({
        segmentDistance(point, a, b),
        segmentDistance(point, b, c),
        segmentDistance(point, c, a),
    });
}

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

std::vector<double> collectIntersectionsX(const TriangleMesh& mesh,
                                          double y,
                                          double z,
                                          double unique_epsilon)
{
    std::vector<double> xs;
    xs.reserve(mesh.triangles.size() / 2);

    for (const StlTriangle& triangle : mesh.triangles) {
        double x = 0.0;
        if (intersectLineXWithTriangleYZ(triangle, y, z, &x)) {
            xs.push_back(x);
        }
    }

    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end(), [unique_epsilon](double lhs, double rhs) {
        return std::abs(lhs - rhs) <= unique_epsilon;
    }), xs.end());

    return xs;
}

void markSurfaceBand(VoxelGrid& grid, const TriangleMesh& mesh)
{
    const double radius = grid.spacing() * 0.75;
    const AABB& bbox = grid.bbox();

    for (const StlTriangle& triangle : mesh.triangles) {
        AABB tri_box;
        for (const Vec3& vertex : triangle.vertices) {
            tri_box.expand(vertex);
        }
        tri_box = tri_box.padded(radius);

        const int x0 = lowerCenterIndex(tri_box.min.x, bbox.min.x, grid.spacing(), grid.nx());
        const int x1 = upperCenterIndex(tri_box.max.x, bbox.min.x, grid.spacing(), grid.nx());
        const int y0 = lowerCenterIndex(tri_box.min.y, bbox.min.y, grid.spacing(), grid.ny());
        const int y1 = upperCenterIndex(tri_box.max.y, bbox.min.y, grid.spacing(), grid.ny());
        const int z0 = lowerCenterIndex(tri_box.min.z, bbox.min.z, grid.spacing(), grid.nz());
        const int z1 = upperCenterIndex(tri_box.max.z, bbox.min.z, grid.spacing(), grid.nz());

        for (int z = z0; z <= z1; ++z) {
            for (int y = y0; y <= y1; ++y) {
                for (int x = x0; x <= x1; ++x) {
                    if (pointTriangleDistance(grid.center(x, y, z), triangle) <= radius) {
                        grid.setOccupied(x, y, z);
                    }
                }
            }
        }
    }
}

}  // namespace

VoxelGrid::VoxelGrid(AABB bbox, double spacing, int nx, int ny, int nz)
    : bbox_(bbox),
      spacing_(spacing),
      nx_(nx),
      ny_(ny),
      nz_(nz)
{
}

bool VoxelGrid::inBounds(int x, int y, int z) const
{
    return x >= 0 && x < nx_ && y >= 0 && y < ny_ && z >= 0 && z < nz_;
}

std::size_t VoxelGrid::index(int x, int y, int z) const
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(nx_) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(ny_) * static_cast<std::size_t>(z));
}

bool VoxelGrid::occupied(int x, int y, int z) const
{
    if (!inBounds(x, y, z)) {
        return false;
    }
    return occupied_.find(index(x, y, z)) != occupied_.end();
}

void VoxelGrid::setOccupied(int x, int y, int z, bool value)
{
    if (!inBounds(x, y, z)) {
        return;
    }
    const std::size_t voxel_index = index(x, y, z);
    if (value) {
        occupied_.insert(voxel_index);
    } else {
        occupied_.erase(voxel_index);
    }
}

Vec3 VoxelGrid::center(int x, int y, int z) const
{
    return {
        bbox_.min.x + (static_cast<double>(x) + 0.5) * spacing_,
        bbox_.min.y + (static_cast<double>(y) + 0.5) * spacing_,
        bbox_.min.z + (static_cast<double>(z) + 0.5) * spacing_,
    };
}

VoxelIndex VoxelGrid::indexToVoxel(std::size_t voxel_index) const
{
    const auto nx_size = static_cast<std::size_t>(nx_);
    const auto ny_size = static_cast<std::size_t>(ny_);
    const int x = static_cast<int>(voxel_index % nx_size);
    const std::size_t yz = voxel_index / nx_size;
    const int y = static_cast<int>(yz % ny_size);
    const int z = static_cast<int>(yz / ny_size);
    return {x, y, z};
}

std::vector<VoxelIndex> VoxelGrid::occupiedVoxels() const
{
    std::vector<VoxelIndex> voxels;
    voxels.reserve(occupied_.size());
    for (std::size_t voxel_index : occupied_) {
        voxels.push_back(indexToVoxel(voxel_index));
    }
    return voxels;
}

std::size_t VoxelGrid::totalVoxelCount() const
{
    return static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) * static_cast<std::size_t>(nz_);
}

std::size_t VoxelGrid::occupiedVoxelCount() const
{
    return occupied_.size();
}

VoxelReport VoxelGrid::report() const
{
    VoxelReport result;
    result.nx = nx_;
    result.ny = ny_;
    result.nz = nz_;
    result.total_voxels = totalVoxelCount();
    result.occupied_voxels = occupiedVoxelCount();
    result.occupancy_ratio = result.total_voxels == 0
        ? 0.0
        : static_cast<double>(result.occupied_voxels) / static_cast<double>(result.total_voxels);
    result.bbox = bbox_;
    return result;
}

VoxelGrid voxelizeMesh(const TriangleMesh& mesh, const VoxelParams& params)
{
    if (params.spacing_mm <= 0.0) {
        throw std::runtime_error("voxel.spacing_mm must be positive");
    }
    if (params.padding_mm < 0.0) {
        throw std::runtime_error("voxel.padding_mm must be non-negative");
    }

    const AABB bbox = mesh.bbox.padded(params.padding_mm);
    const Vec3 size = bbox.size();
    const int nx = axisCount(size.x, params.spacing_mm);
    const int ny = axisCount(size.y, params.spacing_mm);
    const int nz = axisCount(size.z, params.spacing_mm);

    const auto total_voxels = static_cast<std::size_t>(nx) *
        static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
    if (total_voxels > kMaxPhase0Voxels) {
        throw std::runtime_error("Phase 0 voxel grid is too large for the simple CPU voxelizer");
    }

    VoxelGrid grid(bbox, params.spacing_mm, nx, ny, nz);

    const double unique_epsilon = std::max(params.spacing_mm * 1e-6, 1e-9);
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            const Vec3 yz_center = grid.center(0, y, z);
            const std::vector<double> xs = collectIntersectionsX(mesh, yz_center.y, yz_center.z, unique_epsilon);
            for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
                const double x_min = std::min(xs[i], xs[i + 1]);
                const double x_max = std::max(xs[i], xs[i + 1]);
                const int ix0 = clampIndex(static_cast<int>(std::ceil((x_min - bbox.min.x) / params.spacing_mm - 0.5)), nx);
                const int ix1 = clampIndex(static_cast<int>(std::floor((x_max - bbox.min.x) / params.spacing_mm - 0.5)), nx);
                for (int x = ix0; x <= ix1; ++x) {
                    grid.setOccupied(x, y, z);
                }
            }
        }
    }

    markSurfaceBand(grid, mesh);
    return grid;
}

}  // namespace cslc
