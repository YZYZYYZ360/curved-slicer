#include "geometry/bvh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace cslc {
namespace {

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

}  // namespace

TriangleBvh::TriangleBvh(const TriangleMesh& mesh)
    : mesh_(&mesh)
{
    triangle_indices_.reserve(mesh.triangles.size());
    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        triangle_indices_.push_back(i);
    }
}

double TriangleBvh::nearestDistance(const Vec3& point) const
{
    if (mesh_ == nullptr) {
        throw std::runtime_error("TriangleBvh is not initialized");
    }

    double min_distance = std::numeric_limits<double>::infinity();
    for (std::size_t triangle_index : triangle_indices_) {
        min_distance = std::min(min_distance, pointTriangleDistance(point, mesh_->triangles[triangle_index]));
    }
    return min_distance;
}

}  // namespace cslc
