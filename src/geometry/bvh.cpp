#include "geometry/bvh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

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

double axisValue(const Vec3& value, int axis)
{
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

AABB mergeBounds(const AABB& lhs, const AABB& rhs)
{
    AABB result = lhs;
    result.expand(rhs.min);
    result.expand(rhs.max);
    return result;
}

double distanceSquaredToBounds(const Vec3& point, const AABB& bounds)
{
    const auto axis_distance = [](double value, double min_value, double max_value) {
        if (value < min_value) {
            const double d = min_value - value;
            return d * d;
        }
        if (value > max_value) {
            const double d = value - max_value;
            return d * d;
        }
        return 0.0;
    };

    return axis_distance(point.x, bounds.min.x, bounds.max.x) +
        axis_distance(point.y, bounds.min.y, bounds.max.y) +
        axis_distance(point.z, bounds.min.z, bounds.max.z);
}

AABB triangleBounds(const StlTriangle& triangle)
{
    AABB bounds;
    for (const Vec3& vertex : triangle.vertices) {
        bounds.expand(vertex);
    }
    return bounds;
}

Vec3 triangleCentroid(const StlTriangle& triangle)
{
    return (triangle.vertices[0] + triangle.vertices[1] + triangle.vertices[2]) / 3.0;
}

}  // namespace

TriangleBvh::TriangleBvh(const TriangleMesh& mesh)
    : mesh_(&mesh)
{
    triangle_indices_.reserve(mesh.triangles.size());
    triangle_bounds_.reserve(mesh.triangles.size());
    centroids_.reserve(mesh.triangles.size());
    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        triangle_indices_.push_back(i);
        triangle_bounds_.push_back(triangleBounds(mesh.triangles[i]));
        centroids_.push_back(triangleCentroid(mesh.triangles[i]));
    }
    if (!triangle_indices_.empty()) {
        nodes_.reserve(triangle_indices_.size() * 2);
        buildNode(0, triangle_indices_.size());
    }
}

int TriangleBvh::buildNode(std::size_t start, std::size_t count)
{
    Node node;
    node.start = start;
    node.count = count;
    node.bounds = triangle_bounds_[triangle_indices_[start]];
    AABB centroid_bounds;
    centroid_bounds.expand(centroids_[triangle_indices_[start]]);
    for (std::size_t i = start + 1; i < start + count; ++i) {
        const std::size_t tri_index = triangle_indices_[i];
        node.bounds = mergeBounds(node.bounds, triangle_bounds_[tri_index]);
        centroid_bounds.expand(centroids_[tri_index]);
    }

    const int node_index = static_cast<int>(nodes_.size());
    nodes_.push_back(node);

    constexpr std::size_t leaf_size = 12;
    if (count <= leaf_size) {
        return node_index;
    }

    const Vec3 centroid_extent = centroid_bounds.size();
    int axis = 0;
    if (centroid_extent.y > centroid_extent.x && centroid_extent.y >= centroid_extent.z) {
        axis = 1;
    } else if (centroid_extent.z > centroid_extent.x && centroid_extent.z > centroid_extent.y) {
        axis = 2;
    }

    const std::size_t mid = start + count / 2;
    std::nth_element(
        triangle_indices_.begin() + static_cast<std::ptrdiff_t>(start),
        triangle_indices_.begin() + static_cast<std::ptrdiff_t>(mid),
        triangle_indices_.begin() + static_cast<std::ptrdiff_t>(start + count),
        [this, axis](std::size_t lhs, std::size_t rhs) {
            return axisValue(centroids_[lhs], axis) < axisValue(centroids_[rhs], axis);
        });

    nodes_[static_cast<std::size_t>(node_index)].left = buildNode(start, mid - start);
    nodes_[static_cast<std::size_t>(node_index)].right = buildNode(mid, start + count - mid);
    nodes_[static_cast<std::size_t>(node_index)].count = 0;
    return node_index;
}

double TriangleBvh::nearestDistance(const Vec3& point) const
{
    if (mesh_ == nullptr) {
        throw std::runtime_error("TriangleBvh is not initialized");
    }

    double best_sq = std::numeric_limits<double>::infinity();
    std::vector<int> stack;
    if (!nodes_.empty()) {
        stack.push_back(0);
    }

    while (!stack.empty()) {
        const int node_index = stack.back();
        stack.pop_back();
        const Node& node = nodes_[static_cast<std::size_t>(node_index)];
        if (distanceSquaredToBounds(point, node.bounds) > best_sq) {
            continue;
        }

        if (node.left < 0 && node.right < 0) {
            for (std::size_t i = node.start; i < node.start + node.count; ++i) {
                const std::size_t triangle_index = triangle_indices_[i];
                const double distance = pointTriangleDistance(point, mesh_->triangles[triangle_index]);
                best_sq = std::min(best_sq, distance * distance);
            }
            continue;
        }

        const int left = node.left;
        const int right = node.right;
        if (left >= 0 && right >= 0) {
            const double left_dist = distanceSquaredToBounds(point, nodes_[static_cast<std::size_t>(left)].bounds);
            const double right_dist = distanceSquaredToBounds(point, nodes_[static_cast<std::size_t>(right)].bounds);
            if (left_dist < right_dist) {
                if (right_dist <= best_sq) {
                    stack.push_back(right);
                }
                if (left_dist <= best_sq) {
                    stack.push_back(left);
                }
            } else {
                if (left_dist <= best_sq) {
                    stack.push_back(left);
                }
                if (right_dist <= best_sq) {
                    stack.push_back(right);
                }
            }
        } else {
            if (left >= 0) {
                stack.push_back(left);
            }
            if (right >= 0) {
                stack.push_back(right);
            }
        }
    }
    return std::sqrt(best_sq);
}

}  // namespace cslc
