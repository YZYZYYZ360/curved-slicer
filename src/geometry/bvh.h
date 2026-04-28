#pragma once

#include "io/stl_reader.h"

#include <cstddef>
#include <vector>

namespace cslc {

class TriangleBvh {
public:
    TriangleBvh() = default;
    explicit TriangleBvh(const TriangleMesh& mesh);

    double nearestDistance(const Vec3& point) const;

private:
    struct Node {
        AABB bounds;
        int left = -1;
        int right = -1;
        std::size_t start = 0;
        std::size_t count = 0;
    };

    int buildNode(std::size_t start, std::size_t count);

    const TriangleMesh* mesh_ = nullptr;
    std::vector<std::size_t> triangle_indices_;
    std::vector<AABB> triangle_bounds_;
    std::vector<Vec3> centroids_;
    std::vector<Node> nodes_;
};

}  // namespace cslc
