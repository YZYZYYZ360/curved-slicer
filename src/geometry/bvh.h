#pragma once

#include "io/stl_reader.h"

#include <vector>

namespace cslc {

class TriangleBvh {
public:
    TriangleBvh() = default;
    explicit TriangleBvh(const TriangleMesh& mesh);

    double nearestDistance(const Vec3& point) const;

private:
    const TriangleMesh* mesh_ = nullptr;
    std::vector<std::size_t> triangle_indices_;
};

}  // namespace cslc
