#pragma once

#include "core/types.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace cslc {

enum class StlFormat {
    ascii,
    binary,
};

struct StlTriangle {
    Vec3 normal;
    std::array<Vec3, 3> vertices{};
    std::uint16_t attribute_byte_count = 0;
};

struct TriangleMesh {
    std::filesystem::path source_path;
    StlFormat format = StlFormat::ascii;
    std::vector<StlTriangle> triangles;
    AABB bbox;
};

TriangleMesh readStl(const std::filesystem::path& stl_path);

}  // namespace cslc

