#include "io/stl_reader.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cslc {
namespace {

std::string lowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

std::runtime_error parseError(const std::filesystem::path& path, const std::string& message)
{
    return std::runtime_error("STL parse error in " + path.u8string() + ": " + message);
}

void expectToken(std::istream& input, const std::filesystem::path& path, const char* expected)
{
    std::string token;
    if (!(input >> token)) {
        throw parseError(path, std::string("expected '") + expected + "', reached end of file");
    }

    if (lowerCopy(token) != expected) {
        throw parseError(path, std::string("expected '") + expected + "', got '" + token + "'");
    }
}

Vec3 readVec3(std::istream& input, const std::filesystem::path& path)
{
    Vec3 value;
    if (!(input >> value.x >> value.y >> value.z)) {
        throw parseError(path, "expected 3 floating point values");
    }
    return value;
}

bool looksLikeBinaryStl(const std::filesystem::path& path)
{
    const auto file_size = std::filesystem::file_size(path);
    if (file_size < 84) {
        return false;
    }

    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open STL file: " + path.u8string());
    }

    input.seekg(80, std::ios::beg);
    std::uint32_t triangle_count = 0;
    input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
    if (!input) {
        return false;
    }

    const auto expected_size = static_cast<std::uintmax_t>(84) +
        static_cast<std::uintmax_t>(triangle_count) * static_cast<std::uintmax_t>(50);

    return expected_size == file_size;
}

float readFloat(std::istream& input, const std::filesystem::path& path)
{
    float value = 0.0f;
    input.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!input) {
        throw parseError(path, "unexpected end of binary triangle record");
    }
    return value;
}

std::uint16_t readUInt16(std::istream& input, const std::filesystem::path& path)
{
    std::uint16_t value = 0;
    input.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!input) {
        throw parseError(path, "unexpected end of binary attribute record");
    }
    return value;
}

void addTriangle(TriangleMesh& mesh, StlTriangle triangle)
{
    const Vec3 edge_a = triangle.vertices[1] - triangle.vertices[0];
    const Vec3 edge_b = triangle.vertices[2] - triangle.vertices[0];
    if (norm(triangle.normal) == 0.0) {
        triangle.normal = normalized(cross(edge_a, edge_b));
    }

    for (const Vec3& vertex : triangle.vertices) {
        mesh.bbox.expand(vertex);
    }
    mesh.triangles.push_back(triangle);
}

TriangleMesh readBinaryStl(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open STL file: " + path.u8string());
    }

    input.seekg(80, std::ios::beg);
    std::uint32_t triangle_count = 0;
    input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
    if (!input) {
        throw parseError(path, "missing binary triangle count");
    }

    TriangleMesh mesh;
    mesh.source_path = path;
    mesh.format = StlFormat::binary;
    mesh.triangles.reserve(triangle_count);

    for (std::uint32_t i = 0; i < triangle_count; ++i) {
        StlTriangle triangle;
        triangle.normal = {readFloat(input, path), readFloat(input, path), readFloat(input, path)};
        for (Vec3& vertex : triangle.vertices) {
            vertex = {readFloat(input, path), readFloat(input, path), readFloat(input, path)};
        }
        triangle.attribute_byte_count = readUInt16(input, path);
        addTriangle(mesh, triangle);
    }

    return mesh;
}

TriangleMesh readAsciiStl(const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open STL file: " + path.u8string());
    }

    TriangleMesh mesh;
    mesh.source_path = path;
    mesh.format = StlFormat::ascii;

    std::string token;
    while (input >> token) {
        if (lowerCopy(token) != "facet") {
            continue;
        }

        StlTriangle triangle;
        expectToken(input, path, "normal");
        triangle.normal = readVec3(input, path);
        expectToken(input, path, "outer");
        expectToken(input, path, "loop");

        for (Vec3& vertex : triangle.vertices) {
            expectToken(input, path, "vertex");
            vertex = readVec3(input, path);
        }

        expectToken(input, path, "endloop");
        expectToken(input, path, "endfacet");
        addTriangle(mesh, triangle);
    }

    return mesh;
}

}  // namespace

TriangleMesh readStl(const std::filesystem::path& stl_path)
{
    if (!std::filesystem::exists(stl_path)) {
        throw std::runtime_error("STL file does not exist: " + stl_path.u8string());
    }

    TriangleMesh mesh = looksLikeBinaryStl(stl_path) ? readBinaryStl(stl_path) : readAsciiStl(stl_path);
    if (mesh.triangles.empty()) {
        throw std::runtime_error("STL file has no triangles: " + stl_path.u8string());
    }
    if (!mesh.bbox.valid()) {
        throw std::runtime_error("STL file has invalid bounds: " + stl_path.u8string());
    }
    return mesh;
}

}  // namespace cslc

