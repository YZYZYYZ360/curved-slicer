#include "geometry/voxel_grid.h"
#include "io/stl_reader.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct LegacyReadResult {
    std::size_t triangle_count = 0;
    cslc::AABB bbox;
};

struct ModelCase {
    std::string name;
    std::filesystem::path path;
};

std::filesystem::path sourceRoot()
{
#ifdef CSLC_SOURCE_DIR
    return std::filesystem::path(CSLC_SOURCE_DIR);
#else
    return std::filesystem::current_path();
#endif
}

double elapsedMs(Clock::time_point start, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

float readFloat(std::istream& input)
{
    float value = 0.0f;
    input.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!input) {
        throw std::runtime_error("unexpected EOF while reading float");
    }
    return value;
}

std::uint16_t readUInt16(std::istream& input)
{
    std::uint16_t value = 0;
    input.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!input) {
        throw std::runtime_error("unexpected EOF while reading uint16");
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
        throw std::runtime_error("failed to open " + path.u8string());
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

void expand(cslc::AABB& bbox, float x, float y, float z)
{
    bbox.expand({static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)});
}

LegacyReadResult readLegacyStyleBinaryStl(const std::filesystem::path& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("failed to open " + path.u8string());
    }

    input.seekg(80, std::ios::beg);
    std::uint32_t triangle_count = 0;
    input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
    if (!input) {
        throw std::runtime_error("missing binary STL triangle count");
    }

    LegacyReadResult result;
    result.triangle_count = triangle_count;

    for (std::uint32_t i = 0; i < triangle_count; ++i) {
        (void)readFloat(input);
        (void)readFloat(input);
        (void)readFloat(input);

        for (int vertex = 0; vertex < 3; ++vertex) {
            const float x = readFloat(input);
            const float y = readFloat(input);
            const float z = readFloat(input);
            expand(result.bbox, x, y, z);
        }
        (void)readUInt16(input);
    }

    return result;
}

LegacyReadResult readLegacyStyleAsciiStl(const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open " + path.u8string());
    }

    LegacyReadResult result;
    std::string token;
    while (input >> token) {
        if (token != "facet") {
            continue;
        }

        input >> token;  // normal
        float skip = 0.0f;
        input >> skip >> skip >> skip;
        input >> token >> token;  // outer loop
        for (int vertex = 0; vertex < 3; ++vertex) {
            input >> token;
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            input >> x >> y >> z;
            expand(result.bbox, x, y, z);
        }
        input >> token >> token;  // endloop endfacet
        ++result.triangle_count;
    }

    return result;
}

LegacyReadResult readLegacyStyleStl(const std::filesystem::path& path)
{
    return looksLikeBinaryStl(path) ? readLegacyStyleBinaryStl(path) : readLegacyStyleAsciiStl(path);
}

std::string gridString(const cslc::VoxelReport& report)
{
    std::ostringstream output;
    output << report.nx << "x" << report.ny << "x" << report.nz;
    return output.str();
}

std::string sizeString(const cslc::AABB& bbox)
{
    const cslc::Vec3 size = bbox.size();
    std::ostringstream output;
    output << std::fixed << std::setprecision(3)
           << size.x << "x" << size.y << "x" << size.z;
    return output.str();
}

std::vector<ModelCase> modelCases()
{
    const std::filesystem::path root = sourceRoot();
    return {
        {"armadillo_flat", root / "tests" / "models" / "armadillo_flat.stl"},
        {"bunny", root / "tests" / "models" / "bunny(46_35_45).stl"},
        {"mao", root / std::filesystem::u8path(u8"tests/models/毛主席头雕(42_52_70).stl")},
    };
}

}  // namespace

int main(int argc, char** argv)
{
    double spacing_mm = 5.0;
    if (argc >= 2) {
        spacing_mm = std::stod(argv[1]);
    }

    cslc::VoxelParams voxel_params;
    voxel_params.spacing_mm = spacing_mm;
    voxel_params.padding_mm = 0.0;
    voxel_params.sdf_band_mm = 1.0;

    std::cout << "Phase 0 IO/Voxel Benchmark\n";
    std::cout << "spacing_mm=" << spacing_mm << '\n';
    std::cout << "Old-style STL IO is a test-local reader modeled on the old basicDataType/STLReader binary/ascii pull pattern.\n";
    std::cout << "Old project voxelization is not invoked here because it depends on the old OpenVDB/SFF pipeline.\n";
    std::cout << "model,file_mb,triangles,new_stl_io_ms,old_style_stl_io_ms,phase0_voxel_ms,bbox_size_mm,grid,occupied\n";

    for (const ModelCase& model : modelCases()) {
        const auto file_size = std::filesystem::file_size(model.path);

        cslc::TriangleMesh mesh;
        const auto new_read_start = Clock::now();
        mesh = cslc::readStl(model.path);
        const double new_read_ms = elapsedMs(new_read_start, Clock::now());

        const auto legacy_read_start = Clock::now();
        const LegacyReadResult legacy = readLegacyStyleStl(model.path);
        const double legacy_read_ms = elapsedMs(legacy_read_start, Clock::now());

        if (legacy.triangle_count != mesh.triangles.size()) {
            std::cerr << "triangle count mismatch for " << model.name << '\n';
            return 1;
        }

        const auto voxel_start = Clock::now();
        const cslc::VoxelGrid grid = cslc::voxelizeMesh(mesh, voxel_params);
        const double voxel_ms = elapsedMs(voxel_start, Clock::now());
        const cslc::VoxelReport voxel = grid.report();

        std::cout << model.name << ','
                  << std::fixed << std::setprecision(3)
                  << static_cast<double>(file_size) / (1024.0 * 1024.0) << ','
                  << mesh.triangles.size() << ','
                  << new_read_ms << ','
                  << legacy_read_ms << ','
                  << voxel_ms << ','
                  << sizeString(mesh.bbox) << ','
                  << gridString(voxel) << ','
                  << voxel.occupied_voxels << '/' << voxel.total_voxels
                  << '\n';
    }

    return 0;
}

