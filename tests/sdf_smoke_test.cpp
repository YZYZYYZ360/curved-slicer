#include "geometry/sdf.h"
#include "io/stl_reader.h"

#include <filesystem>
#include <iostream>
#include <string>

namespace {

int fail(const std::string& message)
{
    std::cerr << "sdf_smoke_test failed: " << message << '\n';
    return 1;
}

std::size_t sdfIndex(const cslc::SDF& sdf, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(sdf.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(sdf.ny) * static_cast<std::size_t>(z));
}

std::filesystem::path sourceRoot()
{
#ifdef CSLC_SOURCE_DIR
    return std::filesystem::path(CSLC_SOURCE_DIR);
#else
    return std::filesystem::current_path();
#endif
}

}  // namespace

int main()
{
    try {
        const cslc::TriangleMesh mesh =
            cslc::readStl(sourceRoot() / "tests" / "models" / "cube_ascii.stl");

        cslc::VoxelParams params;
        params.spacing_mm = 0.25;
        params.padding_mm = 0.25;
        params.sdf_band_mm = 1.0;

        const cslc::SDF sdf = cslc::buildSDF(mesh, params);
        if (sdf.nx != 6 || sdf.ny != 6 || sdf.nz != 6) {
            return fail("padded unit cube SDF should be 6x6x6 at 0.25mm spacing");
        }

        const double center_value = sdf.values[sdfIndex(sdf, 2, 2, 2)];
        const double corner_value = sdf.values[sdfIndex(sdf, 0, 0, 0)];
        std::cout << "sdf_smoke center=" << center_value << " corner=" << corner_value << '\n';

        if (center_value >= 0.0) {
            return fail("center voxel should be inside cube and negative");
        }
        if (corner_value <= 0.0) {
            return fail("corner voxel should be outside cube and positive");
        }
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }

    return 0;
}
