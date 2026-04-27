#include "field/wavefront.h"
#include "io/stl_reader.h"
#include "surface/iso_surface.h"

#include <filesystem>
#include <iostream>
#include <string>

namespace {

int fail(const std::string& message)
{
    std::cerr << "wavefront_smoke_test failed: " << message << '\n';
    return 1;
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

        cslc::VoxelParams voxel_params;
        voxel_params.spacing_mm = 0.25;
        voxel_params.padding_mm = 0.0;
        voxel_params.sdf_band_mm = 1.0;
        const cslc::VoxelGrid grid = cslc::voxelizeMesh(mesh, voxel_params);

        cslc::WavefrontParams wavefront_params;
        wavefront_params.height_interval_mm = 0.25;
        const cslc::ScalarField phi = cslc::solveWavefront(grid, wavefront_params);

        cslc::IsoExtractParams iso_params;
        iso_params.layer_thickness_mm = 0.25;
        iso_params.phi_start_offset = 0.25;
        iso_params.max_layers = 4;
        const std::vector<double> levels = cslc::planIsoLevels(phi, iso_params);
        if (levels.empty()) {
            return fail("wavefront phi should produce iso levels");
        }

        const cslc::IsoMesh iso_mesh = cslc::extractIsoSurface(phi, levels.front(), 0);
        const int components = cslc::countConnectedComponents(iso_mesh);
        std::cout << "wavefront_smoke vertices=" << iso_mesh.vertices.size()
                  << " triangles=" << iso_mesh.triangles.size()
                  << " components=" << components
                  << " iso=" << levels.front() << '\n';

        if (iso_mesh.vertices.empty() || iso_mesh.triangles.empty()) {
            return fail("wavefront iso surface should not be empty");
        }
        if (components != 1) {
            return fail("cube wavefront iso surface should have one connected component");
        }
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }

    return 0;
}
