#include "app/pipeline.h"
#include "io/config_loader.h"
#include "io/stl_reader.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

int fail(const std::string& message)
{
    std::cerr << "phase0_batch_test failed: " << message << '\n';
    return 1;
}

bool nearlyEqual(double lhs, double rhs, double eps = 1e-9)
{
    return std::abs(lhs - rhs) <= eps;
}

bool nearSize(const cslc::AABB& bbox, double x, double y, double z, double tolerance_mm)
{
    const cslc::Vec3 size = bbox.size();
    return std::abs(size.x - x) <= tolerance_mm &&
        std::abs(size.y - y) <= tolerance_mm &&
        std::abs(size.z - z) <= tolerance_mm;
}

const cslc::ModelReport* findReport(const cslc::BatchReport& report, const std::string& name)
{
    for (const cslc::ModelReport& model : report.models) {
        if (model.name == name) {
            return &model;
        }
    }
    return nullptr;
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
    const std::filesystem::path root = sourceRoot();
    const std::filesystem::path model_path = root / "tests" / "models" / "cube_ascii.stl";
    const std::filesystem::path mao_path =
        root / std::filesystem::u8path(u8"tests/models/毛主席头雕(42_52_70).stl");

    const cslc::TriangleMesh mesh = cslc::readStl(model_path);
    if (mesh.triangles.size() != 12) {
        return fail("cube fixture should contain 12 triangles");
    }
    if (!nearlyEqual(mesh.bbox.min.x, 0.0) || !nearlyEqual(mesh.bbox.max.z, 1.0)) {
        return fail("cube fixture bounds were not read correctly");
    }

    if (!std::filesystem::exists(mao_path)) {
        return fail("Chinese-path mao STL fixture is missing");
    }
    const cslc::TriangleMesh mao_mesh = cslc::readStl(mao_path);
    if (mao_mesh.triangles.empty()) {
        return fail("Chinese-path mao STL should read at least one triangle");
    }
    if (!nearSize(mao_mesh.bbox, 42.0, 52.0, 70.0, 2.0)) {
        return fail("mao AABB should be close to 42x52x70 mm");
    }

    cslc::PipelineConfig config;
    config.io.output_root = std::filesystem::current_path() / "phase0_batch_test_output";
    config.io.models.push_back({"cube_ascii", model_path});
    config.voxel.spacing_mm = 0.25;
    config.voxel.padding_mm = 0.0;
    config.voxel.sdf_band_mm = 1.0;

    const cslc::BatchReport direct_report = cslc::runBatch(config);
    if (direct_report.success_count != 1 || direct_report.failure_count != 0) {
        return fail("direct runBatch(config) should succeed");
    }
    if (direct_report.models.front().voxel.occupied_voxels == 0) {
        return fail("voxelization should mark occupied voxels");
    }
    if (direct_report.models.front().voxel.nx != 4 ||
        direct_report.models.front().voxel.ny != 4 ||
        direct_report.models.front().voxel.nz != 4) {
        return fail("cube voxel grid should be 4x4x4 at 0.25 mm spacing");
    }

    std::ostringstream report_text;
    cslc::printBatchReport(direct_report, report_text);
    if (report_text.str().find("ModelReport name=cube_ascii status=ok") == std::string::npos) {
        return fail("printed report should include a successful ModelReport line");
    }
    std::cout << report_text.str();

    const std::filesystem::path generated_config_path =
        std::filesystem::current_path() / "phase0_batch_test.toml";
    {
        std::ofstream generated_config(generated_config_path);
        generated_config
            << "[io]\n"
            << "output_root = \"phase0_batch_test_output\"\n"
            << "debug_dump_intermediates = false\n\n"
            << "[[io.models]]\n"
            << "name = \"cube_from_config\"\n"
            << "stl_path = \"" << model_path.generic_string() << "\"\n\n"
            << "[voxel]\n"
            << "spacing_mm = 0.25\n"
            << "padding_mm = 0.0\n"
            << "sdf_band_mm = 1.0\n";
    }

    const cslc::PipelineConfig loaded_config = cslc::loadPipelineConfig(generated_config_path);
    if (loaded_config.io.models.size() != 1 ||
        loaded_config.io.models.front().name != "cube_from_config") {
        return fail("config loader should parse [[io.models]]");
    }

    const cslc::BatchReport config_report = cslc::runBatch(generated_config_path);
    if (config_report.success_count != 1 || config_report.failure_count != 0) {
        return fail("runBatch(config_path) should succeed");
    }

    const cslc::PipelineConfig real_config = cslc::loadPipelineConfig(root / "config" / "batch_three_models.toml");
    if (real_config.io.models.size() != 3) {
        return fail("config/batch_three_models.toml should list the three real models");
    }

    const cslc::TriangleMesh bunny_mesh =
        cslc::readStl(root / "tests" / "models" / "bunny(46_35_45).stl");
    if (!nearSize(bunny_mesh.bbox, 46.0, 35.0, 45.0, 2.0)) {
        return fail("bunny AABB should be close to 46x35x45 mm");
    }

    if (!nearSize(mao_mesh.bbox, 42.0, 52.0, 70.0, 2.0)) {
        return fail("mao AABB should be close to 42x52x70 mm");
    }
    std::cout << "phase0_real_models bunny_triangles=" << bunny_mesh.triangles.size()
              << " mao_triangles=" << mao_mesh.triangles.size() << '\n';

    std::filesystem::remove(generated_config_path);
    return 0;
}
