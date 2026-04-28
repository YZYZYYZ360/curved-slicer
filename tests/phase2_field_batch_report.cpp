#include "app/pipeline.h"
#include "io/config_loader.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int fail(const std::string& message)
{
    std::cerr << "phase2_field_batch_report failed: " << message << '\n';
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

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        const std::filesystem::path root = sourceRoot();
        cslc::PipelineConfig config = cslc::loadPipelineConfig(root / "config" / "batch_three_models.toml");
        config.io.output_root = root / "runs" / "phase2_field";
        config.io.debug_dump_intermediates = true;
        config.voxel.spacing_mm = 0.5;
        config.voxel.padding_mm = 0.0;
        config.voxel.sdf_band_mm = 1.0;
        if (argc > 1) {
            const std::string requested = argv[1];
            config.io.models.erase(
                std::remove_if(config.io.models.begin(), config.io.models.end(), [&requested](const cslc::ModelConfig& model) {
                    return model.name != requested;
                }),
                config.io.models.end());
            require(!config.io.models.empty(), "requested phase2 model name was not found");
        }

        const cslc::BatchReport report = cslc::runBatch(config);
        cslc::printBatchReport(report, std::cout);
        require(report.success_count == config.io.models.size(), "all three phase2 models should succeed");
        require(report.failure_count == 0, "phase2 batch should not have failures");

        for (const cslc::ModelReport& model : report.models) {
            require(model.layer_count >= 50, "phase2 model should produce at least 50 layers at 0.5mm");
            require(model.face_count_total > 0, "phase2 model should produce non-empty STL layers");
            require(model.connected_components == 1, "phase2 per-model connected components should be 1");
            require(model.face_count_per_layer.size() == static_cast<std::size_t>(model.layer_count),
                    "per-layer face counts should match layer_count");
            require(model.connected_components_per_layer.size() == static_cast<std::size_t>(model.layer_count),
                    "per-layer connected components should match layer_count");
            for (std::size_t faces : model.face_count_per_layer) {
                require(faces > 0, "each phase2 STL layer should be non-empty");
            }
            std::cout << "phase2_model_cc name=" << model.name
                      << " connected_components=" << model.connected_components
                      << " max_layer_connected_components=" << model.max_layer_connected_components << '\n';
            require(std::filesystem::exists(model.metrics_path), "phase2 metrics.json should be written");
            require(std::filesystem::exists(config.io.output_root / model.name / "phi_points.ply"),
                    "debug phi point cloud should be written");
        }
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }
    return 0;
}
