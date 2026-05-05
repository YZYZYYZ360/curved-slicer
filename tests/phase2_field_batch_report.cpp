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

double largestComponentVertexRatio(const cslc::ModelReport& model)
{
    std::size_t total_vertices = 0;
    std::size_t largest_vertices = 0;
    for (const cslc::ComponentSummary& component : model.component_details) {
        total_vertices += component.vertex_count;
        largest_vertices = std::max(largest_vertices, component.vertex_count);
    }
    return total_vertices == 0 ? 1.0 : static_cast<double>(largest_vertices) / static_cast<double>(total_vertices);
}

bool isPipelineName(const std::string& value)
{
    return value == "scalar" || value == "vector_kuka";
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        const std::filesystem::path root = sourceRoot();

        // Scan argv for --config <path>
        std::filesystem::path config_path;
        bool user_specified_config = false;
        for (int i = 1; i < argc - 1; ++i) {
            if (std::string(argv[i]) == "--config") {
                config_path = std::filesystem::path(argv[i + 1]);
                user_specified_config = true;
                break;
            }
        }
        if (!user_specified_config) {
            config_path = root / "config" / "batch_three_models.toml";
        }

        cslc::PipelineConfig config = cslc::loadPipelineConfig(config_path);

        // Parse remaining positional args (model_name, pipeline), skipping --config and its path
        std::string requested_model;
        std::string pipeline = config.algorithm.pipeline;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--config") {
                ++i; // skip the path that follows
                continue;
            }
            if (isPipelineName(arg)) {
                pipeline = arg;
            } else if (arg != "all" && requested_model.empty()) {
                requested_model = arg;
            }
        }
        require(isPipelineName(pipeline), "pipeline should be scalar or vector_kuka");

        config.algorithm.pipeline = pipeline;
        if (!user_specified_config) {
            config.io.output_root = root / "runs" / (pipeline == "scalar" ? "phase2_scalar" : "phase3_vector_kuka");
        }
        config.io.debug_dump_intermediates = true;
        config.voxel.spacing_mm = 0.5;
        config.voxel.padding_mm = 0.0;
        config.voxel.sdf_band_mm = 1.0;
        if (!requested_model.empty()) {
            config.io.models.erase(
                std::remove_if(config.io.models.begin(), config.io.models.end(), [&requested_model](const cslc::ModelConfig& model) {
                    return model.name != requested_model;
                }),
                config.io.models.end());
            require(!config.io.models.empty(), "requested model name was not found");
        }

        const cslc::BatchReport report = cslc::runBatch(config);
        cslc::printBatchReport(report, std::cout);
        require(report.success_count == config.io.models.size(), "all requested models should succeed");
        require(report.failure_count == 0, "field batch should not have failures");

        for (const cslc::ModelReport& model : report.models) {
            require(model.layer_count >= 47, "model should stay within the Phase 3 15% layer-count tolerance");
            require(model.face_count_total > 0, "model should produce non-empty STL layers");
            const double main_component_ratio = largestComponentVertexRatio(model);
            if (main_component_ratio <= 0.99) {
                std::cerr << "WARN: main component ratio=" << main_component_ratio
                          << " per-model cc=" << model.connected_components
                          << " name=" << model.name
                          << " pipeline=" << pipeline << '\n';
            }
            require(main_component_ratio > 0.99, "main connected component should contain more than 99% of vertices");
            require(model.face_count_per_layer.size() == static_cast<std::size_t>(model.layer_count),
                    "per-layer face counts should match layer_count");
            require(model.connected_components_per_layer.size() == static_cast<std::size_t>(model.layer_count),
                    "per-layer connected components should match layer_count");
            require(model.m1_per_layer.size() == static_cast<std::size_t>(model.layer_count),
                    "per-layer M1 values should match layer_count");
            for (std::size_t faces : model.face_count_per_layer) {
                require(faces > 0, "each STL layer should be non-empty");
            }
            if (pipeline == "vector_kuka") {
                require(model.m2_hemisphere_violation_ratio == 0.0,
                        "vector_kuka hemisphere violation ratio should be zero");
            }
            std::cout << "field_model_cc pipeline=" << pipeline
                      << " name=" << model.name
                      << " connected_components=" << model.connected_components
                      << " max_layer_connected_components=" << model.max_layer_connected_components
                      << " m1_max_abs_mean_curvature=" << model.m1_max_abs_mean_curvature
                      << " m2_hemisphere_violation_ratio=" << model.m2_hemisphere_violation_ratio << '\n';
            require(std::filesystem::exists(model.metrics_path), "phase2 metrics.json should be written");
            require(std::filesystem::exists(config.io.output_root / model.name / "phi_points.ply"),
                    "debug phi point cloud should be written");
        }
    } catch (const std::exception& ex) {
        return fail(ex.what());
    }
    return 0;
}
