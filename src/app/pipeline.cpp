#include "app/pipeline.h"

#include "field/wavefront.h"
#include "geometry/voxel_grid.h"
#include "surface/iso_surface.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace cslc {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void printBounds(const AABB& bbox, std::ostream& output)
{
    const Vec3 size = bbox.size();
    output << "bbox=[("
           << bbox.min.x << ", " << bbox.min.y << ", " << bbox.min.z << "), ("
           << bbox.max.x << ", " << bbox.max.y << ", " << bbox.max.z << ")]";
    output << " size=(" << size.x << ", " << size.y << ", " << size.z << ")mm";
}

IsoExtractParams toIsoExtractParams(const IsoSurfaceConfig& config)
{
    IsoExtractParams params;
    params.layer_thickness_mm = config.layer_thickness_mm;
    params.iso_spacing = config.iso_spacing;
    params.max_layers = config.max_layers;
    params.phi_start_offset = config.phi_start_offset;
    return params;
}

void writeMetricsJson(const ModelReport& report, const std::filesystem::path& output_path)
{
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open metrics output: " + output_path.u8string());
    }

    output << "{\n";
    output << "  \"model\": \"" << report.name << "\",\n";
    output << "  \"M4\": {\n";
    output << "    \"face_count\": " << report.iso_triangles << ",\n";
    output << "    \"connected_components\": " << report.connected_components << "\n";
    output << "  },\n";
    output << "  \"M6\": {\n";
    output << "    \"wavefront_ms\": " << std::fixed << std::setprecision(3) << report.wavefront_ms << ",\n";
    output << "    \"iso_surface_ms\": " << report.iso_surface_ms << "\n";
    output << "  }\n";
    output << "}\n";
}

}  // namespace

BatchReport runBatch(const PipelineConfig& config)
{
    const auto batch_start = Clock::now();
    if (!config.io.output_root.empty()) {
        std::filesystem::create_directories(config.io.output_root);
    }

    BatchReport batch;
    batch.models.reserve(config.io.models.size());

    for (const ModelConfig& model : config.io.models) {
        ModelReport report;
        report.name = model.name;
        report.stl_path = model.stl_path;

        try {
            const auto read_start = Clock::now();
            const TriangleMesh mesh = readStl(model.stl_path);
            const auto read_end = Clock::now();

            const auto voxel_start = Clock::now();
            const VoxelGrid grid = voxelizeMesh(mesh, config.voxel);
            const auto voxel_end = Clock::now();

            report.success = true;
            report.message = "ok";
            report.triangle_count = mesh.triangles.size();
            report.bbox = mesh.bbox;
            report.voxel = grid.report();
            report.read_ms = elapsedMs(read_start, read_end);
            report.voxelize_ms = elapsedMs(voxel_start, voxel_end);

            if (config.algorithm.mode == "wavefront") {
                const auto wavefront_start = Clock::now();
                const ScalarField phi = solveWavefront(grid, config.algorithm.wavefront);
                const auto wavefront_end = Clock::now();

                const auto iso_start = Clock::now();
                const std::vector<double> levels = planIsoLevels(phi, toIsoExtractParams(config.iso_surface));
                if (!levels.empty()) {
                    const double iso_value = levels[levels.size() / 2];
                    const IsoMesh iso_mesh = extractIsoSurface(phi, iso_value, static_cast<int>(levels.size() / 2));
                    report.iso_vertices = iso_mesh.vertices.size();
                    report.iso_triangles = iso_mesh.triangles.size();
                    report.connected_components = countConnectedComponents(iso_mesh);

                    const std::filesystem::path model_output_dir = config.io.output_root / model.name;
                    if (config.io.debug_dump_intermediates) {
                        writeIsoMeshPly(iso_mesh, model_output_dir / "wavefront_mid_iso.ply");
                    }
                }
                const auto iso_end = Clock::now();

                report.wavefront_ms = elapsedMs(wavefront_start, wavefront_end);
                report.iso_surface_ms = elapsedMs(iso_start, iso_end);
                report.metrics_path = config.io.output_root / model.name / "metrics.json";
                writeMetricsJson(report, report.metrics_path);
            }

            ++batch.success_count;
        } catch (const std::exception& ex) {
            report.success = false;
            report.message = ex.what();
            ++batch.failure_count;
        }

        batch.models.push_back(report);
    }

    batch.total_ms = elapsedMs(batch_start, Clock::now());
    return batch;
}

BatchReport runBatch(const std::filesystem::path& config_path)
{
    return runBatch(loadPipelineConfig(config_path));
}

void printBatchReport(const BatchReport& report, std::ostream& output)
{
    output << "Phase 0 Batch Report\n";
    output << "success=" << report.success_count
           << " failure=" << report.failure_count
           << " total_ms=" << std::fixed << std::setprecision(3) << report.total_ms << '\n';

    for (const ModelReport& model : report.models) {
        output << "ModelReport name=" << model.name
               << " status=" << (model.success ? "ok" : "failed")
               << " triangles=" << model.triangle_count
               << " read_ms=" << std::fixed << std::setprecision(3) << model.read_ms
               << " voxelize_ms=" << model.voxelize_ms
               << '\n';

        if (model.success) {
            output << "  ";
            printBounds(model.bbox, output);
            output << '\n';
            output << "  grid=" << model.voxel.nx << "x" << model.voxel.ny << "x" << model.voxel.nz
                   << " occupied=" << model.voxel.occupied_voxels
                   << "/" << model.voxel.total_voxels
                   << " ratio=" << std::fixed << std::setprecision(4) << model.voxel.occupancy_ratio
                   << '\n';
            if (model.wavefront_ms > 0.0 || model.iso_surface_ms > 0.0) {
                output << "  wavefront_ms=" << std::fixed << std::setprecision(3) << model.wavefront_ms
                       << " iso_surface_ms=" << model.iso_surface_ms
                       << " iso_vertices=" << model.iso_vertices
                       << " iso_triangles=" << model.iso_triangles
                       << " connected_components=" << model.connected_components
                       << '\n';
            }
        } else {
            output << "  error=" << model.message << '\n';
        }
    }
}

}  // namespace cslc
