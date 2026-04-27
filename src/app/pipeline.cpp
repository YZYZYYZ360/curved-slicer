#include "app/pipeline.h"

#include "geometry/voxel_grid.h"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <ostream>

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

            // 算法分支（field = Laplacian + Poisson + KUKA 投影）在 Phase 2 启动时接入。
            // 当前 Phase 0+1 基线只验证 read + voxelize；论文对照基线由
            // tests/benchmarks/old_project/ 的旧项目可执行体（方案 C）提供。

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
            if (model.iso_surface_ms > 0.0) {
                output << "  iso_surface_ms=" << std::fixed << std::setprecision(3) << model.iso_surface_ms
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
