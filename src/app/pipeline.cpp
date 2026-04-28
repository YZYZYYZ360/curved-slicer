#include "app/pipeline.h"

#include "field/laplacian.h"
#include "geometry/sdf.h"
#include "geometry/voxel_grid.h"
#include "surface/iso_surface.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#if defined(_WIN32)
#include <windows.h>
#include <psapi.h>
#endif

namespace cslc {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::size_t scalarIndex(const ScalarField& phi, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(phi.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(phi.ny) * static_cast<std::size_t>(z));
}

Vec3 scalarPoint(const ScalarField& phi, int x, int y, int z)
{
    return {
        phi.bbox.min.x + (static_cast<double>(x) + 0.5) * phi.spacing,
        phi.bbox.min.y + (static_cast<double>(y) + 0.5) * phi.spacing,
        phi.bbox.min.z + (static_cast<double>(z) + 0.5) * phi.spacing,
    };
}

double currentPeakRssMb()
{
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS_EX counters{};
    if (GetProcessMemoryInfo(GetCurrentProcess(),
                             reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                             sizeof(counters)) != 0) {
        return static_cast<double>(counters.PeakWorkingSetSize) / (1024.0 * 1024.0);
    }
#endif
    return 0.0;
}

double projectedExtent(const AABB& bbox, const FieldBoundaryConfig& boundary)
{
    const Vec3 direction = normalized({
        boundary.print_direction[0],
        boundary.print_direction[1],
        boundary.print_direction[2],
    });
    const Vec3 size = bbox.size();
    return std::abs(direction.x) * size.x + std::abs(direction.y) * size.y + std::abs(direction.z) * size.z;
}

IsoExtractParams isoParamsFromConfig(const IsoSurfaceConfig& config,
                                     const FieldBoundaryConfig& boundary,
                                     const AABB& bbox)
{
    IsoExtractParams params;
    const double extent = projectedExtent(bbox, boundary);
    params.layer_thickness_mm = extent > 0.0 ? config.layer_thickness_mm / extent : config.layer_thickness_mm;
    params.iso_spacing = config.iso_spacing;
    params.max_layers = config.max_layers;
    params.phi_start_offset = extent > 0.0 ? config.phi_start_offset / extent : config.phi_start_offset;
    return params;
}

std::filesystem::path layerPath(const std::filesystem::path& model_dir, int layer_index)
{
    std::ostringstream name;
    name << "iso_" << std::setw(3) << std::setfill('0') << layer_index << ".stl";
    return model_dir / name.str();
}

void validatePhi(const ScalarField& phi)
{
    double min_value = std::numeric_limits<double>::infinity();
    bool has_value = false;
    for (double value : phi.values) {
        if (std::isinf(value)) {
            throw std::runtime_error("Phase 2 phi field has infinite values");
        }
        if (std::isnan(value)) {
            continue;
        }
        has_value = true;
        min_value = std::min(min_value, value);
    }
    if (!has_value) {
        throw std::runtime_error("Phase 2 phi field has no finite occupied values");
    }
    if (min_value < -1e-8) {
        throw std::runtime_error("Phase 2 phi field has negative values");
    }
}

struct DisjointSet {
    std::vector<int> parent;
    std::vector<int> rank;

    int add()
    {
        const int index = static_cast<int>(parent.size());
        parent.push_back(index);
        rank.push_back(0);
        return index;
    }

    int find(int value)
    {
        if (parent[static_cast<std::size_t>(value)] != value) {
            parent[static_cast<std::size_t>(value)] = find(parent[static_cast<std::size_t>(value)]);
        }
        return parent[static_cast<std::size_t>(value)];
    }

    void unite(int lhs, int rhs)
    {
        int lhs_root = find(lhs);
        int rhs_root = find(rhs);
        if (lhs_root == rhs_root) {
            return;
        }
        if (rank[static_cast<std::size_t>(lhs_root)] < rank[static_cast<std::size_t>(rhs_root)]) {
            std::swap(lhs_root, rhs_root);
        }
        parent[static_cast<std::size_t>(rhs_root)] = lhs_root;
        if (rank[static_cast<std::size_t>(lhs_root)] == rank[static_cast<std::size_t>(rhs_root)]) {
            ++rank[static_cast<std::size_t>(lhs_root)];
        }
    }

    int countRoots()
    {
        std::vector<int> roots;
        roots.reserve(parent.size());
        for (int i = 0; i < static_cast<int>(parent.size()); ++i) {
            roots.push_back(find(i));
        }
        std::sort(roots.begin(), roots.end());
        roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
        return static_cast<int>(roots.size());
    }
};

std::vector<int> vertexComponents(const IsoMesh& mesh, int* component_count)
{
    std::vector<std::vector<std::size_t>> vertex_to_triangles(mesh.vertices.size());
    for (std::size_t tri_index = 0; tri_index < mesh.triangles.size(); ++tri_index) {
        const Tri& tri = mesh.triangles[tri_index];
        vertex_to_triangles[tri.v0].push_back(tri_index);
        vertex_to_triangles[tri.v1].push_back(tri_index);
        vertex_to_triangles[tri.v2].push_back(tri_index);
    }

    std::vector<int> tri_component(mesh.triangles.size(), -1);
    int next_component = 0;
    for (std::size_t start = 0; start < mesh.triangles.size(); ++start) {
        if (tri_component[start] >= 0) {
            continue;
        }
        std::vector<std::size_t> stack{start};
        tri_component[start] = next_component;
        while (!stack.empty()) {
            const std::size_t tri_index = stack.back();
            stack.pop_back();
            const Tri& tri = mesh.triangles[tri_index];
            const std::array<std::size_t, 3> vertices{tri.v0, tri.v1, tri.v2};
            for (std::size_t vertex : vertices) {
                for (std::size_t neighbor : vertex_to_triangles[vertex]) {
                    if (tri_component[neighbor] < 0) {
                        tri_component[neighbor] = next_component;
                        stack.push_back(neighbor);
                    }
                }
            }
        }
        ++next_component;
    }

    std::vector<int> vertex_component(mesh.vertices.size(), -1);
    for (std::size_t tri_index = 0; tri_index < mesh.triangles.size(); ++tri_index) {
        const Tri& tri = mesh.triangles[tri_index];
        vertex_component[tri.v0] = tri_component[tri_index];
        vertex_component[tri.v1] = tri_component[tri_index];
        vertex_component[tri.v2] = tri_component[tri_index];
    }
    *component_count = next_component;
    return vertex_component;
}

using CellKey = std::tuple<long long, long long, long long>;

CellKey cellKey(const Vec3& point, double cell_size)
{
    return {
        static_cast<long long>(std::floor(point.x / cell_size)),
        static_cast<long long>(std::floor(point.y / cell_size)),
        static_cast<long long>(std::floor(point.z / cell_size)),
    };
}

int countLayerShellComponents(const std::vector<IsoMesh>& layers, double connect_radius)
{
    struct VertexRef {
        Vec3 point;
        int component = -1;
    };

    DisjointSet sets;
    std::map<CellKey, std::vector<VertexRef>> previous_index;
    const double radius_sq = connect_radius * connect_radius;

    for (const IsoMesh& layer : layers) {
        int local_components = 0;
        const std::vector<int> local_vertex_components = vertexComponents(layer, &local_components);
        std::vector<int> component_nodes(static_cast<std::size_t>(local_components), -1);
        for (int component = 0; component < local_components; ++component) {
            component_nodes[static_cast<std::size_t>(component)] = sets.add();
        }

        for (std::size_t vertex_index = 0; vertex_index < layer.vertices.size(); ++vertex_index) {
            const int local_component = local_vertex_components[vertex_index];
            if (local_component < 0) {
                continue;
            }
            const int node = component_nodes[static_cast<std::size_t>(local_component)];
            const CellKey key = cellKey(layer.vertices[vertex_index], connect_radius);
            const long long kx = std::get<0>(key);
            const long long ky = std::get<1>(key);
            const long long kz = std::get<2>(key);
            for (long long dz = -1; dz <= 1; ++dz) {
                for (long long dy = -1; dy <= 1; ++dy) {
                    for (long long dx = -1; dx <= 1; ++dx) {
                        const auto found = previous_index.find({kx + dx, ky + dy, kz + dz});
                        if (found == previous_index.end()) {
                            continue;
                        }
                        for (const VertexRef& candidate : found->second) {
                            if (squaredNorm(layer.vertices[vertex_index] - candidate.point) <= radius_sq) {
                                sets.unite(node, candidate.component);
                            }
                        }
                    }
                }
            }
        }

        previous_index.clear();
        for (std::size_t vertex_index = 0; vertex_index < layer.vertices.size(); ++vertex_index) {
            const int local_component = local_vertex_components[vertex_index];
            if (local_component < 0) {
                continue;
            }
            const int node = component_nodes[static_cast<std::size_t>(local_component)];
            previous_index[cellKey(layer.vertices[vertex_index], connect_radius)].push_back({layer.vertices[vertex_index], node});
        }
    }

    return sets.parent.empty() ? 0 : sets.countRoots();
}

std::pair<double, double> finiteRange(const ScalarField& phi)
{
    double min_value = std::numeric_limits<double>::infinity();
    double max_value = -std::numeric_limits<double>::infinity();
    for (double value : phi.values) {
        if (!std::isfinite(value)) {
            continue;
        }
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
    }
    return {min_value, max_value};
}

void writePhiPointCloudPly(const ScalarField& phi, const std::filesystem::path& output_path)
{
    const auto range = finiteRange(phi);
    const double span = std::max(1e-12, range.second - range.first);
    std::size_t finite_count = 0;
    for (double value : phi.values) {
        if (std::isfinite(value)) {
            ++finite_count;
        }
    }

    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open phi PLY output: " + output_path.u8string());
    }

    output << "ply\nformat ascii 1.0\n";
    output << "element vertex " << finite_count << '\n';
    output << "property float x\nproperty float y\nproperty float z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "end_header\n";

    for (int z = 0; z < phi.nz; ++z) {
        for (int y = 0; y < phi.ny; ++y) {
            for (int x = 0; x < phi.nx; ++x) {
                const double value = phi.values[scalarIndex(phi, x, y, z)];
                if (!std::isfinite(value)) {
                    continue;
                }
                const Vec3 point = scalarPoint(phi, x, y, z);
                const double t = std::max(0.0, std::min(1.0, (value - range.first) / span));
                const int red = static_cast<int>(255.0 * t);
                const int green = static_cast<int>(255.0 * (1.0 - std::abs(2.0 * t - 1.0)));
                const int blue = static_cast<int>(255.0 * (1.0 - t));
                output << point.x << ' ' << point.y << ' ' << point.z << ' '
                       << red << ' ' << green << ' ' << blue << '\n';
            }
        }
    }
}

void writeMetricsJson(const ModelReport& report, const std::filesystem::path& output_path)
{
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open metrics output: " + output_path.u8string());
    }

    const auto writeSizeArray = [&output](const std::vector<std::size_t>& values) {
        output << '[';
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i != 0) {
                output << ", ";
            }
            output << values[i];
        }
        output << ']';
    };
    const auto writeIntArray = [&output](const std::vector<int>& values) {
        output << '[';
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i != 0) {
                output << ", ";
            }
            output << values[i];
        }
        output << ']';
    };

    output << "{\n";
    output << "  \"model\": \"" << report.name << "\",\n";
    output << "  \"source\": \"phase2_field\",\n";
    output << "  \"M4\": {\n";
    output << "    \"face_count\": " << report.face_count_total << ",\n";
    output << "    \"face_count_total\": " << report.face_count_total << ",\n";
    output << "    \"face_count_per_layer\": ";
    writeSizeArray(report.face_count_per_layer);
    output << ",\n";
    output << "    \"connected_components\": " << report.connected_components << ",\n";
    output << "    \"max_connected_components_per_layer\": " << report.max_layer_connected_components << ",\n";
    output << "    \"connected_components_per_layer\": ";
    writeIntArray(report.connected_components_per_layer);
    output << ",\n";
    output << "    \"layer_count\": " << report.layer_count << "\n";
    output << "  },\n";
    output << "  \"M6\": {\n";
    output << "    \"sdf_ms\": " << report.sdf_ms << ",\n";
    output << "    \"laplacian_ms\": " << report.laplacian_ms << ",\n";
    output << "    \"poisson_ms\": " << report.poisson_ms << ",\n";
    output << "    \"iso_surface_ms\": " << report.iso_surface_ms << ",\n";
    output << "    \"solver_ms\": " << report.laplacian_ms << ",\n";
    output << "    \"stl_layer_count\": " << report.layer_count << ",\n";
    output << "    \"peak_rss_mb\": " << report.peak_rss_mb << "\n";
    output << "  }\n";
    output << "}\n";
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

            const auto sdf_start = Clock::now();
            const SDF sdf = buildSDF(mesh, config.voxel);
            const auto sdf_end = Clock::now();

            const auto laplacian_start = Clock::now();
            const LaplacianBC bc = generateBC(grid, sdf, config.field_boundary);
            LaplacianParams laplacian_params = config.algorithm.field.laplacian;
            const ScalarField phi = solveLaplacian(grid, bc, laplacian_params);
            validatePhi(phi);
            const auto laplacian_end = Clock::now();

            const std::filesystem::path model_dir = config.io.output_root / model.name;
            std::filesystem::create_directories(model_dir);
            if (config.io.debug_dump_intermediates) {
                writePhiPointCloudPly(phi, model_dir / "phi_points.ply");
            }

            const auto iso_start = Clock::now();
            const std::vector<double> levels = planIsoLevels(
                phi,
                isoParamsFromConfig(config.iso_surface, config.field_boundary, phi.bbox));
            int layer_index = 0;
            std::vector<IsoMesh> layer_meshes;
            layer_meshes.reserve(levels.size());
            for (double level : levels) {
                IsoMesh iso_mesh = extractIsoSurface(phi, level, layer_index);
                if (iso_mesh.triangles.empty()) {
                    continue;
                }
                const int components = countConnectedComponents(iso_mesh);
                writeIsoMeshStl(iso_mesh, layerPath(model_dir, layer_index));

                report.iso_vertices += iso_mesh.vertices.size();
                report.iso_triangles += iso_mesh.triangles.size();
                report.face_count_total += iso_mesh.triangles.size();
                report.face_count_per_layer.push_back(iso_mesh.triangles.size());
                report.connected_components_per_layer.push_back(components);
                report.max_layer_connected_components = std::max(report.max_layer_connected_components, components);
                layer_meshes.push_back(std::move(iso_mesh));
                ++report.layer_count;
                ++layer_index;
            }
            const double shell_connect_radius =
                std::max(config.voxel.spacing_mm * 2.5, config.iso_surface.layer_thickness_mm * 1.75);
            report.connected_components = countLayerShellComponents(layer_meshes, shell_connect_radius);
            const auto iso_end = Clock::now();

            report.success = true;
            report.message = "ok";
            report.triangle_count = mesh.triangles.size();
            report.bbox = mesh.bbox;
            report.voxel = grid.report();
            report.read_ms = elapsedMs(read_start, read_end);
            report.voxelize_ms = elapsedMs(voxel_start, voxel_end);
            report.sdf_ms = elapsedMs(sdf_start, sdf_end);
            report.laplacian_ms = elapsedMs(laplacian_start, laplacian_end);
            report.poisson_ms = 0.0;
            report.iso_surface_ms = elapsedMs(iso_start, iso_end);
            report.peak_rss_mb = currentPeakRssMb();
            report.metrics_path = model_dir / "metrics.json";

            writeMetricsJson(report, report.metrics_path);

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
    output << "Curved Slicer Batch Report\n";
    output << "success=" << report.success_count
           << " failure=" << report.failure_count
           << " total_ms=" << std::fixed << std::setprecision(3) << report.total_ms << '\n';

    for (const ModelReport& model : report.models) {
        output << "ModelReport name=" << model.name
               << " status=" << (model.success ? "ok" : "failed")
               << " triangles=" << model.triangle_count
               << " read_ms=" << std::fixed << std::setprecision(3) << model.read_ms
               << " voxelize_ms=" << model.voxelize_ms
               << " sdf_ms=" << model.sdf_ms
               << " laplacian_ms=" << model.laplacian_ms
               << " poisson_ms=" << model.poisson_ms
               << " iso_surface_ms=" << model.iso_surface_ms
               << " peak_rss_mb=" << model.peak_rss_mb
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
            if (model.layer_count > 0) {
                output << "  iso_surface_ms=" << std::fixed << std::setprecision(3) << model.iso_surface_ms
                       << " iso_vertices=" << model.iso_vertices
                       << " iso_triangles=" << model.iso_triangles
                       << " connected_components=" << model.connected_components
                       << " max_layer_connected_components=" << model.max_layer_connected_components
                       << " layer_count=" << model.layer_count
                       << " face_count_total=" << model.face_count_total
                       << '\n';
                output << "  face_count_per_layer=[";
                for (std::size_t i = 0; i < model.face_count_per_layer.size(); ++i) {
                    if (i != 0) {
                        output << ',';
                    }
                    output << model.face_count_per_layer[i];
                }
                output << "] connected_components_per_layer=[";
                for (std::size_t i = 0; i < model.connected_components_per_layer.size(); ++i) {
                    if (i != 0) {
                        output << ',';
                    }
                    output << model.connected_components_per_layer[i];
                }
                output << "]\n";
            }
            if (!model.metrics_path.empty()) {
                output << "  metrics=" << model.metrics_path.u8string() << '\n';
            }
        } else {
            output << "  error=" << model.message << '\n';
        }
    }
}

}  // namespace cslc
