#include "app/pipeline.h"

#include "field/kuka_projection.h"
#include "field/laplacian.h"
#include "geometry/sdf.h"
#include "geometry/voxel_grid.h"
#include "kinematics/dh_params.h"
#include "kinematics/forward_kin.h"
#include "kinematics/ik_solver.h"
#include "kinematics/reachability.h"
#include "metrics/curvature.h"
#include "path/geodesic_paths.h"
#include "path/pose_from_path.h"
#include "surface/iso_surface.h"
#include "trajectory/poly5_smoother.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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

std::size_t sdfIndex(const SDF& sdf, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(sdf.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(sdf.ny) * static_cast<std::size_t>(z));
}

std::size_t vectorIndex(const VectorField& field, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(field.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(field.ny) * static_cast<std::size_t>(z));
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

double sdfValue(const SDF& sdf, int x, int y, int z)
{
    x = std::max(0, std::min(x, sdf.nx - 1));
    y = std::max(0, std::min(y, sdf.ny - 1));
    z = std::max(0, std::min(z, sdf.nz - 1));
    return sdf.values[sdfIndex(sdf, x, y, z)];
}

Vec3 sdfGradient(const SDF& sdf, int x, int y, int z)
{
    const auto derivative = [&sdf](int x0, int y0, int z0, int axis) {
        int minus_x = x0;
        int minus_y = y0;
        int minus_z = z0;
        int plus_x = x0;
        int plus_y = y0;
        int plus_z = z0;
        if (axis == 0) {
            minus_x = std::max(0, x0 - 1);
            plus_x = std::min(sdf.nx - 1, x0 + 1);
        } else if (axis == 1) {
            minus_y = std::max(0, y0 - 1);
            plus_y = std::min(sdf.ny - 1, y0 + 1);
        } else {
            minus_z = std::max(0, z0 - 1);
            plus_z = std::min(sdf.nz - 1, z0 + 1);
        }

        const int delta = std::abs(plus_x - minus_x) +
            std::abs(plus_y - minus_y) +
            std::abs(plus_z - minus_z);
        if (delta == 0) {
            return 0.0;
        }
        return (sdfValue(sdf, plus_x, plus_y, plus_z) -
                sdfValue(sdf, minus_x, minus_y, minus_z)) /
            (static_cast<double>(delta) * sdf.spacing);
    };

    return {derivative(x, y, z, 0), derivative(x, y, z, 1), derivative(x, y, z, 2)};
}

Vec3 printDirection(const BCParams& params)
{
    const Vec3 direction{
        params.print_direction[0],
        params.print_direction[1],
        params.print_direction[2],
    };
    const Vec3 unit = normalized(direction);
    if (norm(unit) == 0.0) {
        throw std::runtime_error("field.boundary.print_direction must be non-zero");
    }
    return unit;
}

VoxelIndex chooseBottomAnchor(const VoxelGrid& grid, const SDF& sdf, const BCParams& params)
{
    const Vec3 direction = printDirection(params);
    const double sdf_band = params.bottom_sdf_band * grid.spacing();
    VoxelIndex best{0, 0, 0};
    double best_score = -std::numeric_limits<double>::infinity();

    for (const VoxelIndex& voxel : grid.occupiedVoxels()) {
        if (std::abs(sdfValue(sdf, voxel.x, voxel.y, voxel.z)) > sdf_band) {
            continue;
        }
        const Vec3 normal = normalized(sdfGradient(sdf, voxel.x, voxel.y, voxel.z));
        if (norm(normal) == 0.0) {
            continue;
        }
        const double alignment = dot(normal, direction);
        if (alignment >= params.bottom_dot_threshold) {
            continue;
        }
        const double score = -alignment;
        if (score > best_score) {
            best_score = score;
            best = voxel;
        }
    }

    if (best_score < 0.0) {
        throw std::runtime_error("chooseBottomAnchor: no bottom voxel found");
    }
    return best;
}

struct PhiGaugeDiagnostics {
    double min_pre_shift = 0.0;
    double max_pre_shift = 0.0;
    double range_pre_shift = 0.0;
    double min_post_shift = 0.0;
    double max_post_shift = 0.0;
    double progression = 0.0;
    double bbox_extent = 0.0;
    VoxelIndex min_voxel{0, 0, 0};
    VoxelIndex max_voxel{0, 0, 0};
    Vec3 min_point;
    Vec3 max_point;
    double min_sdf = 0.0;
    double max_sdf = 0.0;
    double min_alignment = 0.0;
    double max_alignment = 0.0;
    bool min_on_bottom_boundary = false;
    bool max_on_top_boundary = false;
};

PhiGaugeDiagnostics gaugeShiftPhi(ScalarField& phi,
                                  const VoxelGrid& grid,
                                  const SDF& sdf,
                                  const BCParams& boundary,
                                  bool multi_anchor_used = false)
{
    PhiGaugeDiagnostics diagnostics;
    diagnostics.min_pre_shift = std::numeric_limits<double>::infinity();
    diagnostics.max_pre_shift = -std::numeric_limits<double>::infinity();

    for (int z = 0; z < phi.nz; ++z) {
        for (int y = 0; y < phi.ny; ++y) {
            for (int x = 0; x < phi.nx; ++x) {
                const double value = phi.values[scalarIndex(phi, x, y, z)];
                if (!std::isfinite(value)) {
                    continue;
                }
                if (value < diagnostics.min_pre_shift) {
                    diagnostics.min_pre_shift = value;
                    diagnostics.min_voxel = {x, y, z};
                }
                if (value > diagnostics.max_pre_shift) {
                    diagnostics.max_pre_shift = value;
                    diagnostics.max_voxel = {x, y, z};
                }
            }
        }
    }

    if (!std::isfinite(diagnostics.min_pre_shift) || !std::isfinite(diagnostics.max_pre_shift)) {
        throw std::runtime_error("gaugeShiftPhi requires finite phi values");
    }

    diagnostics.min_point = scalarPoint(phi, diagnostics.min_voxel.x, diagnostics.min_voxel.y, diagnostics.min_voxel.z);
    diagnostics.max_point = scalarPoint(phi, diagnostics.max_voxel.x, diagnostics.max_voxel.y, diagnostics.max_voxel.z);
    const Vec3 direction = printDirection(boundary);
    const auto sampleAlignment = [&sdf, &direction](const VoxelIndex& voxel) {
        const Vec3 normal = normalized(sdfGradient(sdf, voxel.x, voxel.y, voxel.z));
        return norm(normal) == 0.0 ? 0.0 : dot(normal, direction);
    };
    diagnostics.min_sdf = sdfValue(sdf, diagnostics.min_voxel.x, diagnostics.min_voxel.y, diagnostics.min_voxel.z);
    diagnostics.max_sdf = sdfValue(sdf, diagnostics.max_voxel.x, diagnostics.max_voxel.y, diagnostics.max_voxel.z);
    diagnostics.min_alignment = sampleAlignment(diagnostics.min_voxel);
    diagnostics.max_alignment = sampleAlignment(diagnostics.max_voxel);

    const double sdf_band = boundary.bottom_sdf_band * grid.spacing();
    diagnostics.min_on_bottom_boundary =
        std::abs(diagnostics.min_sdf) <= sdf_band &&
        diagnostics.min_alignment < boundary.bottom_dot_threshold;
    diagnostics.max_on_top_boundary =
        std::abs(diagnostics.max_sdf) <= sdf_band &&
        diagnostics.max_alignment > -boundary.bottom_dot_threshold;

    const auto describeVoxel = [](const char* label,
                                  const VoxelIndex& voxel,
                                  const Vec3& point,
                                  double sdf_sample,
                                  double alignment) {
        std::ostringstream message;
        message << label << " voxel=(" << voxel.x << ',' << voxel.y << ',' << voxel.z << ")"
                << " point=(" << point.x << ',' << point.y << ',' << point.z << ")"
                << " sdf=" << sdf_sample
                << " alignment=" << alignment;
        return message.str();
    };

    const Vec3 size = phi.bbox.size();
    diagnostics.bbox_extent = std::max(
        1e-12,
        std::abs(direction.x) * size.x + std::abs(direction.y) * size.y + std::abs(direction.z) * size.z);
    diagnostics.progression = dot(diagnostics.max_point - diagnostics.min_point, direction);
    diagnostics.range_pre_shift = diagnostics.max_pre_shift - diagnostics.min_pre_shift;

    // v4 §9: 多 anchor 模式下底面已构造性钉死，无需 progression / phi_max 检查
    if (!multi_anchor_used) {
        if (diagnostics.progression < 0.5 * diagnostics.bbox_extent) {
            std::ostringstream message;
            message << "phi_max not progressing along print_direction (single-anchor mode)"
                    << " progression=" << diagnostics.progression
                    << " bbox_extent=" << diagnostics.bbox_extent << "; "
                    << describeVoxel("min", diagnostics.min_voxel, diagnostics.min_point,
                                     diagnostics.min_sdf, diagnostics.min_alignment)
                    << "; "
                    << describeVoxel("max", diagnostics.max_voxel, diagnostics.max_point,
                                     diagnostics.max_sdf, diagnostics.max_alignment);
            throw std::runtime_error(message.str());
        }
    }
    if (diagnostics.range_pre_shift < 0.10 * diagnostics.bbox_extent ||
        diagnostics.range_pre_shift > 3.0 * diagnostics.bbox_extent) {
        std::ostringstream message;
        message << "phi_range implausible vs bbox_extent"
                << " range=" << diagnostics.range_pre_shift
                << " bbox_extent=" << diagnostics.bbox_extent;
        throw std::runtime_error(message.str());
    }
    if (diagnostics.min_pre_shift < -0.10 * diagnostics.range_pre_shift) {
        std::ostringstream message;
        message << "phi negative drift > 10% of range"
                << " min=" << diagnostics.min_pre_shift
                << " range=" << diagnostics.range_pre_shift;
        throw std::runtime_error(message.str());
    }

    for (double& value : phi.values) {
        if (std::isfinite(value)) {
            value -= diagnostics.min_pre_shift;
        }
    }

    diagnostics.min_post_shift = 0.0;
    diagnostics.max_post_shift = diagnostics.max_pre_shift - diagnostics.min_pre_shift;
    return diagnostics;
}

double hemisphereViolationRatio(const VoxelGrid& grid,
                                const VectorField& field,
                                const ReachabilityParams& params)
{
    const Vec3 up = normalized(params.workpiece_up);
    if (norm(up) == 0.0) {
        throw std::runtime_error("reachability.workpiece_up must be non-zero");
    }

    std::size_t total = 0;
    std::size_t violations = 0;
    for (const VoxelIndex& voxel : grid.occupiedVoxels()) {
        const Vec3 value = field.values[vectorIndex(field, voxel.x, voxel.y, voxel.z)];
        if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
            continue;
        }
        ++total;
        if (dot(normalized(value), up) + 1e-9 < params.min_dot_threshold) {
            ++violations;
        }
    }
    return total == 0 ? 0.0 : static_cast<double>(violations) / static_cast<double>(total);
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
                                     const AABB& bbox,
                                     bool phi_is_normalized)
{
    IsoExtractParams params;
    const double extent = phi_is_normalized ? projectedExtent(bbox, boundary) : 0.0;
    params.layer_thickness_mm =
        extent > 0.0 ? config.layer_thickness_mm / extent : config.layer_thickness_mm;
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
    double max_value = -std::numeric_limits<double>::infinity();
    bool has_value = false;
    for (double value : phi.values) {
        if (std::isinf(value)) {
            throw std::runtime_error("phi field has infinite values");
        }
        if (std::isnan(value)) {
            continue;
        }
        has_value = true;
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
    }
    if (!has_value) {
        throw std::runtime_error("phi field has no finite occupied values");
    }
    if (min_value < -1e-8) {
        std::ostringstream message;
        message << "phi field has negative values min=" << min_value
                << " max=" << max_value;
        throw std::runtime_error(message.str());
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

struct ComponentNodeInfo {
    std::size_t triangle_count = 0;
    std::size_t vertex_count = 0;
    AABB bbox;
    int layer_min = std::numeric_limits<int>::max();
    int layer_max = std::numeric_limits<int>::min();
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

std::string formatComponentSummary(std::size_t index, const ComponentSummary& component)
{
    std::ostringstream output;
    output << "component[" << index << "] root=" << component.root_id
           << " vertices=" << component.vertex_count
           << " triangles=" << component.triangle_count
           << " bbox=[("
           << component.bbox.min.x << ',' << component.bbox.min.y << ',' << component.bbox.min.z
           << ")..("
           << component.bbox.max.x << ',' << component.bbox.max.y << ',' << component.bbox.max.z
           << ")] z=[" << component.bbox.min.z << ',' << component.bbox.max.z << ']'
           << " layer=[" << component.layer_min << ',' << component.layer_max << ']';
    return output.str();
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

int countLayerShellComponents(const std::vector<IsoMesh>& layers,
                              double connect_radius,
                              std::vector<ComponentSummary>* component_details,
                              std::vector<std::string>* component_summary)
{
    struct VertexRef {
        Vec3 point;
        int component = -1;
    };

    DisjointSet sets;
    std::map<CellKey, std::vector<VertexRef>> previous_index;
    const double radius_sq = connect_radius * connect_radius;
    std::vector<ComponentNodeInfo> node_infos;

    for (const IsoMesh& layer : layers) {
        int local_components = 0;
        const std::vector<int> local_vertex_components = vertexComponents(layer, &local_components);
        std::vector<std::size_t> local_triangle_counts(static_cast<std::size_t>(local_components), 0);
        for (const Tri& triangle : layer.triangles) {
            const int component = local_vertex_components[triangle.v0];
            if (component >= 0) {
                ++local_triangle_counts[static_cast<std::size_t>(component)];
            }
        }

        std::vector<int> component_nodes(static_cast<std::size_t>(local_components), -1);
        for (int component = 0; component < local_components; ++component) {
            component_nodes[static_cast<std::size_t>(component)] = sets.add();
            node_infos.push_back({});
            node_infos.back().triangle_count = local_triangle_counts[static_cast<std::size_t>(component)];
            node_infos.back().layer_min = layer.layer_id;
            node_infos.back().layer_max = layer.layer_id;
        }

        for (std::size_t vertex_index = 0; vertex_index < layer.vertices.size(); ++vertex_index) {
            const int local_component = local_vertex_components[vertex_index];
            if (local_component < 0) {
                continue;
            }
            const int node = component_nodes[static_cast<std::size_t>(local_component)];
            ComponentNodeInfo& node_info = node_infos[static_cast<std::size_t>(node)];
            ++node_info.vertex_count;
            node_info.bbox.expand(layer.vertices[vertex_index]);
            node_info.layer_min = std::min(node_info.layer_min, layer.layer_id);
            node_info.layer_max = std::max(node_info.layer_max, layer.layer_id);

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

    std::vector<ComponentSummary> components;
    if (!sets.parent.empty()) {
        std::map<int, ComponentSummary> by_root;
        for (int node = 0; node < static_cast<int>(node_infos.size()); ++node) {
            const ComponentNodeInfo& node_info = node_infos[static_cast<std::size_t>(node)];
            const int root = sets.find(node);
            auto inserted = by_root.emplace(root, ComponentSummary{});
            ComponentSummary& component = inserted.first->second;
            if (inserted.second) {
                component.root_id = root;
                component.layer_min = std::numeric_limits<int>::max();
                component.layer_max = std::numeric_limits<int>::min();
            }
            component.root_id = root;
            component.triangle_count += node_info.triangle_count;
            component.vertex_count += node_info.vertex_count;
            if (node_info.bbox.valid()) {
                component.bbox.expand(node_info.bbox.min);
                component.bbox.expand(node_info.bbox.max);
            }
            component.layer_min = std::min(component.layer_min, node_info.layer_min);
            component.layer_max = std::max(component.layer_max, node_info.layer_max);
        }
        for (const auto& item : by_root) {
            components.push_back(item.second);
        }
    }

    std::sort(components.begin(), components.end(), [](const ComponentSummary& lhs, const ComponentSummary& rhs) {
        return lhs.vertex_count > rhs.vertex_count;
    });

    std::vector<std::string> summaries;
    const std::size_t print_count = std::min<std::size_t>(5, components.size());
    for (std::size_t i = 0; i < print_count; ++i) {
        summaries.push_back(formatComponentSummary(i, components[i]));
        std::cout << summaries.back() << '\n';
    }
    if (component_details != nullptr) {
        *component_details = components;
    }
    if (component_summary != nullptr) {
        *component_summary = summaries;
    }

    return static_cast<int>(components.size());
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

void writeVoxelOccupiedPly(const VoxelGrid& grid, const std::filesystem::path& output_path)
{
    const auto voxels = grid.occupiedVoxels();
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open voxel PLY: " + output_path.u8string());
    }
    output << "ply\nformat ascii 1.0\n";
    output << "element vertex " << voxels.size() << '\n';
    output << "property float x\nproperty float y\nproperty float z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "end_header\n";
    for (const VoxelIndex& v : voxels) {
        const double x = grid.bbox().min.x + (v.x + 0.5) * grid.spacing();
        const double y = grid.bbox().min.y + (v.y + 0.5) * grid.spacing();
        const double z = grid.bbox().min.z + (v.z + 0.5) * grid.spacing();
        output << x << ' ' << y << ' ' << z << " 255 255 255\n";
    }
}

void writeSdfPointsPly(const VoxelGrid& grid, const SDF& sdf, const std::filesystem::path& output_path)
{
    const auto voxels = grid.occupiedVoxels();
    double min_sdf = std::numeric_limits<double>::infinity();
    double max_sdf = -std::numeric_limits<double>::infinity();
    for (const VoxelIndex& v : voxels) {
        const double val = sdfValue(sdf, v.x, v.y, v.z);
        if (std::isfinite(val)) {
            min_sdf = std::min(min_sdf, val);
            max_sdf = std::max(max_sdf, val);
        }
    }
    const double span = std::max(1e-12, max_sdf - min_sdf);

    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open SDF PLY: " + output_path.u8string());
    }
    output << "ply\nformat ascii 1.0\n";
    output << "element vertex " << voxels.size() << '\n';
    output << "property float x\nproperty float y\nproperty float z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "end_header\n";
    for (const VoxelIndex& v : voxels) {
        const double x = grid.bbox().min.x + (v.x + 0.5) * grid.spacing();
        const double y = grid.bbox().min.y + (v.y + 0.5) * grid.spacing();
        const double z = grid.bbox().min.z + (v.z + 0.5) * grid.spacing();
        const double val = sdfValue(sdf, v.x, v.y, v.z);
        const double t = std::max(0.0, std::min(1.0, (val - min_sdf) / span));
        const int red = static_cast<int>(255.0 * t);
        const int green = static_cast<int>(255.0 * (1.0 - std::abs(2.0 * t - 1.0)));
        const int blue = static_cast<int>(255.0 * (1.0 - t));
        output << x << ' ' << y << ' ' << z << ' '
               << red << ' ' << green << ' ' << blue << '\n';
    }
}

void writeBcVoxelsPly(const VoxelGrid& grid,
                      const LaplacianVectorBC& bc,
                      const Vec3& print_dir,
                      const std::filesystem::path& output_path)
{
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open BC PLY: " + output_path.u8string());
    }
    output << "ply\nformat ascii 1.0\n";
    output << "element vertex " << bc.fixed_indices.size() << '\n';
    output << "property float x\nproperty float y\nproperty float z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "end_header\n";
    for (std::size_t i = 0; i < bc.fixed_indices.size(); ++i) {
        const VoxelIndex& v = bc.fixed_indices[i];
        const double x = grid.bbox().min.x + (v.x + 0.5) * grid.spacing();
        const double y = grid.bbox().min.y + (v.y + 0.5) * grid.spacing();
        const double z = grid.bbox().min.z + (v.z + 0.5) * grid.spacing();
        // v4 §10: 优先用 bc_zones，回退到向量差判定
        bool is_bottom;
        if (!bc.bc_zones.empty()) {
            is_bottom = (bc.bc_zones[i] == BCZone::Bottom);
        } else {
            const Vec3 diff = bc.fixed_vectors[i] - print_dir;
            is_bottom = norm(diff) < 1e-6;
        }
        output << x << ' ' << y << ' ' << z << ' '
               << (is_bottom ? 255 : 0) << ' ' << 0 << ' ' << (is_bottom ? 0 : 255) << '\n';
    }
}

void writeGFieldPly(const VoxelGrid& grid,
                    const VectorField& field,
                    const std::filesystem::path& output_path)
{
    const auto voxels = grid.occupiedVoxels();
    const double seg_len = 0.5 * grid.spacing();
    const Vec3 up{0.0, 0.0, 1.0};

    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open G field PLY: " + output_path.u8string());
    }
    const std::size_t vertex_count = voxels.size() * 2;
    const std::size_t edge_count = voxels.size();
    output << "ply\nformat ascii 1.0\n";
    output << "element vertex " << vertex_count << '\n';
    output << "property float x\nproperty float y\nproperty float z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "element edge " << edge_count << '\n';
    output << "property int vertex1\nproperty int vertex2\n";
    output << "end_header\n";
    for (std::size_t i = 0; i < voxels.size(); ++i) {
        const VoxelIndex& v = voxels[i];
        const double cx = grid.bbox().min.x + (v.x + 0.5) * grid.spacing();
        const double cy = grid.bbox().min.y + (v.y + 0.5) * grid.spacing();
        const double cz = grid.bbox().min.z + (v.z + 0.5) * grid.spacing();
        const Vec3 g = field.values[grid.index(v.x, v.y, v.z)];
        const double dot_up = dot(normalized(g), up);
        const double t = std::max(0.0, std::min(1.0, (dot_up + 1.0) * 0.5));
        const int red = static_cast<int>(255.0 * (1.0 - t));
        const int green = static_cast<int>(255.0 * t);
        const int blue = 0;
        output << cx << ' ' << cy << ' ' << cz << ' ' << red << ' ' << green << ' ' << blue << '\n';
        const double ex = cx + g.x * seg_len;
        const double ey = cy + g.y * seg_len;
        const double ez = cz + g.z * seg_len;
        output << ex << ' ' << ey << ' ' << ez << ' ' << red << ' ' << green << ' ' << blue << '\n';
    }
    for (std::size_t i = 0; i < voxels.size(); ++i) {
        output << (i * 2) << ' ' << (i * 2 + 1) << '\n';
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
    const auto writeDoubleArray = [&output](const std::vector<double>& values) {
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
    output << "  \"source\": \"" << (report.pipeline.empty() ? "field" : report.pipeline) << "\",\n";
    output << "  \"M1\": {\n";
    output << "    \"max_abs_mean_curvature\": " << report.m1_max_abs_mean_curvature << ",\n";
    output << "    \"per_layer\": ";
    writeDoubleArray(report.m1_per_layer);
    output << "\n";
    output << "  },\n";
    output << "  \"M2\": {\n";
    output << "    \"hemisphere_violation_ratio\": " << report.m2_hemisphere_violation_ratio << "\n";
    output << "  },\n";
    output << "  \"phi\": {\n";
    output << "    \"min_pre_shift\": " << report.phi_min_pre_shift << ",\n";
    output << "    \"max_pre_shift\": " << report.phi_max_pre_shift << ",\n";
    output << "    \"range_pre_shift\": " << report.phi_range_pre_shift << ",\n";
    output << "    \"progression\": " << report.phi_progression << ",\n";
    output << "    \"bbox_extent\": " << report.phi_bbox_extent << ",\n";
    output << "    \"min\": " << report.phi_min << ",\n";
    output << "    \"max\": " << report.phi_max << ",\n";
    output << "    \"min_voxel\": [" << report.phi_min_voxel.x << ", "
           << report.phi_min_voxel.y << ", " << report.phi_min_voxel.z << "],\n";
    output << "    \"max_voxel\": [" << report.phi_max_voxel.x << ", "
           << report.phi_max_voxel.y << ", " << report.phi_max_voxel.z << "],\n";
    output << "    \"min_point\": [" << report.phi_min_point.x << ", "
           << report.phi_min_point.y << ", " << report.phi_min_point.z << "],\n";
    output << "    \"max_point\": [" << report.phi_max_point.x << ", "
           << report.phi_max_point.y << ", " << report.phi_max_point.z << "],\n";
    output << "    \"min_sdf\": " << report.phi_min_sdf << ",\n";
    output << "    \"max_sdf\": " << report.phi_max_sdf << ",\n";
    output << "    \"min_alignment\": " << report.phi_min_alignment << ",\n";
    output << "    \"max_alignment\": " << report.phi_max_alignment << ",\n";
    output << "    \"min_on_bottom_boundary\": " << (report.phi_min_on_bottom_boundary ? "true" : "false") << ",\n";
    output << "    \"max_on_top_boundary\": " << (report.phi_max_on_top_boundary ? "true" : "false") << "\n";
    output << "  },\n";
    output << "  \"components\": [\n";
    for (std::size_t i = 0; i < report.component_details.size(); ++i) {
        const ComponentSummary& component = report.component_details[i];
        output << "    {\"root_id\": " << component.root_id
               << ", \"vertices\": " << component.vertex_count
               << ", \"triangles\": " << component.triangle_count
               << ", \"bbox_min\": [" << component.bbox.min.x << ", "
               << component.bbox.min.y << ", " << component.bbox.min.z << "]"
               << ", \"bbox_max\": [" << component.bbox.max.x << ", "
               << component.bbox.max.y << ", " << component.bbox.max.z << "]"
               << ", \"z_range\": [" << component.bbox.min.z << ", " << component.bbox.max.z << "]"
               << ", \"layer_range\": [" << component.layer_min << ", " << component.layer_max << "]}";
        if (i + 1 != report.component_details.size()) {
            output << ',';
        }
        output << '\n';
    }
    output << "  ],\n";
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
    output << "    \"smoothing_ms\": " << report.smoothing_ms << ",\n";
    output << "    \"iso_surface_ms\": " << report.iso_surface_ms << ",\n";
    output << "    \"solver_ms\": " << (report.laplacian_ms + report.poisson_ms) << ",\n";
    output << "    \"stl_layer_count\": " << report.layer_count << ",\n";
    output << "    \"peak_rss_mb\": " << report.peak_rss_mb << ",\n";
    output << "    \"phi_min\": " << report.phi_min << ",\n";
    output << "    \"phi_max\": " << report.phi_max << "\n";
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
        report.pipeline = config.algorithm.pipeline.empty() ? "scalar" : config.algorithm.pipeline;

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

            ScalarField phi;
            bool phi_is_normalized = true;
            double laplacian_ms = 0.0;
            double poisson_ms = 0.0;
            double smoothing_ms = 0.0;
            PhiGaugeDiagnostics gauge_diagnostics;
            bool has_gauge_diagnostics = false;

            if (report.pipeline == "scalar") {
                const auto laplacian_start = Clock::now();
                const LaplacianBC bc = generateBC(grid, sdf, config.field_boundary);
                LaplacianParams laplacian_params = config.algorithm.field.laplacian;
                phi = solveLaplacian(grid, bc, laplacian_params);
                const auto laplacian_end = Clock::now();
                laplacian_ms = elapsedMs(laplacian_start, laplacian_end);
                phi_is_normalized = true;
            } else if (report.pipeline == "vector_kuka") {
                const auto laplacian_start = Clock::now();
                std::cout << "[" << report.name << "] generateVectorBC begin\n" << std::flush;
                const LaplacianVectorBC bc = generateVectorBC(grid, sdf, config.field_boundary);
                std::cout << "[" << report.name << "] generateVectorBC done, fixed_count="
                          << bc.fixed_indices.size() << "\n" << std::flush;
                LaplacianParams laplacian_params = config.algorithm.field.laplacian;
                std::cout << "[" << report.name << "] solveLaplacianVector begin\n" << std::flush;
                const VectorField vector_field = solveLaplacianVector(grid, bc, laplacian_params);
                std::cout << "[" << report.name << "] solveLaplacianVector done\n" << std::flush;
                const auto laplacian_end = Clock::now();
                laplacian_ms = elapsedMs(laplacian_start, laplacian_end);

                const auto poisson_start = Clock::now();
                std::cout << "[" << report.name << "] projectToHemisphere begin\n" << std::flush;
                const VectorField clamped = projectToHemisphere(grid, vector_field, config.kuka.reachability);
                std::cout << "[" << report.name << "] projectToHemisphere done\n" << std::flush;
                report.m2_hemisphere_violation_ratio =
                    hemisphereViolationRatio(grid, clamped, config.kuka.reachability);
                if (report.m2_hemisphere_violation_ratio > 0.0) {
                    throw std::runtime_error("hemisphere clamp left violating vectors");
                }

                const VectorField* poisson_input = &clamped;
                VectorField smoothed_field;
                if (config.algorithm.field.smoothing.passes > 0) {
                    const auto smooth_start = Clock::now();
                    std::cout << "[" << report.name << "] smoothVectorField begin\n" << std::flush;
                    smoothed_field = smoothVectorField(grid, clamped, bc,
                                                       config.algorithm.field.smoothing);
                    std::cout << "[" << report.name << "] smoothVectorField done\n" << std::flush;
                    const auto smooth_end = Clock::now();
                    smoothing_ms = elapsedMs(smooth_start, smooth_end);
                    poisson_input = &smoothed_field;
                }

                PoissonParams poisson_params = config.algorithm.field.poisson;
                // v4 §9+§10: 提取底面+顶面 anchor 集合（优先 bc_zones，回退向量差）
                const Vec3 print_dir = printDirection(config.field_boundary);
                for (std::size_t i = 0; i < bc.fixed_indices.size(); ++i) {
                    if (!bc.bc_zones.empty()) {
                        // geometric_z 策略：bc_zones 明确区分底/顶
                        poisson_params.anchor_voxels.push_back(bc.fixed_indices[i]);
                        poisson_params.anchor_values.push_back(
                            bc.bc_zones[i] == BCZone::Bottom ? 0.0 : 1.0);
                    } else {
                        // bottom_up 策略：向量差判定，仅底面做 anchor
                        const Vec3 diff = bc.fixed_vectors[i] - print_dir;
                        if (norm(diff) < 1e-6) {
                            poisson_params.anchor_voxels.push_back(bc.fixed_indices[i]);
                            poisson_params.anchor_values.push_back(0.0);
                        }
                    }
                }
                poisson_params.log_iterations = true;
                poisson_params.progress_label = report.name;
                std::cout << "[" << report.name << "] solvePoisson begin (anchor_count="
                          << poisson_params.anchor_voxels.size() << ")\n" << std::flush;
                phi = solvePoisson(grid, *poisson_input, poisson_params);
                std::cout << "[" << report.name << "] solvePoisson done\n" << std::flush;
                gauge_diagnostics = gaugeShiftPhi(phi, grid, sdf, config.field_boundary,
                                                    !poisson_params.anchor_voxels.empty());
                has_gauge_diagnostics = true;
                const auto poisson_end = Clock::now();
                poisson_ms = elapsedMs(poisson_start, poisson_end);
                phi_is_normalized = false;

                // Phase 3.5: dump intermediate fields
                if (config.io.debug_dump_intermediates) {
                    const std::filesystem::path dump_dir = config.io.output_root / model.name;
                    std::cout << "[" << report.name << "] dumping intermediates\n" << std::flush;
                    writeVoxelOccupiedPly(grid, dump_dir / "voxel_occupied.ply");
                    writeSdfPointsPly(grid, sdf, dump_dir / "sdf_points.ply");
                    writeBcVoxelsPly(grid, bc, print_dir, dump_dir / "bc_voxels.ply");
                    writeGFieldPly(grid, clamped, dump_dir / "g_field.ply");
                    std::cout << "[" << report.name << "] dumping done\n" << std::flush;
                }
            } else if (report.pipeline == "vector_kuka_v4") {
                // v4 pipeline: same field computation as vector_kuka, then
                // geodesic paths → IK → trajectory smoothing → CSV output
                const auto laplacian_start = Clock::now();
                const LaplacianVectorBC bc = generateVectorBC(grid, sdf, config.field_boundary);
                LaplacianParams laplacian_params = config.algorithm.field.laplacian;
                const VectorField vector_field = solveLaplacianVector(grid, bc, laplacian_params);
                const auto laplacian_end = Clock::now();
                laplacian_ms = elapsedMs(laplacian_start, laplacian_end);

                const auto poisson_start = Clock::now();
                const VectorField clamped = projectToHemisphere(grid, vector_field, config.kuka.reachability);
                report.m2_hemisphere_violation_ratio =
                    hemisphereViolationRatio(grid, clamped, config.kuka.reachability);

                const VectorField* poisson_input = &clamped;
                VectorField smoothed_field;
                if (config.algorithm.field.smoothing.passes > 0) {
                    smoothed_field = smoothVectorField(grid, clamped, bc,
                                                       config.algorithm.field.smoothing);
                    poisson_input = &smoothed_field;
                }

                PoissonParams poisson_params = config.algorithm.field.poisson;
                const Vec3 print_dir = printDirection(config.field_boundary);
                for (std::size_t i = 0; i < bc.fixed_indices.size(); ++i) {
                    if (!bc.bc_zones.empty()) {
                        poisson_params.anchor_voxels.push_back(bc.fixed_indices[i]);
                        poisson_params.anchor_values.push_back(
                            bc.bc_zones[i] == BCZone::Bottom ? 0.0 : 1.0);
                    } else {
                        const Vec3 diff = bc.fixed_vectors[i] - print_dir;
                        if (norm(diff) < 1e-6) {
                            poisson_params.anchor_voxels.push_back(bc.fixed_indices[i]);
                            poisson_params.anchor_values.push_back(0.0);
                        }
                    }
                }
                poisson_params.log_iterations = true;
                poisson_params.progress_label = report.name;
                phi = solvePoisson(grid, *poisson_input, poisson_params);
                gauge_diagnostics = gaugeShiftPhi(phi, grid, sdf, config.field_boundary,
                                                    !poisson_params.anchor_voxels.empty());
                has_gauge_diagnostics = true;
                const auto poisson_end = Clock::now();
                poisson_ms = elapsedMs(poisson_start, poisson_end);
                phi_is_normalized = false;
            } else {
                throw std::runtime_error("algorithm.pipeline must be scalar, vector_kuka, or vector_kuka_v4");
            }

            validatePhi(phi);
            const auto phi_range = finiteRange(phi);

            const std::filesystem::path model_dir = config.io.output_root / model.name;
            std::filesystem::create_directories(model_dir);
            if (config.io.debug_dump_intermediates) {
                writePhiPointCloudPly(phi, model_dir / "phi_points.ply");
            }

            const auto iso_start = Clock::now();
            const std::vector<double> levels = planIsoLevels(
                phi,
                isoParamsFromConfig(config.iso_surface, config.field_boundary, phi.bbox, phi_is_normalized));
            int layer_index = 0;
            std::vector<IsoMesh> layer_meshes;
            layer_meshes.reserve(levels.size());
            for (double level : levels) {
                IsoMesh iso_mesh = extractIsoSurface(phi, level, layer_index);
                if (iso_mesh.triangles.empty()) {
                    continue;
                }
                const int components = countConnectedComponents(iso_mesh);
                const double max_curvature = maxAbsMeanCurvature(computeMeanCurvature(iso_mesh));
                writeIsoMeshStl(iso_mesh, layerPath(model_dir, layer_index));

                report.iso_vertices += iso_mesh.vertices.size();
                report.iso_triangles += iso_mesh.triangles.size();
                report.face_count_total += iso_mesh.triangles.size();
                report.face_count_per_layer.push_back(iso_mesh.triangles.size());
                report.connected_components_per_layer.push_back(components);
                report.max_layer_connected_components = std::max(report.max_layer_connected_components, components);
                report.m1_per_layer.push_back(max_curvature);
                report.m1_max_abs_mean_curvature =
                    std::max(report.m1_max_abs_mean_curvature, max_curvature);
                layer_meshes.push_back(std::move(iso_mesh));
                ++report.layer_count;
                ++layer_index;
            }
            const double shell_connect_radius =
                std::max(config.voxel.spacing_mm * 2.5, config.iso_surface.layer_thickness_mm * 1.75);
            report.connected_components = countLayerShellComponents(
                layer_meshes,
                shell_connect_radius,
                &report.component_details,
                &report.component_summary);
            const auto iso_end = Clock::now();

            // v4 trajectory generation (§14 + §17.5)
            if (report.pipeline == "vector_kuka_v4") {
                const auto traj_start = Clock::now();
                std::cout << "[" << report.name << "] trajectory generation begin\n" << std::flush;

                KR4DHParams dh;
                for (int i = 0; i < 6; ++i) {
                    dh.qlim_deg[i][0] = config.kuka.limits[i].min_deg;
                    dh.qlim_deg[i][1] = config.kuka.limits[i].max_deg;
                }
                dh.tool_z_mm = config.kuka.robot.z_offset;

                JointConfig home;
                home.q_deg = config.kuka.home.q_deg;

                const Vec3 G_dir = normalized(config.kuka.reachability.workpiece_up);

                // Collect all PathPointWithJoints across layers, splitting at NaN
                std::vector<std::vector<PathPointWithJoints>> segments;
                segments.emplace_back();

                for (const auto& iso_mesh : layer_meshes) {
                    if (iso_mesh.vertices.empty() || iso_mesh.triangles.empty()) continue;

                    // Convert IsoMesh to Eigen matrices for geodesic paths
                    const int nv = static_cast<int>(iso_mesh.vertices.size());
                    const int nf = static_cast<int>(iso_mesh.triangles.size());
                    Eigen::MatrixXd V(nv, 3);
                    for (int i = 0; i < nv; ++i) {
                        V(i, 0) = iso_mesh.vertices[i].x;
                        V(i, 1) = iso_mesh.vertices[i].y;
                        V(i, 2) = iso_mesh.vertices[i].z;
                    }
                    Eigen::MatrixXi F(nf, 3);
                    for (int i = 0; i < nf; ++i) {
                        F(i, 0) = static_cast<int>(iso_mesh.triangles[i].v0);
                        F(i, 1) = static_cast<int>(iso_mesh.triangles[i].v1);
                        F(i, 2) = static_cast<int>(iso_mesh.triangles[i].v2);
                    }

                    auto geodesic_paths = generateGeodesicPaths(V, F, config.geodesic);

                    for (const auto& poly : geodesic_paths) {
                        if (poly.points.size() < 2) continue;

                        // Build point + tangent arrays for IK
                        std::vector<Vec3> pts, tans;
                        pts.reserve(poly.points.size());
                        tans.reserve(poly.tangents.size());
                        for (const auto& p : poly.points) {
                            pts.push_back({p.x(), p.y(), p.z()});
                        }
                        for (const auto& t : poly.tangents) {
                            tans.push_back({t.x(), t.y(), t.z()});
                        }

                        auto reach_result = filterReachablePathPoints(
                            pts, tans, G_dir, dh, config.kuka.reachability);

                        // Assemble PathPointWithJoints, split at unreachable points
                        for (size_t i = 0; i < pts.size(); ++i) {
                            if (reach_result.statuses[i] == ReachStatus::OK) {
                                PathPointWithJoints pp;
                                pp.cart_pos = poly.points[i];
                                pp.joint = reach_result.solutions[i];
                                pp.wire_on = true;
                                segments.back().push_back(pp);
                            } else {
                                // NaN: start new segment
                                if (segments.back().size() > 1) {
                                    segments.emplace_back();
                                } else if (!segments.back().empty()) {
                                    segments.back().clear();
                                }
                            }
                        }
                        // Start new segment between polylines
                        if (segments.back().size() > 1) {
                            segments.emplace_back();
                        } else if (!segments.back().empty()) {
                            segments.back().clear();
                        }
                    }
                }

                // Remove empty segments
                segments.erase(
                    std::remove_if(segments.begin(), segments.end(),
                                   [](const auto& s) { return s.size() < 2; }),
                    segments.end());

                // Smooth each segment and write trajectory.csv
                const std::filesystem::path traj_path = model_dir / "trajectory.csv";
                std::ofstream csv(traj_path);
                csv << "timestamp_ms,segment_id,A1,A2,A3,A4,A5,A6,wire_on\n";

                int total_traj_points = 0;
                int segment_id = 0;
                for (const auto& seg : segments) {
                    ++segment_id;
                    auto traj = smoothTrajectoryPoly5(seg, config.trajectory);
                    for (const auto& pt : traj) {
                        csv << std::fixed << std::setprecision(6)
                            << pt.timestamp_ms << "," << segment_id << ","
                            << pt.joint_deg[0] << "," << pt.joint_deg[1] << ","
                            << pt.joint_deg[2] << "," << pt.joint_deg[3] << ","
                            << pt.joint_deg[4] << "," << pt.joint_deg[5] << ","
                            << (pt.wire_on ? 1 : 0) << "\n";
                        ++total_traj_points;
                    }
                }
                csv.close();

                const auto traj_end = Clock::now();
                std::cout << "[" << report.name << "] trajectory done: "
                          << total_traj_points << " points, "
                          << segment_id << " segments, "
                          << std::fixed << std::setprecision(1)
                          << elapsedMs(traj_start, traj_end) << "ms\n" << std::flush;
            }

            report.success = true;
            report.message = "ok";
            report.triangle_count = mesh.triangles.size();
            report.bbox = mesh.bbox;
            report.voxel = grid.report();
            report.read_ms = elapsedMs(read_start, read_end);
            report.voxelize_ms = elapsedMs(voxel_start, voxel_end);
            report.sdf_ms = elapsedMs(sdf_start, sdf_end);
            report.laplacian_ms = laplacian_ms;
            report.poisson_ms = poisson_ms;
            report.smoothing_ms = smoothing_ms;
            report.iso_surface_ms = elapsedMs(iso_start, iso_end);
            report.peak_rss_mb = currentPeakRssMb();
            report.phi_min_pre_shift = has_gauge_diagnostics ? gauge_diagnostics.min_pre_shift : phi_range.first;
            report.phi_max_pre_shift = has_gauge_diagnostics ? gauge_diagnostics.max_pre_shift : phi_range.second;
            report.phi_range_pre_shift =
                has_gauge_diagnostics ? gauge_diagnostics.range_pre_shift : (phi_range.second - phi_range.first);
            report.phi_progression = has_gauge_diagnostics ? gauge_diagnostics.progression : 0.0;
            report.phi_bbox_extent =
                has_gauge_diagnostics ? gauge_diagnostics.bbox_extent : projectedExtent(phi.bbox, config.field_boundary);
            report.phi_min = phi_range.first;
            report.phi_max = phi_range.second;
            if (has_gauge_diagnostics) {
                report.phi_min_voxel = gauge_diagnostics.min_voxel;
                report.phi_max_voxel = gauge_diagnostics.max_voxel;
                report.phi_min_point = gauge_diagnostics.min_point;
                report.phi_max_point = gauge_diagnostics.max_point;
                report.phi_min_sdf = gauge_diagnostics.min_sdf;
                report.phi_max_sdf = gauge_diagnostics.max_sdf;
                report.phi_min_alignment = gauge_diagnostics.min_alignment;
                report.phi_max_alignment = gauge_diagnostics.max_alignment;
                report.phi_min_on_bottom_boundary = gauge_diagnostics.min_on_bottom_boundary;
                report.phi_max_on_top_boundary = gauge_diagnostics.max_on_top_boundary;
            }
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
               << " pipeline=" << model.pipeline
               << " triangles=" << model.triangle_count
               << " read_ms=" << std::fixed << std::setprecision(3) << model.read_ms
               << " voxelize_ms=" << model.voxelize_ms
               << " sdf_ms=" << model.sdf_ms
               << " laplacian_ms=" << model.laplacian_ms
               << " poisson_ms=" << model.poisson_ms
               << " smoothing_ms=" << model.smoothing_ms
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
                       << " m1_max_abs_mean_curvature=" << model.m1_max_abs_mean_curvature
                       << " m2_hemisphere_violation_ratio=" << model.m2_hemisphere_violation_ratio
                       << " phi_min_pre_shift=" << model.phi_min_pre_shift
                       << " phi_max_pre_shift=" << model.phi_max_pre_shift
                       << " phi_range_pre_shift=" << model.phi_range_pre_shift
                       << " phi_progression=" << model.phi_progression
                       << " phi_bbox_extent=" << model.phi_bbox_extent
                       << " phi_min=" << model.phi_min
                       << " phi_max=" << model.phi_max
                       << '\n';
                if (model.pipeline == "vector_kuka" || model.pipeline == "vector_kuka_v4") {
                    output << "  phi_diagnostic min_voxel=(" << model.phi_min_voxel.x << ','
                           << model.phi_min_voxel.y << ',' << model.phi_min_voxel.z << ")"
                           << " min_point=(" << model.phi_min_point.x << ','
                           << model.phi_min_point.y << ',' << model.phi_min_point.z << ")"
                           << " min_sdf=" << model.phi_min_sdf
                           << " min_alignment=" << model.phi_min_alignment
                           << " min_on_bottom_boundary="
                           << (model.phi_min_on_bottom_boundary ? "true" : "false")
                           << " max_voxel=(" << model.phi_max_voxel.x << ','
                           << model.phi_max_voxel.y << ',' << model.phi_max_voxel.z << ")"
                           << " max_point=(" << model.phi_max_point.x << ','
                           << model.phi_max_point.y << ',' << model.phi_max_point.z << ")"
                           << " max_sdf=" << model.phi_max_sdf
                           << " max_alignment=" << model.phi_max_alignment
                           << " max_on_top_boundary="
                           << (model.phi_max_on_top_boundary ? "true" : "false") << '\n';
                }
                for (const std::string& summary : model.component_summary) {
                    output << "  " << summary << '\n';
                }
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
