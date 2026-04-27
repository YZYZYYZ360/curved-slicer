#include "surface/iso_surface.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace cslc {
namespace {

constexpr std::array<std::array<int, 4>, 6> kTetrahedra{{
    {{0, 5, 1, 6}},
    {{0, 1, 2, 6}},
    {{0, 2, 3, 6}},
    {{0, 3, 7, 6}},
    {{0, 7, 4, 6}},
    {{0, 4, 5, 6}},
}};

constexpr std::array<std::array<int, 2>, 6> kTetraEdges{{
    {{0, 1}},
    {{0, 2}},
    {{0, 3}},
    {{1, 2}},
    {{1, 3}},
    {{2, 3}},
}};

std::size_t scalarIndex(const ScalarField& phi, int x, int y, int z)
{
    return static_cast<std::size_t>(x) +
        static_cast<std::size_t>(phi.nx) *
            (static_cast<std::size_t>(y) + static_cast<std::size_t>(phi.ny) * static_cast<std::size_t>(z));
}

Vec3 latticePoint(const ScalarField& phi, int x, int y, int z)
{
    return {
        phi.bbox.min.x + (static_cast<double>(x) + 0.5) * phi.spacing,
        phi.bbox.min.y + (static_cast<double>(y) + 0.5) * phi.spacing,
        phi.bbox.min.z + (static_cast<double>(z) + 0.5) * phi.spacing,
    };
}

Vec3 interpolate(const Vec3& a, const Vec3& b, double va, double vb, double iso)
{
    const double denom = vb - va;
    const double t = std::abs(denom) < 1e-12 ? 0.5 : std::max(0.0, std::min(1.0, (iso - va) / denom));
    return a + (b - a) * t;
}

std::tuple<long long, long long, long long> vertexKey(const Vec3& point)
{
    constexpr double scale = 1'000'000'000.0;
    return {
        static_cast<long long>(std::llround(point.x * scale)),
        static_cast<long long>(std::llround(point.y * scale)),
        static_cast<long long>(std::llround(point.z * scale)),
    };
}

std::size_t addVertex(IsoMesh& mesh,
                      std::map<std::tuple<long long, long long, long long>, std::size_t>& vertex_lookup,
                      const Vec3& point)
{
    const auto key = vertexKey(point);
    const auto found = vertex_lookup.find(key);
    if (found != vertex_lookup.end()) {
        return found->second;
    }

    const std::size_t index = mesh.vertices.size();
    mesh.vertices.push_back(point);
    vertex_lookup.emplace(key, index);
    return index;
}

void addTriangle(IsoMesh& mesh,
                 std::map<std::tuple<long long, long long, long long>, std::size_t>& vertex_lookup,
                 const Vec3& a,
                 const Vec3& b,
                 const Vec3& c)
{
    const std::size_t ia = addVertex(mesh, vertex_lookup, a);
    const std::size_t ib = addVertex(mesh, vertex_lookup, b);
    const std::size_t ic = addVertex(mesh, vertex_lookup, c);
    if (ia == ib || ib == ic || ia == ic) {
        return;
    }
    mesh.triangles.push_back({ia, ib, ic});
}

void polygoniseTetra(IsoMesh& mesh,
                     std::map<std::tuple<long long, long long, long long>, std::size_t>& vertex_lookup,
                     const std::array<Vec3, 4>& points,
                     const std::array<double, 4>& values,
                     double iso)
{
    std::vector<Vec3> intersections;
    intersections.reserve(4);

    for (const auto& edge : kTetraEdges) {
        const int a = edge[0];
        const int b = edge[1];
        const bool a_inside = values[a] <= iso;
        const bool b_inside = values[b] <= iso;
        if (a_inside == b_inside) {
            continue;
        }
        intersections.push_back(interpolate(points[a], points[b], values[a], values[b], iso));
    }

    if (intersections.size() == 3) {
        addTriangle(mesh, vertex_lookup, intersections[0], intersections[1], intersections[2]);
    } else if (intersections.size() == 4) {
        addTriangle(mesh, vertex_lookup, intersections[0], intersections[1], intersections[2]);
        addTriangle(mesh, vertex_lookup, intersections[0], intersections[2], intersections[3]);
    }
}

}  // namespace

std::vector<double> planIsoLevels(const ScalarField& phi, const IsoExtractParams& params)
{
    if (params.layer_thickness_mm <= 0.0) {
        throw std::runtime_error("iso_surface.layer_thickness_mm must be positive");
    }
    if (params.max_layers <= 0) {
        throw std::runtime_error("iso_surface.max_layers must be positive");
    }

    double min_value = std::numeric_limits<double>::infinity();
    double max_value = -std::numeric_limits<double>::infinity();
    for (double value : phi.values) {
        if (!std::isfinite(value)) {
            continue;
        }
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
    }

    if (!std::isfinite(min_value) || !std::isfinite(max_value) || min_value == max_value) {
        return {};
    }

    std::vector<double> levels;
    double iso = min_value + params.phi_start_offset;
    if (iso <= min_value) {
        iso = min_value + params.layer_thickness_mm;
    }
    for (; iso < max_value && static_cast<int>(levels.size()) < params.max_layers; iso += params.layer_thickness_mm) {
        levels.push_back(iso);
    }
    if (levels.empty()) {
        levels.push_back((min_value + max_value) * 0.5);
    }
    return levels;
}

IsoMesh extractIsoSurface(const ScalarField& phi, double iso_value, int layer_id)
{
    if (phi.nx < 2 || phi.ny < 2 || phi.nz < 2) {
        return {};
    }

    IsoMesh mesh;
    mesh.iso_value = iso_value;
    mesh.layer_id = layer_id;
    std::map<std::tuple<long long, long long, long long>, std::size_t> vertex_lookup;

    constexpr std::array<std::array<int, 3>, 8> corner_offsets{{
        {{0, 0, 0}},
        {{1, 0, 0}},
        {{1, 1, 0}},
        {{0, 1, 0}},
        {{0, 0, 1}},
        {{1, 0, 1}},
        {{1, 1, 1}},
        {{0, 1, 1}},
    }};

    for (int z = 0; z + 1 < phi.nz; ++z) {
        for (int y = 0; y + 1 < phi.ny; ++y) {
            for (int x = 0; x + 1 < phi.nx; ++x) {
                std::array<Vec3, 8> cube_points;
                std::array<double, 8> cube_values;
                bool has_finite = false;
                bool has_below = false;
                bool has_above = false;

                for (std::size_t corner = 0; corner < corner_offsets.size(); ++corner) {
                    const int cx = x + corner_offsets[corner][0];
                    const int cy = y + corner_offsets[corner][1];
                    const int cz = z + corner_offsets[corner][2];
                    const double value = phi.values[scalarIndex(phi, cx, cy, cz)];
                    cube_points[corner] = latticePoint(phi, cx, cy, cz);
                    cube_values[corner] = value;
                    if (std::isfinite(value)) {
                        has_finite = true;
                        has_below = has_below || value <= iso_value;
                        has_above = has_above || value > iso_value;
                    }
                }

                if (!has_finite || !has_below || !has_above) {
                    continue;
                }

                for (const auto& tetra : kTetrahedra) {
                    std::array<Vec3, 4> tetra_points;
                    std::array<double, 4> tetra_values;
                    bool tetra_finite = true;
                    for (std::size_t i = 0; i < tetra.size(); ++i) {
                        tetra_points[i] = cube_points[tetra[i]];
                        tetra_values[i] = cube_values[tetra[i]];
                        tetra_finite = tetra_finite && std::isfinite(tetra_values[i]);
                    }
                    if (tetra_finite) {
                        polygoniseTetra(mesh, vertex_lookup, tetra_points, tetra_values, iso_value);
                    }
                }
            }
        }
    }

    return mesh;
}

int countConnectedComponents(const IsoMesh& mesh)
{
    if (mesh.triangles.empty()) {
        return 0;
    }

    std::vector<std::vector<std::size_t>> vertex_to_triangles(mesh.vertices.size());
    for (std::size_t tri_index = 0; tri_index < mesh.triangles.size(); ++tri_index) {
        const Tri& tri = mesh.triangles[tri_index];
        vertex_to_triangles[tri.v0].push_back(tri_index);
        vertex_to_triangles[tri.v1].push_back(tri_index);
        vertex_to_triangles[tri.v2].push_back(tri_index);
    }

    std::vector<bool> visited(mesh.triangles.size(), false);
    int components = 0;
    for (std::size_t start = 0; start < mesh.triangles.size(); ++start) {
        if (visited[start]) {
            continue;
        }
        ++components;
        std::queue<std::size_t> queue;
        visited[start] = true;
        queue.push(start);
        while (!queue.empty()) {
            const std::size_t tri_index = queue.front();
            queue.pop();
            const Tri& tri = mesh.triangles[tri_index];
            const std::array<std::size_t, 3> vertices{tri.v0, tri.v1, tri.v2};
            for (std::size_t vertex : vertices) {
                for (std::size_t neighbor : vertex_to_triangles[vertex]) {
                    if (!visited[neighbor]) {
                        visited[neighbor] = true;
                        queue.push(neighbor);
                    }
                }
            }
        }
    }
    return components;
}

void writeIsoMeshPly(const IsoMesh& mesh, const std::filesystem::path& output_path)
{
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open PLY output: " + output_path.u8string());
    }

    output << "ply\nformat ascii 1.0\n";
    output << "element vertex " << mesh.vertices.size() << '\n';
    output << "property float x\nproperty float y\nproperty float z\n";
    output << "element face " << mesh.triangles.size() << '\n';
    output << "property list uchar int vertex_indices\nend_header\n";
    for (const Vec3& vertex : mesh.vertices) {
        output << vertex.x << ' ' << vertex.y << ' ' << vertex.z << '\n';
    }
    for (const Tri& tri : mesh.triangles) {
        output << "3 " << tri.v0 << ' ' << tri.v1 << ' ' << tri.v2 << '\n';
    }
}

}  // namespace cslc
