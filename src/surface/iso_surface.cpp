#include "surface/iso_surface.h"

#include "surface/mc_lookup_table.h"

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

constexpr std::array<std::array<int, 3>, 8> kCornerOffsets{{
    {{0, 0, 0}},
    {{1, 0, 0}},
    {{1, 1, 0}},
    {{0, 1, 0}},
    {{0, 0, 1}},
    {{1, 0, 1}},
    {{1, 1, 1}},
    {{0, 1, 1}},
}};

constexpr std::array<std::array<int, 2>, 12> kEdgeVertices{{
    {{0, 1}},
    {{1, 2}},
    {{2, 3}},
    {{0, 3}},
    {{4, 5}},
    {{5, 6}},
    {{6, 7}},
    {{4, 7}},
    {{0, 4}},
    {{1, 5}},
    {{2, 6}},
    {{3, 7}},
}};

constexpr std::array<std::array<int, 4>, 6> kFaceVertices{{
    {{0, 1, 5, 4}},
    {{1, 2, 6, 5}},
    {{3, 2, 6, 7}},
    {{0, 3, 7, 4}},
    {{0, 1, 2, 3}},
    {{4, 5, 6, 7}},
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

void addTriangleByIndex(IsoMesh& mesh, std::size_t a, std::size_t b, std::size_t c)
{
    if (a == b || b == c || a == c) {
        return;
    }
    mesh.triangles.push_back({a, b, c});
}

bool faceTest(int face_value, const std::array<double, 8>& cube)
{
    const int face = std::abs(face_value) - 1;
    const auto& vertices = kFaceVertices[static_cast<std::size_t>(face)];
    const double s = cube[vertices[0]] * cube[vertices[2]] - cube[vertices[1]] * cube[vertices[3]];
    return (((s > 0.0) == (face_value > 0)) == (cube[vertices[0]] > 0.0));
}

int interiorTest(int edge, const std::array<double, 8>& cube)
{
    const double aux1 = (cube[1] - cube[0]) * (cube[6] - cube[7]) -
        (cube[5] - cube[4]) * (cube[2] - cube[3]);
    if (std::abs(aux1) < 1e-12) {
        return edge > 0 ? 1 : 0;
    }

    const double aux2 = cube[0] * (cube[6] - cube[7]) -
        cube[4] * (cube[2] - cube[3]) +
        cube[7] * (cube[1] - cube[0]) -
        cube[3] * (cube[5] - cube[4]);
    const double s = -aux2 / (2.0 * aux1);
    if (s < 0.0 || s > 1.0) {
        return edge > 0 ? 1 : 0;
    }

    const double a = cube[0] + (cube[1] - cube[0]) * s;
    const double b = cube[4] + (cube[5] - cube[4]) * s;
    const double c = cube[7] + (cube[6] - cube[7]) * s;
    const double d = cube[3] + (cube[2] - cube[3]) * s;

    const int result = (a >= 0.0 ? 1 : 0) |
        ((b >= 0.0 ? 1 : 0) << 1) |
        ((c >= 0.0 ? 1 : 0) << 2) |
        ((d >= 0.0 ? 1 : 0) << 3);

    switch (result) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 6:
    case 8:
    case 9:
    case 12:
        return edge > 0 ? 1 : 0;
    case 7:
    case 11:
    case 13:
    case 14:
    case 15:
        return edge < 0 ? 1 : 0;
    case 5:
        return ((a * c < b * d) == (edge > 0)) ? 1 : 0;
    case 10:
        return ((a * c >= b * d) == (edge > 0)) ? 1 : 0;
    default:
        return 0;
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

    for (int z = 0; z + 1 < phi.nz; ++z) {
        for (int y = 0; y + 1 < phi.ny; ++y) {
            for (int x = 0; x + 1 < phi.nx; ++x) {
                std::array<Vec3, 8> cube_points;
                std::array<double, 8> cube_values;
                std::array<double, 8> cube_signed;
                bool has_finite = false;
                bool has_below = false;
                bool has_above = false;

                for (std::size_t corner = 0; corner < kCornerOffsets.size(); ++corner) {
                    const int cx = x + kCornerOffsets[corner][0];
                    const int cy = y + kCornerOffsets[corner][1];
                    const int cz = z + kCornerOffsets[corner][2];
                    const double value = phi.values[scalarIndex(phi, cx, cy, cz)];
                    cube_points[corner] = latticePoint(phi, cx, cy, cz);
                    cube_values[corner] = value;
                    cube_signed[corner] = value - iso_value;
                    if (std::isfinite(value)) {
                        has_finite = true;
                        has_below = has_below || value <= iso_value;
                        has_above = has_above || value > iso_value;
                    }
                }

                if (!has_finite || !has_below || !has_above) {
                    continue;
                }

                int raw_case = 0;
                bool all_finite = true;
                for (int corner = 0; corner < 8; ++corner) {
                    all_finite = all_finite && std::isfinite(cube_values[static_cast<std::size_t>(corner)]);
                    raw_case |= (cube_signed[static_cast<std::size_t>(corner)] > 0.0 ? 1 : 0) << corner;
                }
                if (!all_finite || raw_case == 0 || raw_case == 255) {
                    continue;
                }

                const int case_number = marchingCubeSymmetries[raw_case][0];
                if (case_number == 0) {
                    continue;
                }

                int test_result = 0;
                const int face_test_count = faceTest_num[raw_case][0];
                for (int test_index = face_test_count; test_index > 0; --test_index) {
                    const int face = faceTest_num[raw_case][test_index];
                    test_result = (test_result << 1) | (faceTest(face, cube_signed) ? 1 : 0);
                }
                if (interiorTest_num[raw_case] != 0) {
                    test_result |= interiorTest(interiorTest_num[raw_case], cube_signed) << face_test_count;
                }

                const int ambiguity_number = ambiguityTable[case_number][test_result];
                if (ambiguity_number < 0 || triangleTable[raw_case] == nullptr) {
                    continue;
                }

                unsigned char* table_entry = triangleTable[raw_case][ambiguity_number];
                const int triangle_count = table_entry[0] / 3;
                const bool has_center = table_entry[1] != 0;
                const unsigned char* triangle_edges = table_entry + 2;

                std::array<std::size_t, 13> vertex_indices;
                vertex_indices.fill(std::numeric_limits<std::size_t>::max());
                Vec3 center{};
                int center_count = 0;

                for (std::size_t edge = 0; edge < kEdgeVertices.size(); ++edge) {
                    const int a = kEdgeVertices[edge][0];
                    const int b = kEdgeVertices[edge][1];
                    const double va = cube_values[static_cast<std::size_t>(a)];
                    const double vb = cube_values[static_cast<std::size_t>(b)];
                    const bool crosses = (cube_signed[static_cast<std::size_t>(a)] > 0.0) !=
                        (cube_signed[static_cast<std::size_t>(b)] > 0.0);
                    if (!crosses) {
                        continue;
                    }
                    const Vec3 point = interpolate(
                        cube_points[static_cast<std::size_t>(a)],
                        cube_points[static_cast<std::size_t>(b)],
                        va,
                        vb,
                        iso_value);
                    vertex_indices[edge] = addVertex(mesh, vertex_lookup, point);
                    center = center + point;
                    ++center_count;
                }

                if (has_center && center_count > 0) {
                    vertex_indices[12] = addVertex(mesh, vertex_lookup, center * (1.0 / static_cast<double>(center_count)));
                }

                for (int tri = 0; tri < triangle_count; ++tri) {
                    const unsigned char ea = triangle_edges[3 * tri];
                    const unsigned char eb = triangle_edges[3 * tri + 1];
                    const unsigned char ec = triangle_edges[3 * tri + 2];
                    if (ea >= vertex_indices.size() || eb >= vertex_indices.size() || ec >= vertex_indices.size()) {
                        continue;
                    }
                    const std::size_t ia = vertex_indices[ea];
                    const std::size_t ib = vertex_indices[eb];
                    const std::size_t ic = vertex_indices[ec];
                    if (ia == std::numeric_limits<std::size_t>::max() ||
                        ib == std::numeric_limits<std::size_t>::max() ||
                        ic == std::numeric_limits<std::size_t>::max()) {
                        continue;
                    }
                    addTriangleByIndex(mesh, ia, ib, ic);
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

void writeIsoMeshStl(const IsoMesh& mesh, const std::filesystem::path& output_path)
{
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("Failed to open STL output: " + output_path.u8string());
    }

    output << "solid curved_slicer_iso_" << mesh.layer_id << '\n';
    for (const Tri& tri : mesh.triangles) {
        const Vec3& a = mesh.vertices[tri.v0];
        const Vec3& b = mesh.vertices[tri.v1];
        const Vec3& c = mesh.vertices[tri.v2];
        const Vec3 normal = normalized(cross(b - a, c - a));
        output << "  facet normal " << normal.x << ' ' << normal.y << ' ' << normal.z << '\n';
        output << "    outer loop\n";
        output << "      vertex " << a.x << ' ' << a.y << ' ' << a.z << '\n';
        output << "      vertex " << b.x << ' ' << b.y << ' ' << b.z << '\n';
        output << "      vertex " << c.x << ' ' << c.y << ' ' << c.z << '\n';
        output << "    endloop\n";
        output << "  endfacet\n";
    }
    output << "endsolid curved_slicer_iso_" << mesh.layer_id << '\n';
}

}  // namespace cslc
