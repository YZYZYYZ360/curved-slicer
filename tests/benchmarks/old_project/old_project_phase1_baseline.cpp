#include "basicDataType/TEACInclude.h"
#include "basicDataType/STLReader.h"
#include "SUPPORT_FREE_PRINT/SFF.h"

// DEFERRED to Phase 5: old-project baseline artifact generation is kept here for reuse,
// but Phase 2 field-algorithm work no longer depends on running this executable.

#include "io/stl_reader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct ModelCase {
    std::string name;
    std::filesystem::path path;
};

struct OldMesh {
    std::vector<glm::vec3> vertices;
    Cubef cube;
    std::size_t triangle_count = 0;
};

struct LayerMetric {
    std::filesystem::path path;
    std::size_t face_count = 0;
    int connected_components = 0;
};

struct VertexKey {
    long long x = 0;
    long long y = 0;
    long long z = 0;

    bool operator==(const VertexKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VertexKeyHash {
    std::size_t operator()(const VertexKey& key) const
    {
        std::size_t seed = 0;
        hashCombine(seed, key.x);
        hashCombine(seed, key.y);
        hashCombine(seed, key.z);
        return seed;
    }

    static void hashCombine(std::size_t& seed, long long value)
    {
        seed ^= std::hash<long long>{}(value) + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
    }
};

class DisjointSet {
public:
    explicit DisjointSet(std::size_t count)
        : parent_(count),
          rank_(count, 0)
    {
        std::iota(parent_.begin(), parent_.end(), std::size_t{0});
    }

    std::size_t find(std::size_t value)
    {
        if (parent_[value] != value) {
            parent_[value] = find(parent_[value]);
        }
        return parent_[value];
    }

    void unite(std::size_t lhs, std::size_t rhs)
    {
        std::size_t root_lhs = find(lhs);
        std::size_t root_rhs = find(rhs);
        if (root_lhs == root_rhs) {
            return;
        }
        if (rank_[root_lhs] < rank_[root_rhs]) {
            std::swap(root_lhs, root_rhs);
        }
        parent_[root_rhs] = root_lhs;
        if (rank_[root_lhs] == rank_[root_rhs]) {
            ++rank_[root_lhs];
        }
    }

private:
    std::vector<std::size_t> parent_;
    std::vector<unsigned char> rank_;
};

double elapsedMs(Clock::time_point start, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string pathUtf8(const std::filesystem::path& path)
{
    return path.u8string();
}

std::string jsonEscape(const std::string& value)
{
    std::ostringstream out;
    for (char ch : value) {
        switch (ch) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            out << ch;
            break;
        }
    }
    return out.str();
}

template <typename T>
void writeJsonArray(std::ostream& output, const std::vector<T>& values)
{
    output << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            output << ", ";
        }
        output << values[i];
    }
    output << ']';
}

OldMesh readWithOldStlReader(const std::filesystem::path& path)
{
    STLReader reader;
    size_t estimated = 0;
    reader.init(path.string(), &estimated);

    OldMesh mesh;
    mesh.vertices.reserve(estimated * 3);

    STLFacet face{};
    while (reader.getFace(face)) {
        ++mesh.triangle_count;
        for (const Point3f& point : face.p) {
            mesh.vertices.emplace_back(point.x, point.y, point.z);
            mesh.cube.dealPoint(point);
        }
    }

    if (mesh.vertices.empty()) {
        throw std::runtime_error("Old STLReader returned no vertices: " + pathUtf8(path));
    }

    return mesh;
}

void prepareOldVoxel(SupportFreePrint::voxel& result, const OldMesh& mesh, double spacing_mm)
{
    result.sd = SDF();
    result.sd.set_voxel_size(static_cast<float>(spacing_mm));
    result.sd.ReadStl(mesh.vertices);
    result.sd.Init();

    const double padding = spacing_mm * 1.5;
    MeshReconstruction::Rect3 rect;
    rect.min = glm::vec3(
        static_cast<float>(mesh.cube.min.x - padding),
        static_cast<float>(mesh.cube.min.y - padding),
        static_cast<float>(mesh.cube.min.z - padding));
    rect.size = glm::vec3(
        static_cast<float>(mesh.cube.w() + 2.0 * padding),
        static_cast<float>(mesh.cube.h() + 2.0 * padding),
        static_cast<float>(mesh.cube.t() + 2.0 * padding));

    result.init(rect, spacing_mm);
    result.getVoxelModel({
        mesh.cube.w() + 3.0 * spacing_mm,
        mesh.cube.h() + 3.0 * spacing_mm,
        mesh.cube.t() + 3.0 * spacing_mm,
    });
    result.heightInterval = 1.0;
    result.iso_selection_mode = SupportFreePrint::IsoSelectionMode::Median;
    result.use_curvature_penalty = false;
    result.lambda_c = 0.0;
}

bool isLayerStl(const std::filesystem::path& path)
{
    const std::string name = path.filename().string();
    if (name.size() != 7 || name.substr(3) != ".stl") {
        return false;
    }
    return std::isdigit(static_cast<unsigned char>(name[0])) != 0 &&
        std::isdigit(static_cast<unsigned char>(name[1])) != 0 &&
        std::isdigit(static_cast<unsigned char>(name[2])) != 0;
}

std::vector<std::filesystem::path> listLayerStls(const std::filesystem::path& output_dir)
{
    std::vector<std::filesystem::path> layers;
    if (!std::filesystem::exists(output_dir)) {
        return layers;
    }

    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (entry.is_regular_file() && isLayerStl(entry.path())) {
            layers.push_back(entry.path());
        }
    }
    std::sort(layers.begin(), layers.end());
    return layers;
}

void clearPriorOutputs(const std::filesystem::path& output_dir)
{
    std::filesystem::create_directories(output_dir);
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        const auto& path = entry.path();
        if (isLayerStl(path) || path.filename() == "metrics.json") {
            std::filesystem::remove(path);
        }
    }
}

VertexKey quantize(const cslc::Vec3& point)
{
    constexpr double scale = 1'000'000.0;
    return {
        static_cast<long long>(std::llround(point.x * scale)),
        static_cast<long long>(std::llround(point.y * scale)),
        static_cast<long long>(std::llround(point.z * scale)),
    };
}

int countConnectedComponents(const cslc::TriangleMesh& mesh)
{
    if (mesh.triangles.empty()) {
        return 0;
    }

    DisjointSet sets(mesh.triangles.size());
    std::unordered_map<VertexKey, std::size_t, VertexKeyHash> first_triangle_by_vertex;
    first_triangle_by_vertex.reserve(mesh.triangles.size() * 3);

    for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
        const cslc::StlTriangle& triangle = mesh.triangles[triangle_index];
        for (const cslc::Vec3& vertex : triangle.vertices) {
            const VertexKey key = quantize(vertex);
            const auto inserted = first_triangle_by_vertex.emplace(key, triangle_index);
            if (!inserted.second) {
                sets.unite(triangle_index, inserted.first->second);
            }
        }
    }

    std::unordered_set<std::size_t> roots;
    roots.reserve(mesh.triangles.size());
    for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
        roots.insert(sets.find(triangle_index));
    }
    return static_cast<int>(roots.size());
}

LayerMetric analyzeLayer(const std::filesystem::path& layer_path)
{
    const cslc::TriangleMesh mesh = cslc::readStl(layer_path);
    return {
        layer_path,
        mesh.triangles.size(),
        countConnectedComponents(mesh),
    };
}

void writeMetricsJson(const std::filesystem::path& metrics_path,
                      const std::string& model_name,
                      const std::vector<LayerMetric>& layers,
                      double solver_ms)
{
    std::vector<std::size_t> face_counts;
    std::vector<int> connected_components;
    face_counts.reserve(layers.size());
    connected_components.reserve(layers.size());

    std::size_t face_count_total = 0;
    int connected_components_max = 0;
    for (const LayerMetric& layer : layers) {
        face_counts.push_back(layer.face_count);
        connected_components.push_back(layer.connected_components);
        face_count_total += layer.face_count;
        connected_components_max = std::max(connected_components_max, layer.connected_components);
    }

    std::ofstream output(metrics_path);
    if (!output) {
        throw std::runtime_error("Failed to open metrics output: " + pathUtf8(metrics_path));
    }

    output << "{\n";
    output << "  \"model\": \"" << jsonEscape(model_name) << "\",\n";
    output << "  \"source\": \"old_project_baseline\",\n";
    output << "  \"M4\": {\n";
    output << "    \"face_count\": " << face_count_total << ",\n";
    output << "    \"face_count_total\": " << face_count_total << ",\n";
    output << "    \"face_count_per_layer\": ";
    writeJsonArray(output, face_counts);
    output << ",\n";
    output << "    \"connected_components\": " << connected_components_max << ",\n";
    output << "    \"connected_components_per_layer\": ";
    writeJsonArray(output, connected_components);
    output << ",\n";
    output << "    \"layer_count\": " << layers.size() << "\n";
    output << "  },\n";
    output << "  \"M6\": {\n";
    output << "    \"solver_ms\": " << std::fixed << std::setprecision(3) << solver_ms << ",\n";
    output << "    \"stl_layer_count\": " << layers.size() << "\n";
    output << "  }\n";
    output << "}\n";
}

std::vector<ModelCase> makeModelCases(const std::filesystem::path& models_dir)
{
    return {
        {"armadillo_flat", models_dir / "armadillo_flat.stl"},
        {"bunny", models_dir / "bunny(46_35_45).stl"},
        {"mao", models_dir / std::filesystem::u8path(u8"毛主席头雕(42_52_70).stl")},
    };
}

int maxLayerCount(const OldMesh& mesh, double spacing_mm)
{
    return static_cast<int>(std::ceil(mesh.cube.t() / spacing_mm)) + 20;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "usage: old_project_phase1_baseline <models_dir> <spacing_mm> <output_root>\n";
        return 2;
    }

    const std::filesystem::path models_dir = std::filesystem::path(argv[1]);
    const double spacing_mm = std::stod(argv[2]);
    const std::filesystem::path output_root = std::filesystem::path(argv[3]);

    if (spacing_mm <= 0.0) {
        std::cerr << "spacing_mm must be positive\n";
        return 2;
    }

    std::cout << "Old Project Phase 1 Baseline\n";
    std::cout << "models_dir=" << pathUtf8(models_dir) << "\n";
    std::cout << "spacing_mm=" << spacing_mm << "\n";
    std::cout << "output_root=" << pathUtf8(output_root) << "\n";
    std::cout << "model,triangles,solver_ms,layer_count,face_count_total,connected_components_max,output_dir\n";
    std::cout << "phase,detail\n" << std::flush;

    app_log::config().enabled = false;
    std::cout << "init,SDF::Initialize.begin\n" << std::flush;
    SDF::Initialize();
    std::cout << "init,SDF::Initialize.done\n" << std::flush;

    bool any_error = false;
    for (const ModelCase& model : makeModelCases(models_dir)) {
        try {
            const std::filesystem::path output_dir = output_root / model.name;
            std::cout << "model_begin," << model.name << "\n" << std::flush;
            clearPriorOutputs(output_dir);

            std::cout << "old_stl_read_begin," << model.name << "\n" << std::flush;
            const OldMesh mesh = readWithOldStlReader(model.path);
            std::cout << "old_stl_read_done," << model.name << "\n" << std::flush;
            SupportFreePrint::voxel old_voxel;
            std::cout << "old_voxel_prepare_begin," << model.name << "\n" << std::flush;
            prepareOldVoxel(old_voxel, mesh, spacing_mm);
            std::cout << "old_voxel_prepare_done," << model.name << "\n" << std::flush;
            std::vector<glm::vec3> curve_res;
            std::vector<glm::vec4> rgbs;

            std::cout << "getDistanceFiled_begin," << model.name << "\n" << std::flush;
            const auto solver_start = Clock::now();
            SupportFreePrint::getDistanceFiled(
                old_voxel,
                pathUtf8(output_dir),
                curve_res,
                rgbs,
                maxLayerCount(mesh, spacing_mm));
            const double solver_ms = elapsedMs(solver_start, Clock::now());
            std::cout << "getDistanceFiled_done," << model.name << "\n" << std::flush;

            std::vector<LayerMetric> layers;
            std::cout << "postprocess_begin," << model.name << "\n" << std::flush;
            for (const std::filesystem::path& layer_path : listLayerStls(output_dir)) {
                layers.push_back(analyzeLayer(layer_path));
            }
            std::cout << "postprocess_done," << model.name << "\n" << std::flush;

            if (layers.empty()) {
                throw std::runtime_error("getDistanceFiled did not write any layer STL to " + pathUtf8(output_dir));
            }

            writeMetricsJson(output_dir / "metrics.json", model.name, layers, solver_ms);

            std::size_t face_count_total = 0;
            int connected_components_max = 0;
            for (const LayerMetric& layer : layers) {
                face_count_total += layer.face_count;
                connected_components_max = std::max(connected_components_max, layer.connected_components);
            }

            std::cout << model.name << ','
                      << mesh.triangle_count << ','
                      << std::fixed << std::setprecision(3) << solver_ms << ','
                      << layers.size() << ','
                      << face_count_total << ','
                      << connected_components_max << ','
                      << pathUtf8(output_dir) << '\n';
        } catch (const std::exception& ex) {
            any_error = true;
            std::cout << model.name << ",ERROR," << ex.what() << '\n';
        }
    }

    return any_error ? 1 : 0;
}
