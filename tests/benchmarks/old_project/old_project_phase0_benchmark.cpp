#include "basicDataType/TEACInclude.h"
#include "basicDataType/STLReader.h"
#include "SUPPORT_FREE_PRINT/StlToSDF.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
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

struct TimedMesh {
    OldMesh mesh;
    double read_ms = 0.0;
};

struct VoxelStats {
    double sdf_read_stl_ms = 0.0;
    double sdf_init_ms = 0.0;
    double voxel_loop_ms = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    unsigned long long occupied = 0;
};

double elapsedMs(Clock::time_point start, Clock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

TimedMesh readWithOldStlReader(const std::filesystem::path& path)
{
    const auto start = Clock::now();

    STLReader reader;
    size_t estimated = 0;
    reader.init(path.string(), &estimated);

    OldMesh mesh;
    mesh.vertices.reserve(estimated * 3);

    STLFacet face{};
    while (reader.getFace(face)) {
        ++mesh.triangle_count;
        for (const Point3f& p : face.p) {
            mesh.vertices.emplace_back(p.x, p.y, p.z);
            mesh.cube.dealPoint(p);
        }
    }

    TimedMesh result;
    result.mesh = std::move(mesh);
    result.read_ms = elapsedMs(start, Clock::now());
    return result;
}

VoxelStats runOldVoxelPipeline(const OldMesh& mesh, double spacing_mm)
{
    VoxelStats stats;
    SDF sdf;
    sdf.set_voxel_size(static_cast<float>(spacing_mm));

    const auto sdf_read_start = Clock::now();
    sdf.ReadStl(mesh.vertices);
    stats.sdf_read_stl_ms = elapsedMs(sdf_read_start, Clock::now());

    const auto sdf_init_start = Clock::now();
    sdf.Init();
    stats.sdf_init_ms = elapsedMs(sdf_init_start, Clock::now());

    const double len = spacing_mm * 1.5;
    const glm::vec3 min(
        static_cast<float>(mesh.cube.min.x - len),
        static_cast<float>(mesh.cube.min.y - len),
        static_cast<float>(mesh.cube.min.z - len));

    const int x_num = static_cast<int>(std::ceil((mesh.cube.w() + 3.0 * spacing_mm) / spacing_mm));
    const int y_num = static_cast<int>(std::ceil((mesh.cube.h() + 3.0 * spacing_mm) / spacing_mm));
    const int z_num = static_cast<int>(std::ceil((mesh.cube.t() + 3.0 * spacing_mm) / spacing_mm));
    stats.nx = x_num + 1;
    stats.ny = y_num + 1;
    stats.nz = z_num + 1;

    const auto voxel_start = Clock::now();
    const double half_res = spacing_mm * 0.5;
    const double threshold = spacing_mm * 1.73205081 + 0.01;
    unsigned long long occupied = 0;

    for (int z = 0; z <= z_num; ++z) {
        for (int y = 0; y <= y_num; ++y) {
            for (int x = 0; x <= x_num; ++x) {
                const float qx = static_cast<float>(min.x + x * spacing_mm + half_res);
                const float qy = static_cast<float>(min.y + y * spacing_mm + half_res);
                const float qz = static_cast<float>(min.z + z * spacing_mm + half_res);
                if (sdf.Query(qx, qy, qz) <= threshold) {
                    ++occupied;
                }
            }
        }
    }

    stats.voxel_loop_ms = elapsedMs(voxel_start, Clock::now());
    stats.occupied = occupied;
    return stats;
}

std::string bboxSize(const Cubef& cube)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << cube.w() << "x" << cube.h() << "x" << cube.t();
    return out.str();
}

std::vector<ModelCase> makeModelCases(const std::filesystem::path& models_dir)
{
    return {
        {"armadillo_flat", models_dir / "armadillo_flat.stl"},
        {"bunny", models_dir / "bunny(46_35_45).stl"},
        {"mao", models_dir / std::filesystem::u8path(u8"毛主席头雕(42_52_70).stl")},
    };
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "usage: old_project_phase0_benchmark <models_dir> [spacing_mm]\n";
        return 2;
    }

    const std::filesystem::path models_dir = std::filesystem::path(argv[1]);
    const double spacing_mm = argc >= 3 ? std::stod(argv[2]) : 5.0;

    app_log::config().enabled = false;
    SDF::Initialize();

    std::cout << "Old Project Phase 0 Benchmark\n";
    std::cout << "models_dir=" << models_dir.u8string() << "\n";
    std::cout << "spacing_mm=" << spacing_mm << "\n";
    std::cout << "model,file_mb,triangles,old_stl_reader_ms,old_sdf_readstl_ms,old_sdf_init_ms,old_voxel_loop_ms,old_voxel_total_ms,bbox_size_mm,grid,occupied\n";

    for (const ModelCase& model : makeModelCases(models_dir)) {
        try {
            const auto file_size = std::filesystem::file_size(model.path);
            const TimedMesh timed_mesh = readWithOldStlReader(model.path);
            const VoxelStats voxel = runOldVoxelPipeline(timed_mesh.mesh, spacing_mm);
            const double voxel_total = voxel.sdf_read_stl_ms + voxel.sdf_init_ms + voxel.voxel_loop_ms;

            std::cout << model.name << ','
                      << std::fixed << std::setprecision(3)
                      << static_cast<double>(file_size) / (1024.0 * 1024.0) << ','
                      << timed_mesh.mesh.triangle_count << ','
                      << timed_mesh.read_ms << ','
                      << voxel.sdf_read_stl_ms << ','
                      << voxel.sdf_init_ms << ','
                      << voxel.voxel_loop_ms << ','
                      << voxel_total << ','
                      << bboxSize(timed_mesh.mesh.cube) << ','
                      << voxel.nx << 'x' << voxel.ny << 'x' << voxel.nz << ','
                      << voxel.occupied << '/'
                      << static_cast<unsigned long long>(voxel.nx) *
                             static_cast<unsigned long long>(voxel.ny) *
                             static_cast<unsigned long long>(voxel.nz)
                      << '\n';
        } catch (const std::exception& ex) {
            std::cout << model.name << ",ERROR," << ex.what() << '\n';
        }
    }

    return 0;
}

