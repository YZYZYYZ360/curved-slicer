#include "voxel/VoxelizationV6.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

int ringVertexIndex(int lat_steps, int lon_steps, int ring, int j)
{
    (void)lat_steps;
    return 1 + (ring - 1) * lon_steps + (j % lon_steps);
}

double signedVolume(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    double volume = 0.0;
    for (int f = 0; f < F.rows(); ++f) {
        const Eigen::Vector3d a = V.row(F(f, 0));
        const Eigen::Vector3d b = V.row(F(f, 1));
        const Eigen::Vector3d c = V.row(F(f, 2));
        volume += a.dot(b.cross(c)) / 6.0;
    }
    return volume;
}

void flipAllFaces(Eigen::MatrixXi& F)
{
    for (int f = 0; f < F.rows(); ++f) std::swap(F(f, 1), F(f, 2));
}

void buildSphere(int lat_steps, int lon_steps, double radius,
                 Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V.resize(2 + (lat_steps - 1) * lon_steps, 3);
    V.row(0) = Eigen::Vector3d(0.0, 0.0, radius);
    const int south = static_cast<int>(V.rows()) - 1;
    V.row(south) = Eigen::Vector3d(0.0, 0.0, -radius);

    for (int i = 1; i < lat_steps; ++i) {
        const double phi = M_PI * static_cast<double>(i) / static_cast<double>(lat_steps);
        const double z = radius * std::cos(phi);
        const double xy = radius * std::sin(phi);
        for (int j = 0; j < lon_steps; ++j) {
            const double theta = 2.0 * M_PI * static_cast<double>(j) /
                                 static_cast<double>(lon_steps);
            V.row(ringVertexIndex(lat_steps, lon_steps, i, j)) =
                Eigen::Vector3d(xy * std::cos(theta), xy * std::sin(theta), z);
        }
    }

    std::vector<Eigen::Vector3i> faces;
    faces.reserve(static_cast<size_t>(2 * lon_steps * (lat_steps - 1)));
    for (int j = 0; j < lon_steps; ++j) {
        faces.emplace_back(0,
                           ringVertexIndex(lat_steps, lon_steps, 1, j),
                           ringVertexIndex(lat_steps, lon_steps, 1, j + 1));
    }
    for (int i = 1; i < lat_steps - 1; ++i) {
        for (int j = 0; j < lon_steps; ++j) {
            const int a = ringVertexIndex(lat_steps, lon_steps, i, j);
            const int b = ringVertexIndex(lat_steps, lon_steps, i, j + 1);
            const int c = ringVertexIndex(lat_steps, lon_steps, i + 1, j);
            const int d = ringVertexIndex(lat_steps, lon_steps, i + 1, j + 1);
            faces.emplace_back(a, c, b);
            faces.emplace_back(b, c, d);
        }
    }
    for (int j = 0; j < lon_steps; ++j) {
        faces.emplace_back(south,
                           ringVertexIndex(lat_steps, lon_steps, lat_steps - 1, j + 1),
                           ringVertexIndex(lat_steps, lon_steps, lat_steps - 1, j));
    }

    F.resize(static_cast<int>(faces.size()), 3);
    for (int i = 0; i < F.rows(); ++i) F.row(i) = faces[static_cast<size_t>(i)].transpose();
    if (signedVolume(V, F) < 0.0) flipAllFaces(F);
}

int countOcc(const cslc::VoxelizationV6::Grid& grid, cslc::VoxelizationV6::Occ value)
{
    return static_cast<int>(std::count(grid.occupancy.begin(), grid.occupancy.end(), value));
}

int countOccupied(const cslc::VoxelizationV6::Grid& grid)
{
    return static_cast<int>(grid.occupancy.size()) -
        countOcc(grid, cslc::VoxelizationV6::Occ::OUTSIDE);
}

bool hasOutside6Neighbor(const cslc::VoxelizationV6::Grid& grid, const Eigen::Vector3i& v)
{
    static const int offsets[6][3] = {
        {-1, 0, 0}, {1, 0, 0}, {0, -1, 0},
        {0, 1, 0}, {0, 0, -1}, {0, 0, 1},
    };
    for (const auto& o : offsets) {
        const int i = v.x() + o[0];
        const int j = v.y() + o[1];
        const int k = v.z() + o[2];
        if (i < 0 || i >= grid.dims.x() ||
            j < 0 || j >= grid.dims.y() ||
            k < 0 || k >= grid.dims.z()) {
            return true;
        }
        if (grid.occupancy[static_cast<size_t>(grid.idx(i, j, k))] ==
            cslc::VoxelizationV6::Occ::OUTSIDE) {
            return true;
        }
    }
    return false;
}

void readAsciiPly(const std::filesystem::path& path, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    std::ifstream in(path);
    if (!in) throw std::runtime_error("failed to open PLY: " + path.u8string());

    std::string line;
    int n_vertices = -1;
    int n_faces = -1;
    while (std::getline(in, line)) {
        if (line.rfind("element vertex ", 0) == 0) {
            n_vertices = std::stoi(line.substr(15));
        } else if (line.rfind("element face ", 0) == 0) {
            n_faces = std::stoi(line.substr(13));
        } else if (line == "end_header") {
            break;
        }
    }
    if (n_vertices <= 0 || n_faces <= 0) {
        throw std::runtime_error("PLY missing vertex/face counts: " + path.u8string());
    }

    V.resize(n_vertices, 3);
    F.resize(n_faces, 3);
    for (int i = 0; i < n_vertices; ++i) {
        if (!(in >> V(i, 0) >> V(i, 1) >> V(i, 2))) {
            throw std::runtime_error("PLY vertex parse failed");
        }
    }
    for (int f = 0; f < n_faces; ++f) {
        int n = 0;
        if (!(in >> n >> F(f, 0) >> F(f, 1) >> F(f, 2)) || n != 3) {
            throw std::runtime_error("PLY face parse failed");
        }
    }
}

void writeBoundaryPly(const std::filesystem::path& path, const cslc::VoxelizationV6::Grid& grid)
{
    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open boundary PLY: " + path.u8string());
    out << "ply\nformat ascii 1.0\n";
    out << "element vertex " << grid.boundary_voxels.size() << "\n";
    out << "property float x\nproperty float y\nproperty float z\nend_header\n";
    for (const auto& ijk : grid.boundary_voxels) {
        const Eigen::Vector3d p = grid.center(ijk.x(), ijk.y(), ijk.z());
        out << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
    }
}

void writeSlicePgm(const std::filesystem::path& path, const cslc::VoxelizationV6::Grid& grid)
{
    const int k = grid.dims.z() / 2;
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("failed to open PGM: " + path.u8string());
    out << "P5\n" << grid.dims.x() << ' ' << grid.dims.y() << "\n255\n";
    for (int j = grid.dims.y() - 1; j >= 0; --j) {
        for (int i = 0; i < grid.dims.x(); ++i) {
            const auto occ = grid.occupancy[static_cast<size_t>(grid.idx(i, j, k))];
            unsigned char value = 255;
            if (occ == cslc::VoxelizationV6::Occ::INSIDE) value = 160;
            if (occ == cslc::VoxelizationV6::Occ::BOUNDARY) value = 0;
            out.write(reinterpret_cast<const char*>(&value), 1);
        }
    }
}

void writeStatsJson(const std::filesystem::path& path, const cslc::VoxelizationV6::Grid& grid)
{
    const int n_boundary = countOcc(grid, cslc::VoxelizationV6::Occ::BOUNDARY);
    const int n_inside = countOcc(grid, cslc::VoxelizationV6::Occ::INSIDE);
    const int n_outside = countOcc(grid, cslc::VoxelizationV6::Occ::OUTSIDE);
    const Eigen::Vector3d max_corner =
        grid.origin_mm + grid.spacing_mm * grid.dims.cast<double>();

    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open stats JSON: " + path.u8string());
    out << "{\n";
    out << "  \"spacing_mm\": " << grid.spacing_mm << ",\n";
    out << "  \"dims\": [" << grid.dims.x() << ", " << grid.dims.y() << ", "
        << grid.dims.z() << "],\n";
    out << "  \"n_inside\": " << n_inside << ",\n";
    out << "  \"n_boundary\": " << n_boundary << ",\n";
    out << "  \"n_outside\": " << n_outside << ",\n";
    out << "  \"bbox\": {\n";
    out << "    \"min\": [" << grid.origin_mm.x() << ", " << grid.origin_mm.y()
        << ", " << grid.origin_mm.z() << "],\n";
    out << "    \"max\": [" << max_corner.x() << ", " << max_corner.y()
        << ", " << max_corner.z() << "]\n";
    out << "  }\n";
    out << "}\n";
}

void test_unit_sphere_spacing_1()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    constexpr double radius = 10.0;
    constexpr double spacing = 1.0;
    buildSphere(24, 48, radius, V, F);

    cslc::VoxelizationV6::Options opt;
    opt.spacing_mm = spacing;
    opt.boundary_conn = cslc::VoxelizationV6::BoundaryConn::CONN_6;
    const auto grid_c6 = cslc::VoxelizationV6::voxelize(V, F, opt);

    opt.boundary_conn = cslc::VoxelizationV6::BoundaryConn::CONN_18;
    const auto grid_c18 = cslc::VoxelizationV6::voxelize(V, F, opt);

    const double expected_volume =
        4.0 / 3.0 * M_PI * radius * radius * radius / (spacing * spacing * spacing);
    const double area_continuous = 4.0 * M_PI * radius * radius / (spacing * spacing);
    const int occupied = countOccupied(grid_c6);
    const int boundary_c6 = static_cast<int>(grid_c6.boundary_voxels.size());
    const int boundary_c18 = static_cast<int>(grid_c18.boundary_voxels.size());

    std::cout << "  sphere_1mm dims=" << grid_c6.dims.transpose()
              << " occupied=" << occupied
              << " boundary_c6=" << boundary_c6
              << " boundary_c18=" << boundary_c18
              << " area_continuous=" << area_continuous << "\n";

    require((grid_c6.dims.array() == 24).all(), "sphere 1mm: dims are 24^3");
    require((grid_c18.dims.array() == grid_c6.dims.array()).all(),
            "sphere 1mm: boundary connectivity does not affect dims");
    require(occupied == countOccupied(grid_c18),
            "sphere 1mm: boundary connectivity does not affect occupied count");
    require(std::abs(static_cast<double>(occupied) - expected_volume) / expected_volume < 0.05,
            "sphere 1mm: occupied voxel count within 5 percent of volume");
    require(static_cast<double>(boundary_c6) >= 0.70 * area_continuous,
            "sphere 1mm: CONN_6 boundary severe undercount");
    require(static_cast<double>(boundary_c6) <= 1.00 * area_continuous,
            "sphere 1mm: CONN_6 boundary exceeds continuous area");
    require(static_cast<double>(boundary_c18) >= 1.00 * area_continuous,
            "sphere 1mm: CONN_18 boundary under continuous area");
    require(static_cast<double>(boundary_c18) <= 1.30 * area_continuous,
            "sphere 1mm: CONN_18 boundary overcount too high");
    require(boundary_c18 > boundary_c6,
            "sphere 1mm: CONN_18 boundary must be greater than CONN_6");
}

void test_spacing_scaling()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(24, 48, 10.0, V, F);

    cslc::VoxelizationV6::Options opt;
    opt.spacing_mm = 0.5;
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, opt);

    const double expected_volume_voxels =
        4.0 / 3.0 * M_PI * 10.0 * 10.0 * 10.0 / (0.5 * 0.5 * 0.5);
    const int occupied = countOccupied(grid);

    std::cout << "  sphere_0p5mm dims=" << grid.dims.transpose()
              << " occupied=" << occupied << "\n";

    require((grid.dims.array() == 48).all(), "sphere 0.5mm: dims are 48^3");
    require(std::abs(static_cast<double>(occupied) - expected_volume_voxels) /
                expected_volume_voxels < 0.05,
            "sphere 0.5mm: occupied voxel count scales by spacing^3");
}

void test_memory_limit()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(8, 16, 50.0, V, F);

    cslc::VoxelizationV6::Options opt;
    opt.spacing_mm = 0.001;

    bool threw = false;
    try {
        (void)cslc::VoxelizationV6::voxelize(V, F, opt);
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()).find("200000000") != std::string::npos ||
            std::string(e.what()).find("too large") != std::string::npos;
    }
    require(threw, "memory limit: excessive voxel grid throws");
}

void test_boundary_adjacency()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(24, 48, 10.0, V, F);

    cslc::VoxelizationV6::Options opt;
    opt.spacing_mm = 1.0;
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, opt);

    for (const auto& ijk : grid.boundary_voxels) {
        require(grid.occupancy[static_cast<size_t>(grid.idx(ijk.x(), ijk.y(), ijk.z()))] ==
                    cslc::VoxelizationV6::Occ::BOUNDARY,
                "boundary adjacency: listed voxel is marked BOUNDARY");
        require(hasOutside6Neighbor(grid, ijk),
                "boundary adjacency: listed voxel has an OUTSIDE 6-neighbor");
    }
}

int runDemo(const std::filesystem::path& repaired_ply, const std::filesystem::path& out_dir)
{
    std::filesystem::create_directories(out_dir);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    readAsciiPly(repaired_ply, V, F);

    cslc::VoxelizationV6::Options opt;
    opt.spacing_mm = 1.0;
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, opt);

    writeBoundaryPly(out_dir / "bunny_voxel_boundary.ply", grid);
    writeSlicePgm(out_dir / "bunny_voxel_slice_z_mid.pgm", grid);
    writeStatsJson(out_dir / "bunny_voxel_stats.json", grid);

    std::cout << "demo_written=" << out_dir.u8string()
              << " dims=" << grid.dims.transpose()
              << " inside=" << countOcc(grid, cslc::VoxelizationV6::Occ::INSIDE)
              << " boundary=" << grid.boundary_voxels.size()
              << " outside=" << countOcc(grid, cslc::VoxelizationV6::Occ::OUTSIDE)
              << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        if (argc == 4 && std::string(argv[1]) == "--demo") {
            return runDemo(std::filesystem::path(argv[2]), std::filesystem::path(argv[3]));
        }

        test_unit_sphere_spacing_1();
        test_spacing_scaling();
        test_memory_limit();
        test_boundary_adjacency();
        std::cout << "test_voxelization_v6 PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_voxelization_v6 FAILED: " << e.what() << '\n';
        return 1;
    }
}
