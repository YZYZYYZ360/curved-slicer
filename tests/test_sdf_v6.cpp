#include "sdf/SDFV6.h"
#include "surface/iso_surface.h"
#include "voxel/VoxelizationV6.h"

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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

void buildCube(double side, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    const double h = side * 0.5;
    V.resize(8, 3);
    V << -h, -h, -h,
          h, -h, -h,
          h,  h, -h,
         -h,  h, -h,
         -h, -h,  h,
          h, -h,  h,
          h,  h,  h,
         -h,  h,  h;

    F.resize(12, 3);
    F << 0, 2, 1, 0, 3, 2,
         4, 5, 6, 4, 6, 7,
         0, 1, 5, 0, 5, 4,
         3, 6, 2, 3, 7, 6,
         0, 4, 7, 0, 7, 3,
         1, 2, 6, 1, 6, 5;
}

double sphereSdf(const Eigen::Vector3d& p, double radius)
{
    return p.norm() - radius;
}

double cubeSdf(const Eigen::Vector3d& p, double side)
{
    const double h = side * 0.5;
    const Eigen::Vector3d q = p.cwiseAbs() - Eigen::Vector3d::Constant(h);
    const Eigen::Vector3d outside = q.cwiseMax(Eigen::Vector3d::Zero());
    const double inside = std::min(std::max({q.x(), q.y(), q.z()}), 0.0);
    return outside.norm() + inside;
}

template <typename Fn>
void assertAnalyticError(const cslc::SDFV6::Field& field, Fn analytic, double spacing)
{
    double sum_abs = 0.0;
    double max_abs = 0.0;
    for (int k = 0; k < field.grid.dims.z(); ++k) {
        for (int j = 0; j < field.grid.dims.y(); ++j) {
            for (int i = 0; i < field.grid.dims.x(); ++i) {
                const int idx = field.grid.idx(i, j, k);
                const double expected = analytic(field.grid.center(i, j, k));
                const double err = std::abs(static_cast<double>(
                    field.distance_mm[static_cast<size_t>(idx)]) - expected);
                sum_abs += err;
                max_abs = std::max(max_abs, err);
            }
        }
    }
    const double mean_abs = sum_abs / static_cast<double>(field.distance_mm.size());
    std::cout << "  sdf_error mean_abs=" << mean_abs << " max_abs=" << max_abs << "\n";
    require(mean_abs < 0.5 * spacing, "SDF analytic mean absolute error too high");
    require(max_abs < spacing, "SDF analytic max absolute error too high");
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

cslc::ScalarField toScalarField(const cslc::SDFV6::Field& field)
{
    cslc::ScalarField phi;
    phi.spacing = field.grid.spacing_mm;
    phi.nx = field.grid.dims.x();
    phi.ny = field.grid.dims.y();
    phi.nz = field.grid.dims.z();
    phi.bbox.min = {field.grid.origin_mm.x(), field.grid.origin_mm.y(), field.grid.origin_mm.z()};
    const Eigen::Vector3d max_corner =
        field.grid.origin_mm + field.grid.spacing_mm * field.grid.dims.cast<double>();
    phi.bbox.max = {max_corner.x(), max_corner.y(), max_corner.z()};
    phi.values.resize(field.distance_mm.size());
    for (size_t i = 0; i < field.distance_mm.size(); ++i) {
        phi.values[i] = static_cast<double>(field.distance_mm[i]);
    }
    return phi;
}

void appendU32be(std::vector<unsigned char>& out, std::uint32_t value)
{
    out.push_back(static_cast<unsigned char>((value >> 24) & 0xffU));
    out.push_back(static_cast<unsigned char>((value >> 16) & 0xffU));
    out.push_back(static_cast<unsigned char>((value >> 8) & 0xffU));
    out.push_back(static_cast<unsigned char>(value & 0xffU));
}

std::uint32_t crc32(const unsigned char* data, size_t size)
{
    std::uint32_t crc = 0xffffffffU;
    for (size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint32_t>(data[i]);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
        }
    }
    return crc ^ 0xffffffffU;
}

std::uint32_t adler32(const std::vector<unsigned char>& data)
{
    constexpr std::uint32_t mod = 65521U;
    std::uint32_t a = 1U;
    std::uint32_t b = 0U;
    for (unsigned char byte : data) {
        a = (a + byte) % mod;
        b = (b + a) % mod;
    }
    return (b << 16) | a;
}

void appendChunk(
    std::vector<unsigned char>& png,
    const char type[4],
    const std::vector<unsigned char>& payload)
{
    appendU32be(png, static_cast<std::uint32_t>(payload.size()));
    const size_t type_offset = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), payload.begin(), payload.end());
    appendU32be(png, crc32(png.data() + type_offset, png.size() - type_offset));
}

std::vector<unsigned char> zlibStore(const std::vector<unsigned char>& raw)
{
    std::vector<unsigned char> z;
    z.reserve(raw.size() + raw.size() / 65535U * 5U + 8U);
    z.push_back(0x78U);
    z.push_back(0x01U);

    size_t offset = 0;
    while (offset < raw.size()) {
        const size_t count = std::min<size_t>(65535U, raw.size() - offset);
        const bool final = offset + count == raw.size();
        z.push_back(final ? 0x01U : 0x00U);
        const auto len = static_cast<std::uint16_t>(count);
        const auto nlen = static_cast<std::uint16_t>(~len);
        z.push_back(static_cast<unsigned char>(len & 0xffU));
        z.push_back(static_cast<unsigned char>((len >> 8) & 0xffU));
        z.push_back(static_cast<unsigned char>(nlen & 0xffU));
        z.push_back(static_cast<unsigned char>((nlen >> 8) & 0xffU));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(offset),
                 raw.begin() + static_cast<std::ptrdiff_t>(offset + count));
        offset += count;
    }

    appendU32be(z, adler32(raw));
    return z;
}

void writeGrayscalePng(
    const std::filesystem::path& path,
    int width,
    int height,
    const std::vector<unsigned char>& pixels)
{
    require(width > 0 && height > 0, "PNG dimensions must be positive");
    require(pixels.size() == static_cast<size_t>(width) * static_cast<size_t>(height),
            "PNG pixel buffer size mismatch");

    std::vector<unsigned char> raw;
    raw.reserve((static_cast<size_t>(width) + 1U) * static_cast<size_t>(height));
    for (int y = 0; y < height; ++y) {
        raw.push_back(0U);
        const size_t row = static_cast<size_t>(y) * static_cast<size_t>(width);
        raw.insert(raw.end(), pixels.begin() + static_cast<std::ptrdiff_t>(row),
                   pixels.begin() + static_cast<std::ptrdiff_t>(row + width));
    }

    std::vector<unsigned char> png;
    const unsigned char signature[8] = {0x89U, 'P', 'N', 'G', '\r', '\n', 0x1aU, '\n'};
    png.insert(png.end(), signature, signature + 8);

    std::vector<unsigned char> ihdr;
    appendU32be(ihdr, static_cast<std::uint32_t>(width));
    appendU32be(ihdr, static_cast<std::uint32_t>(height));
    ihdr.push_back(8U);
    ihdr.push_back(0U);
    ihdr.push_back(0U);
    ihdr.push_back(0U);
    ihdr.push_back(0U);
    appendChunk(png, "IHDR", ihdr);
    appendChunk(png, "IDAT", zlibStore(raw));
    appendChunk(png, "IEND", {});

    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("failed to open SDF PNG: " + path.u8string());
    out.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
}

void writeSlicePng(const std::filesystem::path& path, const cslc::SDFV6::Field& field)
{
    const int k = field.grid.dims.z() / 2;
    std::vector<unsigned char> pixels(
        static_cast<size_t>(field.grid.dims.x()) * static_cast<size_t>(field.grid.dims.y()));
    size_t pixel = 0;
    for (int j = field.grid.dims.y() - 1; j >= 0; --j) {
        for (int i = 0; i < field.grid.dims.x(); ++i) {
            const float d = field.distance_mm[static_cast<size_t>(field.grid.idx(i, j, k))];
            const double clamped = std::max(-10.0, std::min(10.0, static_cast<double>(d)));
            pixels[pixel++] = static_cast<unsigned char>(
                std::round((clamped + 10.0) / 20.0 * 255.0));
        }
    }
    writeGrayscalePng(path, field.grid.dims.x(), field.grid.dims.y(), pixels);
}

struct DemoTimings {
    double voxelize_s = 0.0;
    double signed_distance_s = 0.0;
    double marching_cubes_s = 0.0;
    double total_s = 0.0;
};

void writeStatsJson(
    const std::filesystem::path& path,
    const cslc::SDFV6::Field& field,
    const DemoTimings& timings,
    double narrow_band_mm)
{
    double min_d = std::numeric_limits<double>::infinity();
    double max_d = -std::numeric_limits<double>::infinity();
    double sum_abs = 0.0;
    for (float d : field.distance_mm) {
        min_d = std::min(min_d, static_cast<double>(d));
        max_d = std::max(max_d, static_cast<double>(d));
        sum_abs += std::abs(static_cast<double>(d));
    }

    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open SDF stats JSON: " + path.u8string());
    out << "{\n";
    out << "  \"method\": \"LIBIGL_SIGNED_DISTANCE\",\n";
    out << "  \"spacing_mm\": " << field.grid.spacing_mm << ",\n";
    out << "  \"n_voxel\": " << field.distance_mm.size() << ",\n";
    out << "  \"min_d\": " << min_d << ",\n";
    out << "  \"max_d\": " << max_d << ",\n";
    out << "  \"mean_abs_d\": " << sum_abs / static_cast<double>(field.distance_mm.size()) << ",\n";
    out << "  \"elapsed_s\": " << timings.total_s << ",\n";
    out << "  \"narrow_band_mm\": " << narrow_band_mm << ",\n";
    out << "  \"elapsed_breakdown\": {\n";
    out << "    \"voxelize_s\": " << timings.voxelize_s << ",\n";
    out << "    \"signed_distance_s\": " << timings.signed_distance_s << ",\n";
    out << "    \"marching_cubes_s\": " << timings.marching_cubes_s << ",\n";
    out << "    \"total_s\": " << timings.total_s << "\n";
    out << "  }\n";
    out << "}\n";
}

void test_sphere_sdf()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    constexpr double radius = 10.0;
    constexpr double spacing = 1.0;
    buildSphere(40, 80, radius, V, F);

    cslc::VoxelizationV6::Options voxel_opt;
    voxel_opt.spacing_mm = spacing;
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, voxel_opt);
    const auto field = cslc::SDFV6::compute(V, F, grid, {});

    assertAnalyticError(field, [](const Eigen::Vector3d& p) { return sphereSdf(p, radius); }, spacing);
    require(field.distance_mm[static_cast<size_t>(field.grid.idx(
                field.grid.dims.x() / 2, field.grid.dims.y() / 2, field.grid.dims.z() / 2))] < 0.0f,
            "sphere center voxel must be negative");
    require(field.distance_mm[0] > 0.0f, "far corner voxel must be positive");
}

void test_cube_sdf()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    constexpr double side = 10.0;
    constexpr double spacing = 1.0;
    buildCube(side, V, F);

    cslc::VoxelizationV6::Options voxel_opt;
    voxel_opt.spacing_mm = spacing;
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, voxel_opt);
    const auto field = cslc::SDFV6::compute(V, F, grid, {});

    assertAnalyticError(field, [](const Eigen::Vector3d& p) { return cubeSdf(p, side); }, spacing);
}

void test_sign_matches_occupancy()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(32, 64, 10.0, V, F);

    cslc::VoxelizationV6::Options voxel_opt;
    voxel_opt.spacing_mm = 1.0;
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, voxel_opt);
    const auto field = cslc::SDFV6::compute(V, F, grid, {});

    for (size_t idx = 0; idx < field.distance_mm.size(); ++idx) {
        const auto occ = field.grid.occupancy[idx];
        const float d = field.distance_mm[idx];
        if (occ == cslc::VoxelizationV6::Occ::INSIDE) {
            require(d < 0.0f, "INSIDE voxel must have negative SDF");
        } else if (occ == cslc::VoxelizationV6::Occ::OUTSIDE) {
            require(d > 0.0f, "OUTSIDE voxel must have positive SDF");
        } else {
            require(std::abs(d) < static_cast<float>(field.grid.spacing_mm),
                    "BOUNDARY voxel should be within one spacing of surface");
        }
    }
}

void test_bvh_fallback_and_narrow_band()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildCube(10.0, V, F);
    cslc::VoxelizationV6::Options voxel_opt;
    voxel_opt.spacing_mm = 1.0;
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, voxel_opt);

    cslc::SDFV6::Options opt;
    opt.method = cslc::SDFV6::Options::Method::BVH_CLOSEST_PLUS_OCCUPANCY;
    const auto field = cslc::SDFV6::compute(V, F, grid, opt);
    require(field.distance_mm.size() == grid.occupancy.size(), "BVH fallback output size");

    opt.narrow_band_mm = 2.0;
    bool threw = false;
    try {
        (void)cslc::SDFV6::compute(V, F, grid, opt);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    require(threw, "narrow_band_mm > 0 must throw until W4-W5");
}

int runDemo(const std::filesystem::path& input_ply, const std::filesystem::path& out_dir)
{
    std::filesystem::create_directories(out_dir);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    readAsciiPly(input_ply, V, F);

    const auto total_start = std::chrono::high_resolution_clock::now();
    DemoTimings timings;

    cslc::VoxelizationV6::Options voxel_opt;
    voxel_opt.spacing_mm = 0.3;
    const auto voxel_start = std::chrono::high_resolution_clock::now();
    const auto grid = cslc::VoxelizationV6::voxelize(V, F, voxel_opt);
    const auto voxel_end = std::chrono::high_resolution_clock::now();
    timings.voxelize_s = std::chrono::duration<double>(voxel_end - voxel_start).count();

    cslc::SDFV6::Options sdf_opt;
    const auto sdf_start = std::chrono::high_resolution_clock::now();
    const auto field = cslc::SDFV6::compute(V, F, grid, sdf_opt);
    const auto sdf_end = std::chrono::high_resolution_clock::now();
    timings.signed_distance_s = std::chrono::duration<double>(sdf_end - sdf_start).count();

    writeSlicePng(out_dir / "bunny_sdf_slice_z_mid.png", field);
    const auto mc_start = std::chrono::high_resolution_clock::now();
    const cslc::IsoMesh iso = cslc::extractIsoSurface(toScalarField(field), 0.0, 0);
    const auto mc_end = std::chrono::high_resolution_clock::now();
    timings.marching_cubes_s = std::chrono::duration<double>(mc_end - mc_start).count();
    timings.total_s = std::chrono::duration<double>(mc_end - total_start).count();
    writeStatsJson(out_dir / "bunny_sdf_stats.json", field, timings, sdf_opt.narrow_band_mm);
    cslc::writeIsoMeshPly(iso, out_dir / "bunny_sdf_isosurface_d0.ply");

    std::cout << "demo_sdf elapsed_s=" << timings.total_s
              << " voxelize_s=" << timings.voxelize_s
              << " signed_distance_s=" << timings.signed_distance_s
              << " marching_cubes_s=" << timings.marching_cubes_s
              << " n_voxel=" << field.distance_mm.size()
              << " iso_vertices=" << iso.vertices.size()
              << " iso_triangles=" << iso.triangles.size()
              << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc == 4 && std::string(argv[1]) == "--demo") {
        return runDemo(argv[2], argv[3]);
    }

    test_sphere_sdf();
    test_cube_sdf();
    test_sign_matches_occupancy();
    test_bvh_fallback_and_narrow_band();

    std::cout << "test_sdf_v6 passed\n";
    return 0;
}
