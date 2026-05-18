#include "path/WeightedDijkstraV6.h"
#include "sdf/SDFV6.h"
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

bool inBounds(const cslc::VoxelizationV6::Grid& grid, int i, int j, int k)
{
    return i >= 0 && i < grid.dims.x() &&
        j >= 0 && j < grid.dims.y() &&
        k >= 0 && k < grid.dims.z();
}

bool isWalkable(const cslc::VoxelizationV6::Grid& grid, int flat, bool walk_boundary = true)
{
    const auto occ = grid.occupancy[static_cast<size_t>(flat)];
    return occ == cslc::VoxelizationV6::Occ::INSIDE ||
        (walk_boundary && occ == cslc::VoxelizationV6::Occ::BOUNDARY);
}

Eigen::Vector3i flatToIJK(const cslc::VoxelizationV6::Grid& grid, int flat)
{
    const int nx = grid.dims.x();
    const int ny = grid.dims.y();
    const int i = flat % nx;
    const int yz = flat / nx;
    const int j = yz % ny;
    const int k = yz / ny;
    return {i, j, k};
}

cslc::SDFV6::Field buildAnalyticSphereField(double radius, double spacing)
{
    cslc::SDFV6::Field field;
    field.grid.origin_mm = Eigen::Vector3d::Constant(-radius - 2.0);
    field.grid.spacing_mm = spacing;
    const int dim = static_cast<int>(std::ceil((2.0 * radius + 4.0) / spacing));
    field.grid.dims = Eigen::Vector3i(dim, dim, dim);
    const int total = dim * dim * dim;
    field.grid.occupancy.assign(static_cast<size_t>(total), cslc::VoxelizationV6::Occ::OUTSIDE);
    field.distance_mm.assign(static_cast<size_t>(total), 0.0f);

    for (int k = 0; k < dim; ++k) {
        for (int j = 0; j < dim; ++j) {
            for (int i = 0; i < dim; ++i) {
                const int flat = field.grid.idx(i, j, k);
                const double d = field.grid.center(i, j, k).norm() - radius;
                field.distance_mm[static_cast<size_t>(flat)] = static_cast<float>(d);
                if (d < 0.0) {
                    field.grid.occupancy[static_cast<size_t>(flat)] =
                        cslc::VoxelizationV6::Occ::INSIDE;
                }
            }
        }
    }

    std::vector<int> boundary;
    for (int k = 0; k < dim; ++k) {
        for (int j = 0; j < dim; ++j) {
            for (int i = 0; i < dim; ++i) {
                const int flat = field.grid.idx(i, j, k);
                if (field.grid.occupancy[static_cast<size_t>(flat)] !=
                    cslc::VoxelizationV6::Occ::INSIDE) {
                    continue;
                }
                bool touches_outside = false;
                const int offsets[6][3] = {
                    {-1, 0, 0}, {1, 0, 0}, {0, -1, 0},
                    {0, 1, 0}, {0, 0, -1}, {0, 0, 1},
                };
                for (const auto& o : offsets) {
                    const int ni = i + o[0];
                    const int nj = j + o[1];
                    const int nk = k + o[2];
                    if (!inBounds(field.grid, ni, nj, nk) ||
                        field.grid.occupancy[static_cast<size_t>(field.grid.idx(ni, nj, nk))] ==
                            cslc::VoxelizationV6::Occ::OUTSIDE) {
                        touches_outside = true;
                        break;
                    }
                }
                if (touches_outside) boundary.push_back(flat);
            }
        }
    }

    field.grid.boundary_voxels.reserve(boundary.size());
    for (int flat : boundary) {
        field.grid.occupancy[static_cast<size_t>(flat)] = cslc::VoxelizationV6::Occ::BOUNDARY;
        field.grid.boundary_voxels.push_back(flatToIJK(field.grid, flat));
    }
    return field;
}

Eigen::Vector3i nearestWalkable(
    const cslc::VoxelizationV6::Grid& grid,
    const Eigen::Vector3d& target,
    bool inside_only = false)
{
    double best = std::numeric_limits<double>::infinity();
    Eigen::Vector3i best_ijk(-1, -1, -1);
    for (int flat = 0; flat < static_cast<int>(grid.occupancy.size()); ++flat) {
        const auto occ = grid.occupancy[static_cast<size_t>(flat)];
        if (inside_only) {
            if (occ != cslc::VoxelizationV6::Occ::INSIDE) continue;
        } else if (!isWalkable(grid, flat)) {
            continue;
        }
        const Eigen::Vector3i ijk = flatToIJK(grid, flat);
        const double d2 = (grid.center(ijk.x(), ijk.y(), ijk.z()) - target).squaredNorm();
        if (d2 < best) {
            best = d2;
            best_ijk = ijk;
        }
    }
    require(best_ijk.x() >= 0, "nearest walkable voxel not found");
    return best_ijk;
}

double pathLengthMm(
    const cslc::VoxelizationV6::Grid& grid,
    const std::vector<Eigen::Vector3i>& path)
{
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        const Eigen::Vector3d a = grid.center(path[i - 1].x(), path[i - 1].y(), path[i - 1].z());
        const Eigen::Vector3d b = grid.center(path[i].x(), path[i].y(), path[i].z());
        len += (a - b).norm();
    }
    return len;
}

double averageDepth(
    const cslc::SDFV6::Field& sdf,
    const std::vector<Eigen::Vector3i>& path)
{
    double sum = 0.0;
    for (const auto& ijk : path) {
        const float d = sdf.distance_mm[static_cast<size_t>(sdf.grid.idx(ijk.x(), ijk.y(), ijk.z()))];
        sum += std::max(0.0, -static_cast<double>(d));
    }
    return path.empty() ? 0.0 : sum / static_cast<double>(path.size());
}

int countWalkableVisited(const cslc::WeightedDijkstraV6::DistanceField& field)
{
    int count = 0;
    for (int flat = 0; flat < static_cast<int>(field.grid.occupancy.size()); ++flat) {
        if (isWalkable(field.grid, flat) &&
            std::isfinite(field.distance[static_cast<size_t>(flat)])) {
            ++count;
        }
    }
    return count;
}

int countWalkable(const cslc::VoxelizationV6::Grid& grid)
{
    int count = 0;
    for (int flat = 0; flat < static_cast<int>(grid.occupancy.size()); ++flat) {
        if (isWalkable(grid, flat)) ++count;
    }
    return count;
}

void test_sphere_antipodal_path()
{
    const auto sdf = buildAnalyticSphereField(10.0, 1.0);
    const Eigen::Vector3i source = nearestWalkable(sdf.grid, Eigen::Vector3d::Zero(), true);
    const Eigen::Vector3i sink = nearestWalkable(sdf.grid, Eigen::Vector3d(9.5, -0.5, -0.5));

    cslc::WeightedDijkstraV6::Options opt;
    opt.penalty.enable_clearance = false;
    const auto field = cslc::WeightedDijkstraV6::computeDistanceField(
        sdf.grid, sdf, {source}, opt);
    const int sink_flat = sdf.grid.idx(sink.x(), sink.y(), sink.z());
    const float dist = field.distance[static_cast<size_t>(sink_flat)];
    const auto path = cslc::WeightedDijkstraV6::tracePath(field, sink);

    const double straight = (sdf.grid.center(source.x(), source.y(), source.z()) -
                             sdf.grid.center(sink.x(), sink.y(), sink.z())).norm();
    const double path_len = pathLengthMm(sdf.grid, path);
    std::cout << "  dijkstra_sphere dist=" << dist
              << " straight=" << straight
              << " path_len=" << path_len
              << " n_path=" << path.size() << "\n";

    require(std::abs(static_cast<double>(dist) - straight) < 0.2 * straight,
            "sphere path cumulative distance outside 20% tolerance");
    require(path_len < 1.3 * straight, "sphere traced path makes a major detour");
}

void test_multi_source_distance_field()
{
    const auto sdf = buildAnalyticSphereField(10.0, 1.0);
    const Eigen::Vector3i s0 = nearestWalkable(sdf.grid, Eigen::Vector3d(-9.5, -0.5, -0.5));
    const Eigen::Vector3i s1 = nearestWalkable(sdf.grid, Eigen::Vector3d(9.5, -0.5, -0.5));
    const Eigen::Vector3i center = nearestWalkable(sdf.grid, Eigen::Vector3d::Zero(), true);

    cslc::WeightedDijkstraV6::Options opt;
    opt.penalty.enable_clearance = false;
    const auto field = cslc::WeightedDijkstraV6::computeDistanceField(
        sdf.grid, sdf, {s0, s1}, opt);

    require(countWalkableVisited(field) == countWalkable(sdf.grid),
            "multi-source Dijkstra must visit all connected walkable sphere voxels");

    const int center_flat = sdf.grid.idx(center.x(), center.y(), center.z());
    const float center_dist = field.distance[static_cast<size_t>(center_flat)];
    require(std::abs(static_cast<double>(center_dist) - 9.0) < 2.0,
            "sphere center should be roughly radius distance from antipodal sources");

    int by_source[2] = {0, 0};
    for (int flat = 0; flat < static_cast<int>(field.grid.occupancy.size()); ++flat) {
        if (!isWalkable(field.grid, flat)) continue;
        const auto sid = field.source_id[static_cast<size_t>(flat)];
        if (sid < 2) ++by_source[sid];
    }
    const int total = by_source[0] + by_source[1];
    const double ratio = static_cast<double>(by_source[0]) / static_cast<double>(total);
    std::cout << "  dijkstra_multisource center_dist=" << center_dist
              << " source0=" << by_source[0]
              << " source1=" << by_source[1]
              << " ratio0=" << ratio << "\n";
    require(ratio > 0.35 && ratio < 0.65, "multi-source source_id split is not geometric");
}

void test_clearance_penalty_moves_path_inward()
{
    const auto sdf = buildAnalyticSphereField(10.0, 1.0);
    const Eigen::Vector3i source = nearestWalkable(sdf.grid, Eigen::Vector3d(8.5, -4.5, -0.5));
    const Eigen::Vector3i sink = nearestWalkable(sdf.grid, Eigen::Vector3d(8.5, 4.5, -0.5));

    cslc::WeightedDijkstraV6::Options low;
    low.penalty.clearance_penalty_weight = 0.0;
    const auto low_field = cslc::WeightedDijkstraV6::computeDistanceField(
        sdf.grid, sdf, {source}, low);
    const auto low_path = cslc::WeightedDijkstraV6::tracePath(low_field, sink);

    cslc::WeightedDijkstraV6::Options high;
    high.penalty.clearance_threshold_mm = 2.0;
    high.penalty.clearance_penalty_weight = 100.0;
    const auto high_field = cslc::WeightedDijkstraV6::computeDistanceField(
        sdf.grid, sdf, {source}, high);
    const auto high_path = cslc::WeightedDijkstraV6::tracePath(high_field, sink);

    const double low_depth = averageDepth(sdf, low_path);
    const double high_depth = averageDepth(sdf, high_path);
    std::cout << "  dijkstra_clearance low_avg_depth=" << low_depth
              << " high_avg_depth=" << high_depth
              << " low_path=" << low_path.size()
              << " high_path=" << high_path.size() << "\n";
    require(high_depth > low_depth + 0.1,
            "high clearance penalty should move path farther from the surface");
}

void test_unreachable_outside_sink()
{
    auto sdf = buildAnalyticSphereField(10.0, 1.0);
    const Eigen::Vector3i source = nearestWalkable(sdf.grid, Eigen::Vector3d::Zero(), true);
    const Eigen::Vector3i sink = nearestWalkable(sdf.grid, Eigen::Vector3d(6.5, 0.5, 0.5), true);
    const int sink_flat = sdf.grid.idx(sink.x(), sink.y(), sink.z());
    sdf.grid.occupancy[static_cast<size_t>(sink_flat)] = cslc::VoxelizationV6::Occ::OUTSIDE;

    cslc::WeightedDijkstraV6::Options opt;
    const auto field = cslc::WeightedDijkstraV6::computeDistanceField(
        sdf.grid, sdf, {source}, opt);
    require(field.distance[static_cast<size_t>(sink_flat)] == cslc::WeightedDijkstraV6::UNREACHABLE,
            "OUTSIDE sink must be unreachable");
    require(field.predecessor[static_cast<size_t>(sink_flat)] == cslc::WeightedDijkstraV6::NO_PRED,
            "OUTSIDE sink predecessor must stay NO_PRED");
    require(cslc::WeightedDijkstraV6::tracePath(field, sink).empty(),
            "OUTSIDE sink trace must be empty");
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
    if (!out) throw std::runtime_error("failed to open PNG: " + path.u8string());
    out.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
}

void writeDistanceSlicePng(
    const std::filesystem::path& path,
    const cslc::WeightedDijkstraV6::DistanceField& field)
{
    const int k = field.grid.dims.z() / 2;
    double max_finite = 0.0;
    for (int j = 0; j < field.grid.dims.y(); ++j) {
        for (int i = 0; i < field.grid.dims.x(); ++i) {
            const int flat = field.grid.idx(i, j, k);
            const float d = field.distance[static_cast<size_t>(flat)];
            if (std::isfinite(d)) max_finite = std::max(max_finite, static_cast<double>(d));
        }
    }
    if (max_finite <= 0.0) max_finite = 1.0;

    std::vector<unsigned char> pixels(
        static_cast<size_t>(field.grid.dims.x()) * static_cast<size_t>(field.grid.dims.y()));
    size_t pixel = 0;
    for (int j = field.grid.dims.y() - 1; j >= 0; --j) {
        for (int i = 0; i < field.grid.dims.x(); ++i) {
            const int flat = field.grid.idx(i, j, k);
            const float d = field.distance[static_cast<size_t>(flat)];
            unsigned char value = 255;
            if (std::isfinite(d)) {
                const double normalized = std::min(1.0, static_cast<double>(d) / max_finite);
                value = static_cast<unsigned char>(std::round(normalized * 230.0));
            }
            pixels[pixel++] = value;
        }
    }
    writeGrayscalePng(path, field.grid.dims.x(), field.grid.dims.y(), pixels);
}

std::vector<Eigen::Vector3i> sortedInsideVoxelsByZ(
    const cslc::VoxelizationV6::Grid& grid,
    bool ascending)
{
    std::vector<Eigen::Vector3i> voxels;
    for (int k = 0; k < grid.dims.z(); ++k) {
        for (int j = 0; j < grid.dims.y(); ++j) {
            for (int i = 0; i < grid.dims.x(); ++i) {
                if (grid.occupancy[static_cast<size_t>(grid.idx(i, j, k))] ==
                    cslc::VoxelizationV6::Occ::INSIDE) {
                    voxels.emplace_back(i, j, k);
                }
            }
        }
    }
    std::sort(voxels.begin(), voxels.end(), [ascending](const auto& a, const auto& b) {
        if (a.z() != b.z()) return ascending ? a.z() < b.z() : a.z() > b.z();
        if (a.y() != b.y()) return a.y() < b.y();
        return a.x() < b.x();
    });
    return voxels;
}

void writePathPly(
    const std::filesystem::path& path,
    const cslc::VoxelizationV6::Grid& grid,
    const std::vector<Eigen::Vector3i>& path_ijk)
{
    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open path PLY: " + path.u8string());
    out << "ply\nformat ascii 1.0\n";
    out << "element vertex " << path_ijk.size() << "\n";
    out << "property float x\nproperty float y\nproperty float z\n";
    out << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    out << "element edge " << (path_ijk.empty() ? 0U : path_ijk.size() - 1U) << "\n";
    out << "property int vertex1\nproperty int vertex2\nend_header\n";
    for (const auto& ijk : path_ijk) {
        const Eigen::Vector3d p = grid.center(ijk.x(), ijk.y(), ijk.z());
        out << p.x() << ' ' << p.y() << ' ' << p.z() << " 255 32 32\n";
    }
    for (size_t i = 1; i < path_ijk.size(); ++i) {
        out << (i - 1U) << ' ' << i << '\n';
    }
}

struct DemoTimings {
    double voxelize_s = 0.0;
    double sdf_s = 0.0;
    double dijkstra_s = 0.0;
    double trace_s = 0.0;
    double total_s = 0.0;
};

void writeDemoStatsJson(
    const std::filesystem::path& path,
    const cslc::WeightedDijkstraV6::DistanceField& field,
    const std::vector<Eigen::Vector3i>& path_ijk,
    int n_sources,
    const DemoTimings& timings)
{
    int n_visited = 0;
    int n_unreachable = 0;
    double max_distance = 0.0;
    for (int flat = 0; flat < static_cast<int>(field.grid.occupancy.size()); ++flat) {
        if (!isWalkable(field.grid, flat)) continue;
        const float d = field.distance[static_cast<size_t>(flat)];
        if (std::isfinite(d)) {
            ++n_visited;
            max_distance = std::max(max_distance, static_cast<double>(d));
        } else {
            ++n_unreachable;
        }
    }

    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open Dijkstra stats JSON: " + path.u8string());
    out << "{\n";
    out << "  \"n_sources\": " << n_sources << ",\n";
    out << "  \"n_visited\": " << n_visited << ",\n";
    out << "  \"n_unreachable\": " << n_unreachable << ",\n";
    out << "  \"max_distance\": " << max_distance << ",\n";
    out << "  \"path_length_mm\": " << pathLengthMm(field.grid, path_ijk) << ",\n";
    out << "  \"path_n_voxel\": " << path_ijk.size() << ",\n";
    out << "  \"elapsed_breakdown\": {\n";
    out << "    \"voxelize_s\": " << timings.voxelize_s << ",\n";
    out << "    \"sdf_s\": " << timings.sdf_s << ",\n";
    out << "    \"dijkstra_s\": " << timings.dijkstra_s << ",\n";
    out << "    \"trace_s\": " << timings.trace_s << ",\n";
    out << "    \"total_s\": " << timings.total_s << "\n";
    out << "  }\n";
    out << "}\n";
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

    const auto sdf_start = std::chrono::high_resolution_clock::now();
    const auto sdf = cslc::SDFV6::compute(V, F, grid, {});
    const auto sdf_end = std::chrono::high_resolution_clock::now();
    timings.sdf_s = std::chrono::duration<double>(sdf_end - sdf_start).count();

    const std::vector<Eigen::Vector3i> bottom = sortedInsideVoxelsByZ(grid, true);
    const std::vector<Eigen::Vector3i> top = sortedInsideVoxelsByZ(grid, false);
    require(bottom.size() >= 5 && !top.empty(), "bunny demo needs at least 5 bottom INSIDE voxels");
    std::vector<Eigen::Vector3i> sources(bottom.begin(), bottom.begin() + 5);
    const Eigen::Vector3i sink = top.front();

    const auto dijkstra_start = std::chrono::high_resolution_clock::now();
    const auto field = cslc::WeightedDijkstraV6::computeDistanceField(grid, sdf, sources, {});
    const auto dijkstra_end = std::chrono::high_resolution_clock::now();
    timings.dijkstra_s = std::chrono::duration<double>(dijkstra_end - dijkstra_start).count();

    const auto trace_start = std::chrono::high_resolution_clock::now();
    const auto path = cslc::WeightedDijkstraV6::tracePath(field, sink);
    const auto trace_end = std::chrono::high_resolution_clock::now();
    timings.trace_s = std::chrono::duration<double>(trace_end - trace_start).count();
    timings.total_s = std::chrono::duration<double>(trace_end - total_start).count();
    require(!path.empty(), "bunny Dijkstra path is empty");

    writePathPly(out_dir / "bunny_path.ply", grid, path);
    writeDistanceSlicePng(out_dir / "bunny_distance_slice_z_mid.png", field);
    writeDemoStatsJson(out_dir / "bunny_dijkstra_stats.json", field, path,
                       static_cast<int>(sources.size()), timings);

    std::cout << "demo_dijkstra elapsed_s=" << timings.total_s
              << " voxelize_s=" << timings.voxelize_s
              << " sdf_s=" << timings.sdf_s
              << " dijkstra_s=" << timings.dijkstra_s
              << " trace_s=" << timings.trace_s
              << " path_voxels=" << path.size()
              << " path_length_mm=" << pathLengthMm(grid, path)
              << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc == 4 && std::string(argv[1]) == "--demo") {
        return runDemo(argv[2], argv[3]);
    }

    test_sphere_antipodal_path();
    test_multi_source_distance_field();
    test_clearance_penalty_moves_path_inward();
    test_unreachable_outside_sink();

    std::cout << "test_weighted_dijkstra_v6 passed\n";
    return 0;
}
