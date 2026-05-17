#include "io/stl_reader.h"
#include "mesh/MeshRepair.h"

#include <Eigen/Dense>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

std::filesystem::path sourceRoot()
{
#ifdef CSLC_SOURCE_DIR
    return std::filesystem::path(CSLC_SOURCE_DIR);
#else
    return std::filesystem::current_path();
#endif
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
    for (int f = 0; f < F.rows(); ++f) {
        std::swap(F(f, 1), F(f, 2));
    }
}

int ringVertexIndex(int lat_steps, int lon_steps, int ring, int j)
{
    (void)lat_steps;
    return 1 + (ring - 1) * lon_steps + (j % lon_steps);
}

void buildSphere(int lat_steps, int lon_steps, double radius,
                 Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    require(lat_steps >= 4, "sphere: lat_steps >= 4");
    require(lon_steps >= 8, "sphere: lon_steps >= 8");

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
    for (int i = 0; i < F.rows(); ++i) {
        F.row(i) = faces[static_cast<size_t>(i)].transpose();
    }

    if (signedVolume(V, F) < 0.0) {
        flipAllFaces(F);
    }
}

Eigen::MatrixXi removeOneVertexStar(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    int target_vertex = 0;
    double best_x = -std::numeric_limits<double>::infinity();
    for (int v = 0; v < V.rows(); ++v) {
        if (std::abs(V(v, 2)) < 25.0 && V(v, 0) > best_x) {
            best_x = V(v, 0);
            target_vertex = v;
        }
    }

    std::vector<Eigen::Vector3i> kept;
    kept.reserve(static_cast<size_t>(F.rows()));
    for (int f = 0; f < F.rows(); ++f) {
        if (F(f, 0) == target_vertex || F(f, 1) == target_vertex ||
            F(f, 2) == target_vertex) {
            continue;
        }
        kept.emplace_back(F(f, 0), F(f, 1), F(f, 2));
    }

    Eigen::MatrixXi result(static_cast<int>(kept.size()), 3);
    for (int i = 0; i < result.rows(); ++i) {
        result.row(i) = kept[static_cast<size_t>(i)].transpose();
    }
    return result;
}

void meshFromStl(const cslc::TriangleMesh& mesh, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V.resize(static_cast<int>(mesh.triangles.size() * 3U), 3);
    F.resize(static_cast<int>(mesh.triangles.size()), 3);
    for (int t = 0; t < static_cast<int>(mesh.triangles.size()); ++t) {
        const auto& tri = mesh.triangles[static_cast<size_t>(t)];
        for (int c = 0; c < 3; ++c) {
            const auto& p = tri.vertices[static_cast<size_t>(c)];
            V.row(3 * t + c) = Eigen::Vector3d(p.x, p.y, p.z);
            F(t, c) = 3 * t + c;
        }
    }
}

void writePly(const std::filesystem::path& path, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open PLY: " + path.u8string());

    out << "ply\nformat ascii 1.0\n";
    out << "element vertex " << V.rows() << "\n";
    out << "property float x\nproperty float y\nproperty float z\n";
    out << "element face " << F.rows() << "\n";
    out << "property list uchar int vertex_indices\nend_header\n";
    for (int i = 0; i < V.rows(); ++i) {
        out << V(i, 0) << ' ' << V(i, 1) << ' ' << V(i, 2) << '\n';
    }
    for (int f = 0; f < F.rows(); ++f) {
        out << "3 " << F(f, 0) << ' ' << F(f, 1) << ' ' << F(f, 2) << '\n';
    }
}

const char* unitGuessName(cslc::MeshRepair::Report::UnitGuess unit)
{
    switch (unit) {
    case cslc::MeshRepair::Report::UnitGuess::MILLIMETERS:
        return "MILLIMETERS";
    case cslc::MeshRepair::Report::UnitGuess::METERS:
        return "METERS";
    case cslc::MeshRepair::Report::UnitGuess::AMBIGUOUS:
        return "AMBIGUOUS";
    }
    return "AMBIGUOUS";
}

void writeReportJson(const std::filesystem::path& path, const cslc::MeshRepair::Report& report)
{
    std::ofstream out(path);
    if (!out) throw std::runtime_error("failed to open report JSON: " + path.u8string());

    out << "{\n";
    out << "  \"n_vertices_in\": " << report.n_vertices_in << ",\n";
    out << "  \"n_triangles_in\": " << report.n_triangles_in << ",\n";
    out << "  \"bbox_diagonal_mm\": " << report.bbox_diagonal_mm << ",\n";
    out << "  \"is_manifold\": " << (report.is_manifold ? "true" : "false") << ",\n";
    out << "  \"n_non_manifold_edges\": " << report.n_non_manifold_edges << ",\n";
    out << "  \"is_orientable\": " << (report.is_orientable ? "true" : "false") << ",\n";
    out << "  \"n_boundary_edges\": " << report.n_boundary_edges << ",\n";
    out << "  \"n_degenerate_tris\": " << report.n_degenerate_tris << ",\n";
    out << "  \"n_duplicate_vertices\": " << report.n_duplicate_vertices << ",\n";
    out << "  \"unit_guess\": \"" << unitGuessName(report.unit_guess) << "\",\n";
    out << "  \"n_vertices_merged\": " << report.n_vertices_merged << ",\n";
    out << "  \"n_triangles_removed_degenerate\": " << report.n_triangles_removed_degenerate << ",\n";
    out << "  \"n_triangles_flipped\": " << report.n_triangles_flipped << ",\n";
    out << "  \"n_holes_filled\": " << report.n_holes_filled << ",\n";
    out << "  \"n_vertices_out\": " << report.n_vertices_out << ",\n";
    out << "  \"n_triangles_out\": " << report.n_triangles_out << ",\n";
    out << "  \"is_manifold_after\": " << (report.is_manifold_after ? "true" : "false") << ",\n";
    out << "  \"is_orientable_after\": " << (report.is_orientable_after ? "true" : "false") << ",\n";
    out << "  \"n_boundary_edges_after\": " << report.n_boundary_edges_after << "\n";
    out << "}\n";
}

void test_clean_sphere()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(16, 32, 100.0, V, F);

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    const auto report = cslc::MeshRepair::repair(V, F, {}, V_out, F_out);

    std::cout << "  clean_sphere V=" << report.n_vertices_out
              << " F=" << report.n_triangles_out
              << " boundary=" << report.n_boundary_edges_after << "\n";

    require(report.is_manifold, "clean sphere: input manifold");
    require(report.is_orientable, "clean sphere: input orientable");
    require(report.n_boundary_edges == 0, "clean sphere: input closed");
    require(report.n_vertices_out == V.rows(), "clean sphere: vertex count unchanged");
    require(report.n_triangles_out == F.rows(), "clean sphere: triangle count unchanged");
    require(report.n_boundary_edges_after == 0, "clean sphere: output closed");
    require(report.is_manifold_after, "clean sphere: output manifold");
    require(report.is_orientable_after, "clean sphere: output orientable");
}

void test_hole_fill()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(16, 32, 100.0, V, F);
    const Eigen::MatrixXi F_hole = removeOneVertexStar(V, F);

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    const auto report = cslc::MeshRepair::repair(V, F_hole, {}, V_out, F_out);

    std::cout << "  hole_fill boundary_in=" << report.n_boundary_edges
              << " holes_filled=" << report.n_holes_filled
              << " boundary_after=" << report.n_boundary_edges_after << "\n";

    require(report.n_boundary_edges > 0, "hole fill: input has boundary");
    require(report.n_holes_filled > 0, "hole fill: filled at least one hole");
    require(report.n_boundary_edges_after == 0, "hole fill: output closed");
    require(report.is_manifold_after, "hole fill: output manifold");
}

void test_reversed_normals()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(16, 32, 100.0, V, F);
    flipAllFaces(F);
    require(signedVolume(V, F) < 0.0, "reversed normals: input volume negative");

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    const auto report = cslc::MeshRepair::repair(V, F, {}, V_out, F_out);

    std::cout << "  reversed_normals flipped=" << report.n_triangles_flipped
              << " volume_after=" << signedVolume(V_out, F_out) << "\n";

    require(report.n_triangles_flipped > 0, "reversed normals: some triangles flipped");
    require(signedVolume(V_out, F_out) > 0.0, "reversed normals: output volume positive");
}

void test_unit_gate()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildSphere(12, 24, 0.05, V, F);

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    bool threw = false;
    try {
        (void)cslc::MeshRepair::repair(V, F, {}, V_out, F_out);
    } catch (const std::runtime_error& e) {
        threw = std::string(e.what()).find("likely meters unit") != std::string::npos;
    }
    require(threw, "unit gate: meter-scale mesh throws explicit unit error");
}

int runDemo(const std::filesystem::path& input_stl, const std::filesystem::path& out_dir)
{
    std::filesystem::create_directories(out_dir);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    meshFromStl(cslc::readStl(input_stl), V, F);
    writePly(out_dir / "bunny_before.ply", V, F);

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    const auto report = cslc::MeshRepair::repair(V, F, {}, V_out, F_out);
    writePly(out_dir / "bunny_after.ply", V_out, F_out);
    writeReportJson(out_dir / "bunny_repair_report.json", report);

    std::cout << "demo_written=" << out_dir.u8string()
              << " before_V=" << V.rows()
              << " after_V=" << V_out.rows()
              << " holes_filled=" << report.n_holes_filled
              << " boundary_after=" << report.n_boundary_edges_after << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        if (argc == 4 && std::string(argv[1]) == "--demo") {
            return runDemo(std::filesystem::path(argv[2]), std::filesystem::path(argv[3]));
        }

        test_clean_sphere();
        test_hole_fill();
        test_reversed_normals();
        test_unit_gate();
        std::cout << "test_mesh_repair PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_mesh_repair FAILED: " << e.what() << '\n';
        return 1;
    }
}
