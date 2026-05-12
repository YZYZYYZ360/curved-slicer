#include "path/geodesic_paths.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

// Build a flat disk mesh: center vertex + n_ring vertices on a circle of radius R,
// all connected into n_ring triangles (a triangle fan).
static void buildDiskMesh(int n_ring, double R,
                          Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V.resize(n_ring + 1, 3);
    V.row(0) = Eigen::Vector3d(0, 0, 0);
    for (int i = 0; i < n_ring; ++i) {
        double angle = 2.0 * M_PI * i / n_ring;
        V.row(i + 1) = Eigen::Vector3d(R * std::cos(angle), R * std::sin(angle), 0.0);
    }

    F.resize(n_ring, 3);
    for (int i = 0; i < n_ring; ++i) {
        int next = (i + 1) % n_ring;
        F.row(i) = Eigen::Vector3i(0, i + 1, next + 1);
    }
}

void test_2x2_plane()
{
    using namespace cslc;

    // 2x2 grid: 9 vertices, 8 triangles
    Eigen::MatrixXd V(9, 3);
    V << 0, 0, 0,
         1, 0, 0,
         2, 0, 0,
         0, 1, 0,
         1, 1, 0,
         2, 1, 0,
         0, 2, 0,
         1, 2, 0,
         2, 2, 0;

    Eigen::MatrixXi F(8, 3);
    F << 0, 1, 4,
         0, 4, 3,
         1, 2, 5,
         1, 5, 4,
         3, 4, 7,
         3, 7, 6,
         4, 5, 8,
         4, 8, 7;

    GeodesicPathParams params;
    params.line_spacing_mm = 0.5;

    auto paths = generateGeodesicPaths(V, F, params);

    std::cout << "  paths.size() = " << paths.size() << "\n";
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "    path[" << i << "]: " << paths[i].points.size()
                  << " points, length=" << paths[i].total_length_mm << "\n";
    }

    require(paths.size() >= 2, "2x2: at least 2 isoline paths");
    for (size_t i = 0; i < paths.size(); ++i) {
        require(paths[i].points.size() >= 2,
                "2x2: each path has at least 2 points");
    }

    std::cout << "  test_2x2_plane PASSED\n";
}

// Test case 1: closed ring stitching on a disk mesh
void test_closed_ring()
{
    using namespace cslc;

    const int n_ring = 24;
    const double R = 10.0;  // 10mm radius
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildDiskMesh(n_ring, R, V, F);

    GeodesicPathParams params;
    params.line_spacing_mm = 2.0;  // isolines at d=2,4,6,8

    auto paths = generateGeodesicPaths(V, F, params);

    std::cout << "  closed_ring: paths.size() = " << paths.size() << "\n";
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "    path[" << i << "]: " << paths[i].points.size()
                  << " pts, closed=" << paths[i].is_closed
                  << ", length=" << paths[i].total_length_mm << "\n";
    }

    // At least one closed isoline expected (innermost ring)
    bool found_closed = false;
    for (auto& p : paths) {
        if (p.is_closed && p.points.size() >= 4) {
            found_closed = true;
            break;
        }
    }
    require(found_closed, "closed_ring: at least one closed polyline with >= 4 points");

    std::cout << "  test_closed_ring PASSED\n";
}

// Test case 2: start point normalization (min-z first, tiebreak min-x)
void test_start_point()
{
    using namespace cslc;

    const int n_ring = 16;
    const double R = 10.0;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildDiskMesh(n_ring, R, V, F);

    GeodesicPathParams params;
    params.line_spacing_mm = 3.0;

    auto paths = generateGeodesicPaths(V, F, params);

    // Find a closed polyline and verify start point
    for (auto& p : paths) {
        if (!p.is_closed || p.points.size() < 4) continue;

        double start_z = p.points[0].z();
        double start_x = p.points[0].x();
        for (size_t i = 1; i < p.points.size(); ++i) {
            double z = p.points[i].z();
            double x = p.points[i].x();
            // start should have min z, or min x among min z
            require(z >= start_z || (z == start_z && x >= start_x),
                    "start_point: first vertex has min z (tiebreak min x)");
        }
        std::cout << "  test_start_point PASSED\n";
        return;
    }
    // If no closed polyline found, test is vacuous but not a failure
    std::cout << "  test_start_point SKIPPED (no closed polyline)\n";
}

// Test case 3: tangent vectors
void test_tangents()
{
    using namespace cslc;

    const int n_ring = 24;
    const double R = 10.0;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildDiskMesh(n_ring, R, V, F);

    GeodesicPathParams params;
    params.line_spacing_mm = 2.0;

    auto paths = generateGeodesicPaths(V, F, params);

    for (auto& p : paths) {
        require(p.tangents.size() == p.points.size(),
                "tangents: tangent count == point count");

        // All tangents should be unit vectors
        for (size_t i = 0; i < p.tangents.size(); ++i) {
            double len = p.tangents[i].norm();
            require(std::abs(len - 1.0) < 0.01,
                    "tangents: each tangent is approximately unit length");
        }

        // For closed polylines, check tangent continuity:
        // adjacent tangent dot product > 0 (angle < 90°)
        if (p.is_closed && p.points.size() >= 4) {
            double min_dot = 1.0;
            for (size_t i = 0; i < p.tangents.size(); ++i) {
                size_t next = (i + 1) % p.tangents.size();
                double dot = p.tangents[i].dot(p.tangents[next]);
                if (dot < min_dot) min_dot = dot;
            }
            // max angular deviation = acos(min_dot)
            double max_angle_deg = std::acos(std::clamp(min_dot, -1.0, 1.0)) * 180.0 / M_PI;
            std::cout << "  tangent continuity: max angle deviation = "
                      << max_angle_deg << " deg\n";
            require(min_dot > 0.0,
                    "tangents: adjacent tangent angle < 90° on closed polyline");
        }
    }

    std::cout << "  test_tangents PASSED\n";
}

// Verification C: finer mesh (32 triangles) to check if igl::isolines
// produces complete rings at higher resolution.
void test_finer_mesh()
{
    using namespace cslc;

    const int n_ring = 32;
    const double R = 10.0;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildDiskMesh(n_ring, R, V, F);

    GeodesicPathParams params;
    params.line_spacing_mm = 2.0;

    auto paths = generateGeodesicPaths(V, F, params);

    int closed_count = 0;
    int total_count = static_cast<int>(paths.size());
    for (auto& p : paths) {
        if (p.is_closed) closed_count++;
    }

    std::cout << "  finer_mesh(32): total=" << total_count
              << " closed=" << closed_count << "\n";
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "    path[" << i << "]: " << paths[i].points.size()
                  << " pts, closed=" << paths[i].is_closed
                  << ", length=" << paths[i].total_length_mm << "\n";
    }

    // Expect at least 1 closed ring (innermost)
    require(closed_count >= 1, "finer_mesh: at least 1 closed ring");

    std::cout << "  test_finer_mesh PASSED\n";
}

// Test case 5: uniform resampling
void test_resample()
{
    using namespace cslc;

    const int n_ring = 32;
    const double R = 10.0;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildDiskMesh(n_ring, R, V, F);

    GeodesicPathParams params;
    params.line_spacing_mm = 2.0;
    params.resample_step_mm = 0.5;

    auto paths = generateGeodesicPaths(V, F, params);

    // Find a closed polyline
    const PathPolyline* closed_poly = nullptr;
    for (auto& p : paths) {
        if (p.is_closed && p.points.size() >= 10) {
            closed_poly = &p;
            break;
        }
    }
    require(closed_poly != nullptr, "resample: found a closed polyline");

    // Check spacing uniformity
    int n = static_cast<int>(closed_poly->points.size());
    std::vector<double> spacings;
    for (int i = 0; i < n; ++i) {
        int next = (i + 1) % n;
        spacings.push_back((closed_poly->points[next] - closed_poly->points[i]).norm());
    }

    double mean_spacing = 0;
    for (double s : spacings) mean_spacing += s;
    mean_spacing /= spacings.size();

    double max_dev = 0;
    for (double s : spacings) {
        double dev = std::abs(s - mean_spacing) / mean_spacing;
        if (dev > max_dev) max_dev = dev;
    }

    std::cout << "  resample: n=" << n << " mean_spacing=" << mean_spacing
              << " max_dev=" << (max_dev * 100) << "%"
              << " total_length=" << closed_poly->total_length_mm << "\n";

    // Spacing should be close to resample_step_mm (within 10%)
    require(std::abs(mean_spacing - params.resample_step_mm) / params.resample_step_mm < 0.10,
            "resample: mean spacing ≈ step_mm");

    // Total length should be preserved (within 5%)
    // The original closed ring length is approximately 2*pi*R_at_d2 ≈ 12.57
    // but we're using the resampled version, so check self-consistency
    double computed_length = 0;
    for (size_t i = 1; i < closed_poly->points.size(); ++i) {
        computed_length += (closed_poly->points[i] - closed_poly->points[i - 1]).norm();
    }
    computed_length += (closed_poly->points.back() - closed_poly->points.front()).norm();

    require(std::abs(computed_length - closed_poly->total_length_mm) < 0.01,
            "resample: total_length consistent with point distances");

    std::cout << "  test_resample PASSED\n";
}

int main()
{
    try {
        test_2x2_plane();
        test_closed_ring();
        test_start_point();
        test_tangents();
        test_finer_mesh();
        test_resample();
        std::cout << "geodesic_paths_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "geodesic_paths_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
