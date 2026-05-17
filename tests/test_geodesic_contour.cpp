#include "path/GeodesicContourPath.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

static void buildDiskFan(int n_segments, double radius,
                         Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    V.resize(n_segments + 1, 3);
    V.row(0) = Eigen::Vector3d(0.0, 0.0, 0.0);

    for (int i = 0; i < n_segments; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(n_segments);
        V.row(i + 1) =
            Eigen::Vector3d(radius * std::cos(theta), radius * std::sin(theta), 0.0);
    }

    F.resize(n_segments, 3);
    for (int i = 0; i < n_segments; ++i) {
        const int next = (i + 1) % n_segments;
        F.row(i) = Eigen::Vector3i(0, i + 1, next + 1);
    }
}

static Eigen::VectorXd radialField(const Eigen::MatrixXd& V)
{
    Eigen::VectorXd field(V.rows());
    for (int i = 0; i < V.rows(); ++i) {
        const double x = V(i, 0);
        const double y = V(i, 1);
        field(i) = std::sqrt(x * x + y * y);
    }
    return field;
}

static double meanFieldValue(const cslc::GeodesicContourPath::Polyline& poly)
{
    double sum = 0.0;
    for (const auto& cp : poly) sum += cp.field_value;
    return sum / static_cast<double>(poly.size());
}

static double polylineLength(const cslc::GeodesicContourPath::Polyline& poly)
{
    double length = 0.0;
    for (size_t i = 1; i < poly.size(); ++i) {
        length += (poly[i].pos - poly[i - 1].pos).norm();
    }
    return length;
}

static int nearestIsoIndex(double value, const std::vector<double>& iso_values)
{
    int best = 0;
    double best_err = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(iso_values.size()); ++i) {
        const double err = std::abs(value - iso_values[static_cast<size_t>(i)]);
        if (err < best_err) {
            best_err = err;
            best = i;
        }
    }
    return best;
}

static void test_extract_radial_disk_contours()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildDiskFan(2048, 1.0, V, F);

    const Eigen::VectorXd field = radialField(V);
    const std::vector<double> iso_values = {0.3, 0.5, 0.7};

    cslc::GeodesicContourPath contour_path(V, F);
    const auto contours = contour_path.extractIsoContours(field, iso_values);

    std::cout << "  extracted_contours=" << contours.size() << "\n";
    require(contours.size() == iso_values.size(),
            "radial_disk: one closed contour per iso value");

    std::vector<bool> seen(iso_values.size(), false);
    for (const auto& poly : contours) {
        require(poly.size() > 32, "radial_disk: contour has enough samples");
        require((poly.front().pos - poly.back().pos).norm() < 1e-3,
                "radial_disk: contour is explicitly closed");

        const int iso_idx = nearestIsoIndex(meanFieldValue(poly), iso_values);
        const double iso = iso_values[static_cast<size_t>(iso_idx)];
        seen[static_cast<size_t>(iso_idx)] = true;

        double radius_sum = 0.0;
        for (const auto& cp : poly) {
            require(std::abs(cp.field_value - iso) < 1e-2,
                    "radial_disk: interpolated field value matches iso");
            radius_sum += cp.pos.norm();
        }

        const double mean_radius = radius_sum / static_cast<double>(poly.size());
        const double length = polylineLength(poly);
        const double expected_length = 2.0 * M_PI * iso;

        std::cout << "    iso=" << iso
                  << " points=" << poly.size()
                  << " mean_radius=" << mean_radius
                  << " length=" << length
                  << " expected=" << expected_length << "\n";

        require(std::abs(mean_radius - iso) / iso < 0.05,
                "radial_disk: mean radius within 5 percent of iso");
        require(std::abs(length - expected_length) / expected_length < 0.05,
                "radial_disk: circumference within 5 percent");
    }

    require(std::all_of(seen.begin(), seen.end(), [](bool v) { return v; }),
            "radial_disk: all requested iso values were returned");
}

static void test_reorder_for_print_orders_by_iso()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    buildDiskFan(128, 1.0, V, F);

    const Eigen::VectorXd field = radialField(V);
    const std::vector<double> iso_values = {0.3, 0.5, 0.7};

    cslc::GeodesicContourPath contour_path(V, F);
    auto contours = contour_path.extractIsoContours(field, iso_values);
    require(contours.size() == 3, "reorder: got three source contours");

    std::vector<cslc::GeodesicContourPath::Polyline> shuffled;
    shuffled.push_back(contours[2]);
    shuffled.push_back(contours[0]);
    shuffled.push_back(contours[1]);

    const auto ordered = contour_path.reorderForPrint(shuffled);
    require(ordered.size() == shuffled.size(), "reorder: preserves contour count");

    double previous_iso = -std::numeric_limits<double>::infinity();
    for (const auto& poly : ordered) {
        const double current_iso = meanFieldValue(poly);
        std::cout << "    reorder_iso=" << current_iso << "\n";
        require(current_iso + 1e-9 >= previous_iso,
                "reorder: contours are sorted by increasing iso");
        previous_iso = current_iso;
    }
}

int main()
{
    try {
        test_extract_radial_disk_contours();
        test_reorder_for_print_orders_by_iso();
        std::cout << "test_geodesic_contour PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_geodesic_contour FAILED: " << e.what() << '\n';
        return 1;
    }
}
