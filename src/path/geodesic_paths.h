#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace cslc {

struct GeodesicPathParams {
    double line_spacing_mm  = 5.0;
    double resample_step_mm = 1.0;
    std::string seed_strategy = "centroid";  // "centroid" or "farthest"
};

struct PathPolyline {
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> tangents;
    bool is_closed = false;
    double total_length_mm = 0.0;
};

// Generate geodesic isoline paths on a triangle mesh.
// V: #V x 3 vertex positions, F: #F x 3 face indices.
// Returns a vector of PathPolyline, one per isoline.
std::vector<PathPolyline> generateGeodesicPaths(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const GeodesicPathParams& params = {});

}  // namespace cslc
