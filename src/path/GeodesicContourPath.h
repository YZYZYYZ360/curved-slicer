#pragma once

#include <Eigen/Dense>

#include <vector>

namespace cslc {

class GeodesicContourPath {
public:
    struct ContourPoint {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        double field_value = 0.0;
    };

    using Polyline = std::vector<ContourPoint>;

    GeodesicContourPath(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

    std::vector<Polyline> extractIsoContours(
        const Eigen::VectorXd& per_vertex_field,
        const std::vector<double>& iso_values) const;

    std::vector<Polyline> reorderForPrint(
        const std::vector<Polyline>& polylines) const;

private:
    Eigen::MatrixXd V_;
    Eigen::MatrixXi F_;
};

}  // namespace cslc
