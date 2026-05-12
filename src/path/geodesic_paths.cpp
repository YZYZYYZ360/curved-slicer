#include "path/geodesic_paths.h"

#include <igl/exact_geodesic.h>
#include <igl/isolines.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cslc {

static int findClosestVertex(const Eigen::MatrixXd& V, const Eigen::Vector3d& target)
{
    int best = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < V.rows(); ++i) {
        double d2 = (V.row(i).transpose() - target).squaredNorm();
        if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
}

std::vector<PathPolyline> generateGeodesicPaths(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const GeodesicPathParams& params)
{
    std::vector<PathPolyline> paths;
    if (V.rows() == 0 || F.rows() == 0) return paths;

    // 1. Select seed vertex
    Eigen::Vector3d centroid = V.colwise().mean();
    int seed = findClosestVertex(V, centroid);

    // 2. Compute geodesic distance field from seed to all vertices
    Eigen::VectorXi VS(1), FS(1), VT(V.rows()), FT(0);
    VS << seed;
    FS << 0;
    for (int i = 0; i < V.rows(); ++i) VT(i) = i;

    Eigen::VectorXd D;
    igl::exact_geodesic(V, F, VS, FS, VT, FT, D);

    // 3. Determine iso-values
    double d_max = D.maxCoeff();
    if (d_max < params.line_spacing_mm) return paths;

    int num_lines = static_cast<int>(d_max / params.line_spacing_mm);
    Eigen::VectorXd vals(num_lines);
    for (int k = 0; k < num_lines; ++k) {
        vals(k) = params.line_spacing_mm * (k + 1);
    }

    // 4. Extract isolines
    Eigen::MatrixXd iV;
    Eigen::MatrixXi iE;
    Eigen::VectorXi I;
    igl::isolines(V, F, D, vals, iV, iE, I);

    // 5. Convert segments to PathPolyline objects (one per iso-value)
    // Group edges by iso-value
    if (iE.rows() == 0) return paths;

    int max_iso = I.maxCoeff();
    paths.resize(max_iso + 1);
    for (int e = 0; e < iE.rows(); ++e) {
        int iso_idx = I(e);
        Eigen::Vector3d p0 = iV.row(iE(e, 0));
        Eigen::Vector3d p1 = iV.row(iE(e, 1));
        paths[iso_idx].points.push_back(p0);
        paths[iso_idx].points.push_back(p1);
        paths[iso_idx].total_length_mm += (p1 - p0).norm();
    }

    // Remove empty entries
    paths.erase(
        std::remove_if(paths.begin(), paths.end(),
                       [](const PathPolyline& p) { return p.points.empty(); }),
        paths.end());

    return paths;
}

}  // namespace cslc
