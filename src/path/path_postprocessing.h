#pragma once

#include <Eigen/Dense>
#include <vector>

namespace cslc {

// 3-point linear smooth (Savitzky-Golay 3-point, 1st-order).
// Endpoints: (5*p0 + 2*p1 - p2) / 6.
// Interior:  (p[i-1] + p[i] + p[i+1]) / 3.
void smoothPolyline_LinearSmooth31(std::vector<Eigen::Vector3d>& points);

// 5-point linear smooth (Savitzky-Golay 5-point, 1st-order, variant 1).
// Endpoints: (3*p0 + 2*p1 + p2 - p4) / 5, (4*p0 + 3*p1 + 2*p2 + p3) / 10.
// Interior:  (p[i-2] + p[i-1] + p[i] + p[i+1] + p[i+2]) / 5.
void smoothPolyline_LinearSmooth51(std::vector<Eigen::Vector3d>& points);

// 5-point linear smooth (Savitzky-Golay 5-point, 2nd-order).
// Endpoints: (31*p0 + 9*p1 - 3*p2 - 5*p3 + 3*p4) / 35, etc.
// Interior:  (-3*(p[i-2]+p[i+2]) + 12*(p[i-1]+p[i+1]) + 17*p[i]) / 35.
void smoothPolyline_LinearSmooth52(std::vector<Eigen::Vector3d>& points);

// Weighted least squares 1D smooth (applied per-component x/y/z).
// window: controls smoothing strength (higher = smoother).
void smoothPolyline_WLS1D(std::vector<Eigen::Vector3d>& points, int window);

}  // namespace cslc
