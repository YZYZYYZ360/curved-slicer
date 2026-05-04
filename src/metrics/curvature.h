#pragma once

#include "io/stl_reader.h"
#include "surface/iso_surface.h"

#include <cstddef>
#include <vector>

namespace cslc {

struct CurvatureSummary {
    std::size_t triangle_count = 0;
    double max_abs_mean_curvature = 0.0;
};

CurvatureSummary computePhase0CurvatureSummary(const TriangleMesh& mesh);
std::vector<double> computeMeanCurvature(const IsoMesh& mesh);
double maxAbsMeanCurvature(const std::vector<double>& values);

}  // namespace cslc
