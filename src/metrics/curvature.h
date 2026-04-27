#pragma once

#include "io/stl_reader.h"

#include <cstddef>

namespace cslc {

struct CurvatureSummary {
    std::size_t triangle_count = 0;
    double max_abs_mean_curvature = 0.0;
};

CurvatureSummary computePhase0CurvatureSummary(const TriangleMesh& mesh);

}  // namespace cslc

