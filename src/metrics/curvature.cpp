#include "metrics/curvature.h"

namespace cslc {

CurvatureSummary computePhase0CurvatureSummary(const TriangleMesh& mesh)
{
    CurvatureSummary summary;
    summary.triangle_count = mesh.triangles.size();
    summary.max_abs_mean_curvature = 0.0;
    return summary;
}

}  // namespace cslc
