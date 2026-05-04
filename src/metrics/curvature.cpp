#include "metrics/curvature.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

namespace cslc {

CurvatureSummary computePhase0CurvatureSummary(const TriangleMesh& mesh)
{
    CurvatureSummary summary;
    summary.triangle_count = mesh.triangles.size();
    summary.max_abs_mean_curvature = 0.0;
    return summary;
}

std::vector<double> computeMeanCurvature(const IsoMesh& mesh)
{
    std::vector<std::vector<std::size_t>> adjacency(mesh.vertices.size());
    for (const Tri& triangle : mesh.triangles) {
        const std::array<std::pair<std::size_t, std::size_t>, 3> edges{{
            {triangle.v0, triangle.v1},
            {triangle.v1, triangle.v2},
            {triangle.v2, triangle.v0},
        }};
        for (const auto& edge : edges) {
            adjacency[edge.first].push_back(edge.second);
            adjacency[edge.second].push_back(edge.first);
        }
    }

    std::vector<double> curvature(mesh.vertices.size(), 0.0);
    for (std::size_t vertex = 0; vertex < adjacency.size(); ++vertex) {
        std::vector<std::size_t>& neighbors = adjacency[vertex];
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
        if (neighbors.empty()) {
            continue;
        }

        Vec3 laplace{};
        double mean_edge_length = 0.0;
        for (std::size_t neighbor : neighbors) {
            const Vec3 delta = mesh.vertices[neighbor] - mesh.vertices[vertex];
            laplace = laplace + delta;
            mean_edge_length += norm(delta);
        }

        const double degree = static_cast<double>(neighbors.size());
        laplace = laplace / degree;
        mean_edge_length /= degree;
        if (mean_edge_length > 1e-12) {
            curvature[vertex] = norm(laplace) / (mean_edge_length * mean_edge_length);
        }
    }
    return curvature;
}

double maxAbsMeanCurvature(const std::vector<double>& values)
{
    double result = 0.0;
    for (double value : values) {
        if (std::isfinite(value)) {
            result = std::max(result, std::abs(value));
        }
    }
    return result;
}

}  // namespace cslc
