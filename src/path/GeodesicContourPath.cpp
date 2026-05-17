#include "path/GeodesicContourPath.h"

#include <igl/isolines.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <utility>

namespace cslc {
namespace {

using Polyline = GeodesicContourPath::Polyline;
using ContourPoint = GeodesicContourPath::ContourPoint;

constexpr double kMergeTolerance = 1e-6;

std::vector<int> mergeVerticesForEdges(
    const Eigen::MatrixXd& iV,
    const std::vector<std::pair<int, int>>& edges,
    double tol)
{
    std::set<int> referenced;
    for (const auto& edge : edges) {
        referenced.insert(edge.first);
        referenced.insert(edge.second);
    }

    std::vector<int> vertices(referenced.begin(), referenced.end());
    std::vector<int> canonical(static_cast<size_t>(iV.rows()));
    std::iota(canonical.begin(), canonical.end(), 0);

    const double tol2 = tol * tol;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const int vi = vertices[i];
        if (canonical[static_cast<size_t>(vi)] != vi) continue;

        for (size_t j = i + 1; j < vertices.size(); ++j) {
            const int vj = vertices[j];
            if (canonical[static_cast<size_t>(vj)] != vj) continue;

            if ((iV.row(vi) - iV.row(vj)).squaredNorm() <= tol2) {
                canonical[static_cast<size_t>(vj)] = vi;
            }
        }
    }

    return canonical;
}

bool isClosed(const Polyline& poly)
{
    return poly.size() >= 3 &&
           (poly.front().pos - poly.back().pos).norm() <= kMergeTolerance;
}

void normalizeStartPoint(Polyline& poly)
{
    if (!isClosed(poly) || poly.size() < 4) return;

    std::vector<ContourPoint> body(poly.begin(), poly.end() - 1);
    auto best = body.begin();
    for (auto it = body.begin() + 1; it != body.end(); ++it) {
        const auto& a = it->pos;
        const auto& b = best->pos;
        if (a.z() < b.z() ||
            (a.z() == b.z() && a.x() < b.x()) ||
            (a.z() == b.z() && a.x() == b.x() && a.y() < b.y())) {
            best = it;
        }
    }

    std::rotate(body.begin(), best, body.end());
    poly = std::move(body);
    poly.push_back(poly.front());
}

double meanFieldValue(const Polyline& poly)
{
    if (poly.empty()) return std::numeric_limits<double>::infinity();

    double sum = 0.0;
    for (const auto& cp : poly) sum += cp.field_value;
    return sum / static_cast<double>(poly.size());
}

std::vector<Polyline> stitchEdgesToPolylines(
    const Eigen::MatrixXd& iV,
    const std::vector<std::pair<int, int>>& edges,
    double iso_value)
{
    if (edges.empty()) return {};

    const auto canonical = mergeVerticesForEdges(iV, edges, kMergeTolerance);

    std::vector<std::pair<int, int>> mapped;
    mapped.reserve(edges.size());
    for (const auto& edge : edges) {
        const int a = canonical[static_cast<size_t>(edge.first)];
        const int b = canonical[static_cast<size_t>(edge.second)];
        if (a != b) mapped.emplace_back(a, b);
    }

    std::map<int, std::vector<std::pair<int, int>>> adjacency;
    for (int edge_idx = 0; edge_idx < static_cast<int>(mapped.size()); ++edge_idx) {
        const auto [a, b] = mapped[static_cast<size_t>(edge_idx)];
        adjacency[a].push_back({b, edge_idx});
        adjacency[b].push_back({a, edge_idx});
    }

    std::vector<int> walk_order;
    walk_order.reserve(adjacency.size());
    for (const auto& [vertex, neighbors] : adjacency) {
        if (neighbors.size() == 1) walk_order.push_back(vertex);
    }
    for (const auto& [vertex, neighbors] : adjacency) {
        if (neighbors.size() != 1) walk_order.push_back(vertex);
    }

    std::vector<bool> used(mapped.size(), false);
    std::vector<Polyline> polylines;

    for (int start : walk_order) {
        for (const auto& [first_neighbor, first_edge] : adjacency[start]) {
            if (used[static_cast<size_t>(first_edge)]) continue;

            std::vector<int> path = {start, first_neighbor};
            used[static_cast<size_t>(first_edge)] = true;

            int previous = start;
            int current = first_neighbor;
            while (current != start) {
                bool found_next = false;
                for (const auto& [neighbor, edge_idx] : adjacency[current]) {
                    if (used[static_cast<size_t>(edge_idx)] || neighbor == previous) {
                        continue;
                    }

                    used[static_cast<size_t>(edge_idx)] = true;
                    path.push_back(neighbor);
                    previous = current;
                    current = neighbor;
                    found_next = true;
                    break;
                }

                if (!found_next) break;
            }

            Polyline poly;
            poly.reserve(path.size() + 1);
            for (int vertex : path) {
                ContourPoint cp;
                cp.pos = iV.row(vertex).transpose();
                cp.field_value = iso_value;
                if (poly.empty() || (cp.pos - poly.back().pos).norm() > kMergeTolerance) {
                    poly.push_back(cp);
                }
            }

            const bool closed = poly.size() >= 3 &&
                                (poly.front().pos - poly.back().pos).norm() <=
                                    kMergeTolerance;
            if (closed) {
                poly.back() = poly.front();
            } else if (current == start && poly.size() >= 3) {
                poly.push_back(poly.front());
            }

            if (poly.size() >= 2) {
                normalizeStartPoint(poly);
                polylines.push_back(std::move(poly));
            }
        }
    }

    return polylines;
}

}  // namespace

GeodesicContourPath::GeodesicContourPath(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F)
    : V_(V), F_(F)
{
    if (V_.cols() != 3) {
        throw std::invalid_argument("GeodesicContourPath: V must have 3 columns");
    }
    if (F_.cols() != 3) {
        throw std::invalid_argument("GeodesicContourPath: F must have 3 columns");
    }
}

std::vector<Polyline> GeodesicContourPath::extractIsoContours(
    const Eigen::VectorXd& per_vertex_field,
    const std::vector<double>& iso_values) const
{
    if (per_vertex_field.size() != V_.rows()) {
        throw std::invalid_argument(
            "GeodesicContourPath: per-vertex field size must match V rows");
    }
    if (V_.rows() == 0 || F_.rows() == 0 || iso_values.empty()) return {};

    Eigen::VectorXd vals(static_cast<int>(iso_values.size()));
    for (int i = 0; i < vals.size(); ++i) vals(i) = iso_values[static_cast<size_t>(i)];

    Eigen::MatrixXd iV;
    Eigen::MatrixXi iE;
    Eigen::VectorXi I;
    igl::isolines(V_, F_, per_vertex_field, vals, iV, iE, I);

    if (iE.rows() == 0) return {};

    std::vector<std::vector<std::pair<int, int>>> edge_groups(iso_values.size());
    for (int edge = 0; edge < iE.rows(); ++edge) {
        const int iso_idx = I(edge);
        if (iso_idx < 0 || iso_idx >= static_cast<int>(iso_values.size())) continue;
        edge_groups[static_cast<size_t>(iso_idx)].push_back({iE(edge, 0), iE(edge, 1)});
    }

    std::vector<Polyline> contours;
    for (size_t iso_idx = 0; iso_idx < edge_groups.size(); ++iso_idx) {
        auto stitched = stitchEdgesToPolylines(
            iV, edge_groups[iso_idx], iso_values[iso_idx]);
        contours.insert(contours.end(),
                        std::make_move_iterator(stitched.begin()),
                        std::make_move_iterator(stitched.end()));
    }

    return contours;
}

std::vector<Polyline> GeodesicContourPath::reorderForPrint(
    const std::vector<Polyline>& polylines) const
{
    std::vector<Polyline> ordered = polylines;
    for (auto& poly : ordered) normalizeStartPoint(poly);

    std::sort(ordered.begin(), ordered.end(),
              [](const Polyline& a, const Polyline& b) {
                  return meanFieldValue(a) < meanFieldValue(b);
              });

    return ordered;
}

}  // namespace cslc
