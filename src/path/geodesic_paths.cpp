#include "path/geodesic_paths.h"

#include <igl/exact_geodesic.h>
#include <igl/isolines.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

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

// Merge vertices that are within tolerance, returning a canonical index map.
static std::vector<int> mergeVertices(const Eigen::MatrixXd& V, double tol)
{
    int n = static_cast<int>(V.rows());
    std::vector<int> canonical(n);
    for (int i = 0; i < n; ++i) canonical[i] = i;

    double tol2 = tol * tol;
    for (int i = 0; i < n; ++i) {
        if (canonical[i] != i) continue;
        for (int j = i + 1; j < n; ++j) {
            if (canonical[j] != j) continue;
            if ((V.row(i) - V.row(j)).squaredNorm() < tol2) {
                canonical[j] = i;
            }
        }
    }
    return canonical;
}

// Stitch isolated edge segments into connected polylines for a single iso-value.
static std::vector<PathPolyline> stitchEdgesToPolylines(
    const Eigen::MatrixXd& iV, const std::vector<std::pair<int,int>>& edges)
{
    if (edges.empty()) return {};

    // Merge nearby vertices to handle igl::isolines creating separate entries
    // at the same geometric position for adjacent triangles.
    auto canonical = mergeVertices(iV, 1e-4);

    // Remap edges through canonical mapping, filtering self-loops
    std::vector<std::pair<int,int>> mapped;
    mapped.reserve(edges.size());
    for (auto& e : edges) {
        int a = canonical[e.first], b = canonical[e.second];
        if (a != b) mapped.push_back({a, b});
    }

    // Build adjacency: canonical_vertex → list of (neighbor, edge_index)
    std::map<int, std::vector<std::pair<int,int>>> adj;
    for (size_t e = 0; e < mapped.size(); ++e) {
        int a = mapped[e].first, b = mapped[e].second;
        adj[a].push_back({b, static_cast<int>(e)});
        adj[b].push_back({a, static_cast<int>(e)});
    }

    // Walk edges to build polylines.
    // Start from degree-1 vertices (dead ends) first so chains are walked
    // endpoint-to-endpoint rather than from a pass-through vertex.
    std::vector<int> walk_order;
    for (auto& [v, neighbors] : adj) {
        if (neighbors.size() == 1) walk_order.push_back(v);
    }
    for (auto& [v, neighbors] : adj) {
        if (neighbors.size() > 1) walk_order.push_back(v);
    }

    std::set<int> used_edges;
    std::vector<PathPolyline> result;

    for (int vstart : walk_order) {
        for (auto& [first_nb, first_ei] : adj[vstart]) {
            if (used_edges.count(first_ei)) continue;

            std::vector<int> path_verts = {vstart, first_nb};
            used_edges.insert(first_ei);

            int cur = first_nb;
            while (true) {
                bool found_next = false;
                for (auto& [nb, ei] : adj[cur]) {
                    if (used_edges.count(ei)) continue;
                    used_edges.insert(ei);
                    path_verts.push_back(nb);
                    cur = nb;
                    found_next = true;
                    break;
                }
                if (!found_next) break;
            }

            // Check closure
            bool closed = (cur == vstart);
            if (!closed) {
                for (auto& [nb, ei] : adj[cur]) {
                    if (nb == vstart && !used_edges.count(ei)) {
                        used_edges.insert(ei);
                        closed = true;
                        break;
                    }
                }
            }

            // Build polyline using canonical vertex positions
            PathPolyline poly;
            poly.is_closed = closed;
            poly.points.reserve(path_verts.size());
            for (int vi : path_verts) {
                poly.points.push_back(iV.row(vi));
            }

            // Remove consecutive duplicate points
            poly.points.erase(
                std::unique(poly.points.begin(), poly.points.end(),
                    [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                        return (a - b).squaredNorm() < 1e-10;
                    }),
                poly.points.end());

            poly.total_length_mm = 0.0;
            for (size_t i = 1; i < poly.points.size(); ++i) {
                poly.total_length_mm += (poly.points[i] - poly.points[i - 1]).norm();
            }
            if (closed && poly.points.size() > 1) {
                poly.total_length_mm +=
                    (poly.points.back() - poly.points.front()).norm();
            }

            if (poly.points.size() >= 2) {
                result.push_back(std::move(poly));
            }
        }
    }

    return result;
}

// Rotate a closed polyline so that the vertex with minimum z is first.
// Tiebreak: minimum x among minimum-z vertices.
static void normalizeStartPoint(PathPolyline& poly)
{
    if (!poly.is_closed || poly.points.size() < 2) return;

    int min_idx = 0;
    double min_z = poly.points[0].z();
    double min_x = poly.points[0].x();
    for (size_t i = 1; i < poly.points.size(); ++i) {
        double z = poly.points[i].z();
        double x = poly.points[i].x();
        if (z < min_z || (z == min_z && x < min_x)) {
            min_z = z;
            min_x = x;
            min_idx = static_cast<int>(i);
        }
    }
    if (min_idx > 0) {
        std::rotate(poly.points.begin(), poly.points.begin() + min_idx,
                    poly.points.end());
    }
}

// Compute tangent vectors for a polyline.
static void computeTangents(PathPolyline& poly)
{
    int n = static_cast<int>(poly.points.size());
    poly.tangents.resize(n);
    if (n == 0) return;
    if (n == 1) { poly.tangents[0] = Eigen::Vector3d::UnitX(); return; }

    if (poly.is_closed) {
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d d = poly.points[(i + 1) % n] -
                                poly.points[(i - 1 + n) % n];
            double len = d.norm();
            if (len > 1e-12) poly.tangents[i] = d / len;
            else poly.tangents[i] = Eigen::Vector3d::UnitX();
        }
    } else {
        poly.tangents[0] = (poly.points[1] - poly.points[0]).normalized();
        if (n >= 2) {
            poly.tangents[n - 1] =
                (poly.points[n - 1] - poly.points[n - 2]).normalized();
        }
        for (int i = 1; i < n - 1; ++i) {
            Eigen::Vector3d d = poly.points[i + 1] - poly.points[i - 1];
            double len = d.norm();
            if (len > 1e-12) poly.tangents[i] = d / len;
            else poly.tangents[i] = Eigen::Vector3d::UnitX();
        }
    }
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

    if (iE.rows() == 0) return paths;

    // 5. Group edges by iso-value
    int max_iso = static_cast<int>(I.maxCoeff());
    std::vector<std::vector<std::pair<int,int>>> edge_groups(max_iso + 1);
    for (int e = 0; e < iE.rows(); ++e) {
        edge_groups[I(e)].push_back({iE(e, 0), iE(e, 1)});
    }

    // 6. Stitch each iso-value's edges, then normalize and compute tangents
    for (auto& group : edge_groups) {
        auto stitched = stitchEdgesToPolylines(iV, group);
        for (auto& poly : stitched) {
            normalizeStartPoint(poly);
            computeTangents(poly);
        }
        paths.insert(paths.end(), stitched.begin(), stitched.end());
    }

    return paths;
}

}  // namespace cslc
