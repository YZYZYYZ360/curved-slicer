#include "mesh/MeshRepair.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cslc {
namespace {

using UnitGuess = MeshRepair::Report::UnitGuess;

struct EdgeKey {
    int a = 0;
    int b = 0;

    EdgeKey() = default;
    EdgeKey(int u, int v) : a(std::min(u, v)), b(std::max(u, v)) {}

    bool operator==(const EdgeKey& other) const
    {
        return a == other.a && b == other.b;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& edge) const
    {
        const auto a = static_cast<std::uint64_t>(static_cast<std::uint32_t>(edge.a));
        const auto b = static_cast<std::uint64_t>(static_cast<std::uint32_t>(edge.b));
        return static_cast<std::size_t>((a << 32U) ^ b);
    }
};

struct DirectedEdgeRef {
    int face = 0;
    bool forward = false;
};

struct MeshAudit {
    bool manifold = false;
    bool orientable = false;
    int non_manifold_edges = 0;
    int boundary_edges = 0;
};

double triangleArea(
    const Eigen::MatrixXd& V,
    int i0,
    int i1,
    int i2)
{
    const Eigen::Vector3d a = V.row(i0);
    const Eigen::Vector3d b = V.row(i1);
    const Eigen::Vector3d c = V.row(i2);
    return 0.5 * (b - a).cross(c - a).norm();
}

double signedVolume(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    double volume = 0.0;
    for (int f = 0; f < F.rows(); ++f) {
        const Eigen::Vector3d a = V.row(F(f, 0));
        const Eigen::Vector3d b = V.row(F(f, 1));
        const Eigen::Vector3d c = V.row(F(f, 2));
        volume += a.dot(b.cross(c)) / 6.0;
    }
    return volume;
}

double bboxDiagonal(const Eigen::MatrixXd& V)
{
    if (V.rows() == 0 || V.cols() != 3) {
        throw std::invalid_argument("MeshRepair: V must be N x 3 with at least one row");
    }

    const Eigen::Vector3d min_corner = V.colwise().minCoeff();
    const Eigen::Vector3d max_corner = V.colwise().maxCoeff();
    return (max_corner - min_corner).norm();
}

void validateFaces(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    if (F.cols() != 3) {
        throw std::invalid_argument("MeshRepair: F must be M x 3");
    }
    for (int f = 0; f < F.rows(); ++f) {
        for (int c = 0; c < 3; ++c) {
            if (F(f, c) < 0 || F(f, c) >= V.rows()) {
                throw std::invalid_argument("MeshRepair: F contains out-of-range vertex index");
            }
        }
    }
}

UnitGuess classifyUnits(double diagonal_mm, double meters_threshold)
{
    if (diagonal_mm < meters_threshold) return UnitGuess::METERS;
    if (diagonal_mm < 10.0) return UnitGuess::AMBIGUOUS;
    return UnitGuess::MILLIMETERS;
}

void addDirectedEdge(
    std::unordered_map<EdgeKey, std::vector<DirectedEdgeRef>, EdgeKeyHash>& edges,
    int face,
    int u,
    int v)
{
    const EdgeKey key(u, v);
    edges[key].push_back({face, u == key.a && v == key.b});
}

std::unordered_map<EdgeKey, std::vector<DirectedEdgeRef>, EdgeKeyHash>
buildEdgeRefs(const Eigen::MatrixXi& F)
{
    std::unordered_map<EdgeKey, std::vector<DirectedEdgeRef>, EdgeKeyHash> edges;
    edges.reserve(static_cast<size_t>(F.rows()) * 3U);
    for (int f = 0; f < F.rows(); ++f) {
        addDirectedEdge(edges, f, F(f, 0), F(f, 1));
        addDirectedEdge(edges, f, F(f, 1), F(f, 2));
        addDirectedEdge(edges, f, F(f, 2), F(f, 0));
    }
    return edges;
}

MeshAudit auditMesh(const Eigen::MatrixXi& F)
{
    MeshAudit audit;
    audit.manifold = true;
    audit.orientable = true;

    const auto edges = buildEdgeRefs(F);
    std::vector<std::vector<std::pair<int, bool>>> adjacency(static_cast<size_t>(F.rows()));

    for (const auto& item : edges) {
        const auto& refs = item.second;
        if (refs.size() == 1U) {
            ++audit.boundary_edges;
        } else if (refs.size() > 2U) {
            ++audit.non_manifold_edges;
            audit.manifold = false;
            audit.orientable = false;
        } else {
            const auto& a = refs[0];
            const auto& b = refs[1];
            const bool must_flip = a.forward == b.forward;
            adjacency[static_cast<size_t>(a.face)].push_back({b.face, must_flip});
            adjacency[static_cast<size_t>(b.face)].push_back({a.face, must_flip});
        }
    }

    std::vector<int> color(static_cast<size_t>(F.rows()), -1);
    for (int seed = 0; seed < F.rows(); ++seed) {
        if (color[static_cast<size_t>(seed)] != -1) continue;

        color[static_cast<size_t>(seed)] = 0;
        std::queue<int> queue;
        queue.push(seed);
        while (!queue.empty()) {
            const int face = queue.front();
            queue.pop();

            for (const auto& [next, must_flip] : adjacency[static_cast<size_t>(face)]) {
                const int wanted = color[static_cast<size_t>(face)] ^ (must_flip ? 1 : 0);
                if (color[static_cast<size_t>(next)] == -1) {
                    color[static_cast<size_t>(next)] = wanted;
                    queue.push(next);
                } else if (color[static_cast<size_t>(next)] != wanted) {
                    audit.orientable = false;
                }
            }
        }
    }

    return audit;
}

struct GridKey {
    long long x = 0;
    long long y = 0;
    long long z = 0;

    bool operator==(const GridKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey& key) const
    {
        std::uint64_t h = 1469598103934665603ULL;
        auto mix = [&h](long long value) {
            h ^= static_cast<std::uint64_t>(value + 0x9e3779b97f4a7c15LL);
            h *= 1099511628211ULL;
        };
        mix(key.x);
        mix(key.y);
        mix(key.z);
        return static_cast<std::size_t>(h);
    }
};

GridKey gridKey(const Eigen::Vector3d& p, double eps)
{
    return {
        static_cast<long long>(std::floor(p.x() / eps)),
        static_cast<long long>(std::floor(p.y() / eps)),
        static_cast<long long>(std::floor(p.z() / eps)),
    };
}

void mergeDuplicateVertices(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    double eps,
    Eigen::MatrixXd& V_out,
    Eigen::MatrixXi& F_out,
    int& merged_count)
{
    if (eps <= 0.0) {
        throw std::invalid_argument("MeshRepair: duplicate vertex epsilon must be positive");
    }

    const double eps2 = eps * eps;
    std::vector<Eigen::Vector3d> unique_vertices;
    unique_vertices.reserve(static_cast<size_t>(V_in.rows()));
    std::vector<int> old_to_new(static_cast<size_t>(V_in.rows()), -1);
    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> cells;
    cells.reserve(static_cast<size_t>(V_in.rows()));

    for (int i = 0; i < V_in.rows(); ++i) {
        const Eigen::Vector3d p = V_in.row(i);
        const GridKey base = gridKey(p, eps);

        int found = -1;
        for (int dz = -1; dz <= 1 && found == -1; ++dz) {
            for (int dy = -1; dy <= 1 && found == -1; ++dy) {
                for (int dx = -1; dx <= 1 && found == -1; ++dx) {
                    const GridKey neighbor{base.x + dx, base.y + dy, base.z + dz};
                    const auto it = cells.find(neighbor);
                    if (it == cells.end()) continue;

                    for (int candidate : it->second) {
                        if ((unique_vertices[static_cast<size_t>(candidate)] - p).squaredNorm() <= eps2) {
                            found = candidate;
                            break;
                        }
                    }
                }
            }
        }

        if (found == -1) {
            found = static_cast<int>(unique_vertices.size());
            unique_vertices.push_back(p);
            cells[base].push_back(found);
        }
        old_to_new[static_cast<size_t>(i)] = found;
    }

    V_out.resize(static_cast<int>(unique_vertices.size()), 3);
    for (int i = 0; i < V_out.rows(); ++i) {
        V_out.row(i) = unique_vertices[static_cast<size_t>(i)].transpose();
    }

    F_out.resize(F_in.rows(), 3);
    for (int f = 0; f < F_in.rows(); ++f) {
        F_out(f, 0) = old_to_new[static_cast<size_t>(F_in(f, 0))];
        F_out(f, 1) = old_to_new[static_cast<size_t>(F_in(f, 1))];
        F_out(f, 2) = old_to_new[static_cast<size_t>(F_in(f, 2))];
    }

    merged_count = V_in.rows() - V_out.rows();
}

int countDegenerateTriangles(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    double area_eps)
{
    int count = 0;
    for (int f = 0; f < F.rows(); ++f) {
        if (F(f, 0) == F(f, 1) || F(f, 1) == F(f, 2) || F(f, 2) == F(f, 0) ||
            triangleArea(V, F(f, 0), F(f, 1), F(f, 2)) < area_eps) {
            ++count;
        }
    }
    return count;
}

void removeDegenerateTriangles(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F_in,
    double area_eps,
    Eigen::MatrixXi& F_out,
    int& removed_count)
{
    std::vector<Eigen::Vector3i> faces;
    faces.reserve(static_cast<size_t>(F_in.rows()));
    for (int f = 0; f < F_in.rows(); ++f) {
        const bool repeated =
            F_in(f, 0) == F_in(f, 1) || F_in(f, 1) == F_in(f, 2) || F_in(f, 2) == F_in(f, 0);
        const bool tiny = !repeated &&
            triangleArea(V, F_in(f, 0), F_in(f, 1), F_in(f, 2)) < area_eps;
        if (repeated || tiny) continue;
        faces.emplace_back(F_in.row(f));
    }

    F_out.resize(static_cast<int>(faces.size()), 3);
    for (int i = 0; i < F_out.rows(); ++i) {
        F_out.row(i) = faces[static_cast<size_t>(i)].transpose();
    }
    removed_count = F_in.rows() - F_out.rows();
}

std::vector<std::vector<int>> boundaryLoops(const Eigen::MatrixXi& F)
{
    const auto edge_refs = buildEdgeRefs(F);
    std::unordered_map<int, std::vector<int>> adjacency;
    std::vector<EdgeKey> boundary_edges;
    for (const auto& item : edge_refs) {
        if (item.second.size() != 1U) continue;
        const auto edge = item.first;
        boundary_edges.push_back(edge);
        adjacency[edge.a].push_back(edge.b);
        adjacency[edge.b].push_back(edge.a);
    }

    std::unordered_map<EdgeKey, bool, EdgeKeyHash> used;
    std::vector<std::vector<int>> loops;

    for (const auto& start_edge : boundary_edges) {
        if (used[start_edge]) continue;
        if (adjacency[start_edge.a].size() != 2U || adjacency[start_edge.b].size() != 2U) {
            used[start_edge] = true;
            continue;
        }

        std::vector<int> loop;
        loop.push_back(start_edge.a);
        int previous = start_edge.a;
        int current = start_edge.b;
        used[start_edge] = true;

        while (true) {
            loop.push_back(current);
            if (current == loop.front()) break;

            const auto it = adjacency.find(current);
            if (it == adjacency.end() || it->second.size() != 2U) {
                loop.clear();
                break;
            }

            int next = it->second[0] == previous ? it->second[1] : it->second[0];
            const EdgeKey next_edge(current, next);
            if (used[next_edge] && next != loop.front()) {
                loop.clear();
                break;
            }
            used[next_edge] = true;
            previous = current;
            current = next;
        }

        if (loop.size() >= 4U && loop.front() == loop.back()) {
            loop.pop_back();
            loops.push_back(std::move(loop));
        }
    }

    return loops;
}

void appendRows(
    Eigen::MatrixXd& V,
    const std::vector<Eigen::Vector3d>& extra_vertices,
    Eigen::MatrixXi& F,
    const std::vector<Eigen::Vector3i>& extra_faces)
{
    const int old_v_rows = V.rows();
    const int old_f_rows = F.rows();

    V.conservativeResize(old_v_rows + static_cast<int>(extra_vertices.size()), 3);
    for (int i = 0; i < static_cast<int>(extra_vertices.size()); ++i) {
        V.row(old_v_rows + i) = extra_vertices[static_cast<size_t>(i)].transpose();
    }

    F.conservativeResize(old_f_rows + static_cast<int>(extra_faces.size()), 3);
    for (int i = 0; i < static_cast<int>(extra_faces.size()); ++i) {
        F.row(old_f_rows + i) = extra_faces[static_cast<size_t>(i)].transpose();
    }
}

int fillBoundaryHoles(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    const auto loops = boundaryLoops(F);
    std::vector<Eigen::Vector3d> centers;
    std::vector<Eigen::Vector3i> new_faces;
    int filled = 0;

    for (const auto& loop : loops) {
        if (loop.size() < 3U || loop.size() > 100U) continue;

        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (int v : loop) center += V.row(v).transpose();
        center /= static_cast<double>(loop.size());

        const int center_index = V.rows() + static_cast<int>(centers.size());
        centers.push_back(center);
        for (size_t i = 0; i < loop.size(); ++i) {
            const int a = loop[i];
            const int b = loop[(i + 1U) % loop.size()];
            new_faces.emplace_back(a, b, center_index);
        }
        ++filled;
    }

    appendRows(V, centers, F, new_faces);
    return filled;
}

int orientConsistentOutward(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    (void)V;
    const auto edge_refs = buildEdgeRefs(F);
    std::vector<std::vector<std::pair<int, bool>>> adjacency(static_cast<size_t>(F.rows()));
    for (const auto& item : edge_refs) {
        const auto& refs = item.second;
        if (refs.size() != 2U) continue;
        const auto& a = refs[0];
        const auto& b = refs[1];
        const bool must_flip = a.forward == b.forward;
        adjacency[static_cast<size_t>(a.face)].push_back({b.face, must_flip});
        adjacency[static_cast<size_t>(b.face)].push_back({a.face, must_flip});
    }

    std::vector<int> flip(static_cast<size_t>(F.rows()), -1);
    for (int seed = 0; seed < F.rows(); ++seed) {
        if (flip[static_cast<size_t>(seed)] != -1) continue;
        flip[static_cast<size_t>(seed)] = 0;
        std::queue<int> queue;
        queue.push(seed);
        while (!queue.empty()) {
            const int face = queue.front();
            queue.pop();
            for (const auto& [next, must_flip] : adjacency[static_cast<size_t>(face)]) {
                const int wanted = flip[static_cast<size_t>(face)] ^ (must_flip ? 1 : 0);
                if (flip[static_cast<size_t>(next)] == -1) {
                    flip[static_cast<size_t>(next)] = wanted;
                    queue.push(next);
                }
            }
        }
    }

    for (int& value : flip) {
        if (value == -1) value = 0;
    }

    for (int f = 0; f < F.rows(); ++f) {
        if (flip[static_cast<size_t>(f)] == 1) std::swap(F(f, 1), F(f, 2));
    }

    if (signedVolume(V, F) < 0.0) {
        for (int f = 0; f < F.rows(); ++f) {
            std::swap(F(f, 1), F(f, 2));
            flip[static_cast<size_t>(f)] ^= 1;
        }
    }

    return static_cast<int>(std::count(flip.begin(), flip.end(), 1));
}

}  // namespace

MeshRepair::Report MeshRepair::repair(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Options& opt,
    Eigen::MatrixXd& V_out,
    Eigen::MatrixXi& F_out)
{
    validateFaces(V_in, F_in);

    Report report;
    report.n_vertices_in = static_cast<int>(V_in.rows());
    report.n_triangles_in = static_cast<int>(F_in.rows());
    report.bbox_diagonal_mm = bboxDiagonal(V_in);
    report.unit_guess =
        classifyUnits(report.bbox_diagonal_mm, opt.meters_threshold_bbox_diagonal);

    if (report.unit_guess == UnitGuess::METERS) {
        throw std::runtime_error(
            "Mesh bbox diagonal < 1 mm, likely meters unit. Specify --unit explicitly.");
    }

    const MeshAudit input_audit = auditMesh(F_in);
    report.is_manifold = input_audit.manifold;
    report.n_non_manifold_edges = input_audit.non_manifold_edges;
    report.is_orientable = input_audit.orientable;
    report.n_boundary_edges = input_audit.boundary_edges;
    report.n_degenerate_tris =
        countDegenerateTriangles(V_in, F_in, opt.degenerate_area_eps_mm2);

    Eigen::MatrixXd V_work;
    Eigen::MatrixXi F_work;
    mergeDuplicateVertices(
        V_in, F_in, opt.duplicate_vertex_eps_mm, V_work, F_work, report.n_vertices_merged);
    report.n_duplicate_vertices = report.n_vertices_merged;

    Eigen::MatrixXi F_no_degenerate;
    removeDegenerateTriangles(
        V_work, F_work, opt.degenerate_area_eps_mm2, F_no_degenerate,
        report.n_triangles_removed_degenerate);
    F_work = std::move(F_no_degenerate);

    if (opt.fill_holes) {
        report.n_holes_filled = fillBoundaryHoles(V_work, F_work);
    }

    if (opt.reorient_outward) {
        report.n_triangles_flipped = orientConsistentOutward(V_work, F_work);
    }

    const MeshAudit output_audit = auditMesh(F_work);
    report.n_vertices_out = static_cast<int>(V_work.rows());
    report.n_triangles_out = static_cast<int>(F_work.rows());
    report.is_manifold_after = output_audit.manifold;
    report.is_orientable_after = output_audit.orientable;
    report.n_boundary_edges_after = output_audit.boundary_edges;

    V_out = std::move(V_work);
    F_out = std::move(F_work);
    return report;
}

}  // namespace cslc
