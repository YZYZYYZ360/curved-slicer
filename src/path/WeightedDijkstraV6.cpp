#include "path/WeightedDijkstraV6.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace cslc {
namespace {

long long totalVoxelCount(const Eigen::Vector3i& dims)
{
    return static_cast<long long>(dims.x()) *
        static_cast<long long>(dims.y()) *
        static_cast<long long>(dims.z());
}

void validateGridAndSdf(
    const VoxelizationV6::Grid& grid,
    const SDFV6::Field& sdf)
{
    if (grid.dims.x() <= 0 || grid.dims.y() <= 0 || grid.dims.z() <= 0) {
        throw std::invalid_argument("WeightedDijkstraV6: grid dims must be positive");
    }
    if (grid.spacing_mm <= 0.0) {
        throw std::invalid_argument("WeightedDijkstraV6: grid spacing must be positive");
    }
    const long long total = totalVoxelCount(grid.dims);
    if (total <= 0 ||
        static_cast<long long>(grid.occupancy.size()) != total ||
        static_cast<long long>(sdf.distance_mm.size()) != total) {
        throw std::invalid_argument("WeightedDijkstraV6: grid/SDF sizes do not match dims");
    }
    if (sdf.grid.dims != grid.dims ||
        sdf.grid.origin_mm != grid.origin_mm ||
        sdf.grid.spacing_mm != grid.spacing_mm) {
        throw std::invalid_argument("WeightedDijkstraV6: SDF grid geometry does not match grid");
    }
}

bool inBounds(const VoxelizationV6::Grid& grid, int i, int j, int k)
{
    return i >= 0 && i < grid.dims.x() &&
        j >= 0 && j < grid.dims.y() &&
        k >= 0 && k < grid.dims.z();
}

bool isWalkable(
    const VoxelizationV6::Grid& grid,
    int flat,
    bool walk_boundary)
{
    const auto occ = grid.occupancy[static_cast<size_t>(flat)];
    return occ == VoxelizationV6::Occ::INSIDE ||
        (walk_boundary && occ == VoxelizationV6::Occ::BOUNDARY);
}

Eigen::Vector3i flatToIJK(const VoxelizationV6::Grid& grid, int flat)
{
    const int nx = grid.dims.x();
    const int ny = grid.dims.y();
    const int i = flat % nx;
    const int yz = flat / nx;
    const int j = yz % ny;
    const int k = yz / ny;
    return {i, j, k};
}

double clearanceMultiplier(
    const SDFV6::Field& sdf,
    int flat,
    const WeightedDijkstraV6::PenaltyOptions& opt)
{
    if (!opt.enable_clearance) return 1.0;

    const double signed_d = static_cast<double>(sdf.distance_mm[static_cast<size_t>(flat)]);
    const double depth_inside = std::max(0.0, -signed_d);
    // Near-surface inside voxels have small depth and therefore get a larger penalty.
    const double violation = std::max(0.0, opt.clearance_threshold_mm - depth_inside);
    return 1.0 + opt.clearance_penalty_weight * violation;
}

struct QueueNode {
    float cost = WeightedDijkstraV6::UNREACHABLE;
    int flat = 0;
};

struct QueueCompare {
    bool operator()(const QueueNode& a, const QueueNode& b) const
    {
        if (a.cost == b.cost) return a.flat > b.flat;
        return a.cost > b.cost;
    }
};

}  // namespace

WeightedDijkstraV6::DistanceField WeightedDijkstraV6::computeDistanceField(
    const VoxelizationV6::Grid& grid,
    const SDFV6::Field& sdf,
    const std::vector<Eigen::Vector3i>& sources,
    const Options& opt)
{
    validateGridAndSdf(grid, sdf);
    if (sources.empty()) {
        throw std::invalid_argument("WeightedDijkstraV6: at least one source is required");
    }
    if (sources.size() > 255U) {
        throw std::invalid_argument("WeightedDijkstraV6: at most 255 sources are supported");
    }

    const int total = static_cast<int>(grid.occupancy.size());
    DistanceField field;
    field.grid = grid;
    field.distance.assign(static_cast<size_t>(total), UNREACHABLE);
    field.predecessor.assign(static_cast<size_t>(total), NO_PRED);
    field.source_id.assign(static_cast<size_t>(total), UNREACHABLE_SOURCE);

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> pq;
    for (size_t s = 0; s < sources.size(); ++s) {
        const Eigen::Vector3i& src = sources[s];
        if (!inBounds(grid, src.x(), src.y(), src.z())) {
            throw std::invalid_argument("WeightedDijkstraV6: source is out of bounds");
        }
        const int flat = grid.idx(src.x(), src.y(), src.z());
        if (!isWalkable(grid, flat, opt.walk_boundary)) {
            throw std::invalid_argument("WeightedDijkstraV6: source must be INSIDE/BOUNDARY");
        }
        field.distance[static_cast<size_t>(flat)] = 0.0f;
        field.predecessor[static_cast<size_t>(flat)] = NO_PRED;
        field.source_id[static_cast<size_t>(flat)] = static_cast<std::uint8_t>(s);
        pq.push({0.0f, flat});
    }

    while (!pq.empty()) {
        const QueueNode node = pq.top();
        pq.pop();
        if (node.cost != field.distance[static_cast<size_t>(node.flat)]) continue;

        const Eigen::Vector3i ijk = flatToIJK(grid, node.flat);
        for (int dk = -1; dk <= 1; ++dk) {
            for (int dj = -1; dj <= 1; ++dj) {
                for (int di = -1; di <= 1; ++di) {
                    if (di == 0 && dj == 0 && dk == 0) continue;
                    const int ni = ijk.x() + di;
                    const int nj = ijk.y() + dj;
                    const int nk = ijk.z() + dk;
                    if (!inBounds(grid, ni, nj, nk)) continue;

                    const int nflat = grid.idx(ni, nj, nk);
                    if (!isWalkable(grid, nflat, opt.walk_boundary)) continue;

                    const double step_len = grid.spacing_mm *
                        std::sqrt(static_cast<double>(di * di + dj * dj + dk * dk));
                    const double edge_cost =
                        step_len * clearanceMultiplier(sdf, nflat, opt.penalty);
                    const float candidate = static_cast<float>(
                        static_cast<double>(node.cost) + edge_cost);
                    if (candidate < field.distance[static_cast<size_t>(nflat)]) {
                        field.distance[static_cast<size_t>(nflat)] = candidate;
                        field.predecessor[static_cast<size_t>(nflat)] =
                            static_cast<std::int32_t>(node.flat);
                        field.source_id[static_cast<size_t>(nflat)] =
                            field.source_id[static_cast<size_t>(node.flat)];
                        pq.push({candidate, nflat});
                    }
                }
            }
        }
    }

    return field;
}

std::vector<Eigen::Vector3i> WeightedDijkstraV6::tracePath(
    const DistanceField& field,
    const Eigen::Vector3i& sink)
{
    const VoxelizationV6::Grid& grid = field.grid;
    const long long total = totalVoxelCount(grid.dims);
    if (total <= 0 ||
        static_cast<long long>(field.distance.size()) != total ||
        static_cast<long long>(field.predecessor.size()) != total) {
        throw std::invalid_argument("WeightedDijkstraV6: invalid DistanceField sizes");
    }
    if (!inBounds(grid, sink.x(), sink.y(), sink.z())) return {};

    int flat = grid.idx(sink.x(), sink.y(), sink.z());
    if (!std::isfinite(field.distance[static_cast<size_t>(flat)])) return {};

    std::vector<Eigen::Vector3i> path;
    path.reserve(128);
    while (flat != NO_PRED) {
        path.push_back(flatToIJK(grid, flat));
        flat = field.predecessor[static_cast<size_t>(flat)];
    }
    return path;
}

}  // namespace cslc
