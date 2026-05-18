#pragma once

#include "sdf/SDFV6.h"
#include "voxel/VoxelizationV6.h"

#include <Eigen/Dense>

#include <cstdint>
#include <limits>
#include <vector>

namespace cslc {

class WeightedDijkstraV6 {
public:
    static constexpr float UNREACHABLE = std::numeric_limits<float>::infinity();
    static constexpr std::int32_t NO_PRED = -1;
    static constexpr std::uint8_t UNREACHABLE_SOURCE = 255;

    struct DistanceField {
        VoxelizationV6::Grid grid;
        std::vector<float> distance;
        std::vector<std::int32_t> predecessor;
        std::vector<std::uint8_t> source_id;
    };

    struct PenaltyOptions {
        bool enable_clearance = true;
        double clearance_threshold_mm = 1.0;
        double clearance_penalty_weight = 10.0;
    };

    struct Options {
        PenaltyOptions penalty;
        bool walk_boundary = true;
    };

    static DistanceField computeDistanceField(
        const VoxelizationV6::Grid& grid,
        const SDFV6::Field& sdf,
        const std::vector<Eigen::Vector3i>& sources,
        const Options& opt);

    static std::vector<Eigen::Vector3i> tracePath(
        const DistanceField& field,
        const Eigen::Vector3i& sink);
};

}  // namespace cslc
