#pragma once

#include "voxel/VoxelizationV6.h"

#include <Eigen/Dense>

#include <vector>

namespace cslc {

class SDFV6 {
public:
    struct Field {
        VoxelizationV6::Grid grid;
        std::vector<float> distance_mm;
    };

    struct Options {
        enum class Method {
            LIBIGL_SIGNED_DISTANCE,
            BVH_CLOSEST_PLUS_OCCUPANCY,
        };

        Method method = Method::LIBIGL_SIGNED_DISTANCE;
        double narrow_band_mm = -1.0;
        bool negative_inside = true;
    };

    static Field compute(
        const Eigen::MatrixXd& V_repaired,
        const Eigen::MatrixXi& F_repaired,
        const VoxelizationV6::Grid& grid,
        const Options& opt);
};

}  // namespace cslc
