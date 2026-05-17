#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <vector>

namespace cslc {

class VoxelizationV6 {
public:
    enum class Occ : std::uint8_t {
        OUTSIDE = 0,
        INSIDE = 1,
        BOUNDARY = 2,
    };

    struct Grid {
        Eigen::Vector3d origin_mm = Eigen::Vector3d::Zero();
        double spacing_mm = 1.0;
        Eigen::Vector3i dims = Eigen::Vector3i::Zero();
        std::vector<Occ> occupancy;
        std::vector<Eigen::Vector3i> boundary_voxels;

        inline int idx(int i, int j, int k) const
        {
            return i + dims.x() * j + dims.x() * dims.y() * k;
        }

        inline Eigen::Vector3d center(int i, int j, int k) const
        {
            return origin_mm +
                spacing_mm * Eigen::Vector3d(i + 0.5, j + 0.5, k + 0.5);
        }
    };

    enum class BoundaryConn { CONN_6, CONN_18 };

    struct Options {
        enum class InsideTest {
            WINDING_NUMBER,
            RAY_PARITY,
        };

        double spacing_mm = 1.0;
        double bbox_padding_mm = 2.0;
        BoundaryConn boundary_conn = BoundaryConn::CONN_6;
        InsideTest inside_test = InsideTest::WINDING_NUMBER;
    };

    static Grid voxelize(
        const Eigen::MatrixXd& V_repaired,
        const Eigen::MatrixXi& F_repaired,
        const Options& opt);
};

}  // namespace cslc
