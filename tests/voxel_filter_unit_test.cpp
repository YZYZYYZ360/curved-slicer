#include "kinematics/reachability.h"
#include "kinematics/forward_kin.h"
#include "path/pose_from_path.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

void test_5x5x5_grid()
{
    using namespace cslc;
    KR4DHParams dh;

    // Use home config to get a known-reachable TCP position and tool orientation.
    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
    CartPose home_pose = forwardKin(home, dh);

    // Extract tool_z from the home pose rotation matrix.
    Eigen::Matrix3d R_home = kukaABCToRotMat(home_pose.A, home_pose.B, home_pose.C);
    Eigen::Vector3d tool_z = R_home.col(2);
    // G = -tool_z (outward surface normal)
    Eigen::Vector3d G_vec = -tool_z;
    Vec3 G(G_vec.x(), G_vec.y(), G_vec.z());

    // Build a 5×5×5 grid centered at the home TCP position.
    double spacing = 10.0;
    int nx = 5, ny = 5, nz = 5;
    Vec3 center(home_pose.X, home_pose.Y, home_pose.Z);
    double half = spacing * 2.5;  // 25mm
    AABB bbox{{center.x - half, center.y - half, center.z - half},
              {center.x + half, center.y + half, center.z + half}};

    VoxelGrid grid(bbox, spacing, nx, ny, nz);

    // Mark all voxels occupied.
    for (int x = 0; x < nx; ++x)
        for (int y = 0; y < ny; ++y)
            for (int z = 0; z < nz; ++z)
                grid.setOccupied(x, y, z, true);

    require(grid.occupiedVoxels().size() == 125, "5x5x5: 125 occupied voxels");

    auto reachable = filterReachableVoxels(grid, G, dh, {}, home);

    int total = 125;
    int ok = static_cast<int>(reachable.size());
    double rate = static_cast<double>(ok) / total;

    std::cout << "  5x5x5 grid: " << ok << "/" << total
              << " reachable (" << static_cast<int>(rate * 100) << "%)\n";

    require(ok > 0, "5x5x5: at least 1 reachable voxel");
    std::cout << "  5x5x5_grid PASSED\n";
}

int main()
{
    try {
        test_5x5x5_grid();
        std::cout << "voxel_filter_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "voxel_filter_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
