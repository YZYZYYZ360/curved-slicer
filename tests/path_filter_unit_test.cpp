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

void test_arc_positive()
{
    using namespace cslc;
    KR4DHParams dh;

    // Use home config to derive a known-valid G direction.
    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
    CartPose home_pose = forwardKin(home, dh);
    Eigen::Matrix3d R_home = kukaABCToRotMat(home_pose.A, home_pose.B, home_pose.C);
    Eigen::Vector3d tool_z = R_home.col(2);
    Eigen::Vector3d G_vec = -tool_z;
    Vec3 G(G_vec.x(), G_vec.y(), G_vec.z());

    // Generate 10 reachable poses by varying J1 (arm swing).
    // Each config has the same J2-J6, so the tool orientation is identical.
    // Only the TCP position changes (arc in XY plane).
    const int N = 10;
    std::vector<Vec3> points(N);
    std::vector<Vec3> tangents(N);

    for (int i = 0; i < N; ++i) {
        double j1_deg = -30.0 + 60.0 * i / (N - 1);  // -30° to +30°
        JointConfig cfg{{j1_deg, 9.95, -62.26, 0.0, -37.69, 0.0}};
        CartPose p = forwardKin(cfg, dh);
        points[i] = Vec3(p.X, p.Y, p.Z);

        // Tangent: direction of TCP motion as J1 changes (approx: perpendicular to radius)
        double a = j1_deg * M_PI / 180.0;
        tangents[i] = Vec3(-std::sin(a), std::cos(a), 0.0);
    }

    auto result = filterReachablePathPoints(points, tangents, G, dh);

    std::cout << "  arc_positive: " << result.num_reachable << "/" << N << "\n";

    require(result.num_reachable == N, "arc: all 10 points reachable");

    for (int i = 0; i < N; ++i) {
        require(result.statuses[i] == ReachStatus::OK,
                "arc: each point status=OK");
    }

    std::cout << "  arc_positive PASSED\n";
}

void test_oow_negative()
{
    using namespace cslc;
    KR4DHParams dh;

    // Same G direction as the positive test.
    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
    CartPose home_pose = forwardKin(home, dh);
    Eigen::Matrix3d R_home = kukaABCToRotMat(home_pose.A, home_pose.B, home_pose.C);
    Eigen::Vector3d tool_z = R_home.col(2);
    Eigen::Vector3d G_vec = -tool_z;
    Vec3 G(G_vec.x(), G_vec.y(), G_vec.z());

    // 3 points: first 2 reachable (from FK), last one out of workspace.
    CartPose p0 = forwardKin({{-20.0, 9.95, -62.26, 0.0, -37.69, 0.0}}, dh);
    CartPose p1 = forwardKin({{20.0, 9.95, -62.26, 0.0, -37.69, 0.0}}, dh);

    std::vector<Vec3> points = {
        Vec3(p0.X, p0.Y, p0.Z),
        Vec3(p1.X, p1.Y, p1.Z),
        Vec3(2000.0, 0.0, 330.0),  // far away → OutOfWorkspace
    };

    double a0 = -20.0 * M_PI / 180.0;
    double a1 = 20.0 * M_PI / 180.0;
    std::vector<Vec3> tangents = {
        Vec3(-std::sin(a0), std::cos(a0), 0.0),
        Vec3(-std::sin(a1), std::cos(a1), 0.0),
        Vec3(0.0, 1.0, 0.0),
    };

    auto result = filterReachablePathPoints(points, tangents, G, dh);

    require(result.num_reachable == 2, "oow: 2 of 3 reachable");
    require(result.statuses[0] == ReachStatus::OK, "oow: point 0 OK");
    require(result.statuses[1] == ReachStatus::OK, "oow: point 1 OK");
    require(result.statuses[2] == ReachStatus::OutOfWorkspace, "oow: point 2 OutOfWorkspace");

    std::cout << "  oow_negative PASSED\n";
}

int main()
{
    try {
        test_arc_positive();
        test_oow_negative();
        std::cout << "path_filter_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "path_filter_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
