#include "kinematics/reachability.h"
#include "kinematics/forward_kin.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

void test_home()
{
    using namespace cslc;
    KR4DHParams dh;
    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
    CartPose pose = forwardKin(home, dh);

    auto r = checkReachability(pose, home, dh);
    require(r.status == ReachStatus::OK, "home: status=OK");
    require(!r.arm_flip, "home: no arm_flip");
    require(r.num_valid >= 1, "home: at least 1 solution");
    std::cout << "  home PASSED\n";
}

void test_out_of_workspace()
{
    using namespace cslc;
    KR4DHParams dh;
    JointConfig ref{{0, 0, 0, 0, 0, 0}};

    // Far away
    CartPose far_pose;
    far_pose.X = 2000; far_pose.Y = 0; far_pose.Z = 330;
    auto r1 = checkReachability(far_pose, ref, dh);
    require(r1.status == ReachStatus::OutOfWorkspace, "far: OutOfWorkspace");

    // Near Z-axis (r < r_min)
    CartPose near_z;
    near_z.X = 10; near_z.Y = 10; near_z.Z = 500;
    auto r2 = checkReachability(near_z, ref, dh);
    require(r2.status == ReachStatus::OutOfWorkspace, "near_z: OutOfWorkspace");

    std::cout << "  out_of_workspace PASSED\n";
}

void test_wrist_singularity()
{
    using namespace cslc;
    KR4DHParams dh;
    // IK solver filters |J5| < 5°. Use J5=6° (just above filter) and set
    // reachability singularity threshold to 10° (stricter than IK's 5°).
    // This tests that reachability detects near-singular poses that the IK
    // solver marginally accepts.
    JointConfig sing{{0.0, -60.0, 30.0, 0.0, 6.0, 0.0}};
    JointConfig ref{{0.0, -60.0, 30.0, 0.0, 6.0, 0.0}};
    CartPose pose = forwardKin(sing, dh);

    double r_tcp = std::sqrt(pose.X * pose.X + pose.Y * pose.Y);
    require(r_tcp > 100.0 && r_tcp < 600.0, "wrist_sing: in workspace");

    ReachabilityParams params;
    params.singularity_a5_deg = 10.0;  // stricter than IK's 5° filter
    auto r = checkReachability(pose, ref, dh, params);
    require(r.status == ReachStatus::NearSingularity, "wrist_sing: NearSingularity");
    require(r.wrist_sing, "wrist_sing: wrist_sing=true");

    std::cout << "  wrist_singularity PASSED\n";
}

void test_shoulder_singularity()
{
    using namespace cslc;
    KR4DHParams dh;
    // Construct config with A2+A3 = 90° → shoulder singularity
    // A2=30° (within [-195,40]), A3=60° (within [-115,150])
    JointConfig sing{{0.0, 30.0, 60.0, 0.0, -45.0, 0.0}};
    JointConfig ref{{0.0, 30.0, 60.0, 0.0, -45.0, 0.0}};
    CartPose pose = forwardKin(sing, dh);

    double r_tcp = std::sqrt(pose.X * pose.X + pose.Y * pose.Y);
    require(r_tcp > 100.0 && r_tcp < 600.0, "shoulder_sing: in workspace");

    auto r = checkReachability(pose, ref, dh);
    require(r.status == ReachStatus::NearSingularity, "shoulder_sing: NearSingularity");
    require(r.shoulder_sing, "shoulder_sing: shoulder_sing=true");

    std::cout << "  shoulder_singularity PASSED\n";
}

int main()
{
    try {
        test_home();
        test_out_of_workspace();
        test_wrist_singularity();
        test_shoulder_singularity();
        std::cout << "reachability_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "reachability_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
