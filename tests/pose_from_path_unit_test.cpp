#include "path/pose_from_path.h"
#include "kinematics/forward_kin.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

static double vec_err(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    return (a - b).norm();
}

void test_orthogonal()
{
    using namespace cslc;
    // G = (0,0,1), tangent = (1,0,0) → tool_z = (0,0,-1), tool_x = (1,0,0)
    Eigen::Vector3d G(0, 0, 1);
    Eigen::Vector3d t(1, 0, 0);
    Eigen::Matrix3d R = constructToolFrame(t, G);

    require(vec_err(R.col(2), Eigen::Vector3d(0, 0, -1)) < 1e-9,
            "orthogonal: tool_z = (0,0,-1)");
    require(vec_err(R.col(0), Eigen::Vector3d(1, 0, 0)) < 1e-9,
            "orthogonal: tool_x = (1,0,0)");
    require(std::abs(R.determinant() - 1.0) < 1e-9,
            "orthogonal: det(R) = 1");

    // toolFrameToCartPose
    Eigen::Vector3d pt(100, 200, 300);
    CartPose pose = toolFrameToCartPose(pt, R);
    require(std::abs(pose.X - 100) < 1e-9, "orthogonal: X");
    require(std::abs(pose.Y - 200) < 1e-9, "orthogonal: Y");
    require(std::abs(pose.Z - 300) < 1e-9, "orthogonal: Z");

    // Verify round-trip: ABC → R should recover the same rotation
    Eigen::Matrix3d R_rt = kukaABCToRotMat(pose.A, pose.B, pose.C);
    require((R - R_rt).norm() < 1e-6, "orthogonal: ABC round-trip");

    std::cout << "  orthogonal PASSED\n";
}

void test_non_orthogonal()
{
    using namespace cslc;
    // G = (0,0,1), tangent = (1,0,1) → tool_z = (0,0,-1)
    // tool_x = proj of (1,0,1) onto ⊥(0,0,-1) = (1,0,0), normalized = (1,0,0)
    Eigen::Vector3d G(0, 0, 1);
    Eigen::Vector3d t(1, 0, 1);
    Eigen::Matrix3d R = constructToolFrame(t, G);

    require(vec_err(R.col(2), Eigen::Vector3d(0, 0, -1)) < 1e-9,
            "non_orth: tool_z = (0,0,-1)");
    require(vec_err(R.col(0), Eigen::Vector3d(1, 0, 0)) < 1e-9,
            "non_orth: tool_x = (1,0,0)");
    require(std::abs(R.determinant() - 1.0) < 1e-9,
            "non_orth: det(R) = 1");

    std::cout << "  non_orthogonal PASSED\n";
}

void test_near_parallel()
{
    using namespace cslc;
    // G = (0,0,1), tangent ≈ (0,0,1) → near-degenerate
    // Should return identity (degenerate fallback)
    Eigen::Vector3d G(0, 0, 1);
    Eigen::Vector3d t(0.001, 0, 1.0);
    Eigen::Matrix3d R = constructToolFrame(t, G);

    // With such a small perpendicular component, tool_x projection is tiny.
    // If norm < 1e-6, returns identity. Otherwise it should still produce
    // a valid rotation matrix.
    double xn = Eigen::Vector3d(0.001, 0, 0).norm();  // = 0.001 > 1e-6
    if (xn < 1e-6) {
        require((R - Eigen::Matrix3d::Identity()).norm() < 1e-9,
                "near_parallel: returns identity");
    } else {
        require(std::abs(R.determinant() - 1.0) < 1e-9,
                "near_parallel: det(R) = 1");
        // tool_z should still be (0,0,-1)
        require(vec_err(R.col(2), Eigen::Vector3d(0, 0, -1)) < 1e-9,
                "near_parallel: tool_z = (0,0,-1)");
    }

    // Truly parallel: tangent = (0,0,1) → should return identity
    Eigen::Vector3d t_par(0, 0, 1);
    Eigen::Matrix3d R_par = constructToolFrame(t_par, G);
    require((R_par - Eigen::Matrix3d::Identity()).norm() < 1e-9,
            "near_parallel: exact parallel → identity");

    std::cout << "  near_parallel PASSED\n";
}

int main()
{
    try {
        test_orthogonal();
        test_non_orthogonal();
        test_near_parallel();
        std::cout << "pose_from_path_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "pose_from_path_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
