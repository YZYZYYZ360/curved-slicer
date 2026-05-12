#include "trajectory/poly5_smoother.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

void test_poly5_boundary()
{
    using namespace cslc;
    auto c = computePoly5(0.0, 10.0, 1.0);

    require(std::abs(c.eval(0)) < 1e-9, "q(0) = 0");
    require(std::abs(c.eval(1) - 10) < 1e-9, "q(T) = 10");
    require(std::abs(c.evalDot(0)) < 1e-9, "q'(0) = 0");
    require(std::abs(c.evalDot(1)) < 1e-9, "q'(T) = 0");
    require(std::abs(c.evalDotDot(0)) < 1e-9, "q''(0) = 0");
    require(std::abs(c.evalDotDot(1)) < 1e-9, "q''(T) = 0");

    // Midpoint: q(0.5) should be 5 (symmetric)
    require(std::abs(c.eval(0.5) - 5.0) < 1e-9, "q(0.5) = 5 (symmetric)");

    // Negative direction
    auto c2 = computePoly5(10.0, 0.0, 2.0);
    require(std::abs(c2.eval(0) - 10) < 1e-9, "neg: q(0) = 10");
    require(std::abs(c2.eval(2)) < 1e-9, "neg: q(T) = 0");
    require(std::abs(c2.evalDot(0)) < 1e-9, "neg: q'(0) = 0");
    require(std::abs(c2.evalDot(2)) < 1e-9, "neg: q'(T) = 0");

    std::cout << "  test_poly5_boundary PASSED\n";
}

void test_dual_time_constraint()
{
    using namespace cslc;
    TrajectoryParams params;
    params.target_line_speed_mm_per_s = 5.0;
    params.max_joint_velocity_deg_per_s = 200.0;

    // Case 1: Cartesian dominant (TCP moves 10mm, joints barely change)
    {
        PathPointWithJoints a, b;
        a.cart_pos = Eigen::Vector3d(0, 0, 0);
        a.joint.q_deg = {0, 0, 0, 0, 0, 0};
        b.cart_pos = Eigen::Vector3d(10, 0, 0);
        b.joint.q_deg = {1, 0, 0, 0, 0, 0};

        double T = computeSegmentDuration(a, b, params);
        double dt_cart = 10.0 / 5.0;
        require(std::abs(T - dt_cart) < 1e-9, "cart dominant: T = d_cart/v");
        std::cout << "  cart_dominant: T=" << T << "s (expected 2.0)\n";
    }

    // Case 2: Joint dominant (TCP barely moves, joint flips 30°)
    {
        PathPointWithJoints a, b;
        a.cart_pos = Eigen::Vector3d(100, 0, 500);
        a.joint.q_deg = {0, 0, 0, 0, 0, 0};
        b.cart_pos = Eigen::Vector3d(100.01, 0, 500);
        b.joint.q_deg = {30, 0, 0, 0, 0, 0};

        double T = computeSegmentDuration(a, b, params);
        double dt_joint = 30.0 / 200.0;
        require(std::abs(T - dt_joint) < 1e-9, "joint dominant: T = max_dq/v_joint");
        std::cout << "  joint_dominant: T=" << T << "s (expected 0.15)\n";
    }

    std::cout << "  test_dual_time_constraint PASSED\n";
}

int main()
{
    try {
        test_poly5_boundary();
        test_dual_time_constraint();
        std::cout << "poly5_smoother_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "poly5_smoother_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
