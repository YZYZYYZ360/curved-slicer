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

int main()
{
    try {
        test_poly5_boundary();
        std::cout << "poly5_smoother_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "poly5_smoother_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
