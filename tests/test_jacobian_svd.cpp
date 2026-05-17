#include "kinematics/Jacobian.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

cslc::KR4Model::JointVector deg(double a1, double a2, double a3, double a4, double a5, double a6)
{
    cslc::KR4Model::JointVector q;
    q << a1, a2, a3, a4, a5, a6;
    return q * (M_PI / 180.0);
}

double conditionFor(const cslc::Jacobian& jacobian, const cslc::KR4Model::JointVector& q)
{
    return jacobian.svd(jacobian.geometricJacobian(q)).cond;
}

}  // namespace

int main()
{
    try {
        const cslc::Jacobian jacobian;

        const cslc::KR4Model::JointVector q1 = deg(20.0, -45.0, 35.0, 30.0, 45.0, -20.0);
        const cslc::KR4Model::JointVector q2 = deg(-55.0, -30.0, 80.0, -40.0, -55.0, 70.0);
        const cslc::KR4Model::JointVector q3 = deg(80.0, -70.0, 55.0, 90.0, 35.0, -110.0);
        const cslc::KR4Model::JointVector q_singular = deg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        const double c1 = conditionFor(jacobian, q1);
        const double c2 = conditionFor(jacobian, q2);
        const double c3 = conditionFor(jacobian, q3);
        const double cs = conditionFor(jacobian, q_singular);

        std::cout << "jacobian_svd cond_non_singular_1=" << c1 << '\n';
        std::cout << "jacobian_svd cond_non_singular_2=" << c2 << '\n';
        std::cout << "jacobian_svd cond_non_singular_3=" << c3 << '\n';
        std::cout << "jacobian_svd cond_elbow_straight_singular=" << cs << '\n';

        require(c1 < 1e4, "non-singular case 1 should have cond < 1e4");
        require(c2 < 1e4, "non-singular case 2 should have cond < 1e4");
        require(c3 < 1e4, "non-singular case 3 should have cond < 1e4");
        require(cs > 1e8, "elbow-straight singular case should have cond > 1e8");

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "test_jacobian_svd FAILED: " << ex.what() << '\n';
        return 1;
    }
}
