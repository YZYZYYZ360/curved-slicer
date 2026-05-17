#include "kinematics/AnalyticIK.h"
#include "kinematics/KR4Model.h"
#include "kinematics/forward_kin.h"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

double normalizeAngle(double rad)
{
    while (rad > M_PI) {
        rad -= 2.0 * M_PI;
    }
    while (rad < -M_PI) {
        rad += 2.0 * M_PI;
    }
    return rad;
}

double wrappedInfNorm(const cslc::KR4Model::JointVector& lhs, const cslc::KR4Model::JointVector& rhs)
{
    double max_error = 0.0;
    for (int i = 0; i < 6; ++i) {
        max_error = std::max(max_error, std::abs(normalizeAngle(lhs(i) - rhs(i))));
    }
    return max_error;
}

void assertWristCenterFromToolOffset(const cslc::KR4Model& model, const cslc::KR4Model::JointVector& q)
{
    const Eigen::Matrix4d T_tcp = model.forwardKinematics(q);
    const Eigen::Vector3d wrist_from_tcp =
        T_tcp.block<3, 1>(0, 3) - model.toolOffset() * T_tcp.block<3, 1>(0, 2);

    const cslc::JointConfig q_deg = cslc::KR4Model::toJointConfigDeg(q);
    const Eigen::Matrix4d T3 = cslc::forwardKinMatrix_partial(q_deg, model.dhParams(), 3);
    const Eigen::Vector4d wrist_local(0.0, 0.0, model.dhParams().d_mm[3], 1.0);
    const Eigen::Vector3d wrist_expected = (T3 * wrist_local).head<3>();

    require((wrist_from_tcp - wrist_expected).norm() < 1e-9,
            "wrist center must use KR4Model::toolOffset() from DH parameters");
}

}  // namespace

int main()
{
    try {
        const cslc::KR4Model model;
        const cslc::AnalyticIK ik(model);
        const auto& limits = model.jointLimits();

        require(std::abs(model.toolOffset() -
                         (model.dhParams().flange_z_mm + model.dhParams().tool_z_mm)) < 1e-12,
                "toolOffset should come from DH flange_z_mm + tool_z_mm");
        require(std::abs(model.toolOffset() - 12.28) < 1e-12,
                "KR4 default toolOffset should be 12.28mm");

        cslc::KR4Model::JointVector home;
        home << 0.0, 9.95 * M_PI / 180.0, -62.26 * M_PI / 180.0,
            0.0, -37.69 * M_PI / 180.0, 0.0;
        assertWristCenterFromToolOffset(model, home);

        std::mt19937 rng(42);
        int any_valid = 0;
        int exact_recovered = 0;
        int at_least_four_valid = 0;
        int branch_hist[9] = {};

        constexpr int kSamples = 1000;
        for (int sample = 0; sample < kSamples; ++sample) {
            cslc::KR4Model::JointVector q;
            for (int axis = 0; axis < 6; ++axis) {
                const double margin = 1.0 * M_PI / 180.0;
                std::uniform_real_distribution<double> dist(limits(axis, 0) + margin, limits(axis, 1) - margin);
                q(axis) = dist(rng);
            }

            const Eigen::Matrix4d T = model.forwardKinematics(q);
            const auto branches = ik.solveAllBranches(T);

            int valid_count = 0;
            bool exact = false;
            for (const cslc::IKBranch& branch : branches) {
                if (!branch.valid) {
                    continue;
                }
                ++valid_count;
                if (wrappedInfNorm(branch.q, q) < 1e-4) {
                    exact = true;
                }
            }

            if (valid_count > 0) {
                ++any_valid;
            }
            if (valid_count >= 4) {
                ++at_least_four_valid;
            }
            if (exact) {
                ++exact_recovered;
            }
            ++branch_hist[std::min(valid_count, 8)];
        }

        std::cout << "kr4_roundtrip random1000 any_valid=" << any_valid
                  << " ge4_valid=" << at_least_four_valid
                  << " exact_recovered=" << exact_recovered << '\n';
        std::cout << "kr4_roundtrip branch_hist";
        for (int i = 0; i <= 8; ++i) {
            std::cout << " " << i << ":" << branch_hist[i];
        }
        std::cout << '\n';

        require(any_valid >= 995, "expected >=995/1000 targets with at least one valid branch");
        require(at_least_four_valid >= 800, "expected >=800/1000 targets with at least four valid branches");
        require(exact_recovered >= 990, "expected >=990/1000 exact branch recoveries");

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "test_kr4_roundtrip FAILED: " << ex.what() << '\n';
        return 1;
    }
}
