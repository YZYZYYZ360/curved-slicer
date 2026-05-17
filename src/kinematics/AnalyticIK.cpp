#include "kinematics/AnalyticIK.h"

#include "kinematics/Jacobian.h"
#include "kinematics/forward_kin.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cslc {
namespace {

constexpr double kPi = M_PI;
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

double normalizeAngle(double rad)
{
    while (rad > kPi) {
        rad -= 2.0 * kPi;
    }
    while (rad < -kPi) {
        rad += 2.0 * kPi;
    }
    return rad;
}

double angleDiff(double lhs, double rhs)
{
    return normalizeAngle(lhs - rhs);
}

bool equivalentWithinLimits(double rad, double min_rad, double max_rad, double* adjusted)
{
    bool found = false;
    double best = rad;
    double best_abs = std::numeric_limits<double>::infinity();
    for (int k = -2; k <= 2; ++k) {
        const double candidate = rad + static_cast<double>(k) * 2.0 * kPi;
        if (candidate < min_rad - 1e-9 || candidate > max_rad + 1e-9) {
            continue;
        }
        const double abs_value = std::abs(candidate);
        if (!found || abs_value < best_abs) {
            found = true;
            best = candidate;
            best_abs = abs_value;
        }
    }
    if (found) {
        *adjusted = best;
    }
    return found;
}

struct J2J3Solution {
    double q2 = 0.0;
    double q3 = 0.0;
    bool valid = false;
};

J2J3Solution solveJ2J3(double radial, double z, const KR4DHParams& dh, int elbow)
{
    const double a2 = dh.a_mm[1];
    const double a3 = dh.a_mm[2];
    const double d4 = dh.d_mm[3];
    const double l3 = std::sqrt(a3 * a3 + d4 * d4);
    const double delta = std::atan2(a3, d4);
    const double distance = std::sqrt(radial * radial + z * z);
    if (distance < 1e-12) {
        return {};
    }

    const double w = radial * radial + z * z + l3 * l3 - a2 * a2;
    double rhs = w / (2.0 * l3 * distance);
    if (rhs < -1.0 - 1e-9 || rhs > 1.0 + 1e-9) {
        return {};
    }
    rhs = std::clamp(rhs, -1.0, 1.0);

    const double gamma = std::atan2(-z, radial);
    const double asin_value = std::asin(rhs);
    const double alpha = elbow == 0 ? asin_value - gamma : kPi - asin_value - gamma;

    const double a = z + l3 * std::cos(alpha);
    const double b = radial - l3 * std::sin(alpha);
    if (std::abs(a) < 1e-12 && std::abs(b) < 1e-12) {
        return {};
    }

    J2J3Solution solution;
    solution.q2 = std::atan2(a, b);
    const double u = alpha - delta;
    solution.q3 = solution.q2 - u;
    solution.valid = true;
    return solution;
}

bool decomposeWrist(const Eigen::Matrix3d& R36, int wrist, double* q4, double* q5, double* q6)
{
    const double cos_q5 = std::clamp(R36(2, 2), -1.0, 1.0);
    const double sin_abs = std::sqrt(std::max(0.0, 1.0 - cos_q5 * cos_q5));
    if (sin_abs < 1e-10) {
        return false;
    }

    const double sin_q5 = wrist == 0 ? sin_abs : -sin_abs;
    *q5 = std::atan2(sin_q5, cos_q5);
    *q4 = std::atan2(-R36(1, 2) / sin_q5, -R36(0, 2) / sin_q5);
    *q6 = std::atan2(-R36(2, 1) / sin_q5, R36(2, 0) / sin_q5);
    return true;
}

bool adjustAllToLimits(KR4Model::JointVector* q, const KR4Model::JointLimits& limits)
{
    for (int i = 0; i < 6; ++i) {
        double adjusted = 0.0;
        if (!equivalentWithinLimits((*q)(i), limits(i, 0), limits(i, 1), &adjusted)) {
            return false;
        }
        (*q)(i) = adjusted;
    }
    return true;
}

bool poseMatches(const Eigen::Matrix4d& lhs, const Eigen::Matrix4d& rhs)
{
    const double pos_error = (lhs.block<3, 1>(0, 3) - rhs.block<3, 1>(0, 3)).norm();
    const double rot_error = (lhs.block<3, 3>(0, 0) - rhs.block<3, 3>(0, 0)).norm();
    return pos_error < 1e-5 && rot_error < 1e-8;
}

}  // namespace

AnalyticIK::AnalyticIK(KR4Model model)
    : model_(model)
{
}

std::array<IKBranch, 8> AnalyticIK::solveAllBranches(const Eigen::Matrix4d& T_target) const
{
    std::array<IKBranch, 8> branches{};
    for (IKBranch& branch : branches) {
        branch.q.setZero();
        branch.valid = false;
        branch.cond_num = std::numeric_limits<double>::infinity();
    }

    const KR4DHParams& dh = model_.dhParams();
    const Eigen::Matrix3d R06 = T_target.block<3, 3>(0, 0);
    const Eigen::Vector3d tcp = T_target.block<3, 1>(0, 3);
    const Eigen::Vector3d wrist_center = tcp - model_.toolOffset() * R06.col(2);
    const double q1_base = std::atan2(wrist_center.y(), wrist_center.x());
    const Jacobian jacobian(model_);

    for (int shoulder = 0; shoulder < 2; ++shoulder) {
        const double q1 = normalizeAngle(q1_base + (shoulder == 0 ? 0.0 : kPi));
        const double radial = std::cos(q1) * wrist_center.x() + std::sin(q1) * wrist_center.y();
        const double z = wrist_center.z() - dh.d_mm[0];

        for (int elbow = 0; elbow < 2; ++elbow) {
            const J2J3Solution j23 = solveJ2J3(radial, z, dh, elbow);
            if (!j23.valid) {
                continue;
            }

            JointConfig q123_deg;
            q123_deg.q_deg[0] = q1 * kRad2Deg;
            q123_deg.q_deg[1] = j23.q2 * kRad2Deg;
            q123_deg.q_deg[2] = j23.q3 * kRad2Deg;
            const Eigen::Matrix3d R03 = forwardKinMatrix_partial(q123_deg, dh, 3).block<3, 3>(0, 0);
            const Eigen::Matrix3d R36 = R03.transpose() * R06;

            for (int wrist = 0; wrist < 2; ++wrist) {
                const int branch_id = shoulder | (elbow << 1) | (wrist << 2);
                double q4 = 0.0;
                double q5 = 0.0;
                double q6 = 0.0;
                if (!decomposeWrist(R36, wrist, &q4, &q5, &q6)) {
                    continue;
                }

                KR4Model::JointVector q;
                q << q1, j23.q2, j23.q3, q4, q5, q6;
                for (int i = 0; i < 6; ++i) {
                    q(i) = normalizeAngle(q(i));
                }
                if (!adjustAllToLimits(&q, model_.jointLimits())) {
                    continue;
                }

                const Eigen::Matrix4d T_check = model_.forwardKinematics(q);
                if (!poseMatches(T_check, T_target)) {
                    continue;
                }

                const SVDResult svd_result = jacobian.svd(jacobian.geometricJacobian(q));
                branches[static_cast<std::size_t>(branch_id)].q = q;
                branches[static_cast<std::size_t>(branch_id)].valid = true;
                branches[static_cast<std::size_t>(branch_id)].cond_num = svd_result.cond;
            }
        }
    }

    return branches;
}

}  // namespace cslc
