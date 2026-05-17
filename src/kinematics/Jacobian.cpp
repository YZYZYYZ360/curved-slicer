#include "kinematics/Jacobian.h"

#include "kinematics/forward_kin.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cslc {
namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

}  // namespace

Jacobian::Jacobian(KR4Model model)
    : model_(model)
{
}

Eigen::Matrix<double, 6, 6> Jacobian::geometricJacobian(const KR4Model::JointVector& q_rad) const
{
    const KR4DHParams& dh = model_.dhParams();
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::array<Eigen::Vector3d, 6> origins;
    std::array<Eigen::Vector3d, 6> axes;

    for (int i = 0; i < 6; ++i) {
        origins[static_cast<std::size_t>(i)] = T.block<3, 1>(0, 3);
        axes[static_cast<std::size_t>(i)] = T.block<3, 1>(0, 2);
        T = T * dhMatrix(q_rad(i) * kRad2Deg, dh.d_mm[i], dh.a_mm[i], dh.alpha_rad[i]);
    }

    Eigen::Matrix4d T_tool = Eigen::Matrix4d::Identity();
    T_tool(2, 3) = model_.toolOffset();
    const Eigen::Vector3d tcp = (T * T_tool).block<3, 1>(0, 3);

    Eigen::Matrix<double, 6, 6> J;
    for (int i = 0; i < 6; ++i) {
        const Eigen::Vector3d z = axes[static_cast<std::size_t>(i)];
        const Eigen::Vector3d o = origins[static_cast<std::size_t>(i)];
        J.block<3, 1>(0, i) = z.cross(tcp - o);
        J.block<3, 1>(3, i) = z;
    }
    return J;
}

SVDResult Jacobian::svd(const Eigen::Matrix<double, 6, 6>& J) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> solver(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

    SVDResult result;
    result.sigma = solver.singularValues();
    result.U = solver.matrixU();
    result.V = solver.matrixV();

    const double sigma_max = result.sigma.size() > 0 ? result.sigma(0) : 0.0;
    const double sigma_min = result.sigma.size() > 0 ? result.sigma(result.sigma.size() - 1) : 0.0;
    result.cond = sigma_min <= 1e-12 ? std::numeric_limits<double>::infinity() : sigma_max / sigma_min;
    return result;
}

}  // namespace cslc
