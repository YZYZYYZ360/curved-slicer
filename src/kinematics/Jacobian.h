#pragma once

#include "kinematics/KR4Model.h"

#include <Eigen/Dense>

namespace cslc {

struct SVDResult {
    Eigen::VectorXd sigma;
    Eigen::MatrixXd U;
    Eigen::MatrixXd V;
    double cond = 0.0;
};

class Jacobian {
public:
    explicit Jacobian(KR4Model model = KR4Model{});

    Eigen::Matrix<double, 6, 6> geometricJacobian(const KR4Model::JointVector& q_rad) const;
    SVDResult svd(const Eigen::Matrix<double, 6, 6>& J) const;

private:
    KR4Model model_;
};

}  // namespace cslc
