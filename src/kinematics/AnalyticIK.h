#pragma once

#include "kinematics/KR4Model.h"

#include <array>

namespace cslc {

struct IKBranch {
    // Branch id bits: bit0=shoulder (0=L, 1=R), bit1=elbow (0=U, 1=D),
    // bit2=wrist (0=F, 1=N). q is radians in KR4 joint order A1..A6.
    KR4Model::JointVector q = KR4Model::JointVector::Zero();
    bool valid = false;
    double cond_num = 0.0;
};

class AnalyticIK {
public:
    explicit AnalyticIK(KR4Model model = KR4Model{});

    std::array<IKBranch, 8> solveAllBranches(const Eigen::Matrix4d& T_target) const;

private:
    KR4Model model_;
};

}  // namespace cslc
