#pragma once

#include "kinematics/cart_pose.h"
#include "kinematics/dh_params.h"

#include <array>

namespace cslc {

// Reachability parameters (defaults from v4 spec §12.3)
struct ReachabilityParams {
    double workspace_r_min_mm          = 100.0;
    double workspace_r_max_mm          = 600.0;
    double singularity_a5_deg          = 5.0;
    double singularity_shoulder_deg    = 5.0;
    double layer_skip_threshold        = 0.30;
    double arm_flip_threshold_deg      = 90.0;
};

enum class ReachStatus : uint8_t {
    OK              = 0,
    OutOfWorkspace  = 1,
    JointLimit      = 2,
    NearSingularity = 3,
    NoSolution      = 4,
};

struct ReachabilityResult {
    ReachStatus status     = ReachStatus::NoSolution;
    bool        arm_flip   = false;
    bool        wrist_sing = false;
    bool        shoulder_sing = false;
    JointConfig solution{};
    int         num_valid  = 0;
};

// Check if a Cartesian pose is reachable by the IK solver.
// Performs workspace check, IK solve, singularity detection, and arm-flip detection.
ReachabilityResult checkReachability(
    const CartPose& pose,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params = {});

}  // namespace cslc
