#pragma once

#include "field/laplacian.h"

namespace cslc {

// Shared reachability parameters used by both kuka_projection and kinematics/reachability.
struct ReachabilityParams {
    // Hemisphere projection (kuka_projection)
    Vec3 workpiece_up{0.0, 0.0, 1.0};
    double min_dot_threshold = 0.05;
    // IK reachability (kinematics/reachability)
    double workspace_r_min_mm          = 100.0;
    double workspace_r_max_mm          = 600.0;
    double singularity_a5_deg          = 5.0;
    double singularity_shoulder_deg    = 5.0;
    double layer_skip_threshold        = 0.30;
    double arm_flip_threshold_deg      = 90.0;
};

VectorField projectToHemisphere(const VoxelGrid& grid,
                                const VectorField& field,
                                const ReachabilityParams& params);

}  // namespace cslc
