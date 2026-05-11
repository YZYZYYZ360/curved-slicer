#include "path/pose_from_path.h"
#include "kinematics/forward_kin.h"

namespace cslc {

Eigen::Matrix3d constructToolFrame(const Eigen::Vector3d& path_tangent,
                                   const Eigen::Vector3d& G_outward)
{
    Eigen::Vector3d tool_z = -G_outward.normalized();
    Eigen::Vector3d t = path_tangent.normalized();

    // Project tangent onto plane perpendicular to tool_z
    Eigen::Vector3d tool_x = t - (t.dot(tool_z)) * tool_z;
    double xn = tool_x.norm();

    // Degenerate: tangent parallel to G_outward → no well-defined tool frame
    constexpr double kEps = 1e-6;
    if (xn < kEps) {
        return Eigen::Matrix3d::Identity();
    }
    tool_x /= xn;

    Eigen::Vector3d tool_y = tool_z.cross(tool_x);

    Eigen::Matrix3d R;
    R.col(0) = tool_x;
    R.col(1) = tool_y;
    R.col(2) = tool_z;
    return R;
}

CartPose toolFrameToCartPose(const Eigen::Vector3d& point,
                             const Eigen::Matrix3d& R_tool)
{
    CartPose pose;
    pose.X = point.x();
    pose.Y = point.y();
    pose.Z = point.z();
    rotMatToKukaABC(R_tool, pose.A, pose.B, pose.C);
    return pose;
}

}  // namespace cslc
