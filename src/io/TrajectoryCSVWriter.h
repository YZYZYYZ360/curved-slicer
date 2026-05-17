#pragma once

#include <string>
#include <vector>

namespace cslc {

class TrajectoryCSVWriter {
public:
    struct Row {
        std::string schema_version = "traj_v1.0";
        std::string run_id;
        int frame_id = 0;
        double timestamp_ms = 0.0;
        double dt_ms = 0.0;
        int layer_id = 0;
        int component_id = 0;
        int path_id = 0;
        int segment_id = 0;
        int transition_id = -1;
        std::string motion_type;
        std::string status;
        bool wire_on = false;
        double A1_deg = 0.0;
        double A2_deg = 0.0;
        double A3_deg = 0.0;
        double A4_deg = 0.0;
        double A5_deg = 0.0;
        double A6_deg = 0.0;
        double tcp_x_mm = 0.0;
        double tcp_y_mm = 0.0;
        double tcp_z_mm = 0.0;
        double tcp_rx_deg = 0.0;
        double tcp_ry_deg = 0.0;
        double tcp_rz_deg = 0.0;
        int ik_branch = 0;
        int yaw_id = 0;
        double feed_mm_s = 0.0;
        double sigma_min = 0.0;
        double joint_margin_min_deg = 0.0;
        double clearance_mm = 0.0;
        std::string validation_level;
        std::string failure_code;
        std::string source_type;
    };

    static void write(const std::string& path, const std::vector<Row>& rows);
};

}  // namespace cslc
