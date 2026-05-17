#include "io/TrajectoryCSVWriter.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace cslc {
namespace {

const char* kSchemaMarker = "# schema=traj_v1.0";

const char* kHeader =
    "schema_version,run_id,frame_id,timestamp_ms,dt_ms,"
    "layer_id,component_id,path_id,segment_id,transition_id,"
    "motion_type,status,wire_on,"
    "A1_deg,A2_deg,A3_deg,A4_deg,A5_deg,A6_deg,"
    "tcp_x_mm,tcp_y_mm,tcp_z_mm,tcp_rx_deg,tcp_ry_deg,tcp_rz_deg,"
    "ik_branch,yaw_id,"
    "feed_mm_s,sigma_min,joint_margin_min_deg,clearance_mm,"
    "validation_level,failure_code,source_type";

std::string formatDouble(double value)
{
    if (std::isnan(value)) return "nan";

    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << value;
    return out.str();
}

const char* formatBool(bool value)
{
    return value ? "true" : "false";
}

}  // namespace

void TrajectoryCSVWriter::write(
    const std::string& path,
    const std::vector<Row>& rows)
{
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("TrajectoryCSVWriter: failed to open output path: " + path);
    }

    output << kSchemaMarker << '\n';
    output << kHeader << '\n';

    for (const Row& row : rows) {
        output
            << row.schema_version << ','
            << row.run_id << ','
            << row.frame_id << ','
            << formatDouble(row.timestamp_ms) << ','
            << formatDouble(row.dt_ms) << ','
            << row.layer_id << ','
            << row.component_id << ','
            << row.path_id << ','
            << row.segment_id << ','
            << row.transition_id << ','
            << row.motion_type << ','
            << row.status << ','
            << formatBool(row.wire_on) << ','
            << formatDouble(row.A1_deg) << ','
            << formatDouble(row.A2_deg) << ','
            << formatDouble(row.A3_deg) << ','
            << formatDouble(row.A4_deg) << ','
            << formatDouble(row.A5_deg) << ','
            << formatDouble(row.A6_deg) << ','
            << formatDouble(row.tcp_x_mm) << ','
            << formatDouble(row.tcp_y_mm) << ','
            << formatDouble(row.tcp_z_mm) << ','
            << formatDouble(row.tcp_rx_deg) << ','
            << formatDouble(row.tcp_ry_deg) << ','
            << formatDouble(row.tcp_rz_deg) << ','
            << row.ik_branch << ','
            << row.yaw_id << ','
            << formatDouble(row.feed_mm_s) << ','
            << formatDouble(row.sigma_min) << ','
            << formatDouble(row.joint_margin_min_deg) << ','
            << formatDouble(row.clearance_mm) << ','
            << row.validation_level << ','
            << row.failure_code << ','
            << row.source_type << '\n';
    }
}

}  // namespace cslc
