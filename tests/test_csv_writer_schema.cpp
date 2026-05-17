#include "io/TrajectoryCSVWriter.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

static std::vector<std::string> splitCsvLine(const std::string& line)
{
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) tokens.push_back(token);
    return tokens;
}

static std::vector<std::string> readLines(const std::filesystem::path& path)
{
    std::ifstream input(path);
    require(input.is_open(), "csv: output file opened");

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
    }
    return lines;
}

static cslc::TrajectoryCSVWriter::Row makeRow(int i)
{
    cslc::TrajectoryCSVWriter::Row row;
    row.run_id = "unit_test_run";
    row.frame_id = i;
    row.timestamp_ms = static_cast<double>(i) * 4.0;
    row.dt_ms = 4.0;
    row.layer_id = 1;
    row.component_id = 2;
    row.path_id = 3;
    row.segment_id = 4;
    row.transition_id = -1;
    row.motion_type = "PRINT";
    row.status = "OK";
    row.wire_on = true;
    row.A1_deg = 10.0 + i;
    row.A2_deg = 20.0 + i;
    row.A3_deg = 30.0 + i;
    row.A4_deg = 40.0 + i;
    row.A5_deg = 50.0 + i;
    row.A6_deg = 60.0 + i;
    row.tcp_x_mm = 100.0 + i;
    row.tcp_y_mm = 200.0 + i;
    row.tcp_z_mm = 300.0 + i;
    row.tcp_rx_deg = 1.0 + i;
    row.tcp_ry_deg = 2.0 + i;
    row.tcp_rz_deg = 3.0 + i;
    row.ik_branch = 5;
    row.yaw_id = 6;
    row.feed_mm_s = 25.0;
    row.sigma_min = 0.123456;
    row.joint_margin_min_deg = 7.5;
    row.clearance_mm = 8.5;
    row.validation_level = "L0";
    row.failure_code = "NONE";
    row.source_type = "PRINT";
    return row;
}

int main()
{
    try {
        const std::vector<std::string> expected_header = {
            "schema_version", "run_id", "frame_id", "timestamp_ms", "dt_ms",
            "layer_id", "component_id", "path_id", "segment_id", "transition_id",
            "motion_type", "status", "wire_on",
            "A1_deg", "A2_deg", "A3_deg", "A4_deg", "A5_deg", "A6_deg",
            "tcp_x_mm", "tcp_y_mm", "tcp_z_mm", "tcp_rx_deg", "tcp_ry_deg", "tcp_rz_deg",
            "ik_branch", "yaw_id",
            "feed_mm_s", "sigma_min", "joint_margin_min_deg", "clearance_mm",
            "validation_level", "failure_code", "source_type",
        };

        std::vector<cslc::TrajectoryCSVWriter::Row> rows;
        for (int i = 0; i < 5; ++i) rows.push_back(makeRow(i));

        const auto path =
            std::filesystem::temp_directory_path() / "curved_slicer_test_traj_v1.csv";
        cslc::TrajectoryCSVWriter::write(path.string(), rows);

        const auto lines = readLines(path);
        require(lines.size() == 7, "csv: schema line + header + five rows");
        require(lines[0] == "# schema=traj_v1.0", "csv: schema marker line");

        const auto header = splitCsvLine(lines[1]);
        require(header == expected_header, "csv: header matches V6_SPEC §6 order");

        for (int i = 0; i < 5; ++i) {
            const auto tokens = splitCsvLine(lines[static_cast<size_t>(i + 2)]);
            require(tokens.size() == 34, "csv: data row has 34 columns");
            require(std::stoi(tokens[2]) == i, "csv: frame_id round-trip");
            require(std::abs(std::stod(tokens[3]) - static_cast<double>(i) * 4.0) < 1e-9,
                    "csv: timestamp_ms round-trip");
            require(std::abs(std::stod(tokens[13]) - (10.0 + i)) < 1e-9,
                    "csv: A1_deg round-trip");
            require(std::abs(std::stod(tokens[18]) - (60.0 + i)) < 1e-9,
                    "csv: A6_deg round-trip");
            require(tokens[12] == "true", "csv: bool serialized as true");
        }

        std::cout << "test_csv_writer_schema PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "test_csv_writer_schema FAILED: " << e.what() << '\n';
        return 1;
    }
}
