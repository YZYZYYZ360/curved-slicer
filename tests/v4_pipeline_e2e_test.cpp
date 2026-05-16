#include "app/pipeline.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

int main()
{
    using namespace cslc;

    const std::filesystem::path config_path = CSLC_SOURCE_DIR "/config/mao_rescaled_v4.toml";

    std::cout << "  Running v4 pipeline on mao_rescaled...\n" << std::flush;

    BatchReport batch = runBatch(config_path);

    require(batch.success_count == 1, "pipeline succeeded");
    require(batch.models.size() == 1, "one model processed");

    const auto& model = batch.models[0];
    require(model.success, ("model success: " + model.message).c_str());

    // Check trajectory.csv exists and has content
    const std::filesystem::path traj_path =
        config_path.parent_path().parent_path() / "runs" / "v4_mao_rescaled" / model.name / "trajectory.csv";

    std::cout << "  trajectory.csv path: " << traj_path.string() << "\n";

    require(std::filesystem::exists(traj_path), "trajectory.csv exists");

    std::ifstream csv(traj_path);
    require(csv.is_open(), "trajectory.csv opened");

    std::string header;
    std::getline(csv, header);
    require(header == "timestamp_ms,segment_id,A1,A2,A3,A4,A5,A6,wire_on",
            "CSV header correct");

    int line_count = 0;
    int max_segment_id = 0;
    double prev_timestamp = -1.0;
    bool first_line = true;

    std::string line;
    while (std::getline(csv, line)) {
        if (line.empty()) continue;
        ++line_count;

        std::istringstream ss(line);
        std::string token;
        double timestamp;
        int segment_id;

        std::getline(ss, token, ','); timestamp = std::stod(token);
        std::getline(ss, token, ','); segment_id = std::stoi(token);

        if (segment_id > max_segment_id) max_segment_id = segment_id;

        // Verify 4ms spacing within same segment
        if (!first_line && segment_id == 1) {
            double dt = timestamp - prev_timestamp;
            require(std::abs(dt - 4.0) < 0.1,
                    ("4ms spacing: dt=" + std::to_string(dt)).c_str());
        }
        prev_timestamp = timestamp;
        first_line = false;
    }

    csv.close();

    std::cout << "  trajectory.csv: " << line_count << " lines, "
              << max_segment_id << " segments\n";

    require(line_count > 1000, ("line_count > 1000: " + std::to_string(line_count)).c_str());
    require(max_segment_id >= 1, "at least 1 segment");

    std::cout << "  v4_pipeline_e2e_test PASSED\n";
    return 0;
}
