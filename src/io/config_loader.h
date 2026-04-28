#pragma once

#include "field/laplacian.h"
#include "field/poisson.h"
#include "geometry/voxel_grid.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace cslc {

struct ModelConfig {
    std::string name;
    std::filesystem::path stl_path;
};

struct IoConfig {
    std::filesystem::path output_root;
    std::filesystem::path runs_csv;
    bool debug_dump_intermediates = false;
    std::vector<ModelConfig> models;
};

struct AlgorithmFieldConfig {
    LaplacianParams laplacian;
    PoissonParams poisson;
};

struct AlgorithmConfig {
    AlgorithmFieldConfig field;
};

using FieldBoundaryConfig = BCParams;

struct IsoSurfaceConfig {
    double layer_thickness_mm = 0.8;
    std::string iso_spacing = "uniform";
    int max_layers = 500;
    double phi_start_offset = 0.0;
};

struct SmoothOrientationConfig {
    double lambda_data = 1.0;
    double lambda_smooth = 10.0;
};

struct SplineConfig {
    bool enabled = true;
    int resample_n_per_segment = 3;
    double tension = 0.5;
};

struct PathConfig {
    std::string strategy = "scanline";
    double line_spacing_mm = 0.4;
    double contour_resample_mm = 0.2;
    int smooth_position_window = 5;
    SmoothOrientationConfig smooth_orientation;
    SplineConfig spline;
};

struct JointAxisLimit {
    double min_deg = 0.0;
    double max_deg = 0.0;
    double speed_deg_per_s = 0.0;
};

struct RobotConfig {
    int status = 4;
    int turn = 28;
    int tool_no = 8;
    int base_no = 6;
    double platform_x = 0.0;
    double platform_y = 0.0;
    double platform_z = 0.0;
    double z_offset = 0.28;
    std::array<double, 3> robot_ini_abc{0.0, 0.0, 0.0};
};

struct KukaConfig {
    std::array<JointAxisLimit, 6> limits{{{-170.0, 170.0, 336.0}, {-195.0, 40.0, 336.0},
        {-115.0, 150.0, 488.0}, {-185.0, 185.0, 600.0}, {-120.0, 120.0, 529.0},
        {-350.0, 350.0, 800.0}}};
    RobotConfig robot;
    std::array<std::array<double, 4>, 4> world_to_base{{{{1.0, 0.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0, 0.0}}, {{0.0, 0.0, 1.0, 0.0}}, {{0.0, 0.0, 0.0, 1.0}}}};
};

struct MetricsConfig {
    bool enabled = true;
    std::vector<int> sample_layer_ids;
    bool output_per_layer = true;
};

struct LoggingConfig {
    std::string level = "info";
};

struct PipelineConfig {
    IoConfig io;
    VoxelParams voxel;
    AlgorithmConfig algorithm;
    FieldBoundaryConfig field_boundary;
    IsoSurfaceConfig iso_surface;
    PathConfig path;
    KukaConfig kuka;
    MetricsConfig metrics;
    LoggingConfig logging;
};

PipelineConfig loadPipelineConfig(const std::filesystem::path& config_path);

}  // namespace cslc
