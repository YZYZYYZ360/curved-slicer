#pragma once

#include "io/config_loader.h"
#include "io/stl_reader.h"

#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

namespace cslc {

struct ComponentSummary {
    int root_id = -1;
    std::size_t triangle_count = 0;
    std::size_t vertex_count = 0;
    AABB bbox;
    int layer_min = 0;
    int layer_max = 0;
};

struct ModelReport {
    std::string name;
    std::filesystem::path stl_path;
    std::string pipeline;
    bool success = false;
    std::string message;
    std::size_t triangle_count = 0;
    AABB bbox;
    VoxelReport voxel;
    double read_ms = 0.0;
    double voxelize_ms = 0.0;
    double sdf_ms = 0.0;
    double laplacian_ms = 0.0;
    double poisson_ms = 0.0;
    double smoothing_ms = 0.0;
    double iso_surface_ms = 0.0;
    double peak_rss_mb = 0.0;
    std::size_t iso_vertices = 0;
    std::size_t iso_triangles = 0;
    int connected_components = 0;
    int max_layer_connected_components = 0;
    int layer_count = 0;
    std::size_t face_count_total = 0;
    std::vector<std::size_t> face_count_per_layer;
    std::vector<int> connected_components_per_layer;
    double m1_max_abs_mean_curvature = 0.0;
    std::vector<double> m1_per_layer;
    double m2_hemisphere_violation_ratio = 0.0;
    double phi_min_pre_shift = 0.0;
    double phi_max_pre_shift = 0.0;
    double phi_range_pre_shift = 0.0;
    double phi_progression = 0.0;
    double phi_bbox_extent = 0.0;
    double phi_min = 0.0;
    double phi_max = 0.0;
    VoxelIndex phi_min_voxel{0, 0, 0};
    VoxelIndex phi_max_voxel{0, 0, 0};
    Vec3 phi_min_point;
    Vec3 phi_max_point;
    double phi_min_sdf = 0.0;
    double phi_max_sdf = 0.0;
    double phi_min_alignment = 0.0;
    double phi_max_alignment = 0.0;
    bool phi_min_on_bottom_boundary = false;
    bool phi_max_on_top_boundary = false;
    std::vector<std::string> component_summary;
    std::vector<ComponentSummary> component_details;
    std::filesystem::path metrics_path;
};

struct BatchReport {
    std::vector<ModelReport> models;
    std::size_t success_count = 0;
    std::size_t failure_count = 0;
    double total_ms = 0.0;
};

BatchReport runBatch(const PipelineConfig& config);
BatchReport runBatch(const std::filesystem::path& config_path);
void printBatchReport(const BatchReport& report, std::ostream& output);

}  // namespace cslc
