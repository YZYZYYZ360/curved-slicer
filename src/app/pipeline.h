#pragma once

#include "io/config_loader.h"
#include "io/stl_reader.h"

#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

namespace cslc {

struct ModelReport {
    std::string name;
    std::filesystem::path stl_path;
    bool success = false;
    std::string message;
    std::size_t triangle_count = 0;
    AABB bbox;
    VoxelReport voxel;
    double read_ms = 0.0;
    double voxelize_ms = 0.0;
    double iso_surface_ms = 0.0;
    std::size_t iso_vertices = 0;
    std::size_t iso_triangles = 0;
    int connected_components = 0;
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
