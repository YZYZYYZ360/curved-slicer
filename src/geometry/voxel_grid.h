#pragma once

#include "core/types.h"
#include "io/stl_reader.h"

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace cslc {

struct VoxelIndex {
    int x = 0;
    int y = 0;
    int z = 0;
};

struct VoxelParams {
    double spacing_mm = 0.5;
    double padding_mm = 2.0;
    double sdf_band_mm = 3.0;
};

struct VoxelReport {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::size_t total_voxels = 0;
    std::size_t occupied_voxels = 0;
    double occupancy_ratio = 0.0;
    AABB bbox;
};

class VoxelGrid {
public:
    VoxelGrid() = default;
    VoxelGrid(AABB bbox, double spacing, int nx, int ny, int nz);

    int nx() const { return nx_; }
    int ny() const { return ny_; }
    int nz() const { return nz_; }
    double spacing() const { return spacing_; }
    const AABB& bbox() const { return bbox_; }

    bool inBounds(int x, int y, int z) const;
    std::size_t index(int x, int y, int z) const;
    bool occupied(int x, int y, int z) const;
    void setOccupied(int x, int y, int z, bool value = true);
    Vec3 center(int x, int y, int z) const;
    VoxelIndex indexToVoxel(std::size_t index) const;
    std::vector<VoxelIndex> occupiedVoxels() const;

    std::size_t totalVoxelCount() const;
    std::size_t occupiedVoxelCount() const;
    VoxelReport report() const;

private:
    AABB bbox_;
    double spacing_ = 0.0;
    int nx_ = 0;
    int ny_ = 0;
    int nz_ = 0;
    std::unordered_set<std::size_t> occupied_;
};

VoxelGrid voxelizeMesh(const TriangleMesh& mesh, const VoxelParams& params);

}  // namespace cslc
