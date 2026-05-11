#pragma once

#include "core/types.h"
#include "field/poisson.h"
#include "geometry/sdf.h"

#include <array>
#include <string>
#include <vector>

namespace cslc {

struct BCParams {
    std::string strategy = "bottom_up";
    std::array<double, 3> print_direction{0.0, 0.0, 1.0};
    // sdf_normal 策略参数（向后兼容）
    double bottom_dot_threshold = -0.5;
    double bottom_sdf_band = 1.0;
    // geometric_z 策略参数（v4 §10）
    double bottom_band_mm = -1.0;   // -1 = 自动 bbox.z * 5%
    double top_band_mm    = -1.0;   // -1 = 自动 bbox.z * 5%
    double band_min_mm    = 0.5;
    double band_max_mm    = 5.0;
};

struct LaplacianBC {
    std::vector<VoxelIndex> fixed_indices;
    std::vector<double> fixed_values;
};

enum class BCZone : uint8_t { Bottom = 0, Top = 1 };

struct LaplacianVectorBC {
    std::vector<VoxelIndex> fixed_indices;
    std::vector<Vec3> fixed_vectors;
    std::vector<BCZone> bc_zones;  // v4 §10: 区分底面/顶面，供 pipeline 锚定 Poisson
};

struct LaplacianParams {
    int max_iterations = 2000;
    double tolerance = 1e-6;
};

struct VectorField {
    AABB bbox;
    double spacing = 0.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<Vec3> values;
};

LaplacianBC generateBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params);
ScalarField solveLaplacian(const VoxelGrid& grid, const LaplacianBC& bc, const LaplacianParams& params);
LaplacianVectorBC generateVectorBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params);
VectorField solveLaplacianVector(const VoxelGrid& grid,
                                 const LaplacianVectorBC& bc,
                                 const LaplacianParams& params);

}  // namespace cslc
