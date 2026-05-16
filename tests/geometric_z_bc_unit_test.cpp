#include "field/laplacian.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int fail(const std::string& message)
{
    std::cerr << "geometric_z_bc_unit_test failed: " << message << '\n';
    return 1;
}

cslc::VoxelGrid makeTestGrid(int nx, int ny, int nz, double spacing)
{
    cslc::VoxelGrid grid(
        {{0.0, 0.0, 0.0},
         {static_cast<double>(nx) * spacing,
          static_cast<double>(ny) * spacing,
          static_cast<double>(nz) * spacing}},
        spacing, nx, ny, nz);
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                grid.setOccupied(x, y, z);
            }
        }
    }
    return grid;
}

// geometric_z 不需要 SDF，构造一个 dummy
cslc::SDF makeDummySdf(int nx, int ny, int nz, double spacing)
{
    cslc::SDF sdf;
    sdf.bbox = {{0.0, 0.0, 0.0},
                {static_cast<double>(nx) * spacing,
                 static_cast<double>(ny) * spacing,
                 static_cast<double>(nz) * spacing}};
    sdf.spacing = spacing;
    sdf.nx = nx;
    sdf.ny = ny;
    sdf.nz = nz;
    sdf.values.assign(static_cast<std::size_t>(nx) * ny * nz, 0.0);
    return sdf;
}

}  // namespace

int main()
{
    try {
        // 10×10×20 立方体，spacing 1mm → bbox.z = 20mm
        constexpr int nx = 10, ny = 10, nz = 20;
        constexpr double spacing = 1.0;
        const cslc::VoxelGrid grid = makeTestGrid(nx, ny, nz, spacing);
        const cslc::SDF sdf = makeDummySdf(nx, ny, nz, spacing);

        cslc::BCParams p;
        p.strategy = "geometric_z";
        // bottom_band = clamp(20 * 0.05, 0.5, 5.0) = 1.0mm
        // top_band = 同上 = 1.0mm
        // → bottom: z_world ∈ [0, 1) → z=0 层 (100 voxels)
        // → top:    z_world ∈ [19, 20] → z=19 层 (100 voxels)

        const cslc::LaplacianBC bc = cslc::generateBC(grid, sdf, p);

        int bottom_count = 0, top_count = 0;
        for (std::size_t i = 0; i < bc.fixed_indices.size(); ++i) {
            const cslc::VoxelIndex& v = bc.fixed_indices[i];
            const double z_world = grid.bbox().min.z + (v.z + 0.5) * grid.spacing();
            if (bc.fixed_values[i] == 0.0) {
                if (z_world >= 1.0) {
                    return fail("bottom BC at z=" + std::to_string(z_world) + " expected < 1.0");
                }
                ++bottom_count;
            } else if (bc.fixed_values[i] == 1.0) {
                if (z_world <= 19.0) {
                    return fail("top BC at z=" + std::to_string(z_world) + " expected > 19.0");
                }
                ++top_count;
            } else {
                return fail("unexpected BC value=" + std::to_string(bc.fixed_values[i]));
            }
        }

        if (bottom_count != 100) {
            return fail("bottom_count=" + std::to_string(bottom_count) + " expected 100");
        }
        if (top_count != 100) {
            return fail("top_count=" + std::to_string(top_count) + " expected 100");
        }

        // 测试 generateVectorBC：底面 vec=print_dir，顶面 vec=print_dir，bc_zones 区分
        const cslc::LaplacianVectorBC vbc = cslc::generateVectorBC(grid, sdf, p);
        if (vbc.bc_zones.size() != vbc.fixed_indices.size()) {
            return fail("bc_zones size mismatch");
        }
        int v_bottom = 0, v_top = 0;
        for (std::size_t i = 0; i < vbc.fixed_indices.size(); ++i) {
            if (vbc.bc_zones[i] == cslc::BCZone::Bottom) ++v_bottom;
            else if (vbc.bc_zones[i] == cslc::BCZone::Top) ++v_top;
        }
        if (v_bottom != 100) {
            return fail("vectorBC bottom_count=" + std::to_string(v_bottom) + " expected 100");
        }
        if (v_top != 100) {
            return fail("vectorBC top_count=" + std::to_string(v_top) + " expected 100");
        }

        std::cout << "geometric_z_bc_unit_test PASSED: bottom=100 top=100\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "geometric_z_bc_unit_test exception: " << e.what() << '\n';
        return 1;
    }
}
