#include "path/geodesic_paths.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

void test_2x2_plane()
{
    using namespace cslc;

    // 2x2 grid: 9 vertices, 8 triangles
    //   v0(0,0) v1(1,0) v2(2,0)
    //   v3(0,1) v4(1,1) v5(2,1)
    //   v6(0,2) v7(1,2) v8(2,2)
    Eigen::MatrixXd V(9, 3);
    V << 0, 0, 0,
         1, 0, 0,
         2, 0, 0,
         0, 1, 0,
         1, 1, 0,
         2, 1, 0,
         0, 2, 0,
         1, 2, 0,
         2, 2, 0;

    Eigen::MatrixXi F(8, 3);
    F << 0, 1, 4,
         0, 4, 3,
         1, 2, 5,
         1, 5, 4,
         3, 4, 7,
         3, 7, 6,
         4, 5, 8,
         4, 8, 7;

    GeodesicPathParams params;
    params.line_spacing_mm = 0.5;  // small spacing to get multiple isolines

    auto paths = generateGeodesicPaths(V, F, params);

    std::cout << "  paths.size() = " << paths.size() << "\n";
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "    path[" << i << "]: " << paths[i].points.size()
                  << " points, length=" << paths[i].total_length_mm << "\n";
    }

    require(paths.size() >= 2, "2x2: at least 2 isoline paths");
    for (size_t i = 0; i < paths.size(); ++i) {
        require(paths[i].points.size() >= 2,
                "2x2: each path has at least 2 points");
    }

    std::cout << "  test_2x2_plane PASSED\n";
}

int main()
{
    try {
        test_2x2_plane();
        std::cout << "geodesic_paths_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "geodesic_paths_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
