#include <igl/exact_geodesic.h>
#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

void test_single_triangle()
{
    // Non-planar quad: two triangles sharing edge v0-v2.
    // v0=(0,0,0), v1=(1,0,0), v2=(0.5,0.5,0.5), v3=(0,1,0)
    // F0 = (0,1,2), F1 = (0,2,3)
    Eigen::MatrixXd V(4, 3);
    V << 0.0, 0.0, 0.0,
         1.0, 0.0, 0.0,
         0.5, 0.5, 0.5,
         0.0, 1.0, 0.0;

    Eigen::MatrixXi F(2, 3);
    F << 0, 1, 2,
         0, 2, 3;

    // Compute geodesic distance from vertex 0 to vertices 1, 2, 3.
    Eigen::VectorXi VS(1), FS(1), VT(3), FT(0);
    VS << 0;
    FS << 0;
    VT << 1, 2, 3;

    Eigen::VectorXd D;
    igl::exact_geodesic(V, F, VS, FS, VT, FT, D);

    std::cout << "  D.size()=" << D.size() << " D=[";
    for (int i = 0; i < D.size(); ++i) std::cout << D(i) << " ";
    std::cout << "]\n";

    // Smoke: verify it runs, produces correct size, non-negative, reasonable
    require(D.size() == 3, "non_planar: D.size() == 3");
    require(D(0) >= 0.0, "non_planar: D(0) >= 0");
    require(D(1) >= 0.0, "non_planar: D(1) >= 0");
    require(D(2) >= 0.0, "non_planar: D(2) >= 0");
    require(D(0) < 10.0 && D(1) < 10.0 && D(2) < 10.0,
            "non_planar: distances reasonable");

    // Edge v0-v1 has Euclidean length 1.0; geodesic should be close.
    // (For non-degenerate meshes, geodesic >= Euclidean.)
    std::cout << "  d(v0,v1)=" << D(0) << " d(v0,v2)=" << D(1)
              << " d(v0,v3)=" << D(2) << "\n";

    std::cout << "  single_triangle PASSED\n";
}

int main()
{
    try {
        test_single_triangle();
        std::cout << "igl_smoke_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "igl_smoke_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
