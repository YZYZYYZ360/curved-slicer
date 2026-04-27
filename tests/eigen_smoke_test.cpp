#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

int fail(const std::string& message)
{
    std::cerr << "eigen_smoke_test failed: " << message << '\n';
    return 1;
}

}  // namespace

int main()
{
    constexpr int width = 25;
    constexpr int height = 40;
    constexpr int n = width * height;

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(n) * 5);

    const auto index = [](int x, int y) {
        return x + width * y;
    };

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int row = index(x, y);
            triplets.emplace_back(row, row, 4.0);
            if (x > 0) {
                triplets.emplace_back(row, index(x - 1, y), -1.0);
            }
            if (x + 1 < width) {
                triplets.emplace_back(row, index(x + 1, y), -1.0);
            }
            if (y > 0) {
                triplets.emplace_back(row, index(x, y - 1), -1.0);
            }
            if (y + 1 < height) {
                triplets.emplace_back(row, index(x, y + 1), -1.0);
            }
        }
    }

    Eigen::SparseMatrix<double> matrix(n, n);
    matrix.setFromTriplets(triplets.begin(), triplets.end());

    const Eigen::VectorXd rhs = Eigen::VectorXd::Ones(n);
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
    solver.setTolerance(1e-10);
    solver.setMaxIterations(10000);
    solver.compute(matrix);
    if (solver.info() != Eigen::Success) {
        return fail("ConjugateGradient factorization failed");
    }

    const Eigen::VectorXd solution = solver.solve(rhs);
    if (solver.info() != Eigen::Success) {
        return fail("ConjugateGradient solve failed");
    }

    const double residual = (matrix * solution - rhs).norm() / rhs.norm();
    std::cout << "eigen_smoke residual=" << residual
              << " iterations=" << solver.iterations() << '\n';

    if (!std::isfinite(residual) || residual >= 1e-6) {
        return fail("relative residual should be < 1e-6");
    }

    return 0;
}
