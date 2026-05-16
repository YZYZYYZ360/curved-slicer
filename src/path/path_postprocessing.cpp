#include "path/path_postprocessing.h"

#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

#include <cmath>
#include <vector>

namespace cslc {

namespace {

// Apply a smoothing function to each x/y/z component independently.
// smooth_fn takes a vector<double> and returns a smoothed vector<double>.
void applyPerComponent(std::vector<Eigen::Vector3d>& points,
                       std::vector<double>(*smooth_fn)(const std::vector<double>&))
{
    if (points.size() < 2) return;

    const int n = static_cast<int>(points.size());
    std::vector<double> x(n), y(n), z(n);
    for (int i = 0; i < n; ++i) {
        x[i] = points[i].x();
        y[i] = points[i].y();
        z[i] = points[i].z();
    }

    auto sx = smooth_fn(x);
    auto sy = smooth_fn(y);
    auto sz = smooth_fn(z);

    for (int i = 0; i < n; ++i) {
        points[i] = Eigen::Vector3d(sx[i], sy[i], sz[i]);
    }
}

// LinearSmooth31 on a 1D signal
std::vector<double> linearSmooth31_impl(const std::vector<double>& input)
{
    const long size = static_cast<long>(input.size());
    std::vector<double> output(size);

    if (size < 3) {
        for (long i = 0; i < size; ++i) output[i] = input[i];
    } else {
        output[0] = (5.0 * input[0] + 2.0 * input[1] - input[2]) / 6.0;
        for (long i = 1; i <= size - 2; ++i) {
            output[i] = (input[i - 1] + input[i] + input[i + 1]) / 3.0;
        }
        output[size - 1] = (5.0 * input[size - 1] + 2.0 * input[size - 2] - input[size - 3]) / 6.0;
    }
    return output;
}

// LinearSmooth51 on a 1D signal
std::vector<double> linearSmooth51_impl(const std::vector<double>& input)
{
    const long size = static_cast<long>(input.size());
    std::vector<double> output(size);

    if (size < 5) {
        for (long i = 0; i < size; ++i) output[i] = input[i];
    } else {
        output[0] = (3.0 * input[0] + 2.0 * input[1] + input[2] - input[4]) / 5.0;
        output[1] = (4.0 * input[0] + 3.0 * input[1] + 2.0 * input[2] + input[3]) / 10.0;
        for (long i = 2; i <= size - 3; ++i) {
            output[i] = (input[i - 2] + input[i - 1] + input[i] + input[i + 1] + input[i + 2]) / 5.0;
        }
        output[size - 2] = (4.0 * input[size - 1] + 3.0 * input[size - 2] + 2.0 * input[size - 3] + input[size - 4]) / 10.0;
        output[size - 1] = (3.0 * input[size - 1] + 2.0 * input[size - 2] + input[size - 3] - input[size - 5]) / 5.0;
    }
    return output;
}

// LinearSmooth52 on a 1D signal
std::vector<double> linearSmooth52_impl(const std::vector<double>& input)
{
    const long size = static_cast<long>(input.size());
    std::vector<double> output(size);

    if (size < 5) {
        for (long i = 0; i < size; ++i) output[i] = input[i];
    } else {
        output[0] = (31.0 * input[0] + 9.0 * input[1] - 3.0 * input[2] - 5.0 * input[3] + 3.0 * input[4]) / 35.0;
        output[1] = (9.0 * input[0] + 13.0 * input[1] + 12.0 * input[2] + 6.0 * input[3] - 5.0 * input[4]) / 35.0;
        for (long i = 2; i <= size - 3; ++i) {
            output[i] = (-3.0 * (input[i - 2] + input[i + 2]) +
                          12.0 * (input[i - 1] + input[i + 1]) + 17.0 * input[i]) / 35.0;
        }
        output[size - 2] = (9.0 * input[size - 1] + 13.0 * input[size - 2] + 12.0 * input[size - 3] + 6.0 * input[size - 4] - 5.0 * input[size - 5]) / 35.0;
        output[size - 1] = (31.0 * input[size - 1] + 9.0 * input[size - 2] - 3.0 * input[size - 3] - 5.0 * input[size - 4] + 3.0 * input[size - 5]) / 35.0;
    }
    return output;
}

// WLS 1D filter on a single signal
std::vector<double> wlsFilter1d_impl(const std::vector<double>& input,
                                      double lambda, double alpha, double smallNum)
{
    const int n = static_cast<int>(input.size());
    if (n < 2) return input;

    // Build weight vector (uniform for simplicity)
    std::vector<double> vecL(n, 1.0);

    // Compute dx weights
    std::vector<double> vecDx(n + 1, 0.0);
    for (int i = 1; i < n; ++i) {
        vecDx[i] = lambda * std::pow(std::abs(vecL[i] - vecL[i - 1]) + smallNum, alpha);
    }

    // Build sparse tridiagonal matrix
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(3 * n - 2);
    for (int i = 0; i < n; ++i) {
        triplets.emplace_back(i, i, 1.0 + vecDx[i] + vecDx[i + 1]);
    }
    for (int i = 1; i < n; ++i) {
        triplets.emplace_back(i, i - 1, -vecDx[i]);
    }
    for (int i = 1; i < n; ++i) {
        triplets.emplace_back(i - 1, i, -vecDx[i]);
    }

    Eigen::SparseMatrix<double> A(n, n);
    A.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) b(i) = input[i];

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(A);
    if (solver.info() != Eigen::Success) return input;

    Eigen::VectorXd x = solver.solve(b);
    if (solver.info() != Eigen::Success) return input;

    return std::vector<double>(x.data(), x.data() + n);
}

}  // namespace

void smoothPolyline_LinearSmooth31(std::vector<Eigen::Vector3d>& points)
{
    applyPerComponent(points, linearSmooth31_impl);
}

void smoothPolyline_LinearSmooth51(std::vector<Eigen::Vector3d>& points)
{
    applyPerComponent(points, linearSmooth51_impl);
}

void smoothPolyline_LinearSmooth52(std::vector<Eigen::Vector3d>& points)
{
    applyPerComponent(points, linearSmooth52_impl);
}

void smoothPolyline_WLS1D(std::vector<Eigen::Vector3d>& points, int window)
{
    if (points.size() < 2 || window < 1) return;

    // Save endpoints for restoration
    Eigen::Vector3d p_start = points.front();
    Eigen::Vector3d p_end = points.back();

    const double lambda = static_cast<double>(window * window);
    constexpr double alpha = 0.5;
    constexpr double smallNum = 0.2;

    auto wls_fn = [&](const std::vector<double>& input) -> std::vector<double> {
        return wlsFilter1d_impl(input, lambda, alpha, smallNum);
    };

    const int n = static_cast<int>(points.size());
    std::vector<double> x(n), y(n), z(n);
    for (int i = 0; i < n; ++i) {
        x[i] = points[i].x();
        y[i] = points[i].y();
        z[i] = points[i].z();
    }

    auto sx = wls_fn(x);
    auto sy = wls_fn(y);
    auto sz = wls_fn(z);

    for (int i = 0; i < n; ++i) {
        points[i] = Eigen::Vector3d(sx[i], sy[i], sz[i]);
    }

    // Restore endpoints
    points.front() = p_start;
    points.back() = p_end;
}

}  // namespace cslc
