#include "path/path_postprocessing.h"

#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>

static void require(bool cond, const char* msg)
{
    if (!cond) throw std::runtime_error(msg);
}

// Build a straight line with Gaussian noise
static std::vector<Eigen::Vector3d> makeNoisyLine(int n, double sigma,
                                                   unsigned seed = 42)
{
    std::mt19937 gen(seed);
    std::normal_distribution<double> dist(0.0, sigma);

    std::vector<Eigen::Vector3d> pts(n);
    for (int i = 0; i < n; ++i) {
        double t = static_cast<double>(i);
        pts[i] = Eigen::Vector3d(t, dist(gen), dist(gen));
    }
    return pts;
}

// Compute variance of deviations from the best-fit line (x-axis)
static double computeNoiseVariance(const std::vector<Eigen::Vector3d>& pts)
{
    double sum_y = 0, sum_z = 0;
    for (auto& p : pts) { sum_y += p.y(); sum_z += p.z(); }
    double mean_y = sum_y / pts.size();
    double mean_z = sum_z / pts.size();

    double var = 0;
    for (auto& p : pts) {
        var += (p.y() - mean_y) * (p.y() - mean_y);
        var += (p.z() - mean_z) * (p.z() - mean_z);
    }
    return var / (2.0 * pts.size());
}

// Test a smoothing function: noise reduction + endpoint protection
void testSmoothingFunction(const char* name,
                           void(*smooth_fn)(std::vector<Eigen::Vector3d>&),
                           double noise_reduction_threshold)
{
    const int n = 20;
    const double sigma = 0.5;

    auto pts = makeNoisyLine(n, sigma);
    auto original = pts;

    double var_before = computeNoiseVariance(pts);
    smooth_fn(pts);
    double var_after = computeNoiseVariance(pts);

    double reduction = 1.0 - var_after / var_before;
    std::cout << "  " << name << ": var_before=" << var_before
              << " var_after=" << var_after
              << " reduction=" << (reduction * 100) << "%\n";

    require(reduction >= noise_reduction_threshold,
            (std::string(name) + ": noise reduction >= threshold").c_str());

    // Endpoint protection: endpoints should not move more than 0.5mm
    double end0_dist = (pts.front() - original.front()).norm();
    double endN_dist = (pts.back() - original.back()).norm();
    std::cout << "    endpoint displacement: start=" << end0_dist
              << " end=" << endN_dist << "\n";
    // Endpoint protection: displacement should be within noise level
    require(end0_dist < sigma * 2.0 + 1e-9,
            (std::string(name) + ": start endpoint displacement < 2*sigma").c_str());
    require(endN_dist < sigma * 2.0 + 1e-9,
            (std::string(name) + ": end endpoint displacement < 2*sigma").c_str());
}

void test_linear_smooth_31()
{
    // 3-point filter: theoretical ~63% reduction for Gaussian noise
    testSmoothingFunction("LinearSmooth31",
                          cslc::smoothPolyline_LinearSmooth31, 0.60);
}

void test_linear_smooth_51()
{
    testSmoothingFunction("LinearSmooth51",
                          cslc::smoothPolyline_LinearSmooth51, 0.75);
}

void test_linear_smooth_52()
{
    // 2nd-order SG: endpoint formula amplifies noise (31/9/-3/-5/3 weights).
    // Theoretical ~48% reduction for Gaussian noise on 20-point line.
    testSmoothingFunction("LinearSmooth52",
                          cslc::smoothPolyline_LinearSmooth52, 0.40);
}

void test_wls_1d()
{
    testSmoothingFunction("WLS1D(window=5)",
                          [](std::vector<Eigen::Vector3d>& p) {
                              cslc::smoothPolyline_WLS1D(p, 5);
                          }, 0.70);
}

int main()
{
    try {
        test_linear_smooth_31();
        test_linear_smooth_51();
        test_linear_smooth_52();
        test_wls_1d();
        std::cout << "path_postprocessing_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "path_postprocessing_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
