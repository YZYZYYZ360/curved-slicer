#include "kinematics/forward_kin.h"
#include "kinematics/ik_solver.h"
#include "kinematics/dh_params.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double sq(double x) { return x * x; }

void require(bool cond, const char* msg)
{
    if (!cond) {
        throw std::runtime_error(msg);
    }
}

static const char* failed_cfg_label = nullptr;

void set_failed_cfg(const char* label) { failed_cfg_label = label; }
const char* get_failed_cfg() { return failed_cfg_label; }

void test_home_smoke()
{
    using namespace cslc;
    KR4DHParams dh;
    set_failed_cfg("home_smoke");

    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
    CartPose tcp = forwardKin(home, dh);

    std::cout << "home FK: X=" << tcp.X << " Y=" << tcp.Y
              << " Z=" << tcp.Z << "\n";

    JointConfig ref = home;
    auto sols = solveAnalyticalIK(tcp, dh, &ref);

    std::cout << "IK solutions: " << sols.size() << "\n";
    for (std::size_t i = 0; i < sols.size(); ++i) {
        const auto& s = sols[i].solution;
        std::cout << "  [" << i << "] "
                  << s.q_deg[0] << " " << s.q_deg[1] << " "
                  << s.q_deg[2] << " " << s.q_deg[3] << " "
                  << s.q_deg[4] << " " << s.q_deg[5] << "\n";
    }

    require(sols.size() >= 1, "home: at least 1 IK solution");
    CartPose fk = forwardKin(sols[0].solution, dh);
    double pos_err = std::sqrt(sq(fk.X - tcp.X) + sq(fk.Y - tcp.Y) + sq(fk.Z - tcp.Z));
    std::cout << "home round-trip pos_err: " << pos_err << " mm\n";
    require(pos_err < 0.1, "home: round-trip pos error < 0.1mm");

    set_failed_cfg(nullptr);
}

static double angle_diff_deg(double a, double b)
{
    double d = a - b;
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

// Plan 1 Step 4.1 test ranges (80-90% of mechanical limits)
struct TestRange { double lo, hi; };
static const TestRange kTestRanges[6] = {
    {-150.0, 150.0},   // J1: [-170, 170]
    {-150.0,   30.0},   // J2: [-195,  40]
    {-100.0, 130.0},   // J3: [-115, 150]
    {-170.0, 170.0},   // J4: [-185, 185]
    {-100.0, 100.0},   // J5: [-120, 120]
    {-300.0, 300.0},   // J6: [-350, 350]
};

bool random_test(int i, int& num_no_sol, int& num_pos_fail)
{
    using namespace cslc;
    static KR4DHParams dh;

    double q[6];
    for (int j = 0; j < 6; ++j) {
        double lo = kTestRanges[j].lo, hi = kTestRanges[j].hi;
        q[j] = lo + (hi - lo) * (double(std::rand()) / RAND_MAX);
    }

    JointConfig original{{q[0], q[1], q[2], q[3], q[4], q[5]}};
    CartPose tcp = forwardKin(original, dh);

    auto sols = solveAnalyticalIK(tcp, dh, &original);

    // Check if any IK solution matches the original joints (1° tolerance)
    for (const auto& s : sols) {
        double joint_err = 0.0;
        for (int j = 0; j < 6; ++j) {
            double d = angle_diff_deg(s.solution.q_deg[j], original.q_deg[j]);
            joint_err += d * d;
        }
        joint_err = std::sqrt(joint_err);
        if (joint_err < 1.0) {
            return true;
        }
    }

    // If no exact match, verify IK solutions reach the correct TCP position
    for (const auto& s : sols) {
        CartPose fk = forwardKin(s.solution, dh);
        double pos_err = std::sqrt(sq(fk.X - tcp.X) + sq(fk.Y - tcp.Y) + sq(fk.Z - tcp.Z));
        if (pos_err < 1.0) {
            return true;
        }
    }

    // Failure categorization
    if (sols.empty()) {
        ++num_no_sol;
    } else {
        ++num_pos_fail;
    }

    if (sols.empty()) {
        std::cout << "  FAIL [i=" << i << "] NO SOLUTION  tcp=("
                  << tcp.X << "," << tcp.Y << "," << tcp.Z << ")\n";
    } else {
        double best_pos_err = 1e9;
        for (const auto& s : sols) {
            CartPose fk = forwardKin(s.solution, dh);
            double pos_err = std::sqrt(sq(fk.X - tcp.X) + sq(fk.Y - tcp.Y) + sq(fk.Z - tcp.Z));
            if (pos_err < best_pos_err) best_pos_err = pos_err;
        }
        std::cout << "  FAIL [i=" << i << "] #sols=" << sols.size()
                  << " best_pos_err=" << best_pos_err << "  tcp=("
                  << tcp.X << "," << tcp.Y << "," << tcp.Z << ")\n";
    }
    return false;
}

int main()
{
    try {
        std::srand(42);
        test_home_smoke();

        set_failed_cfg("random1000");
        int passed = 0;
        int num_no_sol = 0, num_pos_fail = 0;
        constexpr int N = 1000;
        for (int i = 0; i < N; ++i) {
            if (random_test(i, num_no_sol, num_pos_fail)) ++passed;
            if ((i + 1) % 200 == 0)
                std::cout << "  progress: " << (i + 1) << "/" << N << "\n";
        }
        int failed = N - passed;
        double rate = 100.0 * passed / N;
        std::cout << "random round-trip: " << passed << "/" << N
                  << " (" << rate << "%)\n";
        std::cout << "  failures: " << failed
                  << " (no_solution=" << num_no_sol
                  << ", pos_fail=" << num_pos_fail << ")\n";
        require(passed >= 800,
                ("need >= 80% pass, got " + std::to_string(passed) + "/1000").c_str());

        std::cout << "ik_solver_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        const char* ctx = get_failed_cfg();
        std::cerr << "ik_solver_unit_test FAILED" << (ctx ? " [" : "") << (ctx ? ctx : "")
                  << (ctx ? "]" : "") << ": " << e.what() << '\n';
        return 1;
    }
}
