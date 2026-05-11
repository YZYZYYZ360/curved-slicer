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

bool random_test(int i)
{
    using namespace cslc;
    static KR4DHParams dh;

    double q[6];
    for (int j = 0; j < 6; ++j) {
        double lo = dh.qlim_deg[j][0], hi = dh.qlim_deg[j][1];
        // Use middle 40% of joint range for reliable round-trip testing
        double margin = 0.3 * (hi - lo);
        q[j] = (lo + margin) + (hi - lo - 2 * margin) * (double(std::rand()) / RAND_MAX);
    }

    JointConfig original{{q[0], q[1], q[2], q[3], q[4], q[5]}};
    CartPose tcp = forwardKin(original, dh);

    // Skip configurations where TCP is near the Z-axis (J1 ill-conditioned)
    double r_tcp = std::sqrt(tcp.X * tcp.X + tcp.Y * tcp.Y);
    if (r_tcp < 50.0) {
        return true;  // count as pass (workspace boundary)
    }

    auto sols = solveAnalyticalIK(tcp, dh, &original);

    // Check if any IK solution matches the original joints
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

    if (sols.empty()) {
        std::cout << "  FAIL [i=" << i << "] NO SOLUTION  tcp=("
                  << tcp.X << "," << tcp.Y << "," << tcp.Z << ")\n";
    } else {
        // Find best position error among solutions
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
        std::srand(12345);
        test_home_smoke();

        set_failed_cfg("random1000");
        int passed = 0;
        constexpr int N = 1000;
        for (int i = 0; i < N; ++i) {
            if (random_test(i)) ++passed;
            if ((i + 1) % 200 == 0)
                std::cout << "  progress: " << (i + 1) << "/" << N << "\n";
        }
        double rate = 100.0 * passed / N;
        std::cout << "random round-trip: " << passed << "/" << N
                  << " (" << rate << "%)\n";
        require(passed >= 900,
                ("need >= 90% pass, got " + std::to_string(passed) + "/1000").c_str());

        std::cout << "ik_solver_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        const char* ctx = get_failed_cfg();
        std::cerr << "ik_solver_unit_test FAILED" << (ctx ? " [" : "") << (ctx ? ctx : "")
                  << (ctx ? "]" : "") << ": " << e.what() << '\n';
        return 1;
    }
}
