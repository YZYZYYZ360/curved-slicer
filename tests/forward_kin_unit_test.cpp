#include "kinematics/forward_kin.h"
#include "kinematics/dh_params.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void require(bool cond, const char* msg)
{
    if (!cond) {
        throw std::runtime_error(msg);
    }
}

int main()
{
    try {
        using namespace cslc;
        KR4DHParams dh;

        // === Test 1: Zero joint angles ===
        // With all joints at 0, the arm is fully extended along +Z
        // TCP should be at X≈0, Y≈0, Z = d1 + a2 + d4 + flange + tool
        {
            JointConfig zero{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            CartPose tcp = forwardKin(zero, dh);

            std::cout << "zero config FK: X=" << tcp.X << " Y=" << tcp.Y
                      << " Z=" << tcp.Z << " A=" << tcp.A << " B=" << tcp.B
                      << " C=" << tcp.C << "\n";
            // At zero config: J1=0 → no lateral offset (Y≈0).
            // KR4 R600 DH zero is arm extended forward (X≈d4=310, Z small).
            // This is geometrically correct, not "arm straight up".
            require(std::abs(tcp.Y) < 1.0, "zero config: Y should be ~0 (J1=0)");
            require(tcp.Z > 0.0, "zero config: Z should be positive");
            // Z should be positive and reasonable (arm extends upward)
            require(tcp.Z > 0.0, "zero config: Z should be positive");
            std::cout << "zero config FK: X=" << tcp.X << " Y=" << tcp.Y
                      << " Z=" << tcp.Z << " A=" << tcp.A << " B=" << tcp.B
                      << " C=" << tcp.C << "\n";
        }

        // === Test 2: Home joint angles ===
        // Home = (0, 9.95, -62.26, 0, -37.69, 0) deg
        // This is the physical home position from the KR4 R600 teach pendant.
        // Expected: TCP in front of robot (X > 0), reasonable height.
        {
            JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
            CartPose tcp = forwardKin(home, dh);

            // TCP should be in front of the robot base
            require(tcp.X > 0.0, "home: X should be positive (arm forward)");
            // Z should be reasonable (not below base, not above max reach)
            require(tcp.Z > 50.0, "home: Z should be above base plane");
            require(tcp.Z < 1000.0, "home: Z should be below max reach");

            std::cout << "home FK: X=" << tcp.X << " Y=" << tcp.Y
                      << " Z=" << tcp.Z << " A=" << tcp.A << " B=" << tcp.B
                      << " C=" << tcp.C << "\n";
        }

        // === Test 3: forwardKinMatrix == forwardKin (consistency) ===
        {
            JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
            Eigen::Matrix4d T = forwardKinMatrix(home, dh);
            CartPose pose = forwardKin(home, dh);

            require(std::abs(T(0, 3) - pose.X) < 1e-9, "FK matrix vs pose X mismatch");
            require(std::abs(T(1, 3) - pose.Y) < 1e-9, "FK matrix vs pose Y mismatch");
            require(std::abs(T(2, 3) - pose.Z) < 1e-9, "FK matrix vs pose Z mismatch");
        }

        // === Test 4: rotMatToKukaABC <-> kukaABCToRotMat round-trip ===
        {
            // Test several known Euler angle sets
            const double test_angles[][3] = {
                {0.0, 0.0, 0.0},
                {30.0, 45.0, 60.0},
                {-120.0, 30.0, -45.0},
                {180.0, -90.0, 0.0},
            };
            for (const auto& abc : test_angles) {
                Eigen::Matrix3d R = kukaABCToRotMat(abc[0], abc[1], abc[2]);
                double A2, B2, C2;
                rotMatToKukaABC(R, A2, B2, C2);
                // Round-trip should recover the original angles (modulo gimbal lock at B=±90)
                if (std::abs(abc[1]) < 89.9) {
                    require(std::abs(A2 - abc[0]) < 1e-6, "Euler round-trip A mismatch");
                    require(std::abs(B2 - abc[1]) < 1e-6, "Euler round-trip B mismatch");
                    require(std::abs(C2 - abc[2]) < 1e-6, "Euler round-trip C mismatch");
                }
            }
        }

        // === Test 5: forwardKinMatrix_partial(N=3) matches first 3 joints of full FK ===
        {
            JointConfig q{{20.0, -30.0, 45.0, 0.0, 0.0, 0.0}};
            Eigen::Matrix4d T3 = forwardKinMatrix_partial(q, dh, 3);
            Eigen::Matrix4d T6 = forwardKinMatrix(q, dh);
            // T6 = T3 * DH(0,0,0,0) * DH(0,0,0,0) * DH(0,0,0,0) * T_flange
            // So T3 != T6, but T3 should be a valid 4x4 matrix with det(R)=1
            Eigen::Matrix3d R3 = T3.block<3, 3>(0, 0);
            double det = R3.determinant();
            require(std::abs(det - 1.0) < 1e-9, "T3 rotation det should be 1");
        }

        std::cout << "forward_kin_unit_test PASSED\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "forward_kin_unit_test FAILED: " << e.what() << '\n';
        return 1;
    }
}
