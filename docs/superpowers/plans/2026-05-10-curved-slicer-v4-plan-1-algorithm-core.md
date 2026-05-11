# Plan 1: v4 §10-§14 算法核心 Implementation Plan

> **For agentic workers (CLI #2):** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 curved-slicer 项目实现 v4 §10-§14：BC 重构（绝对 z）+ KR4 R600 解析 IK + 双层可达性过滤 + libigl 测地路径 + 5 次多项式关节角轨迹平滑。完成后 pipeline 能从 STL 输入产出 4ms 周期密化的 6 关节角轨迹（不含 §15 输出）。

**Architecture:** 模块化分层，每模块独立可测：
- `kinematics/` ：DH 参数 + 正逆运动学 + 可达性
- `path/` ：测地等值线路径生成
- `trajectory/` ：5 次多项式平滑
- `field/` ：扩展现有 BC 选择策略
- `app/pipeline.cpp` ：串接成 vector_kuka_v4 流水线

**Tech Stack:** C++17, Eigen 3, libigl (新增), tomlplusplus, CMake FetchContent。Windows + MSVC（用户环境）。

**前置条件**：
- 工作目录：`E:\Git_Repository\curved-slicer\`（CLI #2 默认 cwd 是 `C:\Users\26480`，每条 git/cmake 命令前先 `cd /d E:\Git_Repository\curved-slicer`）
- Build 系统：NMake Makefiles + Release（CLI #2 现役 build 目录是 `build_nmake`），与 single-config generator 兼容（命令里**不带** `--config Debug` flag）
- 已存在 phase0/phase2 实现（laplacian / poisson / iso_surface / sdf 全跑通），ctest 7/7 基线绿
- 设计依据：`docs/architecture/V3_TO_V4_AMENDMENTS.md` §10-§14（**所有算法细节、参数表、接口定义都在该文档**，本 plan 不重复）
- 分支：实施在 `v4-plan1-algorithm-core` 分支上做（基于 master HEAD），完成后 PR 回 master

---

## 文件结构概览

### 新增源文件

| 文件 | 责任 | 来源 §  |
|------|------|---------|
| `src/kinematics/cart_pose.h` | `CartPose` / `JointConfig` / `IKStatus` 共享类型 | §11 |
| `src/kinematics/dh_params.h` | `KR4DHParams` 结构 + KR4 R600 默认值 | §11.3 |
| `src/kinematics/forward_kin.h` + `.cpp` | `forwardKin(JointConfig, KR4DHParams) → CartPose` | §11.4 |
| `src/kinematics/ik_solver.h` + `.cpp` | `solveAnalyticalIK(CartPose, JointConfig ref, KR4DHParams) → IKResult` | §11.4 |
| `src/kinematics/reachability.h` + `.cpp` | `checkReachability` + B/C 双层入口 | §12 |
| `src/path/geodesic_paths.h` + `.cpp` | libigl 包装 + isolines 提取 + 起点/方向规范化 | §13 |
| `src/path/path_postprocessing.h` + `.cpp` | wlsFilter1d + LinearSmooth31/51/52（简化迁移）| §13.3 |
| `src/trajectory/poly5_smoother.h` + `.cpp` | 5 次多项式 + 双时间约束 + 4ms 周期采样 | §14 |

### 修改源文件

| 文件 | 改动 | 来源 §  |
|------|------|---------|
| `src/field/laplacian.h` + `.cpp` | `BCParams` 加 `strategy = "geometric_z"` 分支；新版 `generateVectorBC` 走绝对 z + bbox 5% 自适应 band；`generateBC`（标量版）同步 | §10 |
| `src/field/poisson.cpp` | 顶面 Dirichlet phi=1 anchor 选项（原仅底面 anchor）| §10.2 |
| `src/io/config_loader.h` + `.cpp` | `BCParams` 新字段；`KR4DHParams` 默认；`ReachabilityParams`；`GeodesicPathParams`；`TrajectoryParams`；`[kuka.home]` 段 | §10/§11/§12/§13/§14 |
| `src/app/pipeline.cpp` | 新 `vector_kuka_v4` 分支：BC → laplacian → poisson → MC → §13 → §11 → §12 → §14 | §10-§14 |
| `CMakeLists.txt` | 加 libigl FetchContent；加新源文件到 `curved_slicer_phase0`；加新 test targets | — |

### 新增测试文件

| 文件 | 测试目标 |
|------|---------|
| `tests/geometric_z_bc_unit_test.cpp` | §10 几何 z BC：3 种 bbox 的体素分类正确性 |
| `tests/forward_kin_unit_test.cpp` | §11 正运动学：home 关节角对应实测 TCP；零位姿恒等 |
| `tests/ik_solver_unit_test.cpp` | §11 IK round-trip：1000 组随机关节角 → FK → IK，关节角差 < 1e-6 |
| `tests/reachability_unit_test.cpp` | §12 边界 case：关节限位、奇异性、工作空间球壳 |
| `tests/geodesic_paths_unit_test.cpp` | §13 在简单球面 mesh 上路径间距 ±5% |
| `tests/poly5_smoother_unit_test.cpp` | §14 端点零速度、4ms 周期对齐、双时间约束 |
| `tests/v4_pipeline_e2e_test.cpp` | mao_rescaled subset 端到端跑通，输出 N 帧 TrajectoryPoint |

---

## Stage A: §10 BC 重构（1 task）

### Task 1: 几何 z 坐标 BC 替换 SDF normal

**Files:**
- Modify: `src/field/laplacian.h`（`BCParams` 加字段）
- Modify: `src/field/laplacian.cpp`（`generateBC` + `generateVectorBC` 加 strategy 分支）
- Modify: `src/field/poisson.cpp`（顶面 Dirichlet anchor 支持）
- Modify: `src/io/config_loader.cpp`（toml 解析新字段）
- Create: `tests/geometric_z_bc_unit_test.cpp`
- Modify: `CMakeLists.txt`（加新 test target）

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §10.2-§10.3

- [ ] **Step 1.1: 扩展 BCParams 结构**

修改 `src/field/laplacian.h`，把 `BCParams` 改为：
```cpp
struct BCParams {
    std::string strategy = "geometric_z";        // 新默认（取代 "bottom_up"）
    std::array<double, 3> print_direction{0.0, 0.0, 1.0};
    // sdf_normal 策略（向后兼容）保留：
    double bottom_dot_threshold = -0.5;
    double bottom_sdf_band = 1.0;
    // geometric_z 策略新字段：
    double bottom_band_mm = -1.0;                // -1 = 自动 bbox.z * 5%
    double top_band_mm    = -1.0;                // -1 = 自动 bbox.z * 5%
    double band_min_mm    = 0.5;                 // 自动推算的下限
    double band_max_mm    = 5.0;                 // 自动推算的上限
};
```

- [ ] **Step 1.2: 写失败测试**

创建 `tests/geometric_z_bc_unit_test.cpp`：
```cpp
#include "field/laplacian.h"
#include "geometry/voxel_grid.h"
#include "geometry/sdf.h"
#include <cassert>

int main() {
    using namespace cslc;
    // 构造 10mm × 10mm × 20mm 的简单立方体素网格（spacing 1mm）
    VoxelGrid grid = makeTestCubeGrid(10, 10, 20, 1.0);
    SDF sdf;  // 内容不影响 geometric_z 策略

    BCParams p;
    p.strategy = "geometric_z";
    // bottom_band 自动 = clamp(20 * 0.05, 0.5, 5.0) = 1.0mm
    // top_band 同上 = 1.0mm

    LaplacianBC bc = generateBC(grid, sdf, p);

    // 期望：z ∈ [0, 1) → bottom (phi=0)；z ∈ [19, 20] → top (phi=1)
    int bottom_count = 0, top_count = 0;
    for (size_t i = 0; i < bc.fixed_indices.size(); ++i) {
        Vec3 wpos = grid.voxelToWorld(bc.fixed_indices[i]);
        if (bc.fixed_values[i] == 0.0) {
            assert(wpos.z() < 1.0);
            ++bottom_count;
        } else if (bc.fixed_values[i] == 1.0) {
            assert(wpos.z() > 19.0);
            ++top_count;
        }
    }
    assert(bottom_count == 100);  // 1 层 × 10×10 体素
    assert(top_count == 100);
    return 0;
}
```

辅助函数 `makeTestCubeGrid` 加在测试 cpp 顶部（直接 inline 不复用）。

加到 `CMakeLists.txt`：
```cmake
add_executable(geometric_z_bc_unit_test tests/geometric_z_bc_unit_test.cpp)
target_link_libraries(geometric_z_bc_unit_test PRIVATE curved_slicer_phase0)
add_test(NAME geometric_z_bc_unit COMMAND geometric_z_bc_unit_test)
```

- [ ] **Step 1.3: 跑测试确认失败**

```
cd /d E:\Git_Repository\curved-slicer
cmake --build build_nmake --target geometric_z_bc_unit_test
ctest --test-dir build_nmake -R geometric_z_bc_unit -V
```
Expected: FAIL（`generateBC` 还没识别 `strategy = "geometric_z"`，走老逻辑返回错的 BC）

- [ ] **Step 1.4: 实现 geometric_z 分支**

修改 `src/field/laplacian.cpp` 的 `generateBC` 和 `generateVectorBC`，开头加 strategy 分支：
```cpp
LaplacianBC generateBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params) {
    if (params.strategy == "geometric_z") {
        return generateBC_geometric_z(grid, params);
    }
    // 原 bottom_up / sdf_normal 实现保留作为 else 分支
    return generateBC_sdf_normal(grid, sdf, params);
}

static LaplacianBC generateBC_geometric_z(const VoxelGrid& grid, const BCParams& params) {
    LaplacianBC bc;
    double zspan = grid.bbox().zmax - grid.bbox().zmin;
    double bot_band = params.bottom_band_mm > 0 ? params.bottom_band_mm
                    : std::clamp(zspan * 0.05, params.band_min_mm, params.band_max_mm);
    double top_band = params.top_band_mm > 0 ? params.top_band_mm
                    : std::clamp(zspan * 0.05, params.band_min_mm, params.band_max_mm);
    double zmin = grid.bbox().zmin;
    double zmax = grid.bbox().zmax;
    for (auto& v : grid.activeVoxels()) {
        double zw = grid.voxelToWorld(v.idx).z();
        if (zw - zmin < bot_band) {
            bc.fixed_indices.push_back(v.idx);
            bc.fixed_values.push_back(0.0);
        } else if (zmax - zw < top_band) {
            bc.fixed_indices.push_back(v.idx);
            bc.fixed_values.push_back(1.0);  // 顶面 Dirichlet phi=1（§10.2）
        }
    }
    return bc;
}
```

`generateVectorBC` 仿照写一个 `generateVectorBC_geometric_z`：底面体素 fixed_vector = print_direction（指向顶部），顶面体素 fixed_vector = print_direction。

- [ ] **Step 1.5: poisson.cpp 顶面 anchor 支持**

`src/field/poisson.cpp` 现有多 anchor Dirichlet 实现（§9）。检查它能不能接受**两组 anchor 值**（不止 0）。如果不能，扩展 `solvePoisson` 接口接 `vector<pair<VoxelIndex, double>>` 作为 anchor 列表。从 LaplacianBC 直接转入。

- [ ] **Step 1.6: config_loader 加字段解析**

`src/io/config_loader.cpp` 的 `BCParams` 解析段加：
```cpp
bc.strategy = field_boundary_node["strategy"].value_or<std::string>("geometric_z");
bc.bottom_band_mm = field_boundary_node["bottom_band_mm"].value_or<double>(-1.0);
bc.top_band_mm = field_boundary_node["top_band_mm"].value_or<double>(-1.0);
bc.band_min_mm = field_boundary_node["band_min_mm"].value_or<double>(0.5);
bc.band_max_mm = field_boundary_node["band_max_mm"].value_or<double>(5.0);
```

- [ ] **Step 1.7: 跑测试确认通过**

```
cmake --build build_nmake --target geometric_z_bc_unit_test
ctest --test-dir build_nmake -R geometric_z_bc_unit -V
```
Expected: PASS。

- [ ] **Step 1.8: 跑全部 ctest 防回归**

```
ctest --test-dir build_nmake --output-on-failure
```
所有原有测试（laplacian_smoke_test, poisson_smoke_test 等）必须通过。

- [ ] **Step 1.9: Commit**

```
git add src/field/laplacian.h src/field/laplacian.cpp src/field/poisson.cpp src/io/config_loader.cpp tests/geometric_z_bc_unit_test.cpp CMakeLists.txt
git commit -m "feat(field): geometric_z BC strategy with bbox-adaptive band (v4 §10)

- BCParams: add strategy='geometric_z', auto bbox.z*5% bands
- Top BC = Dirichlet phi=1 for layer thickness uniformity
- Backward-compat: sdf_normal strategy retained as fallback
- Tests: voxel classification on 10x10x20 cube"
```

---

## Stage B: §11 笛卡尔位姿 + 解析 IK（6 tasks）

### Task 2: kinematics 模块骨架 + DH 参数

**Files:**
- Create: `src/kinematics/cart_pose.h`
- Create: `src/kinematics/dh_params.h`
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §11.3

- [ ] **Step 2.1: CartPose / JointConfig / IKStatus 类型**

创建 `src/kinematics/cart_pose.h`：
```cpp
#pragma once
#include <Eigen/Dense>
#include <array>

namespace cslc {

struct CartPose {
    double X, Y, Z;     // mm
    double A, B, C;     // deg, KUKA ZYX 欧拉角
};

struct JointConfig {
    std::array<double, 6> q_deg;
};

enum class IKStatus { OK, OutOfWorkspace, JointLimit, NearSingularity };

struct IKResult {
    IKStatus status;
    JointConfig solution;
    std::array<JointConfig, 8> all_solutions;   // 调试用
    int num_valid;                               // 有效解数量 (0-8)
};

}  // namespace cslc
```

- [ ] **Step 2.2: KR4DHParams 结构**

创建 `src/kinematics/dh_params.h`：
```cpp
#pragma once
#include <array>

namespace cslc {

struct KR4DHParams {
    std::array<double, 6> a_mm     = {0.0,    290.0, 20.0,   0.0,   0.0, 0.0};
    std::array<double, 6> alpha_rad= {1.5707963, 3.1415927, -1.5707963, 1.5707963, -1.5707963, 0.0};
    std::array<double, 6> d_mm     = {330.0,  0.0,   0.0,    310.0, 0.0, 0.0};
    double flange_z_mm = 12.0;       // mstraj0110.m Tz(0.012)
    double tool_z_mm   = 0.28;       // 用户标定值，可改
    std::array<std::array<double, 2>, 6> qlim_deg{{
        {-170.0, 170.0}, {-195.0, 40.0}, {-115.0, 150.0},
        {-185.0, 185.0}, {-120.0, 120.0}, {-350.0, 350.0}}};
    std::array<double, 6> qmax_speed_deg_per_s = {336.0, 336.0, 488.0, 600.0, 529.0, 800.0};
};

}  // namespace cslc
```

- [ ] **Step 2.3: 加新文件到 CMakeLists**

修改 `CMakeLists.txt`：
```cmake
target_include_directories(curved_slicer_phase0 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
# 上面已有，不动；只需要确保 kinematics/ 被包含
# 由于 .h-only 类型不需要加 .cpp 到库
```
（这一步只是确认 include path，无需修改）

- [ ] **Step 2.4: Smoke build + commit**

```
cmake --build build_nmake --target curved_slicer_phase0
git add src/kinematics/cart_pose.h src/kinematics/dh_params.h
git commit -m "feat(kinematics): CartPose/JointConfig/KR4DHParams types (v4 §11)"
```

---

### Task 3: 正运动学 forward_kin

**Files:**
- Create: `src/kinematics/forward_kin.h` + `.cpp`
- Create: `tests/forward_kin_unit_test.cpp`
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §11.3 + DH 标准 forward kinematics（连乘 6 个 DH 矩阵 + flange + tool z 偏移）

- [ ] **Step 3.1: 写失败测试（home 姿态对应可信 TCP）**

创建 `tests/forward_kin_unit_test.cpp`：
```cpp
#include "kinematics/forward_kin.h"
#include "kinematics/dh_params.h"
#include <cassert>
#include <cmath>

int main() {
    using namespace cslc;
    KR4DHParams dh;

    // home 关节角（§15.4 现场实测）
    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
    CartPose tcp = forwardKin(home, dh);

    // home 姿态下 A2+A3+A5 = -90°，喷头水平指向工件
    // 期望 TCP 在 base 前方某处（X > 0），高度合理
    assert(tcp.X > 0);
    assert(std::abs(tcp.Z - 330.0 + 290.0 * std::sin(9.95 * M_PI/180.0)) < 50.0);  // 粗略检查

    // 零关节角 → TCP 在 base 正上方，Z = d1 + a2 + d4 + d6 (沿 z 轴)
    JointConfig zero{{0,0,0,0,0,0}};
    CartPose tcp_zero = forwardKin(zero, dh);
    assert(std::abs(tcp_zero.X) < 1e-3);
    assert(std::abs(tcp_zero.Y) < 1e-3);
    assert(tcp_zero.Z > 0);

    return 0;
}
```

加到 `CMakeLists.txt`：
```cmake
add_executable(forward_kin_unit_test tests/forward_kin_unit_test.cpp)
target_link_libraries(forward_kin_unit_test PRIVATE curved_slicer_phase0)
add_test(NAME forward_kin_unit COMMAND forward_kin_unit_test)
```

- [ ] **Step 3.2: 跑测试确认失败**

Expected: 链接错误，`forwardKin` 未定义。

- [ ] **Step 3.3: 实现 forward_kin.h/cpp**

创建 `src/kinematics/forward_kin.h`：
```cpp
#pragma once
#include "kinematics/cart_pose.h"
#include "kinematics/dh_params.h"
#include <Eigen/Dense>

namespace cslc {

// 6 DH + flange + tool 累计变换矩阵
Eigen::Matrix4d forwardKinMatrix(const JointConfig& q, const KR4DHParams& dh);
CartPose forwardKin(const JointConfig& q, const KR4DHParams& dh);

// 标准 DH 单关节变换矩阵（KUKA 用 modified DH 还是 standard DH？
// 用 standard DH： A_i = Rz(θ_i) Tz(d_i) Tx(a_i) Rx(α_i)）
Eigen::Matrix4d dhMatrix(double theta_deg, double d, double a, double alpha_rad);

// 旋转矩阵 → KUKA ZYX 欧拉角 (A=γ around Z, B=β around Y, C=α around X)
void rotMatToKukaABC(const Eigen::Matrix3d& R, double& A, double& B, double& C);

}  // namespace cslc
```

创建 `src/kinematics/forward_kin.cpp`：
```cpp
#include "kinematics/forward_kin.h"
#include <cmath>

namespace cslc {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;
}

Eigen::Matrix4d dhMatrix(double theta_deg, double d, double a, double alpha_rad) {
    double th = theta_deg * kDeg2Rad;
    double ct = std::cos(th), st = std::sin(th);
    double ca = std::cos(alpha_rad), sa = std::sin(alpha_rad);
    Eigen::Matrix4d T;
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0.0,      sa,       ca,      d,
         0.0,     0.0,      0.0,    1.0;
    return T;
}

Eigen::Matrix4d forwardKinMatrix(const JointConfig& q, const KR4DHParams& dh) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i) {
        T = T * dhMatrix(q.q_deg[i], dh.d_mm[i], dh.a_mm[i], dh.alpha_rad[i]);
    }
    // flange + tool 沿 z 偏移
    Eigen::Matrix4d T_flange = Eigen::Matrix4d::Identity();
    T_flange(2, 3) = dh.flange_z_mm + dh.tool_z_mm;
    return T * T_flange;
}

void rotMatToKukaABC(const Eigen::Matrix3d& R, double& A, double& B, double& C) {
    // KUKA ZYX 欧拉角 (intrinsic): yaw(A around Z) → pitch(B around Y') → roll(C around X'')
    B = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    if (std::abs(std::cos(B)) > 1e-9) {
        A = std::atan2(R(1, 0), R(0, 0));
        C = std::atan2(R(2, 1), R(2, 2));
    } else {
        // gimbal lock at B = ±90° (奇异)
        A = std::atan2(-R(0, 1), R(1, 1));
        C = 0.0;
    }
    A *= kRad2Deg; B *= kRad2Deg; C *= kRad2Deg;
}

CartPose forwardKin(const JointConfig& q, const KR4DHParams& dh) {
    Eigen::Matrix4d T = forwardKinMatrix(q, dh);
    CartPose p;
    p.X = T(0, 3);
    p.Y = T(1, 3);
    p.Z = T(2, 3);
    rotMatToKukaABC(T.block<3, 3>(0, 0), p.A, p.B, p.C);
    return p;
}

}  // namespace cslc
```

加到 `CMakeLists.txt` 的 `curved_slicer_phase0` 源列表：
```cmake
src/kinematics/forward_kin.cpp
```

- [ ] **Step 3.4: 跑测试确认通过**

Expected: PASS。

- [ ] **Step 3.5: Commit**

```
git add src/kinematics/forward_kin.h src/kinematics/forward_kin.cpp tests/forward_kin_unit_test.cpp CMakeLists.txt
git commit -m "feat(kinematics): forward kinematics with DH + KUKA ABC euler (v4 §11)"
```

---

### Task 4: 解析 IK 球腕分解

**Files:**
- Create: `src/kinematics/ik_solver.h` + `.cpp`
- Create: `tests/ik_solver_unit_test.cpp`
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §11.4（详细 6 步分解 + 多解处理）

- [ ] **Step 4.1: 写 round-trip 失败测试**

创建 `tests/ik_solver_unit_test.cpp`：
```cpp
#include "kinematics/ik_solver.h"
#include "kinematics/forward_kin.h"
#include <cassert>
#include <random>
#include <cmath>

int main() {
    using namespace cslc;
    KR4DHParams dh;
    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> d_a1(-150, 150);
    std::uniform_real_distribution<double> d_a2(-150, 30);
    std::uniform_real_distribution<double> d_a3(-100, 130);
    std::uniform_real_distribution<double> d_a4(-170, 170);
    std::uniform_real_distribution<double> d_a5(-100, 100);
    std::uniform_real_distribution<double> d_a6(-300, 300);

    int pass = 0;
    for (int i = 0; i < 1000; ++i) {
        JointConfig q_in{{d_a1(rng), d_a2(rng), d_a3(rng), d_a4(rng), d_a5(rng), d_a6(rng)}};
        CartPose pose = forwardKin(q_in, dh);
        IKResult r = solveAnalyticalIK(pose, q_in, dh);  // ref = q_in 自身
        if (r.status != IKStatus::OK) continue;
        // 检查 IK 解 forward 后位姿一致
        CartPose pose_check = forwardKin(r.solution, dh);
        double dx = std::abs(pose.X - pose_check.X);
        double dy = std::abs(pose.Y - pose_check.Y);
        double dz = std::abs(pose.Z - pose_check.Z);
        if (dx < 1e-3 && dy < 1e-3 && dz < 1e-3) ++pass;
    }
    assert(pass > 900);  // 至少 90% 通过（剩余可能落到奇异邻域）
    return 0;
}
```

加到 `CMakeLists.txt`。

- [ ] **Step 4.2: 跑测试确认失败**

Expected: 链接错误。

- [ ] **Step 4.3: 实现 ik_solver**

创建 `src/kinematics/ik_solver.h`：
```cpp
#pragma once
#include "kinematics/cart_pose.h"
#include "kinematics/dh_params.h"

namespace cslc {

IKResult solveAnalyticalIK(
    const CartPose& target,
    const JointConfig& reference,    // 上一帧或 home，用于多解选择
    const KR4DHParams& dh);

// 工具：把 KUKA ABC → 旋转矩阵
Eigen::Matrix3d kukaABCToRotMat(double A, double B, double C);

}  // namespace cslc
```

创建 `src/kinematics/ik_solver.cpp`：实现 §11.4 算法。完整实现较长（~250 LOC），分以下逻辑步骤（在同一个 cpp 内）：

1. `kukaABCToRotMat(A, B, C)` → 3x3 旋转矩阵
2. 算 wrist_center = TCP_pos - (d6 + flange + tool_z) * tool_z（其中 tool_z = R_tool 第三列）
3. A1 = atan2(wc.y, wc.x)（前/后臂两解）
4. 在 A1 旋转后的平面内：
   - r = sqrt(wc.x² + wc.y²) - a3（注意 A3 偏置 a=20mm）
   - s = wc.z - d1
   - 余弦定理算 cos(A3 + π/2) 或类似（具体取决于 DH 几何，需要验证）
   - A3 ±两解（肘上肘下）
5. A2 = atan2(s, r) - atan2(d4 sin(A3'), a2 + d4 cos(A3'))
6. 用前 3 关节算 R0_3，求 R3_6 = R0_3.T * R_tool
7. 球腕 ZYZ 分解 R3_6 → A4, A5, A6（A5 ≥ 0 / A5 ≤ 0 两解）
8. 多解循环（最多 8 解）：剔除越限 + 奇异邻域，按与 reference 加权 L2 距离选最优

关键代码骨架（CLI #2 实施时填完整公式，可参考 mstraj0110.m + 任何标准 6-axis IK 教材）：

```cpp
IKResult solveAnalyticalIK(const CartPose& target, const JointConfig& reference, const KR4DHParams& dh) {
    IKResult r{};
    r.num_valid = 0;
    Eigen::Matrix3d R_tool = kukaABCToRotMat(target.A, target.B, target.C);
    Eigen::Vector3d tcp(target.X, target.Y, target.Z);
    Eigen::Vector3d wrist_center = tcp - (dh.d_mm[5] + dh.flange_z_mm + dh.tool_z_mm) * R_tool.col(2);

    // 工作空间检查
    double r_xy = std::sqrt(wrist_center.x() * wrist_center.x() + wrist_center.y() * wrist_center.y());
    if (r_xy < 1.0) {
        r.status = IKStatus::OutOfWorkspace;
        return r;
    }

    int sol_idx = 0;
    for (int front_back = 0; front_back < 2; ++front_back) {
        double a1 = std::atan2(wrist_center.y(), wrist_center.x());
        if (front_back == 1) a1 += M_PI;  // 后臂态

        // 计算肘部三角形
        double r_planar = std::sqrt(wrist_center.x() * wrist_center.x()
                                  + wrist_center.y() * wrist_center.y()) - dh.a_mm[2];
        if (front_back == 1) r_planar = -r_planar;
        double s_planar = wrist_center.z() - dh.d_mm[0];
        double L = std::sqrt(r_planar * r_planar + s_planar * s_planar);
        double cos_a3_inner = (L * L - dh.a_mm[1] * dh.a_mm[1] - dh.d_mm[3] * dh.d_mm[3])
                            / (2.0 * dh.a_mm[1] * dh.d_mm[3]);
        if (std::abs(cos_a3_inner) > 1.0) continue;  // 不可达

        for (int elbow = 0; elbow < 2; ++elbow) {
            double a3_inner = (elbow == 0) ? std::acos(cos_a3_inner) : -std::acos(cos_a3_inner);
            // a3_inner 是肘部内角，转换为关节 A3：A3 = a3_inner - π/2 (或类似，取决于 DH 偏置)
            double a3 = a3_inner;  // CLI #2 实施时校正
            double a2 = std::atan2(s_planar, r_planar)
                      - std::atan2(dh.d_mm[3] * std::sin(a3), dh.a_mm[1] + dh.d_mm[3] * std::cos(a3));

            // R0_3 from a1, a2, a3
            JointConfig q3{{a1 * 180.0 / M_PI, a2 * 180.0 / M_PI, a3 * 180.0 / M_PI, 0, 0, 0}};
            Eigen::Matrix4d T0_3 = forwardKinMatrix_partial(q3, dh, 3);  // 只乘前 3 个 DH
            Eigen::Matrix3d R0_3 = T0_3.block<3, 3>(0, 0);
            Eigen::Matrix3d R3_6 = R0_3.transpose() * R_tool;

            for (int wrist_flip = 0; wrist_flip < 2; ++wrist_flip) {
                // ZYZ 分解 R3_6 → a4, a5, a6
                double a5 = (wrist_flip == 0)
                          ? std::acos(R3_6(2, 2))
                          : -std::acos(R3_6(2, 2));
                double a4, a6;
                if (std::abs(std::sin(a5)) < 1e-6) {
                    // 奇异：a4 + a6 不可分
                    a4 = 0;
                    a6 = std::atan2(R3_6(1, 0), R3_6(0, 0));
                } else {
                    a4 = std::atan2(R3_6(1, 2) / std::sin(a5), R3_6(0, 2) / std::sin(a5));
                    a6 = std::atan2(R3_6(2, 1) / std::sin(a5), -R3_6(2, 0) / std::sin(a5));
                }

                JointConfig sol{{a1 * 180/M_PI, a2 * 180/M_PI, a3 * 180/M_PI,
                                 a4 * 180/M_PI, a5 * 180/M_PI, a6 * 180/M_PI}};
                // 关节限位检查
                bool in_limits = true;
                for (int i = 0; i < 6; ++i) {
                    if (sol.q_deg[i] < dh.qlim_deg[i][0] || sol.q_deg[i] > dh.qlim_deg[i][1]) {
                        in_limits = false; break;
                    }
                }
                if (!in_limits) continue;
                r.all_solutions[sol_idx++] = sol;
                r.num_valid++;
            }
        }
    }

    if (r.num_valid == 0) {
        r.status = IKStatus::JointLimit;
        return r;
    }

    // 选与 reference 最近的解
    double best_dist = 1e9;
    int best_idx = 0;
    for (int i = 0; i < r.num_valid; ++i) {
        double d = 0;
        for (int j = 0; j < 6; ++j) {
            double diff = r.all_solutions[i].q_deg[j] - reference.q_deg[j];
            // 处理 ±360° 周期
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;
            d += diff * diff;
        }
        if (d < best_dist) { best_dist = d; best_idx = i; }
    }
    r.solution = r.all_solutions[best_idx];
    r.status = IKStatus::OK;
    return r;
}
```

注：完整实现需要 `forwardKinMatrix_partial`（只算前 N 个 DH）。在 `forward_kin.cpp` 加：
```cpp
Eigen::Matrix4d forwardKinMatrix_partial(const JointConfig& q, const KR4DHParams& dh, int N) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < N; ++i) {
        T = T * dhMatrix(q.q_deg[i], dh.d_mm[i], dh.a_mm[i], dh.alpha_rad[i]);
    }
    return T;
}
```
并在 `forward_kin.h` 暴露。

`kukaABCToRotMat` 实现：
```cpp
Eigen::Matrix3d kukaABCToRotMat(double A_deg, double B_deg, double C_deg) {
    double A = A_deg * M_PI / 180, B = B_deg * M_PI / 180, C = C_deg * M_PI / 180;
    Eigen::Matrix3d Rz, Ry, Rx;
    Rz << std::cos(A), -std::sin(A), 0,  std::sin(A), std::cos(A), 0,  0, 0, 1;
    Ry << std::cos(B), 0, std::sin(B),  0, 1, 0,  -std::sin(B), 0, std::cos(B);
    Rx << 1, 0, 0,  0, std::cos(C), -std::sin(C),  0, std::sin(C), std::cos(C);
    return Rz * Ry * Rx;
}
```

加到 `CMakeLists.txt` 的源列表：
```cmake
src/kinematics/ik_solver.cpp
```

- [ ] **Step 4.4: 跑 round-trip 测试**

```
cmake --build build_nmake --target ik_solver_unit_test
ctest --test-dir build_nmake -R ik_solver_unit -V
```
Expected: PASS（至少 900/1000 round-trip 在 1e-3 精度内）。如果 fail，CLI #2 调试 DH 偏置（A3 = 0 时 a3 = 20mm 的几何处理是 IK 中最常出错的环节）。

- [ ] **Step 4.5: Commit**

```
git add src/kinematics/ik_solver.h src/kinematics/ik_solver.cpp src/kinematics/forward_kin.h src/kinematics/forward_kin.cpp tests/ik_solver_unit_test.cpp CMakeLists.txt
git commit -m "feat(kinematics): analytical IK with spherical wrist decoupling (v4 §11)

- 8-solution enumeration (front/back × elbow × wrist_flip)
- Joint limit + workspace filter
- Reference-distance multi-solution selection
- Round-trip test: 1000 random configs, > 90% within 1e-3 mm"
```

---

### Task 5: home 姿态实测验证

**Files:**
- Modify: `tests/ik_solver_unit_test.cpp`（追加测试用例）

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §11.6 + §15.4 现场实测 home 关节角

- [ ] **Step 5.1: 追加 home 验证 + IK 自洽**

在 `tests/ik_solver_unit_test.cpp` 主函数末尾加：
```cpp
// home 关节角 → FK → IK → 关节角应回到 home
JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};
CartPose home_pose = forwardKin(home, dh);
IKResult home_back = solveAnalyticalIK(home_pose, home, dh);
assert(home_back.status == IKStatus::OK);
for (int i = 0; i < 6; ++i) {
    double diff = std::abs(home_back.solution.q_deg[i] - home.q_deg[i]);
    while (diff > 180) diff = std::abs(diff - 360);
    assert(diff < 1e-3);
}
printf("home FK→IK round trip OK\n");
```

- [ ] **Step 5.2: 跑 + commit**

```
ctest --test-dir build_nmake -R ik_solver_unit -V
git add tests/ik_solver_unit_test.cpp
git commit -m "test(kinematics): home pose FK->IK self-consistency (v4 §11.6)"
```

---

### Task 6: 笛卡尔位姿构造（路径点 → CartPose）

**Files:**
- Create: `src/path/pose_from_path.h` + `.cpp`（独立小模块，§13 与 §11 之间的桥）
- Create: `tests/pose_from_path_unit_test.cpp`
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §11.2 + §13.2.1（路径切线方向规范化）

- [ ] **Step 6.1: 写测试**

创建 `tests/pose_from_path_unit_test.cpp`：
```cpp
#include "path/pose_from_path.h"
#include <cassert>

int main() {
    using namespace cslc;
    // 简单 case：路径点 (0,0,0)，切线 (1,0,0)，G = (0,0,1)（垂直向上）
    // 期望：tool_z = (0,0,-1), tool_x = (1,0,0), tool_y = (0,-1,0)
    Eigen::Vector3d p(0, 0, 0), t(1, 0, 0), G(0, 0, 1);
    Eigen::Matrix3d R = constructToolFrame(p, t, G);
    Eigen::Vector3d tool_z = R.col(2);
    Eigen::Vector3d tool_x = R.col(0);
    assert(std::abs(tool_z.z() + 1.0) < 1e-6);
    assert(std::abs(tool_x.x() - 1.0) < 1e-6);
    return 0;
}
```

- [ ] **Step 6.2: 实现**

创建 `src/path/pose_from_path.h`：
```cpp
#pragma once
#include "kinematics/cart_pose.h"
#include <Eigen/Dense>

namespace cslc {

// §11.2 工具坐标系构造
Eigen::Matrix3d constructToolFrame(
    const Eigen::Vector3d& point,         // unused but for clarity
    const Eigen::Vector3d& path_tangent,
    const Eigen::Vector3d& G_outward);

CartPose toolFrameToCartPose(const Eigen::Vector3d& point, const Eigen::Matrix3d& R_tool);

}  // namespace cslc
```

创建 `src/path/pose_from_path.cpp`：
```cpp
#include "path/pose_from_path.h"
#include "kinematics/forward_kin.h"  // for rotMatToKukaABC

namespace cslc {

Eigen::Matrix3d constructToolFrame(
    const Eigen::Vector3d&,
    const Eigen::Vector3d& path_tangent,
    const Eigen::Vector3d& G_outward) {
    Eigen::Vector3d tool_z = -G_outward.normalized();
    Eigen::Vector3d t = path_tangent;
    Eigen::Vector3d tool_x = (t - t.dot(tool_z) * tool_z).normalized();
    Eigen::Vector3d tool_y = tool_z.cross(tool_x);
    Eigen::Matrix3d R;
    R.col(0) = tool_x;
    R.col(1) = tool_y;
    R.col(2) = tool_z;
    return R;
}

CartPose toolFrameToCartPose(const Eigen::Vector3d& point, const Eigen::Matrix3d& R_tool) {
    CartPose p;
    p.X = point.x(); p.Y = point.y(); p.Z = point.z();
    rotMatToKukaABC(R_tool, p.A, p.B, p.C);
    return p;
}

}  // namespace cslc
```

加到 `CMakeLists.txt`：
```cmake
src/path/pose_from_path.cpp
add_executable(pose_from_path_unit_test tests/pose_from_path_unit_test.cpp)
target_link_libraries(pose_from_path_unit_test PRIVATE curved_slicer_phase0)
add_test(NAME pose_from_path_unit COMMAND pose_from_path_unit_test)
```

- [ ] **Step 6.3: 跑 + commit**

```
ctest --test-dir build_nmake -R pose_from_path_unit -V
git add src/path/pose_from_path.h src/path/pose_from_path.cpp tests/pose_from_path_unit_test.cpp CMakeLists.txt
git commit -m "feat(path): tool frame construction from tangent + G (v4 §11.2)"
```

---

## Stage C: §12 双层可达性（3 tasks）

### Task 7: 单点 reachability + 奇异性

**Files:**
- Create: `src/kinematics/reachability.h` + `.cpp`
- Create: `tests/reachability_unit_test.cpp`
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §12.2-§12.3

- [ ] **Step 7.1: 写测试**

创建 `tests/reachability_unit_test.cpp`，覆盖：
1. home 姿态点：reachable
2. 关节限位边界外：JointLimit
3. 工作空间外（远离 base 1000mm）：OutOfWorkspace
4. 腕奇异邻域（A5 ≈ 0）：NearSingularity
5. 臂态翻转检测（reference 与解差 > 90°）

```cpp
#include "kinematics/reachability.h"
#include "kinematics/forward_kin.h"
#include <cassert>

int main() {
    using namespace cslc;
    KR4DHParams dh;
    ReachabilityParams rp;
    JointConfig home{{0.0, 9.95, -62.26, 0.0, -37.69, 0.0}};

    // 1. home 可达
    CartPose home_pose = forwardKin(home, dh);
    auto r1 = checkReachability(home_pose, home, dh, rp);
    assert(r1.status == IKStatus::OK);

    // 2. 工作空间外
    CartPose far_pose{2000.0, 0, 500, 0, 0, 0};
    auto r2 = checkReachability(far_pose, home, dh, rp);
    assert(r2.status == IKStatus::OutOfWorkspace);

    // 3. 腕奇异（A5 = 0）
    JointConfig sing{{30, -30, 30, 45, 0.5, 60}};
    CartPose sing_pose = forwardKin(sing, dh);
    auto r3 = checkReachability(sing_pose, sing, dh, rp);
    assert(r3.status == IKStatus::NearSingularity);

    return 0;
}
```

- [ ] **Step 7.2: 实现**

创建 `src/kinematics/reachability.h`：
```cpp
#pragma once
#include "kinematics/ik_solver.h"

namespace cslc {

struct ReachabilityParams {
    double workspace_r_min_mm           = 100.0;
    double workspace_r_max_mm           = 600.0;
    double singularity_a5_deg           = 5.0;
    double singularity_shoulder_deg     = 5.0;
    double layer_skip_threshold         = 0.30;
    double layer_warn_threshold         = 0.10;
    double arm_flip_threshold_deg       = 90.0;
};

struct ReachabilityResult {
    IKStatus status;
    JointConfig solution;
    bool arm_status_flip;
};

ReachabilityResult checkReachability(
    const CartPose& pose,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params);

}  // namespace cslc
```

创建 `src/kinematics/reachability.cpp`：调用 `solveAnalyticalIK` 然后检查 ws + 奇异性 + 臂态翻转。

```cpp
#include "kinematics/reachability.h"
#include "kinematics/forward_kin.h"
#include <cmath>

namespace cslc {

ReachabilityResult checkReachability(
    const CartPose& pose,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params) {
    ReachabilityResult r{};
    double dist = std::sqrt(pose.X * pose.X + pose.Y * pose.Y + pose.Z * pose.Z);
    if (dist > params.workspace_r_max_mm || dist < params.workspace_r_min_mm) {
        r.status = IKStatus::OutOfWorkspace;
        return r;
    }
    IKResult ik = solveAnalyticalIK(pose, reference, dh);
    if (ik.status != IKStatus::OK) {
        r.status = ik.status;
        return r;
    }
    // 奇异性：A5 接近 0
    if (std::abs(ik.solution.q_deg[4]) < params.singularity_a5_deg) {
        r.status = IKStatus::NearSingularity;
        return r;
    }
    // 肩奇异：A2 + A3 接近 ±90
    double sum23 = ik.solution.q_deg[1] + ik.solution.q_deg[2];
    if (std::abs(sum23 - 90) < params.singularity_shoulder_deg
     || std::abs(sum23 + 90) < params.singularity_shoulder_deg) {
        r.status = IKStatus::NearSingularity;
        return r;
    }
    r.status = IKStatus::OK;
    r.solution = ik.solution;
    // 臂态翻转检测
    r.arm_status_flip = false;
    for (int i = 0; i < 6; ++i) {
        double diff = std::abs(ik.solution.q_deg[i] - reference.q_deg[i]);
        while (diff > 180) diff = std::abs(diff - 360);
        if (diff > params.arm_flip_threshold_deg) {
            r.arm_status_flip = true;
            break;
        }
    }
    return r;
}

}  // namespace cslc
```

- [ ] **Step 7.3: 跑 + commit**

```
ctest --test-dir build_nmake -R reachability_unit -V
git add src/kinematics/reachability.h src/kinematics/reachability.cpp tests/reachability_unit_test.cpp CMakeLists.txt
git commit -m "feat(kinematics): per-point reachability + singularity check (v4 §12)"
```

---

### Task 8: B 层（体素级）reachability filter

**Files:**
- Modify: `src/kinematics/reachability.h` + `.cpp`（加 `filterReachableVoxels`）
- Modify: `tests/reachability_unit_test.cpp`（加体素 case）

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §12.1, §12.4

- [ ] **Step 8.1: 接口扩展**

`reachability.h` 加：
```cpp
#include "geometry/voxel_grid.h"
#include "field/laplacian.h"  // VectorField

std::vector<bool> filterReachableVoxels(
    const VoxelGrid& grid,
    const VectorField& G,
    const JointConfig& home,
    const KR4DHParams& dh,
    const ReachabilityParams& params);
```

- [ ] **Step 8.2: 实现**

`reachability.cpp` 加：
```cpp
std::vector<bool> filterReachableVoxels(
    const VoxelGrid& grid,
    const VectorField& G,
    const JointConfig& home,
    const KR4DHParams& dh,
    const ReachabilityParams& params) {
    auto& voxels = grid.activeVoxels();
    std::vector<bool> reachable(voxels.size(), false);
    for (size_t i = 0; i < voxels.size(); ++i) {
        Eigen::Vector3d wpos = grid.voxelToWorld(voxels[i].idx).cast<double>();
        Eigen::Vector3d g = G.values[i].cast<double>();
        // 简化：tool_x 假设沿任意一个 ⊥g 方向
        Eigen::Vector3d arbitrary(1, 0, 0);
        if (std::abs(arbitrary.dot(g.normalized())) > 0.99) arbitrary = Eigen::Vector3d(0, 1, 0);
        Eigen::Matrix3d R = constructToolFrame(wpos, arbitrary, g);
        CartPose pose = toolFrameToCartPose(wpos, R);
        auto r = checkReachability(pose, home, dh, params);
        reachable[i] = (r.status == IKStatus::OK);
    }
    return reachable;
}
```

需要 `#include "path/pose_from_path.h"` 让 `constructToolFrame` 可见。

- [ ] **Step 8.3: 加测试 + commit**

`tests/reachability_unit_test.cpp` 加一个简单 case：构造 5×5×5 voxel grid，G 全部指向 +z，调用 `filterReachableVoxels`，期望 > 80% reachable（具体阈值视 grid 实际位置）。

```
ctest --test-dir build_nmake -R reachability_unit -V
git add src/kinematics/reachability.h src/kinematics/reachability.cpp tests/reachability_unit_test.cpp
git commit -m "feat(kinematics): B-layer voxel reachability filter (v4 §12.1)"
```

---

### Task 9: C 层（路径点级）reachability filter + 整层放弃逻辑

**Files:**
- Modify: `src/kinematics/reachability.h` + `.cpp`
- Modify: `tests/reachability_unit_test.cpp`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §12.1, §12.3

- [ ] **Step 9.1: 接口**

`reachability.h` 加：
```cpp
struct PathReachabilityResult {
    std::vector<JointConfig> joint_solutions;     // 不可达点 = NaN-填充
    std::vector<int>         unreachable_indices;
    bool                     layer_skipped;
    double                   unreachable_rate;
};

PathReachabilityResult filterReachablePathPoints(
    const std::vector<CartPose>& poses,
    const JointConfig& reference,                  // 第一帧用 home
    const KR4DHParams& dh,
    const ReachabilityParams& params);
```

- [ ] **Step 9.2: 实现**

```cpp
PathReachabilityResult filterReachablePathPoints(
    const std::vector<CartPose>& poses,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params) {
    PathReachabilityResult result;
    result.joint_solutions.reserve(poses.size());
    JointConfig prev = reference;
    int unreachable = 0;
    for (size_t i = 0; i < poses.size(); ++i) {
        auto r = checkReachability(poses[i], prev, dh, params);
        if (r.status == IKStatus::OK && !r.arm_status_flip) {
            result.joint_solutions.push_back(r.solution);
            prev = r.solution;
        } else {
            JointConfig nan_sol{};
            for (auto& q : nan_sol.q_deg) q = std::nan("");
            result.joint_solutions.push_back(nan_sol);
            result.unreachable_indices.push_back(static_cast<int>(i));
            unreachable++;
        }
    }
    result.unreachable_rate = static_cast<double>(unreachable) / poses.size();
    result.layer_skipped = (result.unreachable_rate > params.layer_skip_threshold);
    return result;
}
```

- [ ] **Step 9.3: 测试**

`tests/reachability_unit_test.cpp` 加：构造一个完全可达的简单弧形路径（home 周边小幅运动）→ 期望 0 不可达 + layer_skipped=false。

- [ ] **Step 9.4: 跑 + commit**

```
ctest --test-dir build_nmake -R reachability_unit -V
git add src/kinematics/reachability.h src/kinematics/reachability.cpp tests/reachability_unit_test.cpp
git commit -m "feat(kinematics): C-layer path point reachability + layer skip threshold (v4 §12)"
```

---

## Stage D: §13 测地路径生成（5 tasks）

### Task 10: 加 libigl 依赖到 CMake

**Files:**
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §13.3

- [ ] **Step 10.1: 加 FetchContent**

修改 `CMakeLists.txt`，在 eigen 块之后加：
```cmake
FetchContent_Declare(libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.5.0
    GIT_CONFIG http.sslBackend=openssl
    GIT_SUBMODULES ""
    GIT_PROGRESS TRUE
)
set(LIBIGL_INSTALL OFF CACHE BOOL "" FORCE)
set(LIBIGL_EMBREE OFF CACHE BOOL "" FORCE)
set(LIBIGL_GLFW OFF CACHE BOOL "" FORCE)
set(LIBIGL_OPENGL OFF CACHE BOOL "" FORCE)
set(LIBIGL_PNG OFF CACHE BOOL "" FORCE)
set(LIBIGL_STB OFF CACHE BOOL "" FORCE)
set(LIBIGL_PREDICATES OFF CACHE BOOL "" FORCE)
set(LIBIGL_XML OFF CACHE BOOL "" FORCE)

FetchContent_GetProperties(libigl)
if(NOT libigl_POPULATED)
    FetchContent_Populate(libigl)
endif()

if(NOT TARGET igl::core)
    add_library(igl::core INTERFACE IMPORTED GLOBAL)
    target_include_directories(igl::core INTERFACE ${libigl_SOURCE_DIR}/include)
    target_link_libraries(igl::core INTERFACE Eigen3::Eigen)
endif()
```

后面 `target_link_libraries(curved_slicer_phase0 ...)` 加 `igl::core`：
```cmake
target_link_libraries(curved_slicer_phase0
    PRIVATE
        tomlplusplus::tomlplusplus
        Eigen3::Eigen
        igl::core
)
```

- [ ] **Step 10.2: 测试 igl 头文件可用**

创建 `tests/igl_smoke_test.cpp`：
```cpp
#include <igl/exact_geodesic.h>
#include <igl/isolines.h>
#include <Eigen/Core>
#include <cassert>
int main() {
    Eigen::MatrixXd V(3, 3);
    V << 0,0,0, 1,0,0, 0,1,0;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    Eigen::VectorXi VS(1); VS << 0;
    Eigen::VectorXi FS, VT(2), FT;
    VT << 1, 2;
    Eigen::VectorXd D;
    igl::exact_geodesic(V, F, VS, FS, VT, FT, D);
    assert(D.size() == 2);
    return 0;
}
```

加到 CMakeLists：
```cmake
add_executable(igl_smoke_test tests/igl_smoke_test.cpp)
target_link_libraries(igl_smoke_test PRIVATE Eigen3::Eigen igl::core)
add_test(NAME igl_smoke COMMAND igl_smoke_test)
```

- [ ] **Step 10.3: 跑 + commit**

```
cmake --build build_nmake --target igl_smoke_test
ctest --test-dir build_nmake -R igl_smoke -V
git add CMakeLists.txt tests/igl_smoke_test.cpp
git commit -m "build: add libigl 2.5.0 dependency for geodesic paths (v4 §13)"
```

注：libigl 首次 fetch + build 较慢（5-15 min），CLI #2 实施时确认 cmake configure 完成。

---

### Task 11: 测地距离场 + isoline 提取

**Files:**
- Create: `src/path/geodesic_paths.h` + `.cpp`
- Create: `tests/geodesic_paths_unit_test.cpp`
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §13.2

- [ ] **Step 11.1: 写测试**

创建 `tests/geodesic_paths_unit_test.cpp`：构造一个简单单位球 mesh（icosphere subdivision 2 级），调用 `generateGeodesicPaths`，期望路径间距均匀。

```cpp
#include "path/geodesic_paths.h"
#include <cassert>
#include <Eigen/Dense>

int main() {
    using namespace cslc;
    // 单位正方形 mesh (在 z=0 平面，2×2 mesh: 4 顶点 2 三角面)
    Eigen::MatrixXd V(4, 3);
    V << 0,0,0,  1,0,0,  1,1,0,  0,1,0;
    Eigen::MatrixXi F(2, 3);
    F << 0,1,2,  0,2,3;

    GeodesicPathParams p;
    p.line_spacing_mm = 0.3;
    p.seed_strategy = 0;  // bbox 中心
    auto paths = generateGeodesicPaths(V, F, p);

    assert(paths.size() >= 2);   // 至少几条等值线
    for (const auto& path : paths) {
        assert(path.points.size() >= 2);
        assert(path.tangents.size() == path.points.size());
    }
    return 0;
}
```

- [ ] **Step 11.2: 实现**

创建 `src/path/geodesic_paths.h`：
```cpp
#pragma once
#include <Eigen/Dense>
#include <vector>

namespace cslc {

struct GeodesicPathParams {
    double line_spacing_mm   = 0.4;
    double resample_step_mm  = 0.2;
    int    seed_strategy     = 0;        // 0=bbox center, 1=user-specified
    Eigen::Vector3d user_seed_point{0,0,0};
    int    smooth_window     = 5;
};

struct PathPolyline {
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> tangents;
    bool   is_closed = true;
    double total_length_mm = 0.0;
};

std::vector<PathPolyline> generateGeodesicPaths(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const GeodesicPathParams& params);

}  // namespace cslc
```

创建 `src/path/geodesic_paths.cpp`：
```cpp
#include "path/geodesic_paths.h"
#include <igl/exact_geodesic.h>
#include <igl/isolines.h>
#include <limits>

namespace cslc {

static int findClosestVertex(const Eigen::MatrixXd& V, const Eigen::Vector3d& target) {
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < V.rows(); ++i) {
        double d = (V.row(i).transpose() - target).squaredNorm();
        if (d < best_d) { best_d = d; best = i; }
    }
    return best;
}

std::vector<PathPolyline> generateGeodesicPaths(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const GeodesicPathParams& params) {
    // 1. 选种子顶点
    Eigen::Vector3d seed_target;
    if (params.seed_strategy == 0) {
        seed_target = (V.colwise().minCoeff() + V.colwise().maxCoeff()).transpose() * 0.5;
    } else {
        seed_target = params.user_seed_point;
    }
    int seed_v = findClosestVertex(V, seed_target);
    Eigen::VectorXi VS(1); VS << seed_v;
    Eigen::VectorXi FS, VT(V.rows()), FT;
    for (int i = 0; i < V.rows(); ++i) VT(i) = i;
    Eigen::VectorXd D;
    igl::exact_geodesic(V, F, VS, FS, VT, FT, D);

    // 2. 提取等值线
    std::vector<PathPolyline> paths;
    double d_max = D.maxCoeff();
    for (double iso = params.line_spacing_mm; iso < d_max; iso += params.line_spacing_mm) {
        Eigen::MatrixXd iV;       // 等值线顶点
        Eigen::MatrixXi iE;       // 等值线 edge 索引（pair of indices into iV）
        Eigen::VectorXi iI;       // 每条 edge 在原 mesh 的 face id
        igl::isolines(V, F, D, Eigen::VectorXd::Constant(1, iso), iV, iE, iI);
        if (iE.rows() == 0) continue;

        // 3. 把 edges 拼成 polyline（简单贪心）
        // CLI #2 实施细化：对 iE 做 union-find / 邻接列表，提取每条闭合环
        // 暂用最简单：把所有 iV 顺序放入 polyline
        PathPolyline poly;
        for (int i = 0; i < iV.rows(); ++i) {
            poly.points.push_back(iV.row(i).transpose());
        }
        // 切线：前向差分
        poly.tangents.resize(poly.points.size());
        for (size_t i = 0; i + 1 < poly.points.size(); ++i) {
            poly.tangents[i] = (poly.points[i+1] - poly.points[i]).normalized();
        }
        if (!poly.points.empty()) {
            poly.tangents.back() = poly.tangents.size() > 1 ? poly.tangents[poly.tangents.size()-2] : Eigen::Vector3d(1,0,0);
        }
        paths.push_back(std::move(poly));
    }

    return paths;
}

}  // namespace cslc
```

注：`igl::isolines` 输出的 `iE` 是边集合，CLI #2 实施时需要写一个 polyline-stitching 算法把 edges 拼成连续 polyline（可能需要邻接遍历 + DFS）。上面实现是骨架，仅证明能跑通；真正连续 polyline 在 Task 12 加。

加到 CMakeLists：
```cmake
src/path/geodesic_paths.cpp
add_executable(geodesic_paths_unit_test tests/geodesic_paths_unit_test.cpp)
target_link_libraries(geodesic_paths_unit_test PRIVATE curved_slicer_phase0 igl::core)
add_test(NAME geodesic_paths_unit COMMAND geodesic_paths_unit_test)
```

- [ ] **Step 11.3: 跑 + commit**

```
ctest --test-dir build_nmake -R geodesic_paths_unit -V
git add src/path/geodesic_paths.h src/path/geodesic_paths.cpp tests/geodesic_paths_unit_test.cpp CMakeLists.txt
git commit -m "feat(path): libigl exact_geodesic + isolines extraction (v4 §13.2)"
```

---

### Task 12: Polyline stitching + 起点 + 方向规范化

**Files:**
- Modify: `src/path/geodesic_paths.cpp`（替换 stitching 实现）
- Modify: `tests/geodesic_paths_unit_test.cpp`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §13.2.1

- [ ] **Step 12.1: 实现 polyline stitching**

`src/path/geodesic_paths.cpp` 加辅助函数：
```cpp
static std::vector<PathPolyline> stitchEdgesToPolylines(
    const Eigen::MatrixXd& iV,
    const Eigen::MatrixXi& iE) {
    // 1. 邻接表
    std::vector<std::vector<int>> adj(iV.rows());
    for (int e = 0; e < iE.rows(); ++e) {
        adj[iE(e, 0)].push_back(iE(e, 1));
        adj[iE(e, 1)].push_back(iE(e, 0));
    }
    std::vector<bool> visited(iV.rows(), false);
    std::vector<PathPolyline> result;
    for (int start = 0; start < iV.rows(); ++start) {
        if (visited[start] || adj[start].empty()) continue;
        PathPolyline poly;
        int cur = start, prev = -1;
        while (cur >= 0 && !visited[cur]) {
            visited[cur] = true;
            poly.points.push_back(iV.row(cur).transpose());
            int next = -1;
            for (int nb : adj[cur]) {
                if (nb != prev && !visited[nb]) { next = nb; break; }
            }
            // 闭合环：next == start 但 visited[start] 已标
            prev = cur; cur = next;
        }
        if (poly.points.size() >= 2) {
            poly.is_closed = (adj[start].size() >= 2 && std::find(adj[start].begin(), adj[start].end(), poly.points.size() > 1 ? prev : -1) != adj[start].end());
            result.push_back(std::move(poly));
        }
    }
    return result;
}
```

- [ ] **Step 12.2: 起点规范化（z 最小，§13.2.1）**

```cpp
static void normalizeStartPoint(PathPolyline& poly) {
    if (!poly.is_closed || poly.points.size() < 3) return;
    // 找 z 最小的顶点（同 z 取 x 最小）
    int min_idx = 0;
    for (size_t i = 1; i < poly.points.size(); ++i) {
        if (poly.points[i].z() < poly.points[min_idx].z()
         || (poly.points[i].z() == poly.points[min_idx].z()
             && poly.points[i].x() < poly.points[min_idx].x())) {
            min_idx = static_cast<int>(i);
        }
    }
    if (min_idx == 0) return;
    // 旋转 polyline 让 min_idx 成为起点
    std::rotate(poly.points.begin(), poly.points.begin() + min_idx, poly.points.end());
}
```

- [ ] **Step 12.3: 方向规范化（暂仅按 G 占位，§11 集成时再细化）**

留 TODO 在 generateGeodesicPaths 末尾，§11 流水线集成时由调用方决定是否反转 polyline。

- [ ] **Step 12.4: 切线计算**

```cpp
static void computeTangents(PathPolyline& poly) {
    poly.tangents.resize(poly.points.size());
    int n = static_cast<int>(poly.points.size());
    for (int i = 0; i < n; ++i) {
        int prev = (i - 1 + n) % n;
        int next = (i + 1) % n;
        poly.tangents[i] = (poly.points[next] - poly.points[prev]).normalized();
    }
}
```

替换 generateGeodesicPaths 主循环用上面 3 个 helper。

- [ ] **Step 12.5: 加测试 + commit**

`tests/geodesic_paths_unit_test.cpp` 追加：验证 polyline 闭合 + 起点 z 最小。

```
ctest --test-dir build_nmake -R geodesic_paths_unit -V
git add src/path/geodesic_paths.cpp tests/geodesic_paths_unit_test.cpp
git commit -m "feat(path): polyline stitching + start point normalization (v4 §13.2.1)"
```

---

### Task 13: 路径后处理 wlsFilter + LinearSmooth（简化迁移）

**Files:**
- Create: `src/path/path_postprocessing.h` + `.cpp`
- Create: `tests/path_postprocessing_unit_test.cpp`
- Modify: `CMakeLists.txt`

**算法依据：** 老项目 `pathPostprocessing.cpp/h` 简化版 + `V3_TO_V4_AMENDMENTS.md` §13.3

- [ ] **Step 13.1: 实现 + 测试**

CLI #2 参考老项目 `E:\Git_Repository\STL-free Print\project\` 下的 `pathPostprocessing.cpp/h`，挑出 `wlsFilter1d` + `LinearSmooth31/51/52` 三个函数干净迁移到新文件。删除调试输出、删除老 namespace。

接口：
```cpp
namespace cslc {

void smoothPolyline_LinearSmooth31(std::vector<Eigen::Vector3d>& points);
void smoothPolyline_LinearSmooth51(std::vector<Eigen::Vector3d>& points);
void smoothPolyline_LinearSmooth52(std::vector<Eigen::Vector3d>& points);
void smoothPolyline_WLS1D(std::vector<Eigen::Vector3d>& points, int window);

}
```

测试：构造一段加了高斯噪声的直线，验证平滑后噪声方差降低 > 80%。

- [ ] **Step 13.2: 跑 + commit**

```
ctest --test-dir build_nmake -R path_postprocessing_unit -V
git add src/path/path_postprocessing.h src/path/path_postprocessing.cpp tests/path_postprocessing_unit_test.cpp CMakeLists.txt
git commit -m "feat(path): polyline smoothing (wlsFilter + LinearSmooth) migration (v4 §13.3)"
```

---

### Task 14: 等值线重采样到固定步长

**Files:**
- Modify: `src/path/geodesic_paths.cpp`（加 `resamplePolyline` helper + 用到主函数）

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §13.4 (`resample_step_mm` 参数)

- [ ] **Step 14.1: 实现等步长重采样**

```cpp
static PathPolyline resamplePolyline(const PathPolyline& src, double step_mm) {
    PathPolyline out;
    out.is_closed = src.is_closed;
    if (src.points.size() < 2) return out;
    double accum = 0.0;
    out.points.push_back(src.points[0]);
    for (size_t i = 1; i < src.points.size(); ++i) {
        Eigen::Vector3d seg = src.points[i] - src.points[i-1];
        double seg_len = seg.norm();
        while (accum + seg_len >= step_mm) {
            double t = (step_mm - accum) / seg_len;
            out.points.push_back(src.points[i-1] + t * seg);
            seg = src.points[i] - out.points.back();
            seg_len = seg.norm();
            accum = 0.0;
        }
        accum += seg_len;
    }
    return out;
}
```

调用：`generateGeodesicPaths` 主循环里 `paths.push_back(resamplePolyline(poly, params.resample_step_mm));` 替换原来的直接 push。

- [ ] **Step 14.2: 测试 + commit**

`tests/geodesic_paths_unit_test.cpp` 验证：相邻点间距 ≈ resample_step_mm（±5%）。

```
ctest --test-dir build_nmake -R geodesic_paths_unit -V
git add src/path/geodesic_paths.cpp tests/geodesic_paths_unit_test.cpp
git commit -m "feat(path): polyline uniform resampling (v4 §13.4)"
```

---

## Stage E: §14 5 次多项式平滑（4 tasks）

### Task 15: TrajectoryPoint + 5 次多项式 segment 求解

**Files:**
- Create: `src/trajectory/poly5_smoother.h` + `.cpp`
- Create: `tests/poly5_smoother_unit_test.cpp`
- Modify: `CMakeLists.txt`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §14.2 (5 次多项式闭式解)

- [ ] **Step 15.1: 类型 + 失败测试**

创建 `src/trajectory/poly5_smoother.h`：
```cpp
#pragma once
#include "kinematics/cart_pose.h"
#include <Eigen/Dense>
#include <array>
#include <vector>

namespace cslc {

struct TrajectoryParams {
    double target_line_speed_mm_per_s    = 5.0;
    double sample_period_ms              = 4.0;
    double max_joint_velocity_deg_per_s  = 200.0;
    double max_joint_accel_deg_per_s2    = 500.0;
    double max_joint_delta_per_cycle_deg = 3.0;
    bool   zero_endpoint_velocity        = true;
};

struct PathPointWithJoints {
    Eigen::Vector3d cart_pos;
    JointConfig     joint;
    bool            wire_on = true;
};

struct TrajectoryPoint {
    double timestamp_ms;
    std::array<double, 6> joint_deg;
    std::array<double, 6> joint_velocity_deg_per_s;
    std::array<double, 6> joint_accel_deg_per_s2;
    bool   wire_on;
    double plc_mode  = 1.0;
    double plc_speed = 5.0;
    double plc_ratio = 1000.0;
};

std::vector<TrajectoryPoint> smoothTrajectoryPoly5(
    const std::vector<PathPointWithJoints>& path,
    const TrajectoryParams& params);

// 5 次多项式系数（端点零速度零加速度版）
struct Poly5Coeffs { double c0, c1, c2, c3, c4, c5; };
Poly5Coeffs computePoly5(double q0, double qT, double T);

}  // namespace cslc
```

创建 `tests/poly5_smoother_unit_test.cpp`：
```cpp
#include "trajectory/poly5_smoother.h"
#include <cassert>
#include <cmath>

int main() {
    using namespace cslc;
    // 端点 0→10 deg, T=1s: 验证 q(0)=0, q(T)=10, q'(0)=q'(T)=0, q''(0)=q''(T)=0
    auto c = computePoly5(0.0, 10.0, 1.0);
    auto eval = [&](double t) {
        return c.c0 + c.c1*t + c.c2*t*t + c.c3*t*t*t + c.c4*t*t*t*t + c.c5*t*t*t*t*t;
    };
    auto evald = [&](double t) {
        return c.c1 + 2*c.c2*t + 3*c.c3*t*t + 4*c.c4*t*t*t + 5*c.c5*t*t*t*t;
    };
    assert(std::abs(eval(0)) < 1e-9);
    assert(std::abs(eval(1) - 10) < 1e-9);
    assert(std::abs(evald(0)) < 1e-9);
    assert(std::abs(evald(1)) < 1e-9);
    return 0;
}
```

- [ ] **Step 15.2: 实现 computePoly5**

`src/trajectory/poly5_smoother.cpp`：
```cpp
#include "trajectory/poly5_smoother.h"

namespace cslc {

Poly5Coeffs computePoly5(double q0, double qT, double T) {
    // 边界条件: q(0)=q0, q(T)=qT, q'(0)=q'(T)=0, q''(0)=q''(T)=0
    // 闭式解: c0=q0, c1=c2=0, c3=10D/T³, c4=-15D/T⁴, c5=6D/T⁵, D=qT-q0
    Poly5Coeffs c;
    double D = qT - q0;
    double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;
    c.c0 = q0;
    c.c1 = 0.0;
    c.c2 = 0.0;
    c.c3 = 10.0 * D / T3;
    c.c4 = -15.0 * D / T4;
    c.c5 = 6.0 * D / T5;
    return c;
}

}  // namespace cslc
```

加到 CMakeLists：
```cmake
src/trajectory/poly5_smoother.cpp
add_executable(poly5_smoother_unit_test tests/poly5_smoother_unit_test.cpp)
target_link_libraries(poly5_smoother_unit_test PRIVATE curved_slicer_phase0)
add_test(NAME poly5_smoother_unit COMMAND poly5_smoother_unit_test)
```

- [ ] **Step 15.3: 跑 + commit**

```
ctest --test-dir build_nmake -R poly5_smoother_unit -V
git add src/trajectory/poly5_smoother.h src/trajectory/poly5_smoother.cpp tests/poly5_smoother_unit_test.cpp CMakeLists.txt
git commit -m "feat(trajectory): 5th-order polynomial closed-form solution (v4 §14.2)"
```

---

### Task 16: 双时间约束 + 段长决定

**Files:**
- Modify: `src/trajectory/poly5_smoother.cpp`（加 `computeSegmentDuration`）
- Modify: `tests/poly5_smoother_unit_test.cpp`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §14.3

- [ ] **Step 16.1: 实现 computeSegmentDuration**

`poly5_smoother.cpp` 加：
```cpp
static double computeSegmentDuration(
    const PathPointWithJoints& a,
    const PathPointWithJoints& b,
    const TrajectoryParams& params) {
    double d_cart = (b.cart_pos - a.cart_pos).norm();
    double dt_cart = d_cart / params.target_line_speed_mm_per_s;  // 秒
    double max_dq = 0;
    for (int i = 0; i < 6; ++i) {
        max_dq = std::max(max_dq, std::abs(b.joint.q_deg[i] - a.joint.q_deg[i]));
    }
    double dt_joint = max_dq / params.max_joint_velocity_deg_per_s;
    return std::max(dt_cart, dt_joint);
}
```

- [ ] **Step 16.2: 测试 + commit**

加测试：极端 case（TCP 不动但关节翻 30°）→ dt_joint 主导。

```
git add src/trajectory/poly5_smoother.cpp tests/poly5_smoother_unit_test.cpp
git commit -m "feat(trajectory): dual time constraint (cartesian + joint) (v4 §14.3)"
```

---

### Task 17: 4ms 周期采样 + smoothTrajectoryPoly5 主函数

**Files:**
- Modify: `src/trajectory/poly5_smoother.cpp`
- Modify: `tests/poly5_smoother_unit_test.cpp`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §14.4

- [ ] **Step 17.1: 实现主函数**

```cpp
std::vector<TrajectoryPoint> smoothTrajectoryPoly5(
    const std::vector<PathPointWithJoints>& path,
    const TrajectoryParams& params) {
    std::vector<TrajectoryPoint> out;
    if (path.size() < 2) return out;
    double sample_dt_ms = params.sample_period_ms;
    double timestamp_ms = 0.0;

    // 第一帧
    {
        TrajectoryPoint p0;
        p0.timestamp_ms = 0;
        p0.joint_deg = path[0].joint.q_deg;
        std::fill(p0.joint_velocity_deg_per_s.begin(), p0.joint_velocity_deg_per_s.end(), 0.0);
        std::fill(p0.joint_accel_deg_per_s2.begin(), p0.joint_accel_deg_per_s2.end(), 0.0);
        p0.wire_on = path[0].wire_on;
        out.push_back(p0);
        timestamp_ms = sample_dt_ms;
    }

    for (size_t k = 0; k + 1 < path.size(); ++k) {
        double T_s = computeSegmentDuration(path[k], path[k+1], params);
        // 6 个关节各自 5 次多项式
        std::array<Poly5Coeffs, 6> coefs;
        for (int i = 0; i < 6; ++i) {
            coefs[i] = computePoly5(path[k].joint.q_deg[i], path[k+1].joint.q_deg[i], T_s);
        }
        double T_ms = T_s * 1000.0;
        double t_ms = (out.back().timestamp_ms < 1e-9 ? sample_dt_ms : sample_dt_ms);
        // 在该段内每 sample_dt_ms 采样一次
        for (double tau_ms = sample_dt_ms; tau_ms <= T_ms + 1e-6; tau_ms += sample_dt_ms) {
            double tau_s = tau_ms / 1000.0;
            TrajectoryPoint pt;
            pt.timestamp_ms = timestamp_ms + tau_ms - sample_dt_ms;
            for (int i = 0; i < 6; ++i) {
                auto& c = coefs[i];
                double s = tau_s;
                pt.joint_deg[i] = c.c0 + c.c1*s + c.c2*s*s + c.c3*s*s*s + c.c4*s*s*s*s + c.c5*s*s*s*s*s;
                pt.joint_velocity_deg_per_s[i] = c.c1 + 2*c.c2*s + 3*c.c3*s*s + 4*c.c4*s*s*s + 5*c.c5*s*s*s*s;
                pt.joint_accel_deg_per_s2[i] = 2*c.c2 + 6*c.c3*s + 12*c.c4*s*s + 20*c.c5*s*s*s;
            }
            pt.wire_on = path[k+1].wire_on;
            out.push_back(pt);
        }
        timestamp_ms = out.back().timestamp_ms + sample_dt_ms;
    }

    return out;
}
```

- [ ] **Step 17.2: 测试**

`tests/poly5_smoother_unit_test.cpp` 加：
- 简单 2 点 path，跑 smooth → 验证时间戳间隔恒等于 4ms
- 关节速度峰值 ≤ max_joint_velocity_deg_per_s
- 段间位置连续

- [ ] **Step 17.3: 跑 + commit**

```
ctest --test-dir build_nmake -R poly5_smoother_unit -V
git add src/trajectory/poly5_smoother.cpp tests/poly5_smoother_unit_test.cpp
git commit -m "feat(trajectory): smoothTrajectoryPoly5 main function (v4 §14.4)"
```

---

### Task 18: 增量上限校验

**Files:**
- Modify: `src/trajectory/poly5_smoother.cpp`
- Modify: `tests/poly5_smoother_unit_test.cpp`

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §14.5 + §14.7

- [ ] **Step 18.1: 实现 + 测试**

在 `smoothTrajectoryPoly5` 末尾加校验循环：相邻两点关节角差 < `max_joint_delta_per_cycle_deg`。如果违反，抛 `std::runtime_error`（or 返回空 vector + log）。

测试：故意构造一个段内关节角变化超快的 case，验证校验触发。

- [ ] **Step 18.2: Commit**

```
ctest --test-dir build_nmake -R poly5_smoother_unit -V
git add src/trajectory/poly5_smoother.cpp tests/poly5_smoother_unit_test.cpp
git commit -m "feat(trajectory): per-cycle joint delta cap validation (v4 §14.5)"
```

---

## Stage F: Pipeline 集成 + 端到端测试（2 tasks）

### Task 19: pipeline.cpp 加 vector_kuka_v4 分支

**Files:**
- Modify: `src/app/pipeline.cpp`（加新流水线分支）
- Modify: `src/app/pipeline.h`（如有公共接口变化）
- Modify: `src/io/config_loader.cpp`（若 toml 用 `pipeline = "vector_kuka_v4"` 触发新分支）

**算法依据：** `V3_TO_V4_AMENDMENTS.md` §17.5 (pipeline 数据流图)

- [ ] **Step 19.1: 加新分支**

`src/app/pipeline.cpp` 找到现有 `if (cfg.algorithm.pipeline == "vector_kuka")` 段，复制改造为新分支：
```cpp
if (cfg.algorithm.pipeline == "vector_kuka_v4") {
    // 1. SDF + voxelize（同 vector_kuka）
    // 2. generateBC (geometric_z)
    // 3. solveLaplacianVector → G 场
    // 4. solvePoisson → φ 场
    // 5. iso_surface (MC) → 每层 V/F mesh
    // 6. 对每层：
    //    a. generateGeodesicPaths
    //    b. constructToolFrame + toolFrameToCartPose → vector<CartPose>
    //    c. filterReachablePathPoints → vector<JointConfig>
    //    d. 组装 vector<PathPointWithJoints>
    // 7. 拼接所有层 → vector<PathPointWithJoints>（暂不插 transitions, Plan 2 做）
    // 8. smoothTrajectoryPoly5 → vector<TrajectoryPoint>
    // 9. 输出到 runs/<run_name>/<model_name>/trajectory.csv（暂用 csv，§15a/b 在 Plan 2 做）
}
```

- [ ] **Step 19.2: trajectory.csv 输出**

简单 CSV：每行 `timestamp_ms, A1, A2, A3, A4, A5, A6, wire_on`。

- [ ] **Step 19.3: Commit**

```
git add src/app/pipeline.cpp src/io/config_loader.cpp
git commit -m "feat(pipeline): vector_kuka_v4 branch integrating §10-§14 (v4 §17.5)"
```

---

### Task 20: mao_rescaled 端到端集成测试

**Files:**
- Create: `config/mao_rescaled_v4.toml`（基于 `mao_rescaled.toml` 改 pipeline + 新参数）
- Create: `tests/v4_pipeline_e2e_test.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 20.1: 写新 config**

`config/mao_rescaled_v4.toml`：
```toml
[io]
output_root              = "../runs/v4_plan1_mao_rescaled"
runs_csv                 = "../runs/v4_plan1_mao_rescaled/runs.csv"
debug_dump_intermediates = true

[[io.models]]
name     = "mao_rescaled"
stl_path = "../tests/models/毛主席头雕(42_52_70)_1_Rescaled(0.7).stl"

[voxel]
spacing_mm   = 0.5
padding_mm   = 0.0
sdf_band_mm  = 1.0

[field.boundary]
strategy = "geometric_z"
print_direction = [0.0, 0.0, 1.0]

[algorithm]
pipeline = "vector_kuka_v4"

[algorithm.field.laplacian]
max_iterations = 2000
tolerance      = 1e-6

[algorithm.field.poisson]
max_iterations = 500
tolerance      = 1e-6

[iso_surface]
layer_thickness_mm = 0.2
iso_spacing        = "uniform"

[path]
strategy = "geodesic"
line_spacing_mm  = 0.4
resample_step_mm = 0.2

[trajectory]
target_line_speed_mm_per_s    = 5.0
sample_period_ms              = 4.0
max_joint_velocity_deg_per_s  = 200.0

[kuka.home]
joint_deg = [0.0, 9.95, -62.26, 0.0, -37.69, 0.0]

# kuka.limits / kuka.robot 同 mao_rescaled.toml
```

- [ ] **Step 20.2: 端到端测试**

`tests/v4_pipeline_e2e_test.cpp`：调用 `runPipeline(config_path)`，检查 trajectory.csv 存在 + 至少 1000 行（每行 4ms，覆盖几秒打印）。

```cpp
#include "app/pipeline.h"
#include "io/config_loader.h"
#include <cassert>
#include <fstream>
#include <filesystem>

int main() {
    using namespace cslc;
    auto cfg = loadPipelineConfig("../../config/mao_rescaled_v4.toml");
    runPipeline(cfg);
    auto traj_path = std::filesystem::path("../../runs/v4_plan1_mao_rescaled/mao_rescaled/trajectory.csv");
    assert(std::filesystem::exists(traj_path));
    std::ifstream f(traj_path);
    int line_count = 0;
    std::string line;
    while (std::getline(f, line)) ++line_count;
    assert(line_count > 1000);  // 包括 header
    return 0;
}
```

加到 CMakeLists：
```cmake
add_executable(v4_pipeline_e2e_test tests/v4_pipeline_e2e_test.cpp)
target_link_libraries(v4_pipeline_e2e_test PRIVATE curved_slicer_phase0)
add_test(NAME v4_pipeline_e2e COMMAND v4_pipeline_e2e_test)
```

- [ ] **Step 20.3: 跑 + commit**

```
cd /d E:\Git_Repository\curved-slicer
cmake --build build_nmake --target v4_pipeline_e2e_test
ctest --test-dir build_nmake -R v4_pipeline_e2e -V --timeout 1800
git add config/mao_rescaled_v4.toml tests/v4_pipeline_e2e_test.cpp CMakeLists.txt
git commit -m "test(e2e): v4 plan 1 mao_rescaled end-to-end (v4 §10-§14)"
```

---

## Self-Review Notes

**Spec coverage**：
- ✅ §10 Task 1
- ✅ §11 Tasks 2-6（DH, FK, IK, home 验证, pose construction）
- ✅ §12 Tasks 7-9（单点、B 层、C 层）
- ✅ §13 Tasks 10-14（libigl 集成、isolines、stitching、起点规范化、重采样、后处理）
- ✅ §14 Tasks 15-18（5 次多项式、双时间约束、采样、增量校验）
- ✅ Pipeline 集成 Tasks 19-20

**Placeholder 扫描**：
- Task 4 IK solver 中 `forwardKinMatrix_partial` 在 Task 3 forward_kin.cpp 已声明，CLI #2 在 Task 4 实施时需要回 Task 3 文件加这一函数（已在 Task 4 内说明）
- Task 11 polyline stitching 是骨架版，正式实现在 Task 12 — 已注明
- Task 17 主函数中 `t_ms` 变量初始化逻辑值得 CLI #2 在实施时仔细 review

**类型一致性**：
- `JointConfig.q_deg` 是 `std::array<double, 6>`，全部 task 沿用 ✓
- `CartPose` 字段 X/Y/Z/A/B/C 全部 task 一致 ✓
- `TrajectoryPoint.joint_deg` 是 `std::array<double, 6>`，与 `JointConfig.q_deg` 一致 ✓
- `PathPointWithJoints` 在 §14 (Task 15) 定义，§16/§17 Plan 2 复用 ✓

**已知风险**：
1. Task 4 IK 实现 DH 偏置（A3 = 20mm）几何处理是 IK 中最易错的点。round-trip 测试可能首次 fail，CLI #2 需要 plot 出错的 case 调试。
2. Task 11 libigl 首次 FetchContent + build 慢（5-15 min）。
3. Task 19 pipeline 集成可能与现有 vector_kuka 分支命名冲突，注意配置 `pipeline = "vector_kuka_v4"` 而不是覆盖 `"vector_kuka"`。
4. Task 20 端到端测试可能跑 5-30 min（取决于模型大小 + Poisson 收敛速度）。设了 1800s timeout。

---

## 实施完成验收清单

CLI #2 跑完 Plan 1 后，下面全绿才算完工：

- [ ] `ctest --test-dir build_nmake --output-on-failure` 全部通过（含 Plan 1 所有新增测试 + 原有 7 项测试）
- [ ] mao_rescaled v4 流水线产出 `runs/v4_plan1_mao_rescaled/mao_rescaled/trajectory.csv`，行数 > 1000
- [ ] `git log --oneline | head -30` 看到 Task 1-20 各自独立的 commit
- [ ] CLI #1 review 一份 50 行截取的 trajectory.csv，检查 timestamp 间隔 == 4ms、关节角变化平滑、A1-A6 都在 qlim 内
- [ ] V3_TO_V4_AMENDMENTS.md §10-§14 所有"工程量"估计行的 LOC + 天数与实际偏差 < 30%
