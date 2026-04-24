# 新项目架构设计 · v3（修订版）

> 本文件是 `NEW_PROJECT_ARCHITECTURE_v2.md` 的进一步修订版，在 v2 基础上做了 4 处增补/修正：
>
> 1. 补全 `IsoExtractParams` 和 `PoissonParams` 字段（v2 只给了函数签名）
> 2. 明确 `PathPoint` / `PathLayer` 数据结构（M3 指标的计算依赖）
> 3. Phase 2 / Phase 3 验证标准量化（原"有明显改善"改成数值阈值）
> 4. 纠正映射表与 Phase 1 的矛盾：wavefront 与 laplacian+poisson **并存**，通过配置 `[algorithm] mode` 切换
>
> v2 的 6 项修订（KUKA 非对称极限、batch 模式、`generateBC`、Phase 0-5 路线图、9 条 workaround 处理、4 项直接决策）在 v3 中**全部保留**。
>
> v3 相对 v2 的 diff 主要集中在 §2（新增 4 个接口子节）、§4（新增 `[algorithm]` 配置节）、§6（映射表拆分 1 行、新增 1 行）、§8（Phase 2/3 量化验证），其余从 v2 保留。

---

## §1 目录结构（v2 保留，无改动）

见 v2 §1。子目录仍为 7 个（core/io/geometry/field/surface/path/metrics/app），未因本次新增 `wavefront.h` 增加目录——新文件归入已有的 `src/field/`。

---

## §2 核心接口改动

本节分两部分：
- **§2.1 – §2.6** 保留 v2 §2 的六个接口（stl_reader、laplacian、kuka_projection、smoothing、curvature、pipeline），未改动。
- **§2.7 – §2.10** 为 v3 新增：补全 v2 中只给了函数签名的接口细节。

### §2.1 `io/stl_reader.h`（v2 保留，无改动）

见 v2 §2.1。Windows 中文路径处理规范不变。

### §2.2 `field/laplacian.h`（v2 保留，无改动）

见 v2 §2.2。`generateBC` + `BCParams` 三种策略（bottom_up / bottom_and_top / reeb_graph）不变。

### §2.3 `field/kuka_projection.h`（v2 保留，无改动）

见 v2 §2.3。KR 4 R600 非对称极限 + 6 轴额定速度不变：

| 轴 | min (°) | max (°) | speed (°/s) |
|---|---:|---:|---:|
| A1 | -170 | 170 | 336 |
| A2 | **-195** | **40** | 336 |
| A3 | **-115** | **150** | 488 |
| A4 | -185 | 185 | 600 |
| A5 | -120 | 120 | 529 |
| A6 | -350 | 350 | 800 |

### §2.4 `path/smoothing.h`（v2 保留，无改动）

见 v2 §2.4。线性滑动 + WLS + Catmull-Rom 三件套不变。

### §2.5 `metrics/curvature.h`（v2 保留，无改动）

见 v2 §2.5。只保留 `computeMeanCurvature` + `maxAbsMeanCurvature` 两个函数。

### §2.6 `app/pipeline.h`（v2 保留，无改动）

见 v2 §2.6。`ModelJob` / `PipelineConfig` / `runSingle` / `runBatch` 不变。

### §2.7 `field/poisson.h` —— 补全 `PoissonParams`（v3 修订 1）

v2 的 §6 映射表只声明了"新增 Poisson 求解"，未给接口细节。v3 补上：

```cpp
#pragma once
#include "core/types.h"
#include "field/laplacian.h"          // VectorField
#include "geometry/voxel_grid.h"

namespace cslc {

// Poisson 方程参数
//   Δφ = ∇·G  （G 为投影后的向量场）
//   边界条件：在 anchor_voxel 处钉 φ=0（消除刚性自由度）；其余为齐次 Neumann
struct PoissonParams {
    int max_iterations   = 500;       // CG 上限
    double tolerance     = 1e-6;      // 相对残差
    bool use_precondition = true;     // true=ICC（SimplicialCholesky）/ false=纯 CG

    // anchor：消除零空间。默认用 generateBC 返回的 fixed_indices[0] 即可；
    // 若 bc 为空（理论上不应发生），pipeline 会兜底选 voxel (0,0,0)。
    VoxelIndex anchor_voxel{0, 0, 0};
};

// 标量场（与 VectorField 布局对齐，方便同 grid 上做差分/等值面）
struct ScalarField {
    AABB bbox;
    double spacing = 0.0;
    int nx = 0, ny = 0, nz = 0;
    std::vector<double> values;       // size = nx*ny*nz
};

ScalarField solvePoisson(const VoxelGrid& grid,
                          const VectorField& G,
                          const PoissonParams& params);

}  // namespace cslc
```

**Poisson 右端项说明**：`solvePoisson` 内部用 `G` 的散度 `∇·G` 作为 RHS。`G` 是否已经过 KUKA 投影，由 `pipeline` 决定（Phase 2 用未投影 G；Phase 3 起用投影后 G）。接口本身不区分，保持单一职责。

### §2.8 `surface/iso_surface.h` —— 补全 `IsoExtractParams`（v3 修订 1）

```cpp
#pragma once
#include "core/types.h"
#include "field/poisson.h"            // ScalarField

namespace cslc {

// 等值面层厚规划参数
//   核心流程：planIsoLevels(phi, params) → 一组 iso 值 → 对每个 iso 值做 MC
struct IsoExtractParams {
    double layer_thickness_mm = 0.8;          // 目标相邻层间距（沿 φ 梯度方向）
    std::string iso_spacing   = "uniform";    // "uniform"       : φ 值等间距
                                              // "volume_equal"  : 按实体体积均分（CV 更小，计算更贵）
    int max_layers            = 500;          // 安全上限，防止参数错误导致层数爆炸
    double phi_start_offset   = 0.0;          // 起始 iso 值相对 φ_min 的偏移；
                                              // 常用：=0 表示从底面起；>0 可跳过底层薄壳
};

// 单个 iso 值的提取结果：一个三角网 + 它所在的 iso 值
struct IsoMesh {
    std::vector<Vec3> vertices;
    std::vector<Tri>  triangles;
    double iso_value = 0.0;
    int    layer_id  = -1;
};

// 规划 iso 值序列。返回 iso 值数组，大小 = 层数
std::vector<double> planIsoLevels(const ScalarField& phi,
                                   const IsoExtractParams& params);

// 对 phi 做一次 MC 抽取。拓扑修复（去除孤立面片、合并重合顶点）在内部完成
IsoMesh extractIsoSurface(const ScalarField& phi,
                           double iso_value,
                           int    layer_id);

}  // namespace cslc
```

### §2.9 `path/path_generator.h` —— 明确 `PathPoint` / `PathLayer`（v3 修订 2）

v2 的 §6 映射表只提了 `DataStructs::actPoint → path_generator.PathPoint` 但没给字段。这个结构直接影响 **M3 指标（Δθ/Δs，姿态变化率）** 的计算，v3 必须明确：

```cpp
#pragma once
#include "core/types.h"
#include "surface/iso_surface.h"      // IsoMesh

namespace cslc {

// 路径上的一个采样点
//   position   : 世界坐标系 XYZ
//   normal     : 打印方向单位向量（= KUKA 投影后的 G 场在该点插值结果；也是 TCP 的 Z 轴）
//   a_deg/b_deg/c_deg : KUKA 欧拉角 ABC，单位为**度**（不是弧度！）
//                       由 normal + 姿态参考（robot_ini_abc）反算得到
//   arc_length_mm : 从当前层路径起点累积的弧长，用于计算 M3（Δθ/Δs）
struct PathPoint {
    Vec3   position;
    Vec3   normal;
    double a_deg = 0.0;
    double b_deg = 0.0;
    double c_deg = 0.0;
    double arc_length_mm = 0.0;
};

// 一层（同一 iso 值对应的等值面）生成的全部路径点
//   points 按生成顺序排列（扫描线先水平后竖直 / contour 沿外环→内环等策略决定）
//   如果一层被 MC 切出多个连通域，points 里会有自然的不连续，由 layer_id 共享但通过 contour_id 区分
struct PathLayer {
    int    layer_id        = -1;
    std::vector<PathPoint> points;
    double total_length_mm = 0.0;
};

// 路径生成参数（Phase 4 再定，Phase 3 之前用默认值）
struct PathGenParams {
    std::string strategy    = "scanline";     // "scanline" / "contour_offset"
    double line_spacing_mm  = 0.4;
    double contour_resample_mm = 0.2;
};

PathLayer generatePathForLayer(const IsoMesh& layer_mesh,
                                const ScalarField& phi,      // 用于查 G 方向 → normal
                                const VectorField& G_projected,
                                const PathGenParams& params);

}  // namespace cslc
```

**M3 指标的计算**（落地到 `metrics/metrics.cpp`）：
```
对每层 PathLayer 的相邻两点 P[i], P[i+1]：
  Δθ = acos(dot(n_i, n_{i+1}))           // 法向变化角，度
  Δs = P[i+1].arc_length_mm - P[i].arc_length_mm
  rate_i = Δθ / Δs
M3 = max_i(rate_i)  或  percentile_95_i(rate_i)
```
字段 `arc_length_mm` 必须由 `generatePathForLayer` 在生成时就填好（每点累加上一段欧氏距离），后续平滑重采样后要重新累加。

### §2.10 `field/wavefront.h` —— 基线波前算法的明确接口（v3 修订 4）

v2 映射表已将 `SFF::threadFun_*` → `src/field/wavefront`，但没有给接口。v3 补上，与 `laplacian.h` + `poisson.h` **并存**，通过 `[algorithm] mode` 切换：

```cpp
#pragma once
#include "core/types.h"
#include "field/poisson.h"            // ScalarField
#include "geometry/voxel_grid.h"

namespace cslc {

// 基线算法参数
//   仅保留**干净**的欧氏波前实现（从 SFF::threadFun_first/CurveLayer 精简迁移）
//   **不**包含：
//     - lambda_c * curvPenalty（§7 #5 废弃）
//     - sign=-1 的下方距离翻转（§7 #1 在新算法里自然消失；基线里直接不迁移该分支）
//     - IsoSelectionMode::Median（§7 #4 废弃）
//     - MergeTreatment（§6 废弃）
struct WavefrontParams {
    std::string seed_strategy = "bottom";     // "bottom" : z=z_min+ε 的体素作种子
                                              // （可扩展 "reeb"，Phase 2 先不实现）
    double isovalue_mode      = 0.5;          // 等值面取值模式：
                                              //   0.0 = 取种子距离分布的最小值（φ_min）
                                              //   1.0 = 最大值（φ_max）
                                              //   0.5 = 中位数（经典做法，但**不做** TrimmedMean fallback）
    double height_interval_mm = 1.0;          // 层间距

    // 线程数：小模型串行，大模型 24 线程——以**显式参数**形式保留 §7 #2 的启发式
    int serial_threshold      = 1000;         // next_initVoxels.size() < 阈值时串行
    int parallel_thread_num   = 24;           // 否则用此线程数
};

// 基线求解：返回与新算法 Poisson 同形状的 ScalarField
ScalarField solveWavefront(const VoxelGrid& grid,
                            const WavefrontParams& params = {});

}  // namespace cslc
```

**核心意义**：新老两种算法**输出类型相同**（都是 `ScalarField`），后续 `planIsoLevels` → `extractIsoSurface` → `generatePathForLayer` 可以**共用同一条下游管线**。`main.cpp` / `pipeline.cpp` 里的分支仅在"怎么得到 φ"这一步：

```cpp
ScalarField phi;
if (cfg.algorithm.mode == "wavefront") {
    phi = solveWavefront(grid, cfg.algorithm.wavefront);
} else {
    auto bc  = generateBC(grid, sdf, cfg.field.boundary);
    auto G   = solveLaplacian(grid, bc, cfg.algorithm.field.laplacian);
    auto Gp  = projectToReachableSet(grid, G, limits, w2b);   // Phase 3+
    phi      = solvePoisson(grid, Gp, cfg.algorithm.field.poisson);
}
auto iso_levels = planIsoLevels(phi, cfg.iso_surface);
// ...
```

---

## §3 数据流图（v2 保留，无改动）

见 v2 §3。batch 模式下每个模型独立跑一次。v3 唯一附加说明：数据流在"求 φ"这一步按 `mode` 分叉（wavefront 单步 / field 四步：BC → Laplacian → KUKA 投影 → Poisson），**下游完全相同**。

---

## §4 配置文件（v2 保留 + v3 新增 `[algorithm]` 节）

### §4.1 `config/batch_three_models.toml`

相对 v2 §4.1 的改动仅两处：
1. 新增 `[algorithm]` 顶层节，作为算法模式切换入口
2. 原 `[algorithm.new.*]` / `[algorithm.baseline.*]` 节**保留并重命名**为 `[algorithm.field.*]` / `[algorithm.wavefront.*]`，与 §2.10 的命名对齐

```toml
# ================== config/batch_three_models.toml ==================
# 一次跑三个真实测试模型。路径为绝对路径，包含中文与括号，
# 全部用 UTF-8 存盘，C++ 端通过 std::filesystem::u8path 转成 path 对象。

[io]
output_root = "E:/Git_Repository/STL-free Print/runs/"
runs_csv    = "E:/Git_Repository/STL-free Print/runs/runs.csv"
debug_dump_intermediates = false

[[io.models]]
name     = "armadillo_flat"
stl_path = "E:/baseline/Armadillo-tailless/Model/armadillo_flat.stl"

[[io.models]]
name     = "bunny"
stl_path = "E:/baseline/BUNNY/Model/bunny(46_35_45).stl"

[[io.models]]
name     = "mao"
stl_path = "E:/baseline/MAO/Model/毛主席头雕(42_52_70).stl"

[voxel]
spacing_mm   = 0.5
padding_mm   = 2.0
sdf_band_mm  = 3.0

# --- v3 修订 4：顶层算法模式切换 ---
# 这是对照实验的唯一开关。同一份代码、同一份 config 主体，
# 只改 mode 就能切换到另一条算法分支。
[algorithm]
mode = "field"          # "wavefront" | "field"

# 基线：纯欧氏波前（从老项目 SFF::getDistanceFiled 精简迁移，剥离所有补丁）
[algorithm.wavefront]
seed_strategy       = "bottom"
isovalue_mode       = 0.5
height_interval_mm  = 1.0
serial_threshold    = 1000          # § 2.10 明确保留的启发式参数
parallel_thread_num = 24

# 新算法：Laplacian + Poisson（+ Phase 3 起接入 KUKA 投影）
[field.boundary]
strategy              = "bottom_up"          # "bottom_up" / "bottom_and_top" / "reeb_graph"
print_direction       = [0.0, 0.0, 1.0]
bottom_dot_threshold  = -0.5
bottom_sdf_band       = 1.0

[algorithm.field.laplacian]
max_iterations      = 2000
tolerance           = 1e-6
normalize_each_iter = false

[algorithm.field.poisson]
max_iterations   = 500
tolerance        = 1e-6
use_precondition = true
# anchor_voxel 不写在 config 里，由 pipeline 根据 BC 自动选

[iso_surface]
layer_thickness_mm = 0.8
iso_spacing        = "uniform"      # "uniform" | "volume_equal"
max_layers         = 500
phi_start_offset   = 0.0

[path]
strategy              = "scanline"
line_spacing_mm       = 0.4
contour_resample_mm   = 0.2
smooth_position_window = 5

[path.smooth_orientation]
lambda_data   = 1.0
lambda_smooth = 10.0

[path.spline]
enabled                = true
resample_n_per_segment = 3
tension                = 0.5

# --- KUKA 部分从 v2 保留（非对称 + 速度） ---
[kuka.limits.a1]
min_deg = -170.0
max_deg =  170.0
speed_deg_per_s = 336.0

[kuka.limits.a2]
min_deg = -195.0
max_deg =   40.0
speed_deg_per_s = 336.0

[kuka.limits.a3]
min_deg = -115.0
max_deg =  150.0
speed_deg_per_s = 488.0

[kuka.limits.a4]
min_deg = -185.0
max_deg =  185.0
speed_deg_per_s = 600.0

[kuka.limits.a5]
min_deg = -120.0
max_deg =  120.0
speed_deg_per_s = 529.0

[kuka.limits.a6]
min_deg = -350.0
max_deg =  350.0
speed_deg_per_s = 800.0

[kuka.robot]
status      = 4
turn        = 28
tool_no     = 8
base_no     = 6
platform_x  = 0.0
platform_y  = 0.0
platform_z  = 0.0
z_offset    = 0.28
robot_ini_abc = [0.0, 0.0, 0.0]

[kuka.world_to_base]
world_to_base = [
  [1.0, 0.0, 0.0, 0.0],
  [0.0, 1.0, 0.0, 0.0],
  [0.0, 0.0, 1.0, 0.0],
  [0.0, 0.0, 0.0, 1.0],
]

[metrics]
enabled          = true
sample_layer_ids = []
output_per_layer = true

[logging]
level = "info"
```

### §4.2 单模型 `config/default.toml`

结构同 §4.1，`[[io.models]]` 只有一条。

### §4.3 中文路径处理（v2 保留，无改动）

见 v2 §4.3。代码层规范 + CMake 层 `/utf-8` + `UNICODE/_UNICODE` 不变。

---

## §5 第三方依赖（v2 保留，无改动）

仍然是 Eigen + toml++ 两个外部库。

---

## §6 老项目模块映射表（v3 拆分 1 行、新增 1 行）

相对 v2 §6 的改动用 **【v3】** 标注；v2 的 **【v2】** 标注保留。未标注的条目从 v1/v2 继承。

| 老项目模块 | 新项目位置 | 处理方式 | 说明 |
|---|---|---|---|
| `basicDataType/STLReader.cpp` | `src/io/stl_reader` | 原样迁移 | 170 行，路径参数用 `std::filesystem::path` **【v2】** |
| `StlToSDF.h` (OpenVDB) | `src/geometry/sdf` + `bvh` | 完全重写 | narrow-band + BVH |
| `SFF::voxel` | `src/geometry/voxel_grid` | 完全重写 | 拆职责 |
| `SFF::getFirstCurvelayer` (z=2) | — | 废弃 | 被 `generateBC` 取代 |
| `SFF::threadFun_first/CurveLayer`（纯波前内核，剥离补丁） | `src/field/wavefront` | 精简迁移 **【v3】** | 作为**基线算法**保留，与新算法并存 |
| **`SFF::getDistanceFiled`（纯波前主循环，剥离补丁）** | `src/field/wavefront` | **精简迁移** **【v3】** | 基线算法入口；`mode="wavefront"` 时调用。与 v2 仅把该模块整体标"精简迁移"相比，v3 明确**只迁移干净部分** |
| **`SFF::getDistanceFiled`（补丁逻辑：sign=-1、curvPenalty、Merge 调用等）** | — | **完全废弃** **【v3】** | 新旧两种算法都不需要这些补丁；详细去向见 §7 workaround 表 |
| `SFF::selectLayerIsoValue` (三模式) | — | 废弃 | 补丁，见 §7 #4 |
| `SFF::CuttingMeshwithMT` | `src/surface/iso_surface` 内 | 完全重写 | |
| `SFF::MergeTreatment` | — | 废弃 | |
| `SFF::OrientationScan` | — | 废弃 | |
| **（新增）** Laplacian BC 生成 | `src/field/laplacian.generateBC` | 完全重写 **【v2】** | |
| **（新增）** Laplacian 求解 | `src/field/laplacian` | 完全重写 | **仅 `mode="field"` 时使用** **【v3】** |
| **（新增）** Poisson 求解 | `src/field/poisson` | 完全重写 | `PoissonParams` 见 §2.7 **【v3】** |
| **（新增）** KUKA 投影 | `src/field/kuka_projection` | 完全重写 | 非对称极限 **【v2】**；Phase 3 起接入 |
| `MarchingCubes.cpp` + MC 查找表 | `src/surface/iso_surface` + `mc_lookup_table.h` | 精简迁移 | 剥离 GMP，查找表原样；`IsoExtractParams` 见 §2.8 **【v3】** |
| `MarchingTriangles.cpp` | — | 废弃 | |
| `CurvatureField.h/cpp` (548 行) | `src/metrics/curvature.h` | 精简迁移 **【v2】** | 只留 `computeMeanCurvature` |
| `CurvatureFieldMapper::queryNearestVertex` | — | 废弃 | O(n) 性能陷阱 |
| `pathPostprocessing::LinearSmooth31/51` | `src/path/smoothing.smoothPositionsLinear` | 精简迁移 | |
| `pathPostprocessing::wlsFilter1d` | `src/path/smoothing.smoothOrientationsWLS` | 精简迁移 | |
| `pathPostprocessing::smoothAngle` | — | 完全重写 | 几何错 + 死参 |
| **（新增）** 样条平滑 | `src/path/smoothing.resampleCatmullRom` | 完全重写 **【v2】** | |
| `curveFitting/BSpline.cpp/.h` | — | 废弃 **【v2】** | |
| `curveFitting/Bezier.cpp/.h` | — | 废弃 **【v2】** | |
| `curveFitting/CatmullRom.cpp/.h` | `src/path/smoothing` 内部 | 精简迁移 **【v2】** | 核心公式内联到 smoothing.cpp |
| `SUPPORT_FREE_PRINT/BSpline.cpp/.h` | — | 废弃 **【v2】** | |
| `Offline.cpp` (KUKA 导出) | `src/io/robot_writer` | 完全重写 | 见 CURRENT_PROJECT_ANALYSIS §7.1 |
| `TurinOfflineExporter` | — | 废弃 **【v2】** | |
| `IRobotOfflineExporter` | — | 废弃 | 不做插件化 |
| **（若存在）** Abaqus 导出 | — | 废弃 **【v2】** | 全文检索未找到；FEM 不在本研究范围 |
| **（若存在）** 碰撞回避 | — | 废弃 **【v2】** | `collisionDetection.h` 未被主流程调用 |
| `DataStructs::printPoint` | `src/path/kuka_export.KukaPoint` | 精简迁移 | |
| `DataStructs::actPoint` | `src/path/path_generator.PathPoint` | 精简迁移 | 字段见 §2.9 **【v3】** |
| `SpatialProperty.h` | — | 废弃 | |
| `exact_geodesic.cpp` | — | 废弃 | |
| `Slicer.h/cpp` (传统平面切片) | — | 废弃 | |
| `basicDataType/TopologicBuilder*` | — | 废弃 | |
| `Dual_Contouring.cpp/.h` | — | 废弃 | 已注释 |
| `CSLCReader` | — | 废弃 | |
| `UDPConnect` | — | 废弃 | 已注释 |
| `clipper.cpp` | — | 废弃 | |
| `main.cpp` (OpenGL 主循环) | `src/app/main.cpp`（重写） | 完全重写 | CLI only |
| `main_ui.cpp` (ImGui 面板) | `src/app/pipeline` | 完全重写 | |
| `main_ui.h` (static atomic 全局标志位) | — | 废弃 | |
| `file.cpp` (文件对话框) | — | 废弃 | |
| `AppLog.h` | `src/core/log` | 精简迁移 | |
| `Profiler.h` | `src/core/timer` | 精简迁移 | |
| `GLutils/` / `Simulation/` | — | 废弃 | |

**v3 数量汇总重统计**：原样迁移 1 / 精简迁移 11（+wavefront 基线 +PathPoint 字段明确化不算新增）/ 完全重写 10 / 废弃 28（+`getDistanceFiled` 的补丁逻辑单独拆出）。新增模块 6：Laplacian BC + Laplacian 求解 + Poisson + KUKA 投影 + 样条 + **明确的 wavefront 基线接口**。

**关键纠正**：v2 映射表里把 `SFF::threadFun_*` 与 `SFF::getDistanceFiled` 都直接标"精简迁移到 `field/wavefront`"，但没有区分"纯波前部分"与"补丁逻辑"。v3 把后者显式拆成两行，明确：
- 波前**内核**（种子 → BFS/Dijkstra 风格扩散 → 归一化）：**迁移**进 `field/wavefront`，作为基线算法。
- 波前**补丁**（`sign=-1` 翻转、`lambda_c*curvPenalty`、`MergeTreatment` 调用、`IsoSelectionMode::Median` 分支、`#if 0` 旧代码等）：**全部废弃**，不迁移。

这样 `[algorithm] mode` 切换才有意义：两条算法路径**都是干净实现**，对照实验才能准确反映"新方法 vs 基线方法"而不是"新方法 vs 一堆补丁"。

---

## §7 老项目 workaround 明确处理（v2 保留，无改动）

见 v2 §7。9 条 workaround（sign=-1、线程启发式、TrimmedMean、iso=median、curvPenalty、#if 0、platform_z+0.28、边界不一致、线程数硬编码）处理方式不变。

---

## §8 分阶段路线图 —— Phase 0 → Phase 5（v3 修订 3：量化验证标准）

总预算 **12 周**（14 周学期 - 2 周 buffer）不变。每阶段结束都要编译过 + 跑完至少 1 个模型。

### Phase 0 · 骨架 + IO  **[1 周]**（v2 保留，无改动）

目标、文件清单、验证标准见 v2 §8 Phase 0。

### Phase 1 · 基线算法迁移（纯波前）  **[1 周]**（v2 保留 + 微调）

**目标**：复现老项目干净输出（剥离补丁后），作为论文对照组。

**新增/修改文件**（相对 v2 Phase 1 的改动：`wavefront.h` 接口明确化）：
```
src/geometry/voxel_grid.h/cpp
src/geometry/bvh.h/cpp
src/geometry/sdf.h/cpp
src/surface/iso_surface.h/cpp        ← 含 IsoExtractParams（§2.8）
src/surface/mc_lookup_table.h
src/field/wavefront.h/cpp            ← 【v3】明确接口，剥离补丁
src/app/pipeline.cpp                 ← 接入 mode="wavefront" 分支（§2.10 的分支代码）
src/path/path_generator.h            ← 【v3】先把 PathPoint/PathLayer 结构定下来，实现 Phase 4 再做
```

**验证标准（v3 量化）**：
- `mode=wavefront` + armadillo_flat，等值面几何形态与老项目**剥离补丁后版本**肉眼一致（导出 PLY → MeshLab 对比）
- 三模型全部跑完不崩（armadillo_flat / bunny / mao）
- `metrics.json` 可填 **M4（面片数/连通域数）** 与 **M6（求解耗时 ms）**，其他指标可为 0
- **M4 基线要求**：三模型各自的**连通域数 = 1**（模型 STL 本身是单连通体的前提下）

### Phase 2 · Laplacian + Poisson  **[2 周]**（v3 修订 3：验证标准量化）

**目标**：新算法核心就位，能在三模型上生成 φ 并提取等值面。

**新增/修改文件**：
```
src/field/laplacian.h/cpp            ← 含 generateBC（strategy="bottom_up"）
src/field/poisson.h/cpp              ← 含 PoissonParams（§2.7）
src/surface/iso_levels.cpp           ← Phase 2 末期升级：uniform + CV 反馈
src/app/pipeline.cpp                 ← mode="field" 走 Laplacian → Poisson（Phase 2 暂不投影）
src/metrics/curvature.h/cpp          ← computeMeanCurvature（M1）
src/metrics/metrics.h/cpp            ← M1/M4/M5/M6 填充
```

**验证标准（v3 量化）**：

所有阈值以 Phase 1 基线（纯波前）的数值为参照系。先把 Phase 1 的 M1/M4/M5 存成 `runs/phase1_baseline.json`，Phase 2 的对比自动化脚本 `scripts/compare_phases.py` 读此文件做对比。

- **模型 1、2（armadillo_flat、bunny）**：M1（max|H|）相对 Phase 1 基线**不劣化超过 10%**。
  解读：形态相对简单的模型，新算法不应比基线更糟；允许略差（数值误差 + Laplacian 平滑带来的细节损失），但不超过 10%。
- **模型 3（mao）**：M1 相对 Phase 1 基线**降低 ≥ 50%**。
  解读：mao 是曲率复杂模型，正是新算法的用武之地；若 M1 没有显著下降，说明 Laplacian+Poisson 没有体现优势，需要反查 BC、求解器参数、iso 间距。
- **所有模型**：M4（连通域数）= 1。
  解读：φ 场应是连通实体上的单调标量场，若某层 MC 出现多个连通域，通常意味着 BC 策略不当或 φ 存在局部极值。
- **所有模型**：M5（层厚 CV）≤ 15%。
  解读：层厚均匀性是论文的核心卖点。CV 超过 15% 说明 `planIsoLevels` 需要从 uniform 升级到 volume_equal，或 `phi_start_offset` 未合理剔除底面薄壳。

**若未达标，排查顺序**：
1. 先看可视化（`debug_dump_intermediates=true` 导出 φ 的 PLY），确认 φ 形态合理（从底面等值面平滑向上扩散）；
2. 检查 BC：底面体素数量是否 < 100？`bottom_dot_threshold` 是否太严？
3. 检查 Poisson 求解：残差收敛到 tolerance 否？anchor 位置是否合理？
4. 检查 iso 值分布：打印所有层的 iso_value，确认等距且跨度覆盖 [φ_min + offset, φ_max]；
5. 最后才怀疑 `IsoExtractParams` 的 `iso_spacing` 需要从 uniform 切到 volume_equal。

注：`bottom_and_top` / `reeb_graph` 策略 Phase 2 **只有接口不实现**（`generateBC` 里给出 `throw std::runtime_error("not implemented until Phase 4+")`）。

### Phase 3 · KUKA 约束嵌入  **[1 周]**（v3 修订 3：验证标准量化）

**目标**：向量场投影到 KR 4 R600 允许集，跑到底。

**新增/修改文件**：
```
src/field/kuka_projection.h/cpp      ← 含 kr4r600_defaults()
src/app/pipeline.cpp                 ← 在 solveLaplacian 后、solvePoisson 前插入 projectToReachableSet
```

**验证标准（v3 量化）**：

对比 Phase 2（开启投影）vs Phase 2（关闭投影）在同一三模型上的指标。

- Phase 2 已通过的 **M1、M4、M5 不劣化超过 15%**。
  解读：引入约束必然对场有扭曲，允许一定劣化，但 15% 以内；超过说明投影过于激进或 Ω 定义有问题。
- **M2（最大悬垂角）**：所有层的法向在 KUKA 可达范围内（即映射到允许集 Ω 内），具体指标：
  - 逐层计算每点法向对应的关节角是否落在 §2.3 表中各轴的 `[min_deg, max_deg]` 内；
  - 对每层统计 **不可达点占比 = (不可达点数 / 本层总点数)**；
  - Phase 3 验收要求：**所有层所有模型，不可达点占比 = 0%**（硬约束，不是软性阈值）。
  - 若某层出现不可达点 > 0，说明投影有 bug 或 `WorldToBase` 标定错了。
- **M3（姿态变化率）**：相邻采样点 Δθ/Δs 的 **95 分位数 ≤ 30°/mm**。
  解读：KUKA 实际运动速度 ~500°/s，若 TCP 线速度 10 mm/s，则 Δθ/Δs 最多允许 50°/mm；预留安全余量取 30°/mm。Phase 3 时 PathPoint 还未生成（Phase 4 才生成），这里**先在等值面采样点上估算**（用 G_projected 在每层顶点上的方向作为 normal，沿相邻顶点计算 Δθ）。
- 三模型都能**跑完不崩**（开启投影后，Poisson 仍能收敛到 tolerance 以内）。

**若未达标**：
- M2 不可达点 > 0 → 检查 `WorldToBase` 标定、`robot_ini_abc` 是否合理；
- M1/M4/M5 劣化 > 15% → 投影过激，调整 `projectToReachableSet` 内部的松弛策略（允许 G 在边界附近平滑过渡而非硬投影）。

### Phase 4 · 路径 + XYZABC 完整闭环  **[1 周]**（v2 保留，无改动）

目标、文件清单、验证标准见 v2 §8 Phase 4。Phase 4 将 `PathPoint.arc_length_mm` 在 `generatePathForLayer` 生成时就填好；M3 指标切换到 path 级别（从 Phase 3 的等值面采样级别升级）。

### Phase 5 · 论文实验  **[6 周，非编码阶段]**（v2 保留，无改动）

见 v2 §8 Phase 5。对照实验明确为 **`mode=wavefront` vs `mode=field`** 两组（v3 修订 4 的直接收益：一行 config 开关）。消融实验：开/关 KUKA 投影；`iso_spacing=uniform` vs `volume_equal`；`bottom_up` vs `bottom_and_top`（若 Phase 4 末实现）。

**总计**：1 + 1 + 2 + 1 + 1 + 6 = **12 周**，剩 2 周 buffer。

---

## §9 校验 checklist

沿用 v2 §9 的五大类自检，v3 新增一类"修订 1-4 落地"自检：

**v2 原有检查项（全部仍然满足）**：见 v2 §9，KUKA / 路径和 IO / 边界条件 / 路线图 / Workaround / 决策 / 子目录 ≤ 7 / 依赖 ≤ 3 / 头文件 < 100 行 / 无过度设计 / 中文注释 —— 逐项仍通过。

**v3 修订 1-4 落地自检**：

- [x] **`PoissonParams` 字段明确** —— §2.7：`max_iterations` / `tolerance` / `use_precondition` / `anchor_voxel` 四字段齐全
- [x] **`IsoExtractParams` 字段明确** —— §2.8：`layer_thickness_mm` / `iso_spacing` / `max_layers` / `phi_start_offset` 四字段齐全，`iso_spacing` 枚举明确为 `"uniform"` / `"volume_equal"` 二选一
- [x] **`PathPoint` 字段完整** —— §2.9：`position` / `normal` / `a_deg,b_deg,c_deg`（明确度数单位）/ `arc_length_mm`（M3 指标依赖）
- [x] **`PathLayer` 字段完整** —— §2.9：`layer_id` / `points` / `total_length_mm`
- [x] **M3 指标计算逻辑落地** —— §2.9 末尾给出"Δθ/Δs"公式与出处（基于 `PathPoint.arc_length_mm`）
- [x] **Phase 2 验证标准量化** —— §8 Phase 2：M1 相对 Phase 1 基线的不劣化阈值（简单模型 10%、复杂模型 -50%）、M4=1、M5≤15%
- [x] **Phase 3 验证标准量化** —— §8 Phase 3：Phase 2 指标不劣化 15%、M2 不可达点占比 = 0%、M3 95 分位 ≤ 30°/mm
- [x] **映射表与 Phase 1 矛盾解决** —— §6 把 `SFF::getDistanceFiled` 拆成"纯波前"（精简迁移到 `field/wavefront`）+ "补丁逻辑"（完全废弃）两行；§2.10 给出 `WavefrontParams` 与 `solveWavefront` 接口
- [x] **wavefront 与 laplacian+poisson 并存** —— §2.10 两者输出同为 `ScalarField`，下游管线共享；§4.1 `[algorithm] mode` 作为切换开关；§8 Phase 5 明确为对照实验
- [x] **`[algorithm]` 配置节到位** —— §4.1 新增 `[algorithm] mode` + `[algorithm.wavefront]` + `[algorithm.field.laplacian]` + `[algorithm.field.poisson]`，命名与接口 §2.7 / §2.10 对齐

---

*v3 修订截止：2026-04-24。本文件仅含架构，不含实现代码。*
*v3 相对 v2 的 diff 集中在 §2（新增 §2.7–§2.10 共 4 个接口子节）、§4.1（新增 `[algorithm]` 节与参数命名对齐）、§6（映射表拆分 1 行 + 标注精细化）、§8 Phase 2 / Phase 3（量化验证标准）。其余从 v2 保留。*
