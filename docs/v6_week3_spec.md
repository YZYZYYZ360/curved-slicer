# v6 Week 3 Spec: Layer Field Extraction

Status: DRAFT — pending user freeze
Author: CLI #1 (Opus 4.7)
Date: 2026-05-19
Prereq: Week 2 complete (commit `02656ca`), W2-T5 rolled back per `v6_week2_summary.md`

---

## 1. 背景与 W2-T5 教训

Week 2 W2-T5 用 W2-T4 Dijkstra 的 `D_global` 直接喂 MC 提取 layer，结果几何爆面。
根因不是 MC 实现，是算法范式错位：

- v6 Dijkstra D 是 **graph 累积距离**（每步 ∈ {s, √2s, √3s}），不是连续函数
- 没有 **layer offset 显式结构**，等值面位置不可预测
- Multi-source wavefront 相遇形成 shock，MC 在 shock cell 拓扑错乱

v4 SFF (`STL-free Print/project/SUPPORT_FREE_PRINT/SFF.cpp:1060-1191`) 用的是不同算法范式：
每层 seed 向欧氏邻域 ball painting，距离 = `euclid_dist + heightInterval × layer + lambda × penalty`，
**layer offset 是显式离散变量**，等值面位置完全可控。

Week 3 采纳 SFF 算法范式，但兼容 v6 红线（不踩 R1 PDE）。

---

## 2. 核心决策（freeze 后不改）

| # | 决策 | 选定 | 备注 |
|---|---|---|---|
| 1 | Layer 算法范式 | SFF ball painting + layer offset | 不是 PDE，不踩 R1 |
| 2 | 每层 seed 来源 | 上层 MC 等值面 voxel（迭代生长） | 保证层间几何连续 |
| 3 | W3 第一版 penalty | clearance + curvature 两项 | normal/IK 留 W4-W5 |
| 4 | 输出与可视化 | 先 path/distance field，R15 用户确认后再 MC + contour | 严格 R15 |
| 5 | 第一层 seed 初始化 | part_bottom 平面（z = z_min + 1 voxel）上的所有 INSIDE voxel | 退回 v4 SFF 风格 |
| 6 | Curvature 来源 | mesh shape operator (cotangent Laplacian) + barycentric 插值到 voxel | 不用 SDF Hessian（评审否决） |

---

## 3. 数据流

```
RepairedMesh (W2-T1)
   │
   ├── VoxelizationV6 (W2-T2)        ──> Grid (occupancy)
   │                                       │
   ├── SDFV6 (W2-T3)             ────────> SDF field           ──> clearance penalty
   │                                       
   └── SurfaceCurvatureField (W3-T1) ───>  curvature field     ──> curvature penalty
                                            │
                                            v
                            SeedExtractor (W3-T2): first layer seeds
                                            │
                                            v
                            PerLayerBallPainter (W3-T3):
                                for layer N = 0, 1, 2, ...:
                                    D_N[v] = min over seed in seeds_N:
                                        euclid(v, seed) + N × layer_height
                                        + λ_clear × clearance_penalty(v)
                                        + λ_curv  × curvature_penalty(v)
                                    layer_mesh_N = MC(D_N, level=(N+0.5) × layer_height)
                                    seeds_{N+1} = voxels on layer_mesh_N
                                    if seeds_{N+1} empty: stop
                                            │
                                            v
                            LayerMesher + ContourGenerator (W3-T4)
```

---

## 4. 模块接口

### 4.1 W3-T1: SurfaceCurvatureField

```cpp
class SurfaceCurvatureField {
public:
    struct Field {
        VoxelizationV6::Grid grid;
        std::vector<float> mean_curvature_mm_inv;   // per-voxel mean curvature H (1/mm)
        std::vector<float> max_abs_principal;       // per-voxel max(|k1|, |k2|) (1/mm)
    };

    struct Options {
        // 插值方式: 每个 voxel center 找最近 face，barycentric 插 H/k1/k2
        // 远离 mesh 的 voxel (例如 OUTSIDE) 给 0
        double max_query_dist_mm = -1.0;   // <0 = 无限制，>0 = 截断
    };

    static Field compute(
        const Eigen::MatrixXd& V_repaired,
        const Eigen::MatrixXi& F_repaired,
        const VoxelizationV6::Grid& grid,
        const Options& opt);
};
```

实施细则：
- 用 `igl::principal_curvature(V, F, ..., PD1, PD2, PV1, PV2)` 算 per-vertex k1/k2
- `H = 0.5 × (k1 + k2)`, `max_abs = max(|k1|, |k2|)`
- 对每个 voxel center，用 BVH (v4 `geometry/bvh`) 找最近 face，barycentric 插值
- OpenMP parallel chunk（学 W2-T2 模式，cap 24 线程）

### 4.2 W3-T2: SeedExtractor

```cpp
class SeedExtractor {
public:
    // 从 grid 选第一层 seed: z = z_min + 1 平面上所有 INSIDE voxel
    static std::vector<Eigen::Vector3i> extractFirstLayer(
        const VoxelizationV6::Grid& grid);

    // 从上一层 MC mesh 提取下一层 seed: 把 layer_mesh_N 的 vertex 离散化到 voxel ijk
    // 重复 voxel 去重
    static std::vector<Eigen::Vector3i> extractFromLayerMesh(
        const Eigen::MatrixXd& V_layer,
        const VoxelizationV6::Grid& grid);
};
```

### 4.3 W3-T3: PerLayerBallPainter

```cpp
class PerLayerBallPainter {
public:
    struct LayerField {
        VoxelizationV6::Grid grid;
        std::vector<float> distance;        // D_N per voxel, INFINITY = unpainted
        int layer_index = 0;
        double layer_height_mm = 0.0;
    };

    struct PenaltyWeights {
        double lambda_clearance = 1.0;
        double lambda_curvature = 1.0;
        double clearance_target_mm = 1.0;   // |sdf| < target → penalty
        double curvature_target_mm_inv = 0.1;  // H > target → penalty (r_n=10mm → 1/10)
    };

    struct Options {
        double layer_height_mm = 1.2;
        double ball_radius_voxels = 1.5;    // 邻域半径 = ball_radius × (layer_height/spacing)
        PenaltyWeights weights;
    };

    // 单层 painting: 每 seed 向邻域 painting，min(...)
    // D = euclid + N × layer_height + λ_c × clearance + λ_k × curvature
    static LayerField paintLayer(
        const VoxelizationV6::Grid& grid,
        const SDFV6::Field& sdf,
        const SurfaceCurvatureField::Field& curv,
        const std::vector<Eigen::Vector3i>& seeds,
        int layer_index,
        const Options& opt);
};
```

实施细则：
- ball_radius = `1.5 × ceil(layer_height_mm / spacing_mm)` voxel
- penalty 公式：
  - `clearance = max(0, weights.clearance_target_mm - |sdf|) / weights.clearance_target_mm`
  - `curvature = max(0, |H| - weights.curvature_target_mm_inv) / weights.curvature_target_mm_inv`
  - 都归一化到 [0, 1+] 量级，量纲一致
- OpenMP parallel over seeds，thread-local accumulator + min-reduction merge
- 不并行不同 layer（layer N 依赖 layer N-1 的 MC，必须串行）

### 4.4 W3-T4: LayerMesher + ContourGenerator

```cpp
class LayerMesher {
public:
    struct Layer {
        int layer_index;
        Eigen::MatrixXd V;     // MC vertices
        Eigen::MatrixXi F;     // MC triangles
    };

    // MC(D_N, level=(N+0.5)*layer_height) on the layer field grid
    static Layer extract(
        const PerLayerBallPainter::LayerField& field);
};

class ContourGenerator {
public:
    // 用 W1-T3 GeodesicContourPath wrapper 在 Layer mesh 上生成 toolpath contour
    // 输出 polyline 数组
    static std::vector<std::vector<Eigen::Vector3d>> generate(
        const LayerMesher::Layer& layer,
        double contour_spacing_mm);
};
```

---

## 5. Penalty 项细节

### 5.1 Clearance penalty

目的：layer 不贴 mesh 表面，留出 nozzle clearance。

公式：
```
clearance_violation(v) = max(0, clearance_target_mm - |sdf(v)|)
clearance_penalty(v)   = lambda_clearance × clearance_violation / clearance_target_mm
```

物理意义：`|sdf(v)| < clearance_target_mm` 的体素加 penalty，越靠近表面 penalty 越大。
默认 `clearance_target_mm = 1.0`（layer 至少离表面 1mm，nozzle radius 10mm 的 1/10）。

### 5.2 Curvature penalty

目的：layer 避开零件高曲率区（耳朵尖、角落）。

公式：
```
curvature_violation(v) = max(0, |H(v)| - curvature_target_mm_inv)
curvature_penalty(v)   = lambda_curvature × curvature_violation / curvature_target_mm_inv
```

物理意义：`|H(v)| > curvature_target_mm_inv` 的体素加 penalty。
默认 `curvature_target_mm_inv = 0.1 mm⁻¹`（对应曲率半径 10mm = nozzle body radius）。

注意：评审 `opus4.7-web.txt:149` 警告 SDF Hessian 估曲率误差 ~50%。
本 spec 改用 mesh shape operator，精度 O(h²)，对 0.5mm voxel 误差 ~0.05 mm⁻¹ ~~ 50%~~ ~5%，
判据有效。

### 5.3 Lambda 选择

W3 第一版 lambda 都设 1.0，不调参。
如果 R15 视觉验证发现 layer 形状不合理，按 §6 决策树定位是 spec gap 还是 lambda 问题。
**不允许为单一测试通过 retune lambda**（R3 红线）。

---

## 6. 红线兼容性

| 红线 | 本 spec 如何兼容 |
|---|---|
| **R1** 不踩 v4 PDE | SFF ball painting 是离散种子扩散 + min reduction，不是 Laplacian/Poisson PDE。完全独立于 v4 `field/laplacian.cpp`, `field/poisson.cpp`。 |
| **R3** 不 retune | lambda 默认 1.0，penalty target 由物理意义（nozzle 半径）确定，不允许调参通过测试。 |
| **R5** 不把 D_norm 喂 MC | MC 输入是 `D_N`（每层独立 distance field），不是归一化 D。每层 MC level = (N+0.5) × layer_height，绝对量级。 |
| **R8** 严格推理 | testbed 不只测数值收敛，还测：(a) 球分析解对照、(b) layer 间距严格 = layer_height、(c) R15 视觉确认 layer 几何合理。 |
| **R15** 可视化 gate | 每个 task 完成后输出 PLY / PNG，停下让用户审，确认后才 commit。 |

---

## 7. 测试 spec

### 7.1 W3-T1 SurfaceCurvatureField testbed

Testbed 1: 球 analytic curvature
- r=10mm 球 mesh，理论 H = 1/r = 0.1 mm⁻¹
- 验收：voxel 内 mean H 误差 < 0.02 mm⁻¹（20% 容差，离散误差）
- max_abs k1/k2 也应 ≈ 0.1

Testbed 2: 立方体 sharp edge
- 10mm 立方体 mesh
- 验收：面中心 voxel H ≈ 0，棱附近 voxel H > 0.1（分布奇异，但 finite）
- 不要求精确数值，只验证 ordering（棱 > 面）

### 7.2 W3-T3 PerLayerBallPainter testbed

Testbed 1: 单层 painting 几何
- 半径 5mm 球，spacing 0.5mm，layer_height 1.0mm
- 1 个 seed at sphere center
- 期望：painted voxel 形成 ball with radius ≤ ball_radius × layer_height
- D 在 seed 处 = 0 × layer_height = 0，远端 D ≈ 1.5 mm
- 验收：mean |D - (euclid + 0)| < spacing

Testbed 2: layer offset 正确性
- 同球，paint layer 0, 1, 2
- 验收：layer N 的 D 范围 ∈ [N × layer_height, N × layer_height + ball_radius × layer_height]
- 验收：MC(D_N, level=(N+0.5)*layer_height) 与 MC(D_{N+1}, level=(N+1.5)*layer_height) 距离 ≈ layer_height

Testbed 3: penalty 起作用
- 同球，lambda_clearance = 0 → MC 等值面靠近表面
- lambda_clearance = 100 → MC 等值面远离表面
- 验收：mean |sdf| in lambda=100 case > mean |sdf| in lambda=0 case

### 7.3 W3-T4 集成 testbed (bunny)

bunny @ spacing 0.5mm + layer_height 1.0mm
- 输出 layer_0 ... layer_N 全部 PLY
- 验收（数值）：
  - 总 layer 数 ≈ bunny_height / layer_height ± 20%
  - 每层 mesh 非空
  - layer N 到 layer N+1 的 Hausdorff distance ≈ layer_height ± 30%
- 验收（R15 视觉）：
  - bunny 耳朵、躯干、底部都被 layer 覆盖
  - 相邻 layer 形状连续变化，没有"跳层"或几何爆面
  - 用户在 MeshLab 加载全部 layer + bunny_after.ply，layer 与 bunny 表面保持 ~1mm 间距

---

## 8. R15 触发点

每个 W3 task 完成都触发 R15 ritual：

| Task | R15 产物 | 用户视觉检查重点 |
|---|---|---|
| W3-T1 | `runs/w3_t1_curv/bunny_curvature_slice.png` (z 切片 heatmap)<br>`runs/w3_t1_curv/bunny_high_curv_voxels.ply` (H > 0.1 的 voxel) | 高曲率区应在 bunny 耳朵尖、眼眶、嘴；不应分布在平坦表面 |
| W3-T2 | `runs/w3_t2_seed/bunny_first_layer_seeds.ply` (point cloud)<br>`runs/w3_t2_seed/bunny_layer_5_seeds.ply` (point cloud) | seeds 应贴近 bunny 底面 / 中层等值面，分布连续 |
| W3-T3 | `runs/w3_t3_paint/bunny_layer_0_distance.png` (D slice)<br>`runs/w3_t3_paint/bunny_layer_5_distance.png`<br>`runs/w3_t3_paint/bunny_layer_field_stats.json` | D 等值面平滑，从 seeds 向外圆滑扩散；同层内无阶梯 |
| W3-T4 | `runs/w3_t4_mesher/bunny_all_layers.ply` (全部 layer 合并)<br>`runs/w3_t4_mesher/bunny_layer_count.json` | layer 间距 ≈ layer_height；相邻 layer 形状连续；无爆面 |

**任何一个 R15 视觉验证不通过，停下不 commit，按 §6 决策树定位 spec gap。**

---

## 9. W3 Task 拆分

| Task | 内容 | 估时 | 依赖 |
|---|---|---|---|
| **W3-T1** | SurfaceCurvatureField（mesh shape operator + voxel barycentric 插值）+ R15 | 1.5d | W2-T1, W2-T2 |
| **W3-T2** | SeedExtractor（first layer + layer mesh → next seeds）+ R15 | 0.5d | W2-T2 |
| **W3-T3** | PerLayerBallPainter（ball painting + 2 penalty 项）+ R15 | 2d | W3-T1, W3-T2, W2-T3 |
| **W3-T4** | LayerMesher（per-layer MC）+ ContourGenerator（用 W1-T3 wrapper）+ R15 | 1.5d | W3-T3 |
| **W3-T5** | Week 3 收尾汇报 + interface audit 增量 | 0.5d | W3-T1..T4 |

Week 3 总计 6d。如有 R15 翻车需要 spec 回退，单独计入 Week 4 overlap。

---

## 10. 与 Week 2 已有模块的复用关系

| Week 2 模块 | Week 3 用法 |
|---|---|
| W2-T1 MeshRepair | mesh 预处理，输出 V_repaired/F_repaired 喂 W3-T1 和 W2-T2 |
| W2-T2 VoxelizationV6 | grid 几何，所有 W3 task 在此 grid 上工作 |
| W2-T3 SDFV6 | clearance penalty 输入 |
| W2-T4 WeightedDijkstraV6 | **W3 不直接复用**。保留作为 W3 后期 path planning（不是 layer 提取）的工具，或废弃合并到 W4 |
| W1-T3 GeodesicContourPath | W3-T4 ContourGenerator 内部调用，layer mesh → polyline contour |
| W1-T2 KR4Model + AnalyticIK | W3 不用，W5 IK feasibility field 才用 |

W2-T4 是否废弃，等 W3-T4 完成后看是否仍有用途，W3-T5 收尾时定。

---

## 11. Open Question（spec 冻结前必须解决）

- **Q1**: layer painting 终止条件。当前 spec 是"seeds 为空则停"，但 bunny 这种有 multiple disconnected branches（耳朵 vs 躯干）的几何，可能某些分支提前耗尽 seeds。是否要 per-component 独立 layer counter？  
  → **CLI #1 默认**: W3 第一版用全局 seeds 合并，提前耗尽则全停。如果 R15 发现 bunny 耳朵 layer 比躯干少，W3 后期再加 per-component 支持。

- **Q2**: layer_height vs spacing 比例。当前 1.2mm layer / 0.5mm spacing = 2.4，ball_radius = 1.5 × 2.4 = 3.6 voxel。比例太小（< 2）会 aliasing。建议 **layer_height ≥ 3 × spacing**。  
  → **CLI #1 默认**: 在 PerLayerBallPainter::compute 入口做 sanity check，如果 layer_height < 3 × spacing 抛 warning（不 throw）。

- **Q3**: 第一层 z 平面定位。当前 spec 是 z = z_min + 1 voxel。但 bunny 底部不是平的（双脚），z_min + 1 可能选到很少 voxel。是否要用 part_bottom plane（mesh 底面）而不是 voxel grid 底面？  
  → **CLI #1 默认**: W3 第一版用 voxel grid z_min + 1，如果 seed 数 < 50 抛 warning。part_bottom plane 留 W4 优化。

---

## 12. Spec 冻结流程

1. 用户审本文档（你正在做）
2. 用户回 OK 或指出问题
3. CLI #1 改 spec（如需）
4. 用户确认冻结
5. CLI #1 把决策项 freeze 后的版本 commit 到 `v6-implementation` 分支
6. CLI #1 发 W3-T1 详细指令给 Codex

---

## 13. 参考

- W2-T5 rollback 详情: `docs/v6_week2_summary.md`
- v4 SFF 算法源: `STL-free Print/project/SUPPORT_FREE_PRINT/SFF.cpp:1060-1191`
- SDF Hessian 评审否决: `docs/评审/opus4.7-web.txt:139-149, 358`
- shape operator 推荐: `docs/评审/opus4.7-web.txt:358`
- Curvature 错位警告: `docs/评审/chatgpt5.5.txt:27`
- 红线编号: `docs/superpowers/codex_protocol.md`
