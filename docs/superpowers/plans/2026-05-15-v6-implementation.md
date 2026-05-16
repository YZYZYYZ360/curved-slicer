# v6.2 12 周实施计划

> **Spec 依据**：`docs/architecture/V6_SPEC.md`
> **目标**：L4（bunny 实机 + armadillo dry-run + mao_regularized full dry-run + 论文）
> **执行者**：CLI #2
> **依赖工具**：libigl, Eigen, yaml-cpp, OpenMP, FCL (optional, Week 8-9)
> **不在 12 周内**：RSI 在线、FCL 必须 build、复现外部方法、α/β/γ 完整消融

---

## 0. 前置事项（Week 0，即启动前）

| 任务 | 谁做 | 截止 |
|---|---|---|
| CLI #2 完成 v4 Plan 1 收尾（Task 19+20）并 commit | CLI #2 | 启动前 |
| 切新分支 `v6-implementation`（基于 master HEAD） | CLI #2 | Week 1 Day 1 |
| 启动指令通知 CLI #2 v4 Plan 1 作废，v4 PDE 代码保留作 baseline M1 | CLI #1 | 启动前 |
| docs/architecture/V6_SPEC.md 阅读完成 | CLI #2 | Week 1 Day 1 |
| 本计划 docs/superpowers/plans/2026-05-15-v6-implementation.md 阅读完成 | CLI #2 | Week 1 Day 1 |

---

## Week 1：工程骨架 + v4 复用 + 坐标系审计

**目标**：建立可编译工程，把 v4 §11/§13/§14 包装为可调用模块，做坐标系审计避免后续混乱。

### 任务

- **W1-T1**：建立 CMake 工程 + 三 CLI 空壳（0.5 天）
  - 文件：`CMakeLists.txt`, `src/app/{curved_slicer,toolpath_validate,export_krl}.cpp`, `configs/*.yaml`
  - 依赖：Eigen, libigl, yaml-cpp（vcpkg 装）
  - 验收：`curved_slicer --help` / `toolpath_validate --help` / `export_krl --help` 正常输出

- **W1-T2**：v4 §11 KR4 模块包装（1.5 天）
  - 文件：`src/robot/KR4Model.cpp`, `AnalyticIK.cpp`, `Jacobian.cpp`
  - 依赖：v4 §11 源码
  - 关键改造：IK 一次返回 8 branches（不调 8 次）
  - 验收：1000 个随机 joint → FK → IK round-trip，至少找回等价解；测试 home pose 准确

- **W1-T3**：v4 §13 路径模块接口包装（0.5 天）
  - 文件：`src/path/GeodesicContourPath.cpp`
  - 接口：输入 layer mesh，输出 polyline list
  - 验收：sphere 测试 mesh 生成至少 1 条闭合 polyline

- **W1-T4**：v4 §14 trajectory/KRL 模块包装（1 天）
  - 文件：`src/robot/TimeParameterization.cpp`, `src/export/{CSVWriter,KRLWriter}.cpp`
  - 验收：dummy trajectory → CSV/KRL，字段完整

- **W1-T5（关键）**：**Interface Audit**（0.5 天）
  - 文件：`docs/v4_v6_interface_audit.md`
  - 列出 v4 每个模块 API 与 v6 需求的 gap，量化重写工作量
  - 修正 Opus 3.3："70% 复用"是乐观估计的问题
  - 验收：每个 v4 模块标注 [直接复用 / 小改写 / 中改写 / 重写]

- **W1-T6（关键）**：**坐标系审计文档**（0.5 天）
  - 文件：`docs/frames.md`
  - 内容：STL frame, voxel grid frame, robot base frame, TCP frame, tool_z, KRL frame
  - 画一张 frame diagram，所有 transform 都有命名
  - 验收：审计文档评审通过

### 风险

- v4 代码耦合严重，封装超 2 天 → fallback：只保留 v4 代码路径，写 adapter 不重构

---

## Week 2：Mesh repair + voxel/SDF + 单位正确的 Dijkstra

**目标**：完成不含 IK 的几何加权距离场，切出第一版 bunny 弯曲层。

### 任务

- **W2-T1**：MeshRepair + MeshRegularization 轻度版（1 天）
  - 文件：`src/geometry/{MeshRepair,MeshRegularization}.cpp`
  - 依赖：libigl
  - 输出：`mesh_clean.ply`
  - 验收：bunny 通过 manifold check + hole count + self-intersection report

- **W2-T2**：Voxelizer + SDFGrid（1.5 天）
  - 文件：`src/geometry/{Voxelizer,SDFGrid}.cpp`
  - 依赖：libigl winding number
  - 输出：`occupancy.vti`, `sdf.vti`
  - 验收：sphere 体积误差 < 5%；inside/outside 可视化正确

- **W2-T3**：WeightedDijkstraField 基础版（1.5 天）
  - 文件：`src/field/{WeightedDijkstraField,FieldWeights}.cpp`
  - 28 邻接体素图 + 边权（仅 ‖u-v‖_mm，不加 penalty）
  - 输出：`D0_mm.vti`
  - 验收：sphere/cube 上 D0_mm 从 bottom 单调增加；**单位严格 mm**

- **W2-T4（关键）**：**Field Unit Test**（0.5 天）
  - 文件：`tests/test_field_units.cpp`
  - 内容：拒绝 D_norm 进 MarchingCubes 的 unit test
  - 必须 fail 如果 normalized field 被用于切片
  - 验收：CMake test 跑通

- **W2-T5**：MarchingCubes layers（1 天）
  - 文件：`src/layers/MarchingCubesLayers.cpp`
  - 依赖：libigl/copyleft MC
  - 输出：`layers/layer_NNN.ply`
  - 验收：bunny at h=1.2mm 输出 >20 层，无空文件

### 风险

- 0.5mm voxel 跑太慢 → Week 2 全部用 0.8mm voxel，0.5mm 延后到 Week 10

---

## Week 3：基础场权重 + Penalty 归一化 + Height-sweep min + layer metrics

**目标**：完成 α/β/γ 几何版（无 IK），建立 PenaltyNormalizer 防 αβγδ 量纲爆炸，提前做 height-sweep 最小版。

### 任务

- **W3-T1**：CavityRadius Estimator（1 天）
  - 文件：`src/geometry/CavityRadiusEstimator.cpp`
  - 算法：每 surface-near voxel 32 方向 ray casting，percentile_20 / 2
  - 输出：`cavity_radius.vti`
  - 验收：narrow cavity synthetic model 高风险区可视化正确

- **W3-T2**：CurvatureEstimator（1 天）
  - 文件：`src/geometry/CurvatureEstimator.cpp`
  - 算法：`igl::principal_curvature` 顶点级 → voxel 插值
  - 输出：κ_max field
  - 验收：sphere 曲率估计误差中位数 < 25%；cube corner 标高风险即可

- **W3-T3（关键）**：**PenaltyNormalizer**（0.5 天）
  - 文件：`src/field/PenaltyNormalizer.cpp`
  - 实现 winsorize + clamp 到 [0,1]
  - 所有 penalty 在边权里仅以 normalized form 出现
  - 验收：所有 p_*_norm ∈ [0,1]，p05/p95 记录到 `penalty_stats.json`

- **W3-T4**：边权集成 α/β/γ（0.5 天）
  - 文件：`src/field/FieldWeights.cpp`
  - 实现 p_clear, p_orient (blended B(v)), p_curv 三项 raw + norm
  - **关键**：p_orient 使用 blended preferred direction（P0-3）
  - 验收：bunny 跑 D_final 不含 IK，与 D_0 有可见差异

- **W3-T5**：Inter-layer Support Checker（1 天）
  - 文件：`src/layers/SupportChecker.cpp`
  - 算法：每层每点查 layer_{k-1} 最近距离 ≤ support_radius
  - 输出：`support_metrics.csv`, `support_heatmap.ply`
  - 验收：sphere 层间 support pass rate >90%；故意增大 layer height 后下降

- **W3-T6（关键）**：**Height-Sweep Diagnostic 最小版**（提前到 Week 3，P2-3）
  - 文件：`src/diagnostics/HeightSweepDiagnostic.cpp`
  - 实现 component_count(z) + persistence threshold filter
  - 输出：`height_events.csv`
  - 验收：synthetic merge/split model 检测正确

- **W3-T7**：Layer Metrics + Component Labeler（1 天）
  - 文件：`src/layers/{LayerMetrics,ComponentLabeler}.cpp`
  - 输出：`layer_metrics.csv` (area, component count, largest ratio, broken flag, deleted area ratio)
  - **不删小组件**（P0-6），标注 MAIN/SMALL_COMPONENT/BROKEN_LAYER
  - 验收：synthetic two-island model component count 正确

- **W3-T8**：bunny + armadillo geometry-only smoke run（0.5 天）
  - 文件：`scripts/run_bunny_geo.sh`, `scripts/run_armadillo_geo.sh`
  - 输出：两模型 layers + metrics
  - 验收：bunny broken layers <10%；armadillo 可出完整 metrics

### 风险

- Curvature 噪声太大 → γ=0 fallback，curvature 仅作 diagnostic 不进边权

---

## Week 4：IK Feasibility Field（v6 窄新意）

**目标**：实现 pointwise IK margin 注入 scalar field（核心 contribution 1），含 branch consistency。

### 任务

- **W4-T1**：稳定 tool axis frame（0.5 天）
  - 文件：`src/field/IKFeasibilityField.cpp`
  - 关键：tool_z = -∇SDF（**Opus O-1.1 修正，不用 ∇D_0**）
  - 输出：`tool_axis.vti`
  - 验收：无 NaN；∇SDF≈0 区域标 invalid

- **W4-T2**：IKCandidateGenerator（1.5 天）
  - 文件：`src/robot/IKCandidateGenerator.cpp`
  - 关键：一次返回 8 branches；J_ik 内部 a/b/c 三项各 normalize 到 [0,1]
  - 验收：bunny 10k voxel 采样运行时间可接受；valid IK ratio 报告

- **W4-T3**：p_ik penalty 实现 + 粗 grid（1 天）
  - 文件：`src/field/IKFeasibilityField.cpp`
  - 关键：1.6mm 粗 grid（2× slicing grid），trilinear 插值回 0.8mm
  - OpenMP 多线程
  - 输出：`p_ik_raw.vti`, `p_ik_norm.vti`
  - 验收：关节限位 / 奇异附近 penalty 升高；mao_regularized 0.8mm 总耗时 <2 分钟（8 线程）

- **W4-T4（关键）**：**Branch Consistency 项**（Opus O-A）（0.5 天）
  - 文件：`src/field/IKFeasibilityField.cpp`
  - 算法：1-ring 邻域内最优 branch 主导比例
  - `p_ik = J_ik_min + λ_consist · branch_volatility`，默认 λ_consist=0.5
  - 验收：synthetic wrist-flip case 在 branch boundary 处 penalty 明显高于内部

- **W4-T5**：D_final_mm + Bounded Smoothing → D_slice（1 天）
  - 文件：`src/field/{WeightedDijkstraField,FieldSmoothing}.cpp`
  - 关键：D_slice = bounded edge-aware Laplacian(D_final)，ε = 0.3 × layer_height（P0-5）
  - 输出：`D_final_mm.vti`, `D_slice_mm.vti`, `D_norm.vti`(仅可视化)
  - 验收：δ=0 时 D_final ≈ D_0；δ>0 时 field_deviation_from_D0 有记录

- **W4-T6（关键）**：**p_ik Cache Contract**（0.5 天）
  - 实现 cache 机制
  - 同 model_transform + robot + yaw_samples → 复用 p_ik.vti
  - δ 改变不重算 p_ik
  - 验收：第二次跑 δ=0.5 时 p_ik 不重算（log 显示 cache hit）

- **W4-T7**：IK field ablation smoke test（0.5 天）
  - 文件：`scripts/ablate_delta_smoke.sh`
  - 跑 δ ∈ {0, 0.25, 0.5, 1.0} 四组 bunny layers
  - 验收：metrics 中有 field_deviation_from_D0；δ=1.0 时 deviation 显著大于 δ=0.25

### 风险

- p_ik 太慢 → 进一步 down-sample 到 2.4mm grid + 只算 surface-near 体素

---

## Week 5：路径生成 + Raster Fallback

**目标**：从 layers 生成可被 IK DP 消化的 polylines，含 raster fallback。

### 任务

- **W5-T1**：复用 v4 §13 GeodesicContourPath（1 天）
  - 文件：`src/path/GeodesicContourPath.cpp`
  - 关键：复用 v4 已修复的 polyline stitching
  - 输出：`paths/layer_NNN_component_C.polyline`
  - 验收：bunny 至少 80% layers 有路径

- **W5-T2**：Path Stitcher + component-aware（1 天）
  - 文件：`src/path/PathStitcher.cpp`
  - 关键：6 项 stitching 通过判据
  - 验收：multi-component synthetic model 不崩；每 component 有 status

- **W5-T3**：Resampling + simplification（0.5 天）
  - 文件：`src/path/PathResampler.cpp`
  - 输出：0.2mm 间距 polyline
  - 验收：点间距 mean error <10%

- **W5-T4**：RasterFallbackPath（1.5 天）
  - 文件：`src/path/RasterFallbackPath.cpp`
  - 关键：plane-family intersection + PCA + g_med 几何补偿（Δs = line_spacing × clamp(g_med, 0.5, 1.0)）
  - 验收：geodesic 故意失败时 fallback 生效；spacing_p95 ≤ 1.5 × line_spacing

- **W5-T5**：PathSpacingMetrics + per-component routing（0.5 天）
  - 文件：`src/path/PathSpacingMetrics.cpp`
  - 输出：`path_metrics.csv`（path mode, length, closed/open, point count, component id）
  - 验收：raster 段标 RASTER_FALLBACK；geodesic 段标 GEODESIC

- **W5-T6**：armadillo dry-run smoke（**提前于原计划**）（0.5 天）
  - 文件：`scripts/run_armadillo_smoke.sh`
  - 验收：armadillo 跑完前 5 层路径，无 crash

### 风险

- `exact_geodesic` 在破碎层上不稳定 → raster 作主路径，geodesic 作可选

---

## Week 6：Open-loop IK DP + 时间参数化

**目标**：先解决非闭合 / 分段路径的 IK branch continuity，验证 DP 算法。

### 任务

- **W6-T1**：Candidate pruning top-K（1 天）
  - 文件：`src/robot/IKCandidateGenerator.cpp`
  - 关键：每点保留 top-K=12 states（按 J_ik 排序）
  - 验收：candidate dump 中无点超过 K；保留最低 penalty states

- **W6-T2**：Open-loop Viterbi DP（1.5 天）
  - 文件：`src/robot/CyclicIKDP.cpp`（先实现 `solveOpen()`）
  - 关键：DP 用 provisional Δt = d_cart/v_target；stretch_limit=3 软化（P0-8）
  - 验收：synthetic circle path 能找到连续 branch；max |Δq| 满足

- **W6-T3**：Time Parameterization 初版（1 天）
  - 文件：`src/robot/TimeParameterization.cpp`
  - 关键：Δt_final = max(d_cart/v, |Δq|/qdot, sqrt(|Δ²q|/qacc))
  - 验收：速度约束无 violation；CSV 可视化关节角连续

- **W6-T4**：Quintic Interpolator（P0-9 修正）（0.5 天）
  - 文件：`src/trajectory/QuinticInterpolator.cpp`
  - 关键：**只在 path 首尾零速度**；中间点 chord-length finite difference
  - 4ms 周期密化
  - 验收：synthetic path 端点 v=0，中间 v≠0；4ms 帧无 NaN

- **W6-T5**：Singularity / joint-limit report（0.5 天）
  - 文件：`src/robot/IKMetrics.cpp`
  - 输出：`ik_metrics.json`（pointwise_ik_success_rate, dp_success_rate, min_sigma_min, joint margin）
  - 验收：报告失败点 index

- **W6-T6**：bunny path dry-run（0.5 天）
  - 文件：`scripts/run_bunny_ik_open.sh`
  - 验收：bunny IK pass rate >70%；若 <70%，调 model_transform 不调算法

### 风险

- 模型摆放导致 reachability 差 → 引入 `--model-transform`，先移到最佳工作区

---

## Week 7：Cyclic DP + Yaw Adaptive

**目标**：解决闭合曲线腕翻转和首尾不连续。**Transition 推到 Week 8 前半，减少 Week 7 双高难叠加**。

### 任务

- **W7-T1**：Cyclic DP fixed-point iteration（P0-8, Opus 1.4）（1.5 天）
  - 文件：`src/robot/CyclicIKDP.cpp`（加 `solveCyclic()`）
  - 关键：不用枚举起点（O(K³N)），用 2 次 open DP 含 closure cost（O(K²N)）
  - 验收：circle path 首尾 ‖Δq‖ < threshold；输出 closure_gap_deg

- **W7-T2**：endpoint repair fallback（0.5 天）
  - 关键：cyclic 失败 → 局部重排起点或插入 transition
  - 验收：故意 wrist-flip case 不静默输出假 OK，status 设为 CYCLIC_REPAIR_FAIL

- **W7-T3**：Yaw Adaptive Refinement（1 天）
  - 文件：`src/robot/IKCandidateGenerator.cpp`
  - 规则：失败段 12 yaw → 24 yaw
  - 验收：只在 fail segments 增加 yaw，不全局爆炸

- **W7-T4**：Branch/yaw Visualization（0.5 天）
  - 文件：`src/diagnostics/FailureHeatmap.cpp`
  - 输出：branch_id / yaw_id 染色 ply
  - 验收：能定位 wrist flip 位置

- **W7-T5**：armadillo dry-run（IK 完整）（1 天）
  - 文件：`scripts/run_armadillo_ik.sh`
  - 验收：armadillo IK 报告完整；失败段标注；程序不 abort

### 风险

- Cyclic DP 调试失控 → 论文降级 open DP + endpoint repair，cyclic 放 appendix

---

## Week 8：TransitionPlanner（前半）+ Collision Validator（后半）+ Experiment Runner 骨架

**目标**：把 Week 7 的 Transition 任务移到这里 + 实现 Stage 6 碰撞验证 L1/L2。

### 任务

- **W8-T1（前半）**：**TransitionPlanner**（P0-7）（2 天）
  - 文件：`src/trajectory/TransitionPlanner.cpp`
  - 关键：6 keypoint (P0-P5)，LIN 全段，4 级 fallback
  - 输出：`transition_segments.csv`, `transition_metrics.json`
  - 验收：
    - synthetic two-island layer：transition 生成 + wire_off + collision proxy pass
    - synthetic blocked case：第一 z_safe 失败，重试更高 z_safe 成功
    - unavoidable blocked：status = TRANSITION_FAIL，KRL 不导出假连接

- **W8-T2（后半）**：**PrintedState 模块**（P0-6 关键）（1 天）
  - 文件：`src/collision/PrintedState.cpp`
  - 关键：按 path 顺序增量更新（不只是 per-layer）
  - 同 layer 内 path-to-path 自碰撞必须显式处理
  - 验收：synthetic case 同层第 2 个 path 撞第 1 个 → 正确报 collision

- **W8-T3**：NozzleModel + SDFClearancePrefilter（L1）（0.5 天）
  - 文件：`src/collision/{NozzleModel,SDFClearancePrefilter}.cpp`
  - 验收：synthetic case 明显远离的帧不进 L2；接近的标 high-risk

- **W8-T4**：CapsuleSampler + igl::AABB L2（1 天）
  - 文件：`src/collision/{CapsuleSampler,FCLValidator}.cpp`（FCLValidator 用 igl::AABB 实现）
  - 验收：capsule 穿模 → COLLISION_FAIL；自由 → OK

- **W8-T5**：FCL 可选 build attempt（0.5 天，可放弃）
  - 尝试 vcpkg install fcl:x64-windows
  - 1 天装不通 → 放弃 FCL，论文写 "Conservative capsule-sampling collision validation"
  - 验收：若装通，FCL 作 L2 升级版

- **W8-T6**：**Experiment Runner 骨架**（提前到 Week 8）（1 天）
  - 文件：`scripts/run_matrix.py`
  - 关键功能：`--resume`, `--cache`, `--max-parallel 2`, 每 run 写 `run_status.json`
  - 验收：bunny smoke 4 methods 能跑通

### 风险

- TransitionPlanner 6-keypoint 调试超 2 天 → 简化为 4-keypoint（v4 §16 原版），4 级 fallback 保留

---

## Week 9：Post-quintic L3 + KRL OK + Schema 冻结 + Physical Print Rehearsal

**目标**：完成 Stage 6 闭环，**Schema v1.0 冻结**，Week 10 前做 physical print rehearsal 降风险。

### 任务

- **W9-T1**：Quintic dense sampling + L3 validation（1.5 天）
  - 文件：`src/trajectory/QuinticInterpolator.cpp`, `src/collision/CollisionValidator.cpp`
  - L3：4ms 全帧 SDF prefilter + stride FCL + high-risk frames
  - 输出：`validation_report.json`（checked frames, FCL calls, runtime）
  - 验收：bunny 完整 dense validation < 5 分钟

- **W9-T2**：KRLWriter OK + valid transitions 模式（P0-7）（1 天）
  - 文件：`src/export/KRLWriter.cpp`
  - 关键：export if status==OK and motion_type ∈ {PRINT, RETRACT, TRAVEL, APPROACH}
  - 验收：KRL 包含连续 OK + 有效 transition；不含任何 *_FAIL 段

- **W9-T3**：Failure Heatmap（0.5 天）
  - 文件：`src/diagnostics/FailureHeatmap.cpp`
  - 输出：`failure_heatmap.ply`（按 status type 染色）
  - 验收：IK / collision / sing / vel 颜色可区分

- **W9-T4（关键）**：**Schema v1.0 冻结**（P2-1, P2-2, Opus 4.1）（0.5 天）
  - 文件：`schemas/{trajectory_v1.md, diagnostic_v1.schema.json, validation_report_v1.schema.json}`
  - 必需字段：34 列 trajectory.csv（含 feed_mm_s, sigma_min, joint_margin, clearance_mm, validation_level, failure_code）
  - Invariants：frame_id 递增, timestamp = frame × dt, OK+PRINT → wire_on=1, etc.
  - 验收：`scripts/validate_schema.py runs/bunny/proposed/diagnostic.json` 通过

- **W9-T5**：bunny full dry-run + bunny smoke matrix（1 天）
  - 文件：`scripts/run_bunny_full.sh`
  - 4 methods（M1 v4 PDE / M2 / M3 / M4）smoke run
  - 验收：bunny 至少一个连续 OK segment 可上机

- **W9-T6（关键）**：**Physical Print Rehearsal**（Opus 推荐，0.5 天）
  - 用 bunny KRL 文件干跑 30 秒（无挤出），观察机器人运动
  - 不真打，纯检验 KRL 兼容性 + 机器人响应
  - 验收：机器人按 KRL 跑，无奇怪跳变；TCP 轨迹与设计一致

- **W9-T7**：bunny 实机 success criteria 写死（OPEN-12）（0.5 天）
  - 文件：`docs/physical_print_criteria.md`
  - 定义："Hausdorff ≤ 3mm AND no layer separation AND bead overlap ≥ 50%"
  - 写在 docs 里，Week 10 不许改

### 风险

- KRL 格式与控制器不匹配 → Week 9 末发现，Week 10 前手工适配

---

## Week 10：bunny 实机（重点）+ bunny full matrix

**目标**：bunny 实机打印 + bunny full matrix。**armadillo smoke 提前到 Week 7 已完成，本周不再做**。

### 任务

- **W10-T1**：机器人 dry-run + TCP/base 校准（1 天）
  - 文件：`docs/calibration.md`
  - 4-point 或 6-point TCP 校准
  - 输出：TCP 偏差估计 + base frame transform
  - 验收：TCP repeatability 误差 < 1mm

- **W10-T2**：bunny 实机阶梯尝试（2 天）
  - **优先**：bunny 完整低速打印（满足 `success_criteria.md`）
  - **Day 3 不通**：降级 small bunny (50% 缩放)
  - **Day 4 不通**：降级 rounded cube
  - **Day 5 不通**：仅 robot dry-run + extrusion-off
  - 输出：照片 / 视频 / 失败记录
  - 验收：至少一段曲面路径实际出料成功

- **W10-T3**：bunny full matrix（1 天）
  - 4 methods × bunny = 4 runs
  - 锁定 model_transform / voxel / spacing / thresholds
  - 验收：4 个 method 都有 trajectory.csv 输出，metrics 完整

- **W10-T4**：实机数据整理（0.5 天）
  - 文件：`runs/physical_print/report.md`
  - 验收：论文可用图至少 3 张

- **W10-T5**：armadillo smoke matrix（0.5 天）
  - 4 methods × armadillo smoke = 4 runs（仅 layer + path，不完整 dry-run）
  - 验收：4 个 method 都不 crash

### 风险

- 实机校准超 1 天 → 砍 small bunny + cube 尝试，直接 robot dry-run
- bunny full matrix 跑不完 → 推到 Week 11

---

## Week 11：armadillo full + mao stress test + Diagnostic + RegularizationReport

**目标**：完成复杂模型 dry-run + 失败定位。

### 任务

- **W11-T1**：armadillo full matrix（4 methods × armadillo full = 4 runs）（1.5 天）
  - 验收：能输出 IK/collision failure segments；不要求全 OK

- **W11-T2**：mao_regularized smoothing 强度固定（0.5 天）
  - 在 Week 11 前做最后一次 smoke：强度调到 < 5mm 抹平
  - 固定 OPEN-1 参数，写入 `configs/mao_regularization.yaml`
  - **不再为 IK pass rate 调整**（避免 hyperparameter mining）

- **W11-T3**：mao_regularized stress test（4 methods × mao_regularized = 4 runs）（1.5 天）
  - 文件：`scripts/run_mao_stress.sh`
  - 验收：程序完整跑完；输出 failure heatmap；不 abort
  - 若 preprocessing gate 不过 → 降到 L3 stress test，记录在 paper

- **W11-T4**：Failure vs Topology Event Overlay（1 天）
  - 文件：`src/diagnostics/FailureHeatmap.cpp` 扩展
  - 输出：论文图（mao + armadillo 各 1 张）
  - 验收：failure heatmap 与 height_sweep events 在同一图

- **W11-T5**：RegularizationMetrics（0.5 天）
  - 文件：`src/diagnostics/RegularizationMetrics.cpp`
  - 输出：mao_original vs mao_regularized 对比（Hausdorff / normal / silhouette IoU / volume）
  - 验收：论文 figure 可直接用

- **W11-T6**：α/β/γ 消融决定（OPEN-8）（0 天，仅决策）
  - 看 Week 10-11 进度
  - 有时间 → Week 12 加 6 runs（bunny + armadillo 单因素）
  - 没时间 → 论文写 limitation："per-weight ablation left to future work"

### 风险

- mao 0.5mm 无法运行 → 用 1.0mm；0.5mm 只给局部 crop

---

## Week 12：Final Matrix + δ Ablation + 论文写作

**目标**：形成可投/可答辩实验包。

### 任务

- **W12-T1**：Final matrix frozen run（1 天）
  - `python scripts/run_matrix.py --models bunny armadillo mao_regularized --methods v4_pde dijkstra_geom dijkstra_mfg proposed_ik --resume --cache --max-parallel 2`
  - 验收：12 runs 全部完成；schema 全部通过 validate_schema.py

- **W12-T2**：δ Ablation（bunny + armadillo）（0.5 天）
  - δ ∈ {0, 0.25, 0.5, 1.0}
  - 8 extra runs（mao 只跑 δ*）
  - 验收：δ ablation table 完整

- **W12-T3**：α/β/γ 消融（如果时间允许）（0.5 天，**可砍**）
  - bunny + armadillo 上 α=0/β=0/γ=0 单因素
  - 6 extra runs
  - 验收：若做则有 ablation table；否则 limitations 写明

- **W12-T4**：Tables + Plots（1 天）
  - 文件：`scripts/make_tables.py`, `scripts/make_plots.py`
  - 主表（Table 1）：4 methods × 3 models
  - Capability table（Table 2）
  - δ ablation table（Table 3）
  - Failure heatmaps（Fig）
  - Regularization comparison（mao Fig）
  - Height-sweep overlay（Fig）

- **W12-T5**：异常 run 重跑（0.5 天）
  - 最多重跑 2-3 个 run
  - 不再 retune 参数

- **W12-T6**：论文写作（1.5 天）
  - 实验章节 + failure analysis
  - Limitations + baseline discussion
  - 排版

### 风险

- Final matrix 跑超 1 天 → Week 11 提前预跑分担

---

## 实验矩阵汇总

### 主对照（必做）
- 4 methods × 3 models = **12 runs**

### δ 消融（必做）
- bunny + armadillo × δ ∈ {0, 0.25, 0.5, 1.0} = **8 runs**
- mao 只跑 δ* = **0 extra**

### α/β/γ 消融（OPEN-8，可砍）
- bunny + armadillo × {α=0, β=0, γ=0} = **6 runs**

### 总实验量
- 必做：20 runs
- 可选：6 runs
- 加 buffer：~25 runs

### 时间分布
- Week 9 末：bunny smoke 4 runs
- Week 10：bunny full 4 runs
- Week 11：armadillo full 4 + mao stress 4 = 8 runs
- Week 12 Day 1：补跑或 final frozen
- Week 12 Day 2-3：δ ablation 8 runs

---

## 关键交付物清单

实施完成后，仓库必须有：

### 代码
- `src/` 完整实现（30+ 模块）
- `tests/` 完整 unit test + integration test
- `CMakeLists.txt` 可在 Windows + MSVC + NMake 编译

### 文档
- `docs/architecture/V6_SPEC.md`（已有）
- `docs/superpowers/plans/2026-05-15-v6-implementation.md`（本文件）
- `docs/v4_v6_interface_audit.md`
- `docs/frames.md`
- `docs/calibration.md`
- `docs/physical_print_criteria.md`

### 数据
- `runs/bunny/{v4_pde,dijkstra_geom,dijkstra_mfg,proposed_ik}/`
- `runs/armadillo/{...}/`
- `runs/mao_regularized/{...}/`
- `runs/physical_print/`

### 论文素材
- `paper/method.md` (Section 3-4)
- `paper/experiments.md` (Section 5)
- `paper/limitations.md` (Section 6)
- `paper/figures/` (failure heatmaps, regularization comparison, etc.)
- `paper/tables/` (Table 1-3 CSV)

### Schema
- `schemas/trajectory_v1.md`
- `schemas/diagnostic_v1.schema.json`
- `schemas/validation_report_v1.schema.json`

### 实验脚本
- `scripts/run_matrix.py`
- `scripts/collect_metrics.py`
- `scripts/make_tables.py`
- `scripts/validate_schema.py`
- `scripts/check_trajectory_csv.py`

---

## 风险总图与降级路径

| Week | 风险点 | 失败 fallback |
|---|---|---|
| Week 1 | v4 封装超 2 天 | 写 adapter 不重构 |
| Week 2 | 0.5mm voxel 太慢 | 全 0.8mm |
| Week 3 | curvature 噪声 | γ=0，仅 diagnostic |
| Week 4 | p_ik 太慢 | 2.4mm 粗 grid |
| Week 5 | geodesic 不稳定 | raster 作主路径 |
| Week 6 | model placement 差 | --model-transform 移到 sweet spot |
| Week 7 | cyclic DP 调试失控 | 论文降级 open DP，cyclic 放 appendix |
| Week 8 | TransitionPlanner 超时 | 简化为 4-keypoint（v4 §16） |
| Week 8 | FCL 装不通 | 用 SDF/capsule proxy |
| Week 9 | KRL 不兼容 | 手工适配 |
| Week 10 | bunny 实机失败 | 阶梯降级 small bunny → rounded cube → dry-run only |
| Week 11 | mao 不过 gate | 降到 L3 stress test |
| Week 12 | matrix 跑不完 | 砍 α/β/γ ablation；先 final 12 + δ 8 |

---

## 给 CLI #2 的执行准则

1. **每周末必须有可视化输出**（PLY/STL/PNG），人眼审过才能进下周
2. **Schema 一旦冻结（Week 9 末）不许改字段**
3. **不准 retune 参数让数字好看**（hyperparameter mining 红线）
4. **失败必须落盘**（IK_FAIL / COLLISION_FAIL / TRANSITION_FAIL 都要有 index, 位置, 层号）
5. **mao 的"成功"是完整失败定位报告，不是全 OK trajectory**
6. **Physical print 只押一个小模型**，不押 mao，不押完整 armadillo
7. **每个 task 完成后跑全部 ctest 防回归**
8. **每个 commit 独立可 build**（不打包多 task）
9. **报告失败时必须 dump 数据**（不只是"失败了"，要给 CLI #1 判 root cause 的材料）
10. **遇到 conceptual question 立即停下问 CLI #1**，不自己拍板

---

## 总结

12 周计划 = ChatGPT 6 轮 + Opus 2 轮评审收敛后的可执行版本。
- L4 目标合理但 aggressive，含完整 fallback 路径
- 关键调整：Transition 推到 Week 8、armadillo smoke 提前到 Week 7、Schema Week 9 冻结、Physical rehearsal Week 9 末
- 关键新增：Interface audit (Week 1)、Penalty normalizer (Week 3)、Branch consistency (Week 4)、p_ik cache contract (Week 4)、PrintedState 优先实现 (Week 8)
- 总实验量 ~20 runs (必做) + 14 runs (可选)，10-12 周可达
