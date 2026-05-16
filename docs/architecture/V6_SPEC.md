# v6.2 算法规范：基于制造性加权标量场的六轴 FDM 曲面层切片

> **状态**：经 6 轮 ChatGPT 5.5 + 2 轮 Claude Opus 4.7 评审收敛 + 用户最终拍板。
> **日期**：2026-05-15
> **作用**：CLI #2 实施的唯一权威 spec。v4 §10-§17 + v5 拓扑路由方案均**作废**。
> **v4 PDE 实现保留**作论文 baseline 比较，不再继续维护。

---

## 0. 一句话概述

在体素图上做**多约束加权 Dijkstra 距离场**，把"间隙不足 / 朝向偏离 / 曲率过大 / 机器人 IK 边际度低"四类**可制造性风险**编码为边权惩罚，得到的距离场等值面作为弯曲层；再做路径级 IK 动态规划 + 保守碰撞验证 + KRL 离线导出，在 bunny / armadillo / mao_regularized 三模型上完成 dry-run，bunny 低速实机打印。

---

## 1. 背景与目标

### 1.1 硬件

| 项 | 配置 |
|---|---|
| 机器人 | KUKA KR4 R600（6 关节球腕，关节限位已知，最大伸展 ~600mm）|
| 喷头 | FDM，圆柱近似 r_n=10mm, h_n=30mm |
| 通信 | KRL 离线 .src/.dat（主线），RSI Ethernet 4ms（v4 §15b 设计保留，本 spec 不实施） |
| 工件 | 固定，机器人 6 DOF 绕工件运动 |

### 1.2 输入输出

- **输入**：STL 网格 + `robot.yaml`（关节限位/DH 参数）+ `nozzle.yaml`（r_n, h_n）+ `weights.yaml`（α/β/γ/δ）+ `model_transform.yaml`（手工配置的位姿变换）
- **输出**：`trajectory.csv`（每帧 6 关节角 + 状态标签）+ `validation_report.json` + `failure_heatmap.ply` + `diagnostic.json` + KRL `.src/.dat`（仅可执行段）

### 1.3 三模型目标（L4）

| 模型 | 处理 | 期望产出 |
|---|---|---|
| **bunny** | Stanford 标准 bunny | 完整 dry-run + 低速实机打印一件 |
| **armadillo** | Stanford 标准 armadillo | 完整 dry-run + failure heatmap |
| **mao_regularized** | mao 头雕重度 smoothing 后（抹平 < 5mm 细节）| 完整 dry-run，trajectory 全 OK |
| mao_original | 原始 mao | 仅作 geometry deviation 对比，不打印 |

**mao_regularized 定义**：抹平 < r_n/2 = 5mm 的所有细节。视觉上不像毛主席本人，但保留头型轮廓。论文叙事：**manufacturability-regularized head-shape benchmark with quantified geometric loss**。

---

## 2. 四类可制造性指标（取代旧的"四约束"措辞）

⚠️ **重要**：这些是**指标**，不是算法保证的硬约束。Scalar field 提供 soft bias 减少失败率；约束满足由 **post-validation** 判定；失败区域显式上报。

| 编号 | 名称 | 描述 | 检查阶段 |
|---|---|---|---|
| **M1 ASN 粘合** | 邻接性 | 每体素至少有 1 个 ASN 邻居在前层（Wang 2018 定义，18 邻居：6 面 + 12 棱）| Stage 3 inter-layer support check |
| **M2 IK 可达性** | 机器人运动 | 解析 IK 解 + 关节限位 + 远奇异 + 工作空间 | Stage 5 IK DP |
| **M3 喷头碰撞** | 几何干涉 | 圆柱体 vs 已打印部分（含同层自碰撞）| Stage 6 三层验证 |
| **M4 曲率干涉** | 局部几何 | max(\|κ₁\|, \|κ₂\|) ≤ 1/r_n（最大主曲率，via shape operator）| Stage 0 检查 + Stage 1 标记 |

---

## 3. 算法 6 阶段详解

### Stage 0 — 几何调理（Geometry Conditioning）

**目的**：把工艺上不可打印的几何特征转为"可制造近似"。承认 r_n=10mm 的物理极限。

**输入**：原始 STL 网格 M
**输出**：`mesh_clean.ply`（轻度清理）+ `mesh_regularized.ply`（重度正则化，仅 mao 需要）

**A. 轻度几何清理**（所有模型）
- 网格修复（`igl::resolve_duplicated_faces`、孔洞填充、流形检查）
- 自交解决
- 抽稀去除 < bead_width = 0.5mm 的几何噪声
- **硬约束**：`Hausdorff(M, mesh_clean) ≤ 0.5mm`，法向偏离 ≤ 5°

**B. 重度正则化**（仅 mao_regularized）
- 主动 Laplacian smoothing 抹平 < r_n/2 = 5mm 的细节
- 薄壁 dilation 到 2 × bead_width
- **不再保留 Hausdorff ≤ 0.5mm 硬约束**（与正则化目标矛盾）
- 必报 `geometry_regularization` metrics（见 §6.3）

**输出对比报告**（mao 必须）：
- hausdorff_p95, hausdorff_max
- normal_deviation_p95
- volume_change_ratio
- silhouette_iou_front, silhouette_iou_side
- 论文图：原始 vs 正则化对比渲染

---

### Stage 1 — 体素化 + 基础场

**目的**：把连续模型离散到体素，预计算后续阶段需要的标量/向量场。

**输入**：`mesh_clean.ply` 或 `mesh_regularized.ply` + `model_transform.yaml`
**输出**：voxel grid + SDF + cavity_radius + κ_max + N_smooth

**步骤**：

1. **应用 model_transform**：把 mesh 平移/旋转到机器人 sweet spot（手工 yaml，见 §4 D11）
2. **体素化**：默认 0.8mm 间距（所有模型开发期一致）；Week 10+ bunny 可选 0.5mm
3. **SDF**：`igl::signed_distance` 或 fast marching，narrow band 模式
4. **cavity_radius field**（P0-4 定义）：
   ```
   对每个 surface-near voxel v:
     for d_i in 32 sphere directions:
       cast ray from v + small offset * inward_normal
       l_i = distance to first opposite-surface hit
     cavity_radius(v) = percentile_20(l_i) / 2
   ```
   论文叫 "local opening-radius proxy"，**不叫 nozzle clearance**
5. **曲率场 κ_max**：用 `igl::principal_curvature` 在 mesh 顶点上算 κ₁, κ₂，再插值到 voxel
6. **法向场 N_smooth**：从 mesh 顶点法向插值到 voxel，再做 1-2 轮邻域平均

**预筛**：标记 hard infeasible voxels（cavity_radius < r_n / 2 的体素直接打 SHADOW，进 Stage 2 时跳过）

---

### Stage 2 — 制造性加权标量距离场（4-Pass，**核心算法**）

⚠️ **关键原则**（P0-1, P0-2）：所有 penalty 项必须 **normalize 到 [0,1]** 才能进边权；diagnostic.json 同时记录 raw / norm 分布。

#### 通用 PenaltyNormalizer

对每个 raw penalty 场 p_raw：
```
p_clip(v) = winsorize(p_raw, p05, p95)
p_norm(v) = clamp((p_clip - p05) / (p95 - p05 + ε), 0, 1)
```
在 diagnostic.json 记录：
```json
"penalty_stats": {
  "p_clear_raw_p05/p50/p95": ...,
  "p_clear_norm_mean/std": ...,
  "p_ik_raw_p95": ...,
  ...
}
```

#### Pass 1：纯几何距离场 D_0

**边权**（4 项 normalized penalties，无 IK 项）：
```
w(u,v) = ‖u-v‖_mm × [1 + α·p_clear_n + β·p_orient_n + γ·p_curv_n]
```

各项 raw 定义：
```
p_clear_raw(v) = max(0, r_n + margin - cavity_radius(v))²        # mm²
p_curv_raw(v)  = max(0, κ_max(v) - 1/r_n)²                       # 1/mm⁴

# P0-3 修正：p_orient 用 blended preferred direction
B(v) = normalize((1 - η) · build_direction + η · N_smooth(v)),  η = 0.3
p_orient_raw(u,v) = 1 - |normalize(v-u) · B(v)|                  # 无量纲
```

**算法**：
- 起点：geometric bottom band 体素（z 最小的一层）
- Dijkstra（28 邻接 + 边权 w）
- 输出 D_0_mm（**单位严格毫米**）

#### Pass 2：估算工具姿态（仅锁 tool_z，yaw 留自由）

⚠️ **Opus O-1.1 修正**：tool_z 用 **∇SDF** 而非 ∇D_0（避免加权梯度与几何法线脱钩）

```
tool_z(v) = -normalize(∇SDF(v))   # 几何外法向反方向
# 稳定参考轴构造（避免选 path tangent，path 还不存在）
ref = world_x  if |world_x · tool_z| < 0.9 else world_y
tool_x_ref(v) = normalize(ref - dot(ref, tool_z) * tool_z)
tool_y_ref(v) = cross(tool_z, tool_x_ref)
yaw ψ 留作 Stage 5 的 DP 状态变量
```

#### Pass 3：IK 可达性场 p_ik

**关键优化**（ChatGPT Q3）：
- IK 一次返回 8 branches（不调 8 次）
- p_ik 用 **粗 grid（1.6mm，slicing grid 的 2×）**，trilinear 插值回 0.8mm
- 多线程 OpenMP

```
对每个 coarse voxel v（1.6mm grid，仅 surface-near band）:
  for ψ in 12 yaw_samples (均匀 30° 分):
    R_ψ = compose(tool_z(v), tool_x_ref(v), ψ)
    sols = analytical_IK_all_8_branches(v, R_ψ)
    
    # 快筛 + 仅 top candidates 算精确 SVD
    candidates = filter(sols, joint_limits_OK)
    candidates = filter(candidates, |sin(q5)| > 0.05)  # 腕奇异 cheap proxy
    top3 = sort by joint_margin_proxy, take top 3
    
    for q in top3:
      J = jacobian(q)
      sigma_min = svd(J).min
      J_ik(q) = a · jl_penalty_norm(q) + b · sing_penalty_norm(σ_min) + c · ws_penalty_norm(q)
      
    J_ik_best_branch(ψ) = min J_ik over q
    branch_id_best(ψ) = argmin
  
  J_ik_min(v) = min J_ik_best_branch over ψ
  best_branch_at_v = argmin (ψ, branch_id_best)

# Opus O-A 修正：加 branch consistency 项
branch_volatility(v) = 1 - (max_count of best_branch_at_neighbor(v) / |1-ring neighbors|)
# 0 = 邻域内 branch 一致，1 = 频繁切换

p_ik_raw(v) = J_ik_min(v) + λ_consist · branch_volatility(v)    # 默认 λ_consist = 0.5
```

**J_ik 内部权重 a/b/c**（Opus 1.3）必须分别 normalize：
```
jl_penalty_norm(q)  = Σᵢ (1 - (q_i - q_mid_i) / (q_range_i / 2))²  ∈ [0, 1]
sing_penalty_norm(σ) = exp(-σ / σ_ref) where σ_ref = 0.05    ∈ [0, 1]
ws_penalty_norm(q)  = ramp(‖TCP‖, r_min, r_max)              ∈ [0, 1]
默认：a = b = c = 1/3
```

最后 trilinear 插值 p_ik 回 0.8mm grid，做 PenaltyNormalizer → p_ik_norm。

#### Pass 4：最终距离场 D_final + bounded smoothing → D_slice

**Pass 4a：含 IK 项的 Dijkstra**
```
w(u,v) = ‖u-v‖_mm × [1 + α·p_clear_n + β·p_orient_n + γ·p_curv_n + δ·p_ik_n(v)]
D_final_mm = Dijkstra(同 Pass 1 设置)
```

**Pass 4b：Bounded edge-aware smoothing**（P0-5）
```
D_s = D_final
repeat T = 5 次:
  D_tmp(v) = (1/|N(v)|) · Σ_{u ∈ N(v)} D_s(u)     # 邻域均值
  D_s(v) = clamp(D_tmp(v), D_final(v) - ε, D_final(v) + ε)
  ε = 0.3 × layer_height_mm
D_slice_mm = D_s
```

**Pass 4c：单位审计**
- 保留 D_final_mm（原始 Dijkstra 累积）
- 保留 D_slice_mm（smoothed，给 MC 用）
- 输出 D_norm_unitless（仅可视化）
- **断言禁止 MarchingCubes 用 D_norm**：CMake test 加 unit test 防混用

⚠️ **Opus 病灶 B 说明**：D_final_mm 虽然名字带 _mm，**不是真实物理距离**，而是"加权最短路径累积距离"。在高 penalty 区域，1mm 的 D_final 对应几何上更短距离。论文必须说明：
> The scalar field D is a manufacturability-weighted shortest-path distance, not Euclidean distance. Iso-surface spacing in D corresponds to actual layer thickness modulated by local penalty. Effective vs nominal layer thickness scatter is reported in Fig. X.

---

### Stage 3 — 等值面提取 + 层支撑检查

**输入**：D_slice_mm
**输出**：layers/*.ply + layer_metrics.csv

**步骤**：

1. **Marching Cubes**：在 D_slice_mm 上 at iso = k × layer_height_mm (k = 1, 2, ..., N)
   - 默认 layer_height_mm = 1.2
   - 输出每层 mesh → `layers/layer_NNN.ply`

2. **组件标记**（Opus 6 + P0 P0-6 集成）：**不删小组件**
   ```
   for each layer:
     run connected components on mesh
     label each component: MAIN (largest), SMALL_COMPONENT, BROKEN_LAYER (max < 80%)
   ```
   下游 Stage 4 按 policy 决定取舍：
   - `--component-policy all`：全部进 path generation
   - `--component-policy largest`：仅 MAIN（默认）
   - `--component-policy area-threshold`：area > τ 的进

3. **Inter-layer support check**（M1 后验，P0-1 关键）：
   ```
   support_radius = layer_height_mm × 1.5     # 经验默认
   for each layer k:
     for each sampled point p in layer_k:
       d_to_prev = min distance from p to layer_{k-1} mesh
       support_ok(p) = d_to_prev ≤ support_radius
     support_pass_rate(k) = ratio of support_ok
   broken_flag(k) = support_pass_rate < 0.9
   ```
   输出：`support_metrics.csv`, `support_heatmap.ply`

---

### Stage 4 — 层内路径生成（component-level mode）

**输入**：每层 mesh + components
**输出**：每层若干 polyline + path_metrics.csv

**两种 path mode（按 component 选择）**：

#### Mode A：Geodesic contour（复用 v4 §13）
```
对每个 component:
  d_geo = igl::exact_geodesic from seed (centroid 最近顶点)
  for i in 1..N_lines:
    iso_d = i × line_spacing_mm    # 默认 0.8mm
    polylines_i = igl::isolines(V, F, d_geo, iso_d)
  for each polyline:
    stitch (复用 v4 已修复的 polyline stitching)
    normalize start point (z 最小，复用 v4 §13.2.1)
    compute tangents (Frenet 中心差分)
    resample 0.2mm
```

#### Mode B：Raster fallback（P0 P0-4, Q9 几何补偿）
触发条件（per component）：geodesic 失败时
```
stitching 通过判据（per component）：
  closed_loop_ratio ≥ 0.95
  open_endpoint_count ≤ 2
  max_endpoint_gap ≤ 1.5 × resample_step_mm
  coverage_gap_p95 ≤ 1.5 × line_spacing_mm
  self_intersection_count < threshold
全部通过 → 用 Mode A
否则 → 该 component 用 Mode B
```

Mode B 算法：
```
对每个 fallback component:
  PCA 三轴 e1/e2/e3
  raster direction = e1
  
  # 几何补偿：plane spacing 按 surface projection 修正
  g(p) = sqrt(1 - dot(e1, n(p))^2)        # tangent component
  g_med = median g over component samples
  Δs = line_spacing_mm × clamp(g_med, 0.5, 1.0)
  
  for s = s_min, s_min + Δs, ...:
    iso_polylines = mesh ∩ plane(dot(x, e1) = s)
    stitch + resample
  
  排序：蛇形（snake order）减少 retract
  
  通过判据：
    spacing_p95_mm ≤ 1.5 × line_spacing_mm
    spacing_max_mm ≤ 2.0 × line_spacing_mm
  不过 → Δs *= 0.75 重试；3 次仍不过 → 标 PATH_FAIL
```

每条 polyline 输出 metadata：
```json
{
  "layer_id": 17, "component_id": 2, "path_id": 0,
  "path_mode": "GEODESIC" | "RASTER_FALLBACK",
  "n_points": 143,
  "is_closed": true,
  "fail_reason": null
}
```

---

### Stage 5 — 六轴轨迹生成

**输入**：每层 polylines
**输出**：trajectory_keypoints.csv（DP 输出）→ trajectory_dense.csv（quintic 后）

#### 5a：IK 候选枚举

对每个 polyline point P：
- 工具姿态：tool_z 沿层法向（从 mesh 顶点法向插值），tool_x 沿路径切线投影
- 12 yaw samples（30° 等分）
- 每个 yaw 算 8 IK branches（一次性返回）
- = 最多 96 候选

#### 5b：维特比 DP 选 branch 序列

```
state = (ik_branch ∈ [0,7], yaw_id ∈ [0,11])     # 96 states max
top-K 剪枝：每点保留按 J_ik_total 排序的前 12 候选     # K = 12

transition cost(state_i → state_{i+1}):
  w_q · ‖Δq‖²        joint angle continuity
  w_v · vel_penalty   velocity penalty
  w_s · sing_penalty  singularity proximity
  w_y · |Δyaw_id|     yaw continuity (限相邻 ≤ 2 bin)

hard constraints（P0-8 软化）：
  DP 阶段用 provisional Δt_cart = d_cart / v_target
  |Δq| ≤ qdot_max × Δt_cart × stretch_limit (= 3)
  branch compatibility
  yaw |Δyaw_id| ≤ 2

复杂度：O(K² × N) = O(144 N)
```

**闭合路径**（cyclic DP，P0-8 / Opus 1.4 修正）：
- ⚠️ **不**用枚举起点的 O(K³N)
- 改用 **fixed-point iteration**：
  ```
  1. 先跑 open DP，取最优解 q_open
  2. 强制起点 state = q_open[0]，再跑一次 open DP 含 closure cost(q_N, q_0)
  3. 收敛或迭代上限 2 次
  4. 输出 closure_gap_deg，若 > threshold 标 CYCLIC_REPAIR_FAIL
  ```

#### 5c：时间参数化（DP 完成后）

```
对每段 (q_i, q_{i+1}):
  Δt_final = max(
    d_cart_i / v_target_mm_per_s,
    max_j |Δq_j| / qdot_max_j,
    sqrt(max_j |Δ²q_j| / qacc_max_j)
  )
```

#### 5d：五次多项式插值（P0-9 修正）

- **只在 path 首尾零速度**
- 中间 waypoint 速度用 chord-length finite difference 估
- 加速度连续（除非首尾点）
- 采样到 4ms 周期密化 → trajectory_dense.csv

```
v_i = (q_{i+1} - q_{i-1}) / (t_{i+1} - t_{i-1})    # 中间点
v_0 = 0, v_N = 0                                    # 端点
分段五次多项式 (q_i, v_i) → (q_{i+1}, v_{i+1})
按 4ms 步长求值
```

---

### Stage 6 — 碰撞验证 + Transition + 输出

#### 6a：PrintedState 模块（P0-6 关键）

**关键**：必须按 path 顺序增量更新，不只是按 layer：
```python
printed_state = PlatformGeometry()     # 初始仅平台

for layer in layers:
    for component in sorted_components(layer):
        for path in sorted_paths(component):
            # 验证 path 上每个 frame 不撞 printed_state
            for frame in path.frames:
                if not validate(frame, printed_state):
                    mark frame as COLLISION_FAIL
                    break
            
            if all frames OK and path.wire_on:
                # 增量加入 swept bead mesh
                printed_state.add(path.swept_bead_mesh)
        
        # 同 component 内 path 间也要 validate transition
        if next_path exists:
            transition = TransitionPlanner.build(this_path.end, next_path.start, printed_state)
            validate transition
```

BVH 重建策略：每 layer 完成时 rebuild printed_state BVH（不做 per-path 增量，太慢）

#### 6b：三层碰撞验证

| Level | 算法 | 速度 | 用途 |
|---|---|---|---|
| **L1** | SDF/voxel clearance prefilter | 极快 | 去 90% 明显无碰撞帧 |
| **L2** | Capsule sampling vs printed BVH（`igl::AABB` 或可选 FCL）| 中速 | high-risk 帧详查 |
| **L3** | Post-quintic dense validation | 慢 | 4ms 全帧 SDF + stride FCL |

**FCL 处理（P2-2）**：默认走 L1+L2 的 SDF/capsule proxy。FCL 作 Week 8-9 可选增强，1 天装不通就放弃，论文写 "Conservative capsule-sampling collision validation"。

#### 6c：TransitionPlanner（P0-7, ChatGPT Q1）

**6 keypoint 版本**（不是 v4 §16 的 4 点）：

```
P0 = 上段末点 (last TCP, tool_z_end)
P1 = 局部回抽: P0 + d_retract × (-tool_z_end)
P2 = 升到 z_safe: same xy as P1, z = z_safe
P3 = 水平移到下段入口上方: xy from next_first_pt, z = z_safe
P4 = 局部接近: next_first_pt + d_retract × (-tool_z_start)
P5 = 下段首点

d_retract = max(5mm, 0.5 × r_n, bead + collision_margin) = 5-10mm
z_safe = local_max_z + max(10mm, r_n, 0.5 × h_n) ≥ 15mm
```

**5 段全用 LIN**（不用 PTP，PTP 扫掠不可控）：
- P0→P1: LIN slow, wire_off
- P1→P2: LIN
- P2→P3: LIN
- P3→P4: LIN
- P4→P5: LIN

**Transition 必须 collision check**：
```
for transition segment:
  IK check (per keypoint + dense interpolation)
  velocity check
  singularity check
  collision check (vs current printed_state)
```

**4 级 fallback**（transition 失败时）：
```
F1: z_safe += 10mm，最多重试 3 次
F2: tool_z 切换到 vertical-safe orientation，yaw 重选
F3: z_safe 平面上做 2D detour（左/右/前/后 4 个 corner waypoint）
F4: 仍失败 → status = TRANSITION_FAIL，不导出该连接，后续段标 DISCONNECTED_OK
```

#### 6d：输出

**trajectory.csv**（schema_version: traj_v1.0，见 §6.1）

每段 motion_type ∈ {PRINT, RETRACT, TRAVEL, APPROACH, DWELL}
每段 status ∈ {OK, IK_FAIL, COLLISION_FAIL, SING_RISK, VEL_FAIL, ACC_FAIL, PATH_FAIL, TRANSITION_FAIL, DISCONNECTED_OK, BROKEN_LAYER, SKIPPED}

**KRL 导出规则**（P0-7 修正，**不是仅 PRINT_OK**）：
```
export if status == OK and motion_type in {PRINT, RETRACT, TRAVEL, APPROACH}
即：PRINT 是 wire_on=1 的真实沉积
   RETRACT/TRAVEL/APPROACH 是 wire_on=0 的有效运动
均导出到 .src/.dat
KRL 不导出任何 *_FAIL 段
```

**输出文件**：
- `trajectory.csv` (per-frame)
- `validation_report.json` (详细 collision stats)
- `failure_heatmap.ply` (按 failure type 染色)
- `transition_metrics.json` (transition 段统计)
- `diagnostic.json` (run-level summary, 见 §6.3)
- KRL `*.src` + `*.dat`

---

### Stage 7 — 诊断（post-hoc）

**Height-sweep connectivity diagnostic**（Opus + ChatGPT 共识：仅诊断，不参与算法）

```python
for z in sample_heights (default 100 samples):
    components_z = mesh ∩ plane(z = z)
    component_count(z) = len(components_z)
    detect merge/split events

apply persistence threshold τ = 2mm
output height_events.csv
output diagnostic figure: component_count(z) curve + IK/collision failure overlay
```

**论文图**：1-2 张，展示失败位置与 height events 的关联（不 claim "topology predicts failure"，只 claim "failure regions correlate with critical events"）。

---

## 4. 关键设计决策表（v6.2 全集）

| # | 决策 | 来源 |
|---|---|---|
| D1 | 不用 PDE（Helmholtz curl 问题），不用 Wang 贪婪（shadow region），不用拓扑路由（反例多） | 收敛 |
| D2 | 用 Dijkstra 加权最短路径 + 4 个 penalty term + IK-aware | 核心 |
| D3 | mao 重度 smoothing → mao_regularized，原 mao 仅作 deviation 对比 | 用户决定 |
| D4 | trajectory.csv 全段 status flag，KRL 输出 PRINT + 有效 transition（**P0-7 修正**） | ChatGPT |
| D5 | 三模型 voxel size: **开发期全 0.8mm**，Week 10+ bunny 可选 0.5mm | 用户 + P1-5 |
| D6 | layer_height 1.2mm 全模型一致 | 默认 |
| D7 | yaw 采样 12，IK DP top-K=12 | ChatGPT |
| D8 | **路径 Y**：KRL 离线主线，RSI 后续提效（v4 §15b 保留不实施） | **用户最终** |
| D9 | bunny 实机打印（成功是亮点），armadillo/mao dry-run | 用户决定 |
| D10 | bunny 实机阶梯 fallback：full bunny → small bunny → rounded cube → robot dry-run only | 用户决定 |
| D11 | model_transform 手工 yaml + 5k 顶点 IK pass > 90% sanity check | ChatGPT Q5 |
| D12 | FCL 不设硬依赖，主线 SDF + capsule sampling proxy | ChatGPT Q2 |
| D13 | **Baseline v4 PDE 自比**（不用 Planar-Z，不复现 Wang 2018） | **用户最终** |
| D14 | 不复现 Wang/Li/Jayakody，做 capability table | ChatGPT Q7 |
| D15 | v4 PDE 实现保留作 baseline，CLI #2 完成 v4 Plan 1 后切新分支 | 用户 X |
| D16 | Schema versioning 必须做（traj_v1.0 / diag_v1.0）| ChatGPT Q10 |
| D17 | 实验矩阵：M1 v4 PDE + M2 Dijkstra-geom + M3 Dijkstra-mfg + M4 Proposed × 3 models = 12 runs | 用户修订 |
| D18 | CLI 间数据流 file-based | 用户决定 |
| D19 | 路径间距统一用 target_line_spacing，raster 按 PCA 几何补偿 | ChatGPT Q9 |
| D20 | 拓扑分析仅作 diagnostic，不进 algorithm/title | 收敛 |
| D21 | **α/β/γ 消融延后**：spec 留 hook，Week 11-12 视时间决定做不做 | **用户最终** |
| D22 | **Penalty 全部 normalize 到 [0,1]**（P0-2）| Opus + ChatGPT |
| D23 | **p_orient 用 blended preferred direction B(v)**（P0-3） | ChatGPT |
| D24 | **D_slice = bounded smoothing(D_final)** 后再 MC（P0-5） | ChatGPT |
| D25 | **PrintedState 按 path 顺序增量更新**（P0-6） | ChatGPT |
| D26 | **DP-Δt 拆两阶段**（P0-8）：DP 用 provisional + stretch_limit | ChatGPT |
| D27 | **五次多项式只在 path 首尾零速度**（P0-9） | ChatGPT |
| D28 | **mao_regularized 必须过 preprocessing gate** 才进 full dry-run（P0-10） | ChatGPT |
| D29 | **Pass 3 加 branch_consistency**（Opus O-A） | Opus |
| D30 | **Pass 2 tool_z 用 ∇SDF 不是 ∇D_0**（Opus O-1.1） | Opus |
| D31 | **J_ik 内部 a/b/c 各 normalize**（Opus 1.3） | Opus |

---

## 5. 工程架构

### 5.1 三 CLI（file-based）

```bash
# CLI 1: 曲面层生成
curved_slicer \
  --input data/bunny.stl \
  --robot configs/kr4_r600.yaml \
  --nozzle configs/nozzle_fdm_10mm.yaml \
  --voxel 0.8 \
  --layer-height 1.2 \
  --weights configs/default_weights.yaml \
  --enable-ik-field true \
  --out runs/bunny_proposed

# CLI 2: 路径 + IK + 碰撞 dry-run
toolpath_validate \
  --layers runs/bunny_proposed/layers \
  --robot configs/kr4_r600.yaml \
  --nozzle configs/nozzle_fdm_10mm.yaml \
  --yaw-samples 12 \
  --top-k 12 \
  --out runs/bunny_proposed_validate

# CLI 3: KRL 导出
export_krl \
  --trajectory runs/bunny_proposed_validate/trajectory.csv \
  --out runs/bunny_proposed_krl
```

### 5.2 目录结构

```
src/
  app/                    # 三 CLI 入口
    curved_slicer.cpp
    toolpath_validate.cpp
    export_krl.cpp
  geometry/
    MeshRepair.cpp/.h
    MeshRegularization.cpp/.h
    Voxelizer.cpp/.h
    SDFGrid.cpp/.h
    CurvatureEstimator.cpp/.h
    CavityRadiusEstimator.cpp/.h
    HausdorffMetrics.cpp/.h
  field/
    WeightedDijkstraField.cpp/.h
    FieldWeights.cpp/.h
    PenaltyNormalizer.cpp/.h     # P0-2 新增
    IKFeasibilityField.cpp/.h
    FieldSmoothing.cpp/.h         # P0-5 新增
    FieldIO.cpp/.h
  layers/
    MarchingCubesLayers.cpp/.h
    LayerMetrics.cpp/.h
    SupportChecker.cpp/.h
    ComponentLabeler.cpp/.h
  path/
    GeodesicContourPath.cpp/.h    # 复用 v4 §13
    RasterFallbackPath.cpp/.h     # 新增
    PathStitcher.cpp/.h
    PathResampler.cpp/.h
    PathSpacingMetrics.cpp/.h
  robot/
    KR4Model.cpp/.h               # 复用 v4 §11
    AnalyticIK.cpp/.h
    Jacobian.cpp/.h
    IKCandidateGenerator.cpp/.h
    CyclicIKDP.cpp/.h
    TimeParameterization.cpp/.h
  trajectory/
    TransitionPlanner.cpp/.h      # P0-7 新增
    QuinticInterpolator.cpp/.h    # P0-9 修正
  collision/
    NozzleModel.cpp/.h
    PrintedState.cpp/.h           # P0-6 新增
    SDFClearancePrefilter.cpp/.h
    CapsuleSampler.cpp/.h
    FCLValidator.cpp/.h           # optional Week 8-9
  export/
    CSVWriter.cpp/.h
    KRLWriter.cpp/.h              # 复用 v4 §15a + OK + transitions
    DiagnosticWriter.cpp/.h
  diagnostics/
    HeightSweepDiagnostic.cpp/.h  # 提前到 Week 3-4 (P2-3)
    FailureHeatmap.cpp/.h
    RegularizationMetrics.cpp/.h  # mao 必须
configs/
  kr4_r600.yaml
  nozzle_fdm_10mm.yaml
  default_weights.yaml
  experiment_{bunny,armadillo,mao}.yaml
  model_transform_{bunny,armadillo,mao}.yaml
scripts/
  run_matrix.py                    # 实验运行器
  collect_metrics.py
  make_tables.py
  validate_schema.py               # P2 schema 检查
  check_trajectory_csv.py
schemas/
  trajectory_v1.md
  diagnostic_v1.schema.json
  validation_report_v1.schema.json
tests/
  test_dijkstra_field.cpp
  test_field_units.cpp             # P2 unit test 防 D_norm 混用
  test_penalty_normalizer.cpp
  test_ik_solver.cpp
  test_cyclic_dp.cpp
  test_transition_planner.cpp
  test_printed_state.cpp
  test_collision_validator.cpp
```

### 5.3 v4 复用映射

⚠️ **Opus 3.3 警告**：v4 "70% 复用" 是乐观估计，实际 30-50%。Week 1 必须做 interface audit。

| v4 模块 | v6 用途 | 改写量 |
|---|---|---|
| v4 §11 IK | 包装为 KR4Model + AnalyticIK，扩展为一次返回 8 branches | 中改写 |
| v4 §12 reachability | voxel/SDF/nozzle 工具函数复用 | 直接复用 |
| v4 §13 geodesic | wrap 为 GeodesicContourPath；polyline stitching 已修复直接用 | 小改写 |
| v4 §14 五次多项式 | 扩展为 chord-length finite difference 速度估计；time parameterization 拆两阶段 | 中改写 |
| v4 §15a KRL writer | 扩展为 OK + valid transitions 模式 | 小改写 |
| v4 §15b RSI sender | 保留 spec，不实施 | 不动 |
| v4 §10 BC + Poisson | **作废**，保留作 baseline M1 比较 | 不动 |

---

## 6. 数据 Schema

### 6.1 trajectory.csv（traj_v1.0）

**必需字段（34 列）**：

```
schema_version, run_id, frame_id, timestamp_ms, dt_ms,
layer_id, component_id, path_id, segment_id, transition_id,
motion_type, status, wire_on,
A1_deg, A2_deg, A3_deg, A4_deg, A5_deg, A6_deg,
tcp_x_mm, tcp_y_mm, tcp_z_mm, tcp_rx_deg, tcp_ry_deg, tcp_rz_deg,
ik_branch, yaw_id,
feed_mm_s, sigma_min, joint_margin_min_deg, clearance_mm,
validation_level, failure_code, source_type
```

**Invariants（CMake test 强制）**：
- frame_id 严格 +1 递增
- timestamp_ms = frame_id × dt_ms 严格
- status == OK 且 motion_type == PRINT 时 wire_on == 1
- status == OK 且 motion_type ∈ {RETRACT, TRAVEL, APPROACH} 时 wire_on == 0
- transition_id != -1 时 motion_type ∈ {RETRACT, TRAVEL, APPROACH}

**status 枚举**：
```
OK                  ：可执行帧
IK_FAIL             ：解析 IK 无解或 DP 拒绝
COLLISION_FAIL      ：碰撞验证失败
SING_RISK           ：奇异性边际过低
VEL_FAIL / ACC_FAIL ：超速度/加速度
PATH_FAIL           ：路径生成失败
TRANSITION_FAIL     ：transition planner 失败
DISCONNECTED_OK     ：可执行但前置 transition 失败
BROKEN_LAYER        ：层 component 破碎
SKIPPED             ：手动跳过
```

**motion_type 枚举**：
```
PRINT, RETRACT, TRAVEL, APPROACH, DWELL
```

### 6.2 diagnostic.json（diag_v1.0）

顶层结构：
```json
{
  "schema_version": "diag_v1.0",
  "run_id": "bunny_proposed_001",
  "created_at": "2026-05-15T00:00:00",
  "method_id": "proposed_ik",
  "model_id": "bunny",
  "input_mesh": "data/bunny.stl",
  "regularized_mesh": "...",
  "git_commit": "abc123",
  "config": {...},
  "artifacts": {...},
  "summary": {...},
  "penalty_stats": {...},                  // P0-2 新增
  "geometry_regularization": {...},
  "field_metrics": {...},
  "layer_metrics_summary": {...},
  "path_metrics_summary": {...},
  "ik_metrics": {...},
  "collision_metrics": {...},
  "transition_metrics": {...},
  "height_sweep_diagnostic": {...},
  "failure_summary": {...},
  "per_layer": [...]
}
```

详细字段见 `schemas/diagnostic_v1.schema.json`，Week 10 冻结，Week 11-12 所有实验必须通过 schema check。

### 6.3 geometry_regularization（mao 必须）

```json
{
  "enabled": true,
  "source_model_id": "mao_original",
  "regularized_model_id": "mao_regularized",
  "hausdorff_mean_mm": 1.8,
  "hausdorff_p95_mm": 4.6,
  "hausdorff_max_mm": 8.9,
  "normal_deviation_mean_deg": 8.2,
  "normal_deviation_p95_deg": 27.0,
  "volume_change_ratio": 0.06,
  "silhouette_iou_front": 0.81,
  "silhouette_iou_side": 0.84,
  "silhouette_view_angles_deg": {"front": [0,0,0], "side": [0,90,0]}    // Opus 4.2 修正
}
```

---

## 7. 实验设计（修订版）

### 7.1 主对照矩阵（4 methods × 3 models = 12 runs）

| Method ID | 配置 | 说明 |
|---|---|---|
| **M1** v4_pde | v4 §10 PDE 切片 | 作者自己的前作，**同范式 baseline**（用户决定）|
| **M2** dijkstra_geom | α=β=γ=δ=0，纯 distance field | 验证加权 vs 不加权 |
| **M3** dijkstra_mfg | α, β, γ enabled, δ=0 | 验证制造性 bias 但无 IK |
| **M4** proposed_ik | α, β, γ + δ=δ* | v6.2 完整版 |

**注意**：M1 v4 PDE 是用户决定的 baseline。它是 ASN/curved-layer 同范式的方法，但论文叙事需说明：
> "We use our prior PDE-based scalar field approach (v4) as a same-paradigm baseline. It represents a class of curl-free scalar field methods. We do not claim superiority over published methods such as Wang 2018, Li 2022, or Jayakody 2024; capability comparisons against these works are provided in Table 2."

### 7.2 Capability Table（取代 Performance Table 对外部方法的比较）

| Method | Curved layers | Support-free | Collision-aware | IK-aware field | Robot dry-run |
|---|---|---|---|---|---|
| Wang 2018 | reported | reported | heuristic | not reported | not reported |
| Li 2022 | reported | reported | reported | not KR4-specific | not reported |
| Jayakody 2024 | reported | reported | reported | orientation-focused | not reported |
| **Ours** | **evaluated** | **evaluated post hoc** | **SDF+capsule validated** | **pointwise KR4** | **yes** |

**P1-2 修正**：不用 ✓/✗，改用 reported / not reported / evaluated。

### 7.3 主表（论文 Table 1）

| Method | Coverage | Broken Layers | IK Pass | Collision Fail | KRL OK Ratio | Runtime |
|---|---|---|---|---|---|---|
| M1 v4 PDE | ... | ... | ... | ... | ... | ... |
| M2 Dijkstra δ=0 | ... | ... | ... | ... | ... | ... |
| M3 Dijkstra+mfg | ... | ... | ... | ... | ... | ... |
| M4 Proposed | ... | ... | ... | ... | ... | ... |

**KRL OK Ratio = executable_OK_path_length / total_required_path_length**（不是绝对长度）。

mao_regularized 单独一栏：geometry_deviation_from_mao_original（hausdorff_p95, normal_dev_p95, silhouette_iou, volume_change）

### 7.4 Coverage 双重定义

```
layer_coverage_ratio       : MC 层覆盖模型体积/表面比例
printable_coverage_ratio   : wire_on OK path length / total required path length
```

### 7.5 δ 消融（小规模）

仅 bunny + armadillo 上做：
- δ = 0, 0.25, 0.5, 1.0（4 配置 × 2 模型 = 8 extra runs）
- mao_regularized 只跑 δ = δ*（最佳）

### 7.6 α/β/γ 消融（**延后，Week 11-12 视时间决定**）

Spec 留 hook：CLI 支持 `--weights configs/ablation_alpha0.yaml` 等。
若 Week 11 末有 buffer，做 bunny + armadillo 上的 α=0/β=0/γ=0 单因素消融（6 extra runs）。
若无时间，论文 limitations 写：
> "Per-weight ablation (α/β/γ) was not exhaustively conducted due to time constraints; we report joint default values motivated by physical interpretation. Sensitivity analysis is left to future work."

### 7.7 实验时间安排（Week 10-12）

- **Week 9 末**：bunny 跑 M2/M3/M4 smoke matrix（确认 schema 正确，pipeline 不崩）
- **Week 10**：bunny full matrix（4 methods）+ armadillo smoke matrix；锁定 model_transform / voxel / spacing / thresholds
- **Week 11**：armadillo full matrix + mao_regularized smoke + full matrix 预跑；定 mao smoothing 强度（在 Week 11 前固定避免 hyperparameter mining）
- **Week 12**：final frozen matrix + tables + plots + 论文写作

### 7.8 缓存策略

每个模型缓存（model_transform + robot 一致时复用）：
```
mesh_clean.ply, mesh_regularized.ply
voxel_grid.vti, sdf.vti, cavity_radius.vti
p_clear.vti, p_curv.vti
p_ik.vti           # 只按 model_transform + robot + nozzle 算一次
D_0_mm.vti
```

每个 method 重新生成：
```
D_final_mm.vti, D_slice_mm.vti
layers/*.ply, paths/*.polyline
trajectory.csv, reports
```

**关键**：δ 消融时**不重算 p_ik**，只重跑 Pass 4 Dijkstra。否则 δ 扫描会反复花 ~5 分钟算 IK field。

---

## 8. 失败模式与降级

### 8.1 三级交付目标

| 级别 | 内容 | 触发条件 |
|---|---|---|
| **L4 full** | bunny 实机 + armadillo dry-run + mao_regularized full dry-run + IK ablation + FCL | 理想情况 |
| **L3 acceptable** | bunny dry-run + small physical + armadillo dry-run + mao layers + heatmap + KRL | bunny full 实机失败 |
| **L2 emergency** | bunny geometry+IK+KRL + armadillo geometry+IK + mao 仅可视化 + robot dry-run video | mao full dry-run 失败 |

### 8.2 mao_regularized preprocessing gate（P0-10）

mao 进入 full dry-run 前必须满足：
```
pointwise IK success > 95%       (5k surface samples)
layer broken ratio < 5%
path generation success > 95%
cavity high-risk voxel ratio < 10%
support pass rate > 90%
```

不过 gate → 选项：
- 加大 smoothing 强度，重跑 Stage 0
- 调整 model_transform
- 降到 L3 目标（仅 dry-run layer + path，不强求 trajectory 全 OK）

⚠️ **关键**：smoothing 强度在 Week 3-4 内部 smoke test 后固定，**Week 10-12 不再为 IK pass rate 调整**（避免 Opus 2.2 hyperparameter mining 指控）

### 8.3 已识别 15 个 pitfall

| # | Pitfall | 严重 | Fallback |
|---|---|---|---|
| 1 | Pass 2-4 鸡生蛋 | 中 | 两遍即停，不迭代 |
| 2 | p_ik 不连续 | 高 | winsorize + Gaussian smooth + δ 上限 + hard infeasible mask |
| 3 | yaw 12 不够 | 中 | 失败段 12→24 adaptive |
| 4 | Cyclic DP 难调 | 中 | 先 open DP + endpoint repair；Week 7 才上 cyclic |
| 5 | FCL BVH 增量 | 中 | 按 layer rebuild，不 per-path 增量 |
| 6 | Post-quintic FCL 爆炸 | 高 | L1/L2/L3 分层 + stride |
| 7 | mao 语义损失 | 中 | RegularizationReport 量化 |
| 8 | 权重调参 | 高 | 固定 αβγ 默认值，δ 扫描，α/β/γ 消融延后 |
| 9 | bunny 实机失败 | 高 | 阶梯 fallback（full → small → cube → dry-run only） |
| 10 | Baseline 不够 | 中 | M1 v4 PDE + capability table |
| 11 | Dijkstra 内存爆 | 中 | 默认 0.8mm voxel |
| 12 | 坐标系错乱 | 高 | Week 1 frame audit + 文档 |
| 13 | 路径数量爆炸 | 中 | path simplification + top-K |
| 14 | KRL OK 不连续 | 高 | TransitionPlanner 6 keypoint（**P0-7 已修**） |
| 15 | FDM extrusion 未建模 | 低 | 论文写为 geometric/kinematic validation |

---

## 9. 论文叙事

### 9.1 标题

**`Robot-Aware Curved-Layer Slicing with Manufacturability-Weighted Scalar Fields for Six-Axis FDM: A KR4 R600 Pipeline`**

（"Manufacturability-Weighted" 取代 "Clearance-Weighted"，P1-1 修正）

### 9.2 Contributions（4 项）

```
1. A manufacturability-weighted scalar-field formulation that injects a
   KR4-specific pointwise IK margin (including joint-limit, singularity,
   and workspace margins) before curved-layer extraction. Branch
   consistency is incorporated to reduce path-level discontinuity risk.

2. A practical six-axis FDM pipeline that combines geometry
   conditioning, curved-layer extraction, component-aware path generation
   with raster fallback, cyclic IK branch selection with yaw sampling,
   conservative SDF-based collision validation, transition planning with
   safe re-positioning, and KRL-compatible export.

3. A diagnostic framework that reports layer quality, IK feasibility,
   collision risk, transition failures, and height-sweep connectivity
   events, supporting failure mode analysis on geometrically complex
   models.

4. A three-model evaluation including a physical low-speed print on
   bunny, full dry-run feasibility analysis on armadillo, and stress-test
   evaluation on a regularized version of a complex bust model
   (mao_regularized) with quantified geometric loss.
```

### 9.3 禁止的 claim

- ❌ "First IK-aware curved slicer"
- ❌ "Topology-adaptive routing"
- ❌ "All three models successfully printable"
- ❌ "Better than Jayakody 2024 quantitatively"
- ❌ "Hard constraint satisfaction by edge weights"
- ❌ "Topological equivalence after PDE smoothing"
- ❌ "Open-source" (除非真开源)
- ❌ "Support-free guaranteed" (P0-1)
- ❌ "Collision-free guaranteed" (P0-1)

### 9.4 Limitations 章节必写

1. Constraint satisfaction is enforced by post-validation, not by hard constraints in the scalar field. The field provides soft guidance; failure regions are explicitly reported.
2. mao_regularized loses sub-5mm geometric details, quantified by Hausdorff / normal deviation / silhouette IoU / volume change metrics.
3. Collision validation uses a conservative SDF/capsule sampling proxy. Exact mesh-mesh continuous collision detection is left to future work.
4. Path-level IK continuity is enforced via Viterbi DP after the scalar field stage. Pointwise IK margin in the field is a necessary but not sufficient condition for path-level feasibility.
5. Real-time RSI online control is not validated in this work; trajectories are exported as offline KRL.
6. The height-sweep diagnostic provides post-hoc failure explanation, not algorithm routing decisions.
7. Per-weight (α, β, γ) ablation was not exhaustively conducted due to time constraints; default values are motivated by physical interpretation.
8. Baseline comparison uses our prior PDE-based approach. External published methods (Wang 2018, Li 2022, Jayakody 2024) are compared qualitatively via capability table.

---

## 10. 遗留问题（实施期决定）

| ID | 问题 | 默认处理 | 截止 |
|---|---|---|---|
| OPEN-1 | mao_regularized smoothing 强度参数 | Week 3-4 smoke test 固定，不为 IK pass rate 调整 | Week 4 |
| OPEN-2 | δ* 最优值 | Week 10 ablation (δ ∈ {0, 0.25, 0.5, 1.0}) 选 | Week 10 |
| OPEN-3 | KUKA TCP 校准方法 | 4-point 或 6-point，现场决定 | Week 10 |
| OPEN-4 | bunny 实机喷头 r_n 实测 | 现场测后填 nozzle yaml | Week 10 |
| OPEN-5 | 平台坐标系与 KUKA base 关系 | Week 10 frame audit | Week 10 |
| OPEN-6 | FCL 是否成功 build | Week 8 决定，1 天装不通用 SDF proxy | Week 8 |
| OPEN-7 | RSI 在线 demo 是否做 | 视 Week 12 时间，optional | Week 12 |
| OPEN-8 | α/β/γ 消融是否做 | Week 11 末看时间，做 bunny+armadillo 单因素 / 不做写 limitation | Week 11 |
| OPEN-9 | η (p_orient blend) | Week 3 smoke 后定，默认 0.3 | Week 3 |
| OPEN-10 | λ_consist (branch consistency) | Week 4 smoke 后定，默认 0.5 | Week 4 |
| OPEN-11 | model_transform per model | Week 2 手工 yaml + sanity check | Week 2 |
| OPEN-12 | bunny 实机 success criteria | "Hausdorff ≤ 3mm AND no layer separation AND bead overlap ≥ 50%"，Week 9 写死 | Week 9 |

---

## 总结

v6.2 = v6.1 + 17 项 reviewer patch + 用户 3 项最终拍板（路径 Y, v4 PDE baseline, α/β/γ 延后）。

**算法核心**：制造性加权 Dijkstra + pointwise IK 边际 + 后验验证 + transition planner。
**工程范围**：3 CLI + file-based + 复用 v4 ~40% 代码 + 4 新模块（PenaltyNormalizer, FieldSmoothing, PrintedState, TransitionPlanner）。
**12 周可交付**：含 fallback，bunny 实机有阶梯降级，mao 失败有 stress test 叙事。
**论文叙事谨慎**：窄 claim + capability table + 必报 limitations + 不假装超越外部方法。

**新颖性核心**：pointwise IK margin + branch consistency in scalar field construction（窄但可立）。
**工程价值核心**：KR4 R600 完整开源 pipeline（稳，可立）。
**论文叙事核心**：诚实承认 soft bias + post validation；mao 是 manufacturability-regularized benchmark 不是完整重建。
