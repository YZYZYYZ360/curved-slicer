# v4 -> v6 Interface Audit

## 1. 概述

Week 1 的复用策略是：v4 baseline M1 保持可运行、可复现实验，不在原文件上改算法行为。
v6 主路径只通过新增 wrapper 或新模块接入可复用能力；v4 PDE 主链路保留为 baseline，不进入 v6 weighted-Dijkstra 主链路。
对 v4 已有但接口不足的模块，v6 在相邻目录新增 wrapper，避免破坏 v4 trace 与回归测试。
对 placeholder 级输出模块，v6 直接新写 writer，同时把 v4 placeholder 留作历史 baseline。

## 2. 模块清单

| v4 模块 | LOC | 标签 | v6 使用位置（stage / week / wrapper 文件） | 备注 |
|---|---:|---|---|---|
| `src/core/types.h` | ~113 | 【直接复用】 | Stage 1-6 基础类型 | `Vec3` / `Tri` / `AABB` 继续作为 legacy mesh/field 类型。 |
| `src/io/stl_reader.{h,cpp}` | ~184 | 【直接复用】 | Stage 1 mesh input | STL 读入可继续复用；mesh repair 仍需 Week 2 新增。 |
| `src/io/config_loader.{h,cpp}` | ~516 | 【baseline 冻结，v6 不调用】 | v6 CLI 暂不接 v4 TOML loader | v4 pipeline config 保留；v6 后续按 CLI/yaml 入口另行设计。 |
| `src/io/robot_writer.{h,cpp}` | ~45 | 【废弃，v6 重写】 | Stage 6 / W1-T4 `src/io/KRLWriter.*` | 现有实现只是 Phase0 placeholder 注释，不是真 KRL writer。 |
| `src/geometry/voxel_grid.{h,cpp}` | ~363 | 【直接复用】 | Stage 2 voxel/SDF | 基础 voxel grid 可复用；weighted graph/Dijkstra 不在 v4 中。 |
| `src/geometry/bvh.{h,cpp}` | ~235 | 【直接复用】 | Stage 2 SDF / Stage 6 validation 起点 | 三角距离查询基础可复用；swept collision 仍需新模块。 |
| `src/geometry/sdf.{h,cpp}` | ~178 | 【直接复用】 | Stage 2 SDF | 可生成初始 SDF；v6 仍需梯度/符号质量审查与后续 collision sampling。 |
| `src/surface/iso_surface.{h,cpp}` + `mc_lookup_table.h` | ~1522 | 【直接复用】 | Stage 3/4 layer extraction 候选 | Marching Cubes 能复用；必须遵守 R5，不把 `D_norm` 传入 MC。 |
| `src/metrics/curvature.{h,cpp}` | ~77 | 【直接复用】 | Stage 3 penalty/diagnostics 候选 | 可作曲率诊断起点；v6 penalty 需要更明确的 surface/path curvature 定义。 |
| `src/field/laplacian.{h,cpp}` | ~527 | 【baseline 冻结，v6 不调用】 | v6 主路径不用，M1 实验保留 | 红线 R1；v4 PDE baseline 的 Laplacian 核心。 |
| `src/field/poisson.{h,cpp}` | ~255 | 【baseline 冻结，v6 不调用】 | v6 主路径不用，M1 实验保留 | 红线 R1；v4 PDE baseline 的 Poisson 核心。 |
| `src/field/kuka_projection.{h,cpp}` | ~70 | 【baseline 冻结，v6 不调用】 | v6 主路径不用，M1 实验保留 | 红线 R1；v6 IK feasibility 改用 W1-T2/W4 路线。 |
| `src/field/field_smoothing.{h,cpp}` | ~93 | 【baseline 冻结，v6 不调用】 | v6 主路径不用，M1 实验保留 | 红线 R1；不得为 pass rate retune smoothing。 |
| `src/kinematics/cart_pose.h` + `dh_params.h` | ~69 | 【直接复用】 | Stage 2/5 / W1-T2 `KR4Model` | DH 参数与 pose structs 是 KR4 wrapper 的基础契约。 |
| `src/kinematics/forward_kin.{h,cpp}` | ~104 | 【直接复用】 | Stage 2/5 / W1-T2 `KR4Model::forwardKinematics` | v4 FK 不改；wrapper 统一 frame contract: `kuka.world_to_base`。 |
| `src/kinematics/ik_solver.{h,cpp}` | ~292 | 【新增 wrapper】 | Stage 2 IK field + Stage 5 DP / W1-T2 `AnalyticIK` | v4 first-valid solver保留；v6 新增 8-branch enumerator。M1 exact recovery 498/1000 是 baseline 特征。 |
| `src/kinematics/reachability.{h,cpp}` | ~185 | 【baseline 冻结，v6 不调用】 | v6 主路径不用，M1 实验保留 | v6 后续用 IK margin/branch volatility，不复用 v4 greedy reachability filter。 |
| `src/path/geodesic_paths.{h,cpp}` | ~313 | 【新增 wrapper】 | Stage 4 / W1-T3 `GeodesicContourPath` | v4 public API 只生成 geodesic field contours；v6 wrapper expose 任意 scalar field -> stitched contours。 |
| `src/path/path_postprocessing.{h,cpp}` | ~179 | 【直接复用】 | Stage 4 path cleanup 候选 | 平滑/后处理函数可复用；v6 broken component policy 仍需新逻辑。 |
| `src/path/pose_from_path.{h,cpp}` | ~54 | 【直接复用】 | Stage 5 path pose construction | Stage 5 才用 path tangent 构造 tool frame；Stage 2 IK field 不用此 pose 逻辑。 |
| `src/trajectory/poly5_smoother.{h,cpp}` | ~151 | 【新增 wrapper】 | Stage 5d / W1-T4 `TimeParameterization` | v4 行为是 every-waypoint zero velocity；v6 ENDPOINTS_ONLY_ZERO 到 W6 实现。 |
| `src/app/pipeline.{h,cpp}` | ~1622 | 【baseline 冻结，v6 不调用】 | M1 baseline runner only | 内含 v4 9-col `trajectory.csv` writer，已被 `v4_pipeline_e2e_test` 锁死；v6 CLI 不改它。 |

Summary:

- 【直接复用】11 个模块
- 【新增 wrapper】3 个模块
- 【baseline 冻结，v6 不调用】7 个模块
- 【废弃，v6 重写】1 个模块

## 3. v6 stage -> v4 模块反向映射

| v6 stage | v4 / W1 模块映射 | 说明 |
|---|---|---|
| Stage 1 (mesh 处理) | `io/stl_reader`, `core/types`; Week 2 新增 mesh repair | STL 读入可复用；repair/watertight checks 不是 v4 能力。 |
| Stage 2 (voxel/SDF/距离场) | `geometry/voxel_grid`, `geometry/bvh`, `geometry/sdf`; `KR4Model`, `AnalyticIK`, `Jacobian` | 基础 voxel/SDF 复用；multi-constraint Dijkstra 和 IK feasibility field 是 v6 新代码。 |
| Stage 3 (penalty + Dijkstra) | `metrics/curvature` 候选；`surface/iso_surface` 候选；field PDE 模块不调用 | v4 PDE 不进入 v6 主路径；penalty graph、normal/clearance/IK terms 需要新实现。 |
| Stage 4 (path generation) | W1-T3 `GeodesicContourPath`; v4 `geodesic_paths` 只作算法参考/基线 | v6 输入是 `D_slice` 等任意 scalar field，wrapper 已 expose `field -> contour`。 |
| Stage 5 (IK + time param) | W1-T2 `KR4Model` / `AnalyticIK` / `Jacobian`; `pose_from_path`; W1-T4 `TimeParameterization` | IK 需要 all-branch data；time param 当前只复现 M1 all-zero mode，ENDPOINTS_ONLY_ZERO 到 W6。 |
| Stage 6 (export) | W1-T4 `TrajectoryCSVWriter` / `KRLWriter`; v4 `robot_writer` 不复用 | v6 输出 34-col `traj_v1.0` + minimal KRL；v4 9-col CSV 留在 baseline pipeline。 |

## 4. 风险与 gap

- Mesh repair / mesh validation：v4 只有 STL reader，没有 manifold repair、hole filling、unit normalization gate；Week 2 需要补。
- Weighted Dijkstra field：v4 PDE 模块不能表达 clearance/normal/curvature/IK penalty graph；v6 Stage 2-3 必须新写。
- IK baseline limitation：v4 IK M1 probe 记录为 498/1000 exact recovery；W1-T2 enumerator 已到 1000/1000 exact recovery、812/1000 cases with >=4 valid branches。
- Geodesic contour接口：v4 `generateGeodesicPaths()` 不接受任意 scalar field；W1-T3 wrapper 已补 `field -> iso contours`，但 non-manifold/tangent contours 仍需后续 policy。
- Time parameterization：v4 every-waypoint zero velocity 是 M1 特征；v6 红线 R8 要 endpoints-only zero，W1-T4 仅预留并抛错，W6 必须实现。
- Export：v4 `robot_writer` 是 placeholder，v6 KRL writer 目前是 minimal `.src`；W8 后仍需 status 强制过滤、transition-aware export、`.dat`/controller syntax review。
- Validation/collision：v4 `bvh`/`sdf` 是基础几何工具，不等价于 swept nozzle collision；Stage 6 validation 仍需新模块。
- Config boundary：v4 TOML config loader 支撑 baseline pipeline；v6 CLI/yaml schema 尚未冻结，后续不要把 v4 config 误当 v6 contract。
