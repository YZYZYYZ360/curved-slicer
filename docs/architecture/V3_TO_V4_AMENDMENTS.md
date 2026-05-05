# 架构修订：v3 → v4 增补与废止

> 修订日期：2026-04-27（§1-§4），2026-04-27 二次修订（§5）
> 触发原因：见各小节
> 本文件不重写 v3 全文，仅记录相对 v3 的差异。v3 的所有未提及部分仍然有效。

---

## §1 总体定位变化

**v3 设计**：新仓库内同时实现 wavefront 基线算法 与 Laplacian+Poisson 新算法，通过 `[algorithm] mode` 切换，做对照实验。

**v4 修订**：
- 新仓库**仅实现新算法**（Laplacian + Poisson + KUKA 投影 + iso-surface + 路径）。
- 论文对照基线由 `tests/benchmarks/old_project/` 内已有的旧项目可执行体提供（编译旧项目源码，调用 `SFF::getDistanceFiled` + OpenVDB 链路），不在新仓库重复实现。
- 新旧两端各自输出**等值面 PLY + metrics.json**，论文层面做对照即可。

理由：
1. v3 §2.10 设想的"干净 wavefront 基线"在实施时，要么变成对老算法的虚假迁移（实际是 Dijkstra 重写、注释作为映射假象），要么需要逐行复刻老算法的所有补丁——前者欺骗性、后者维护成本高。
2. 老项目源码已可独立编译运行（`tests/benchmarks/old_project/old_project_phase0_benchmark.exe` 已验证），把它升级为完整的"基线产物生成器"成本远低于在新仓库二次开发。
3. 让"基线 = 原作者算法不动"更诚实，论文论述也更清晰。

---

## §2 v3 章节修订清单

| v3 章节 | 修订动作 |
|---|---|
| §2.10 `field/wavefront.h` 接口章节 | **整节废止**。`field/wavefront.h` / `.cpp` 文件已删除，`WavefrontParams` 类型已删除。对应 `src/field/laplacian.h` + `src/field/poisson.h` 仍保留（Phase 2 实现） |
| §3 数据流图末尾"按 mode 分叉"附加说明 | 废止。新仓库不再分叉，只跑 field 路径 |
| §4.1 `[algorithm] mode` 顶层节 | **整节废止**。`config/batch_three_models.toml` 已删除该节 |
| §4.1 `[algorithm.wavefront]` 节 | **整节废止**。`config/batch_three_models.toml` 已删除该节 |
| §4.1 `[algorithm.field.laplacian]` / `[algorithm.field.poisson]` | **保留**。新算法继续使用 |
| §6 映射表中 `SFF::threadFun_first/CurveLayer` → `src/field/wavefront` 一行 | 改为：→ — 废弃（方案 C 基线由旧项目可执行体提供） |
| §6 映射表中 `SFF::getDistanceFiled（纯波前主循环）` → `src/field/wavefront` 一行 | 同上，改为废弃 |
| §6 映射表中 `mc_lookup_table.h` 一行 | **保留**。Phase 2 启动时把 `iso_surface.cpp` 从 tetra-decomposition 切换为表驱动 MC，届时启用 |
| §7 #2 "线程数启发式 在 wavefront 模块保留" | 改为"老项目里保留，新仓库不复用——方案 C 下不需要" |
| §8 Phase 1 整节 | 重新定位（见下文 §3） |
| §9 自检清单中 wavefront 相关项 | 替换为方案 C 相关项（见下文 §4） |
| **§2.2 / §2.7 向量场 Laplacian + Poisson 接口** | **§5 修订**：Phase 2 改为标量 Laplacian 直接求 φ，废弃向量 G 与 Poisson 调用；向量 + Poisson 推迟到 Phase 3 与 KUKA 投影一起实现 |
| **§8 Phase 2 / Phase 3 验收量化** | **§5.5 修订**：M4 口径从"per-layer cc=1"改为"per-model cc=1"；Phase 3 验收对比 Phase 2 标量结果 |

---

## §3 v3 §8 Phase 1 重新定位

**v3 原 Phase 1**：迁移波前算法到 `src/field/wavefront`，作为基线对照。

**v4 修订 Phase 1**：仍是 1 周预算，但任务定位为**通用基础设施 + 命名对齐**。Phase 1 实际产出（已落地）：
- 命名/路径与 v3 §6 对齐（`io/robot_writer`、`metrics/curvature`、`app/pipeline`）
- `geometry/bvh`（最近距离查询，Phase 2 SDF + Phase 2 BC 生成都要用）
- `geometry/voxel_grid` 改 sparse 存储（应对大模型 + 高分辨率）
- `surface/iso_surface` + `mc_lookup_table.h`（Phase 2 把 tetra 替换为 MC，本阶段先用 tetra 跑通框架）
- `field/laplacian.h` + `field/poisson.h` 接口 stub（Phase 2 实现）

**v4 Phase 1 验证标准**（替换原 v3 §8 Phase 1 验证）：
- ctest 全绿（phase0_batch_test、config_loader_unit、eigen_smoke、sdf_smoke 共 4 项；wavefront_smoke 已删除）
- 三模型 spacing=0.5mm 跑通 read + voxelize（耗时 < 10 min，RSS < 8GB）——已实测 213s / 171MB
- 老项目可执行体在 `tests/benchmarks/old_project/` 能编译运行（已验证）
- v3 §9 子目录 ≤ 7、依赖 ≤ 3、headers < 100 行 等约束仍然成立

---

## §4 方案 C 落地任务（Phase 2 启动前）

老项目可执行体当前只输出体素化耗时与体素计数（见 `tests/benchmarks/old_project/old_project_phase0_benchmark.cpp`）。要做"论文基线"，扩展为：

| 输出物 | 现状 | Phase 2 启动前要做 |
|---|---|---|
| ~~ScalarField (φ 场)~~ | ~~老项目内部有，未导出~~ | **取消**：M1/M4/M5/M6 全部基于 iso-surface mesh，不需要 φ 场原值。强行导出会要求暴露 `getDistanceFiled` 内部局部变量 `curvelayer_voxels`，违反"只读老项目"约束。 |
| 各层等值面 STL | 老项目已支持 —— `SFF::getDistanceFiled` 调 `getMeshCurvelayer/getSingleMeshlayer`，二者在 `file_name` 非空时调 `buildLayerStlPath` 自动写盘到 `<file_name>/NNN.stl`（3 位零填充） | 直接消费这一已有机制：调用时传 `file_name = "runs/phase1_old_baseline/<model>"`，跑完后读取所有 `NNN.stl` 即可 |
| metrics.json | 老项目无统一输出 | 在 benchmark cpp 内做后处理：用新项目 `readStl` 加载每张 STL，统计 `face_count` 与 `connected_components`，输出与新项目同 schema：`{"model": ..., "source": "old_project_baseline", "M4": {"face_count": N, "connected_components": K, "layer_count": L}, "M6": {"solver_ms": ms}}` |
| 中层 PLY 对比文件 | 同上 | **取消独立导出**：直接用各层 STL 即可（MeshLab 原生支持）；如果论文真的需要 PLY，Phase 5 写一行 `scripts/stl_to_ply.py` 转换即可，不阻塞当前阶段 |
| 中文路径支持 | 已通过（boost + UTF-8） | 保留 |

新项目侧对应的对照器（Phase 2 末再实现，**不阻塞 Phase 2 算法开发**）：
- `scripts/compare_baselines.py` 读两端 metrics.json，生成对比表
- `runs/comparison/<model>/`：side-by-side STL + 数值表

新增 v4 自检条目：

- [x] 新仓库 src/ 全文不再含 `wavefront`、`WavefrontParams`、`algorithm.mode` 字符串
- [x] `config/batch_three_models.toml` 不再含 `[algorithm.wavefront]`、`[algorithm] mode`
- [x] `tests/config_loader_unit_test.cpp` 不再断言 mode 或 wavefront 字段
- [x] `CMakeLists.txt` 不再编译 `wavefront.cpp` / `wavefront_smoke_test` / `phase1_wavefront_batch_report`
- [ ] ~~老项目 benchmark 调用 `getDistanceFiled` 触发逐层 STL 写盘~~ → **推迟到 Phase 5**（见 v4 §4.1 baseline 备选方案）
- [x] `scripts/compare_baselines.py` 框架已落地（自我对比验证通过；具体基线源 Phase 5 决定）

### §4.1 baseline 备选方案（Phase 5 论文写作期决定）

A1 实施过程中发现 `SFF::getDistanceFiled` 在 standalone benchmark（无 main_ui 上下文）下挂死，所有内部诊断走 `app_log::logf` 默认 `OutputDebugString` 路径，console 不可见，定位预估 1-3 天且可能要求修改老项目。Phase 2 算法主任务**不依赖**baseline，故 A1 推迟。Phase 5 写论文时三选一：

- **(a) 平面切片基线（推荐）**：在已有的 OpenVDB SDF 上按 z 等距切等值面。`tests/benchmarks/old_project/` 现有代码已经把 SDF 拉起来了，只需加 50 行 MC 调用 + 后处理。论文叙事："vs traditional planar slicing"
- **(b) 重启 A1**：花 1 天预算给 Codex 授权"老项目 SFF.cpp 里只加诊断 `printf` 不改逻辑"的有限修改权，定位 getDistanceFiled 卡点
- **(c) GUI 手动**：跑老项目原生 main_ui.exe 三模型，手动收集 STL 输出。土法但有效

**φ 场对比的取消理由**（v4 §4 表中已说明）：M1（max\|H\|）/ M4（面片数 + 连通域）/ M5（层厚 CV）/ M6（耗时）四项全部基于 iso-surface mesh。φ 场只是中间产物，不进论文表格。强行导出会要求修改老项目，得不偿失。

---

## §5 Phase 2 算法路径修订：标量 Laplacian（2026-04-27 二次修订）

### §5.1 决策背景

Codex 实施 v3 §2.2/§2.7 时实测发现严重问题：

在均匀 Dirichlet BC（底面 G = `[0,0,1]`）+ 自由 Neumann 下，向量场 Laplacian `∇²G = 0` 的唯一解是常量 `G = [0,0,1]` 全空间。然后 `∇·G = 0`，Poisson 的 RHS 全 0，解 φ 退化为齐次解（按 anchor 钉到 0），等值面就是水平面切片——和"传统平面切片"无异。

armadillo_flat spacing=0.5mm 实测 layer_count=55，per-layer connected_components 最高 5，正是水平切非凸模型的物理事实。

**根因**：向量场 Laplacian + Poisson 流水线**只有在 KUKA 投影修改 G 之后**才有非平凡解。原 v3 §8 把 KUKA 投影排在 Phase 3 是设计漏洞——没有它 Phase 2 数学上就是恒等变换。

### §5.2 Phase 2 算法重新定位

| | 旧 Phase 2（v3） | 新 Phase 2（v4 §5） |
|---|---|---|
| 解什么 | 向量场 G（Laplacian），再用 G 求 φ（Poisson） | **标量场 φ 直接求**（标量 Laplacian） |
| 方程 | `∇²G = 0`，`∇²φ = ∇·G` | `∇²φ = 0` |
| BC | 底面 G = 单位 print_direction，侧/顶 Neumann | 底面 φ = 0，**顶面 φ = 1**，侧 Neumann |
| 输出维度 | 3 分量向量场 | 1 分量标量场 |
| 模块依赖 | laplacian + poisson | 仅 laplacian |
| 等值面解释 | φ 等值面 = 切片层 | φ 等值面 = 切片层（同前） |

这是 curved-layer slicing 文献中最常见的"engineering-simplified version"实现。无需 KUKA 投影即产出非平凡曲面层（标量 harmonic 场会自然贴合非凸几何，绕开悬垂区域）。

### §5.3 Phase 3 重新定位

**旧 Phase 3**：向 Phase 2 的向量 Laplacian 输出注入 KUKA projection。
**新 Phase 3**：实现完整向量 Laplacian + KUKA projection + Poisson 流水线，作为 Phase 2 标量版本的**升级版**。

论文叙事的好处：Phase 2 (scalar baseline) vs Phase 3 (vector + KUKA) 形成**算法内生对照**。比"老项目 vs 新项目"对比更可控、更说得清楚算法贡献。

### §5.4 v3 接口的具体修订

#### §5.4.1 `field/laplacian.h`（v4）

```cpp
struct BCParams {
    std::string strategy = "bottom_up";  // 仅 bottom_up；其他 throw
    std::array<double, 3> print_direction{0.0, 0.0, 1.0};
    double bottom_dot_threshold = -0.5;
    double bottom_sdf_band = 1.0;
};

struct LaplacianBC {
    std::vector<VoxelIndex> fixed_indices;
    std::vector<double> fixed_values;     // ← 改为标量（v3 是 std::vector<Vec3>）
};

LaplacianBC generateBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params);
//   bottom_up 策略：
//     底面体素 φ = 0（局部表面法向 · print_direction < bottom_dot_threshold，
//                      在 |SDF| < bottom_sdf_band * spacing 的 narrow band 内）
//     顶面体素 φ = 1（局部表面法向 · print_direction > -bottom_dot_threshold，
//                      在 narrow band 内）
//   两端都钉死，避免常解

struct LaplacianParams {
    int max_iterations = 2000;
    double tolerance = 1e-6;
    // normalize_each_iter 字段 v4 删除（标量场不需要）
};

// 返回类型从 VectorField 改为 ScalarField
ScalarField solveLaplacian(const VoxelGrid& grid, const LaplacianBC& bc, const LaplacianParams& params);
//   求解 ∇²φ = 0，在 fixed_indices 上钉 fixed_values（Dirichlet），其余自由 Neumann
```

`VectorField` 类型保留在 laplacian.h 里**作为 Phase 3+ 的占位**——Phase 3 启动时再恢复 `solveLaplacianVector` 函数。Phase 2 只走标量分支。

#### §5.4.2 `field/poisson.h`（v4）

Phase 2 标量 Laplacian 不调用 Poisson。`field/poisson.h` 保留两件事：
1. `struct ScalarField` 定义（laplacian.h 的返回类型也用它）
2. `struct PoissonParams` + `solvePoisson` inline stub，throw `not_implemented`，Phase 3 实现

#### §5.4.3 `surface/iso_surface.h`（不变）

`planIsoLevels` / `extractIsoSurface` 接口和 v3 §2.8 完全一致，因为它们消费 `ScalarField`，标量 Laplacian / 向量 Laplacian + Poisson 哪条路径产出的都接受。

#### §5.4.4 `pipeline.cpp` 流水线（v4 Phase 2）

```
read STL → voxelize → buildSDF → generateBC → solveLaplacian → planIsoLevels →
extractIsoSurface（每层）→ 写 iso_NNN.stl → 计算 metrics → 写 metrics.json
```

相比 v3 设计**移除了**：solvePoisson 调用、向量场 G 中间产物（v3 流水线把 G 灌给 Poisson 的中间步骤）。

`config_loader.h` 里的 `algorithm.field.poisson` 字段保留（toml 仍能解析），但 Phase 2 不读取。Phase 3 再消费。

### §5.5 验收标准修正

#### §5.5.1 M4（连通域）口径修正

旧 M4：`connected_components_per_layer = 1`（**错**：armadillo 等非凸模型水平切就是 5-6 片，物理事实）。

新 M4 双层定义：

| 指标 | 定义 | Phase 2 阈值 | 用途 |
|---|---|---|---|
| **per-model connected_components** | 把所有层的等值面顶点集合作为单一三角网，计算连通域数 | = 模型本身的连通片数（armadillo_flat / bunny / mao 都是 1） | 硬阈值，必须达成 |
| **max(per-layer cc)** | 各层连通域数的最大值 | 仅记录到 metrics.json，不作阈值 | 描述指标，用于论文 |

per-model cc = 1 的物理意义：模型是单连通体，则它的所有等值面应共同构成一个连通的"分层壳"——每相邻两层至少在某个共享顶点连通（实际上是边连通）。如果 per-model cc > 1，意味着某层完全孤立，这是算法异常。

#### §5.5.2 Phase 2 完整验收（v4）

- 三模型 spacing=0.5mm 全跑完不崩
- 总耗时 < 30 min（armadillo 实测 27s/单模型，三模型应在 5-10 min 范围）
- layer_count >= 50（实测 armadillo 55）
- **per-model connected_components = 1**（armadillo_flat / bunny / mao 全部）
- **max(per-layer cc)** 记录到 metrics.json 但不作阈值
- φ 场可视化（debug PLY）合理：从底面 0 平滑过渡到顶面 1，无 NaN/Inf/负值
- ctest 全绿

#### §5.5.3 Phase 3 量化验证（升级 v3 §8 Phase 3）

Phase 3 实现完整向量 Laplacian + KUKA projection + Poisson 后，**与 Phase 2 标量结果对比**：

- M1（max\|H\|）相对 Phase 2 下降 ≥ 30%（向量场 + KUKA 投影应能产生更平滑的层）
- M2（最大悬垂角）严格在 KR 4 R600 可达范围（**所有层不可达点占比 = 0%**，硬阈值）
- per-layer connected_components = 1（KUKA 投影后等值面应该是单片，因为受机器人姿态约束的 G 场会沿连通域方向流动；不再是 Phase 2 的"per-model cc=1"软口径）
- 三模型 spacing=0.5mm Phase 2 已通过的 layer_count / face_count_total 不劣化超 15%

### §5.6 Codex 已落地代码的回滚清单

| 文件 | v3 实现 | v4 §5 处理 |
|---|---|---|
| `src/field/laplacian.h` | LaplacianBC.fixed_vectors (Vec3) + solveLaplacian → VectorField | 改 fixed_values (double) + 返回 ScalarField；删 normalize_each_iter |
| `src/field/laplacian.cpp` | 三分量 Eigen 求解 | 单标量 Eigen 求解，BC 含底面+顶面 |
| `src/field/poisson.h` | inline solvePoisson stub | 保留 stub（throw not_implemented），Phase 3 实现 |
| `src/field/poisson.cpp` | Codex 已实现完整 Poisson | **删除该 .cpp**；poisson 实现 Phase 3 重做 |
| `src/app/pipeline.cpp` | Laplacian → Poisson → 等值面 | 删 Poisson 调用；ScalarField 直接来自 solveLaplacian |
| `tests/laplacian_smoke_test.cpp` | 验证向量 G 场 | 改为验证标量 φ 场（cube：底面 0、顶面 1、中段平滑递增） |
| `tests/poisson_smoke_test.cpp` | 验证 Poisson | **删除**（Phase 3 重写） |
| `src/app/pipeline.cpp` 内 ModelReport.poisson_ms | 已添加字段 | 保留字段（向后兼容），值始终 0 |

---

*v4 修订是局部增量，不替代 v3。v3 是"原始设计"，v4 是"实施中的现实修正"。§5 是相对 v3 的最深修订，因为它涉及算法路径变更，不只是工程妥协。*

---

## §6 KUKA 投影路径重新定位：硬件实情 + MATLAB 委托（2026-04-27 三次修订）

### §6.1 决策背景

Phase 3 启动前用户更正了硬件配置——v3 §2.3 KUKA 投影设计基于错误假设。

**实际硬件**（参照实验室照片 + 用户既有 MATLAB 工程 `C:\Users\26480\Desktop\机器人交接\后端机器人轨迹生成\程序\轨迹生成`）：
- KUKA 6 轴机器人，**工件夹具+打印基板固定在机械臂法兰上**（动工件）
- **挤出喷头固定在世界系上方**（不动喷头）
- 已标定参数：
  - 喷头世界坐标 `PTXYZ = [461.89, 5.64, 706.63] mm`（机器人 base 坐标系）
  - 模型坐标系（model frame）原点在模型底部中心，与法兰坐标系（flange frame）的 Z 偏移 **12.00 mm**
- DH 参数（来自 `mstraj0110.m`）：
  - A1 [-170°, 170°], A2 [-40°, 195°], A3 [-205°, 60°], A4 [-185°, 185°], A5 [-120°, 120°]
  - **注意**：A2/A3 范围与 v3 §2.3 表中的 KR 4 R600 规范有差异——以 MATLAB DH 为准（实际机器人或工件挂载导致的有效范围变化）

v3 §2.3 设计的"逐体素 KUKA 可达性投影"在动工件场景下**不直接适用**：
- 在动喷头场景下："TCP 必须到达 voxel 位置 + TCP +Z 必须等于 G" 是体素级局部约束
- 在动工件场景下：约束是"工件旋转使 voxel 处的 G 与世界 +Z 对齐 + flange 必须把 voxel 移到 PTXYZ"——后者是位置约束，逐体素都不同；joint-space 路径连续性（相邻 voxel 间不能 180° 翻转）才是真正的硬约束
- 完整 6 自由度 IK + joint-space 平滑已由 MATLAB 工程实现（`convert.m` 做 model→base 转换，`mstraj0110.m` 做 trajectory generation with `qlim`）

### §6.2 Phase 3 KUKA-aware 简化

**新策略**：C++ 项目**不做 IK，不做完整 KUKA 投影**。这部分委托给 MATLAB（已经是 working pipeline）。

C++ Phase 3 的"KUKA-aware"简化为两件事：

1. **Hemisphere clamp**：投影 G 场使每点的 G_z > ε（在工件 frame 内），物理意义是"打印方向不能指向工件下方（重力方向）"。这是动工件场景下唯一与 KUKA 几何相关的硬约束。
2. **Smooth G via non-trivial BC**：让 Vector Laplacian 解非平凡：
   - 底面 BC：G = +工件 Z（与平面打印基板对齐，所有底面体素同方向，因为基板是平的）
   - 顶面 BC：G = local outward SDF normal（**与 v3 §2.3 不同**——顶面 BC 用局部曲面法向，所以 G 在不同 voxel 处不同，避免 §5.1 的均匀 BC 退化问题）
   - 侧面：free Neumann

这样 Phase 3 的算法变化**仍然有意义**：
- Phase 2 标量：每层是 harmonic 函数等值面，仅靠 BC 不同（底=0、顶=1）创造非平凡解
- Phase 3 向量：每点 G 是各向不同的方向，Poisson φ 是 G 的可积分代理。**non-uniform BC 是关键**，Hemisphere clamp 是收尾保证可印性

### §6.3 v3 §2.3 接口的具体修订

`field/kuka_projection.h` 的接口变化：

```cpp
// v3 §2.3 原版接口保留 KukaLimits / WorldToBase / isReachable / projectToReachableSet
// v4 §6 不实现 isReachable（详细 IK 委托 MATLAB），projectToReachableSet 简化为 hemisphere clamp

struct ReachabilityParams {
    Vec3 workpiece_up{0.0, 0.0, 1.0};  // 工件本地 +Z 方向（即重力反方向当工件水平时）
    double min_dot_threshold = 0.05;   // G · workpiece_up >= threshold 方为"可印"
                                       // 0.05 ≈ 87° 偏离 workpiece +Z 仍允许，避免严格水平
};

VectorField projectToHemisphere(const VoxelGrid& grid,
                                  const VectorField& G,
                                  const ReachabilityParams& params);
//   对每个 occupied voxel：
//     若 G · workpiece_up < min_dot_threshold:
//       G' = G - (G·workpiece_up - min_dot_threshold) * workpiece_up
//       G' = normalize(G')
//     否则保留 G
//   即"把指向下半球的 G 旋转到 workpiece_up 附近的可印锥内"
```

**注意**：不在 C++ 实现 `isReachable`。joint-limit 检查由 MATLAB 在 IK 阶段做。

### §6.4 Phase 3 接口最终态

```cpp
// field/laplacian.h（保留 Phase 2 标量版本 + 新增向量版本）
struct LaplacianVectorBC {
    std::vector<VoxelIndex> fixed_indices;
    std::vector<Vec3> fixed_vectors;      // 单位向量
};

LaplacianVectorBC generateVectorBC(const VoxelGrid& grid, const SDF& sdf, const BCParams& params);
//   bottom_up 策略：
//     底面体素 G = +print_direction（默认 [0,0,1]，所有底面体素同向）
//     顶面体素 G = local outward SDF normal（用 SDF 梯度算，归一化）
//     ←—— 注意：顶面 G **变化**（每点不同），这是 §6.2 的关键
//   筛选条件同 Phase 2 标量版本

VectorField solveLaplacianVector(const VoxelGrid& grid, const LaplacianVectorBC& bc, const LaplacianParams& params);

// field/kuka_projection.h 见 §6.3

// field/poisson.h（恢复 Phase 2 删除的 .cpp，行为与 v3 §2.7 一致）
ScalarField solvePoisson(const VoxelGrid& grid, const VectorField& G_projected, const PoissonParams& params);
```

### §6.5 Phase 3 流水线（pipeline.cpp 第 3 条分支）

```cpp
if (config.algorithm.pipeline == "vector_kuka") {
    auto bc = generateVectorBC(grid, sdf, config.field_boundary);
    auto G = solveLaplacianVector(grid, bc, config.algorithm.field.laplacian);
    auto G_clamped = projectToHemisphere(grid, G, config.kuka.reachability);
    auto phi = solvePoisson(grid, G_clamped, config.algorithm.field.poisson);
    // 下游 planIsoLevels → MC → metrics 共用
}
```

config 新增：
```toml
[kuka.reachability]
workpiece_up = [0.0, 0.0, 1.0]
min_dot_threshold = 0.05
```

### §6.6 Phase 3 验收标准（v4 §5.5.3 修正版）

|  指标 | Phase 2 标量基线 | Phase 3 向量+hemisphere clamp 目标 |
|---|---|---|
| **per-model cc** | 1（已达成） | 1（保持） |
| **per-layer cc max** | 72 / 29 / 91 | **下降 ≥ 50%**（向量 + 非均匀 BC 的算法贡献） |
| **M1 max\|H\|** | 待计算 | **下降 ≥ 30%** vs Phase 2 |
| **M2 hemisphere 违反点占比** | N/A | **= 0%**（hemisphere clamp 后所有 G 应满足约束） |
| 总耗时 | 9.9 min | < 15 min（多了 Poisson 求解） |
| layer_count / face_count | 56 / 56 / 88 等 | 不劣化超 15% |

**关键**：per-layer cc 不再要求 = 1 硬阈值。armadillo 等非凸模型，4 腿在低 iso 区会自然分多片——这是几何事实，KUKA 投影改不了。Phase 3 要求是"**显著下降**"，证明算法有效。

### §6.7 MATLAB 集成定位

C++ Phase 3 输出**仍然是 STL + metrics.json + φ PLY**（与 Phase 2 同 schema）。MATLAB 集成是 **Phase 4 范围**：

- Phase 4 增加 `path/path_generator` 模块：把每层 IsoMesh + φ 场转成 7 列 `(X, Y, Z, A, B, C, OnOff)` 路径文本
- Phase 4 输出与 MATLAB `readText.m` 兼容的 layer files
- 用户跑 MATLAB 的 `print0409.m` 验证（IK 不报错、joint 在 qlim 内）= Phase 4 验收

这一段在 v3 §2.6 / §2.9 / §8 Phase 4 已有规划，v4 §6 仅明确"不在 C++ 做 IK"的边界。

### §6.8 后续 3+2 轴中心送丝设备适配（暂不考虑）

用户提及实验室未来会切到 3+2 轴中心送丝工艺。**Phase 5 之前不考虑**。届时主要影响：
- Hardware kinematic：3 平移 + 2 旋转，比 6 自由度自由度更少
- 中心送丝：喷头与工件相对位置约束更紧
- 可能需要重写 `kuka_projection.h` → `kinematic_reachability.h`

但这是 Phase 5+ 论文写作完成后的事，**当前架构不为它预留接口**（避免过度设计）。

---

*v4 §6 修订：从"逐体素 6 自由度 IK 投影"简化为"hemisphere clamp + 委托 MATLAB"。本质是把 v3 §2.3 中 60% 的复杂度移到 MATLAB（已实现），保留 C++ 中真正与曲面层算法相关的 40%（hemisphere 投影 + 非均匀 BC）。*

---

## §7 G 场平滑预处理：解决高频几何模型 Poisson 不收敛（2026-04-29 四次修订）

### §7.1 决策背景

D20 mao_rescaled 测试结果（2026-04-29，CLI #2 执行）：

| 模型 | 体素 | Poisson IC-CG 收敛 | 最终 error | 备注 |
|---|---:|---|---|---|
| armadillo_flat | 61K | ✅ | < 1e-6 | 通过 |
| bunny | 170K | ✅ | < 1e-6 | 通过 |
| **mao_rescaled (0.7x)** | **195K** | ❌ **停滞在 ~0.02** | 0.0204 | **关键判别** |
| mao 原版 | 568K | ❌ 跑满 500 迭代 | 不可见 | — |

**关键观察**：mao_rescaled 体素数 (195K) 比 bunny (170K) 多 25K 却不收敛，**纯规模假设破产**（v4 §6 §6.6 已注明的根因 A 不成立）。坐实根因 B：**mao 模型的高频几何特征**（发丝、眉毛、嘴部细节）在 hemisphere clamp 后产生局部 G 不连续，div_G 含高频成分，IC-CG 预条件子对高频残差不起作用。

迭代误差轨迹（mao_rescaled）：
```
iter=50  error=0.344041  ← 起点
iter=100 error=0.10946   ← 下降 68%
iter=150 error=0.0472637 ← 下降 57%
iter=200 error=0.0254715 ← 下降 46%（低频快速消除）
iter=250 error=0.0266632 ← 几乎不动
iter=300 error=0.0240076 ← 几乎不动
iter=350 error=0.0244061 ← 几乎不动
iter=400 error=0.0221289 ← 几乎不动
iter=450 error=0.0224022 ← 几乎不动
iter=500 error=0.0204361 ← 几乎不动（高频卡死）
```

前 200 次清掉低频（每段 -50%~-66%），后 300 次卡在高频残差（每段 -5%）。

### §7.2 解决方案：G_clamped 低通平滑预处理

在 `projectToHemisphere` 之后、`solvePoisson` 之前，插入一步 **G 场低通平滑**：用 6 邻居加权平均把 G 场的高频成分抹掉，让 div_G 变成低频场，IC-CG 预条件子能高效收敛。

**算法**：
```
对每个 occupied voxel：
  if 该 voxel 是 BC 体素（fixed_indices 中的体素）:
    G'(voxel) = G(voxel)                              # BC 体素不平滑
  else:
    sum = G(voxel) * w_center
    count = w_center
    for 6 个邻居 in {±x, ±y, ±z}:
      if neighbor occupied 且 不在 BC:
        sum += G(neighbor) * w_neighbor
        count += w_neighbor
    G'(voxel) = normalize(sum / count)                # 加权平均后再归一化保持单位向量

可重复执行 N 次（N 越大平滑越强）
```

参数：
- `w_center`：中心权重（默认 1.0）
- `w_neighbor`：邻居权重（默认 1.0；调小到 0.5 让中心更受偏向）
- `passes`：平滑次数（默认 1 次）。每次抹掉一个 voxel 尺度的高频；2-3 次足以应对 mao 的发丝细节
- BC 体素不参与平滑——保持顶面/底面 BC 锚定

**为什么有效**：
- div_G 是 G 的一阶导数，G 平滑则 ∇·G 也平滑
- 平滑后的 G 在尖锐特征处的不连续被"晕开"，覆盖到 1-2 个 voxel 范围
- IC-CG 预条件子对低频残差有效，收敛率恢复

**为什么不破坏算法语义**：
- BC（顶/底面方向钉定）保留——平滑只动 BC 之间的体素
- 平滑量 ≤ 1-2 个 voxel = 0.5-1mm 几何精度损失，远小于 layer_thickness（0.8mm）
- hemisphere clamp 已保证 G_z > 0.05，平滑不会让 G 跨越上下半球

### §7.3 接口设计

新增文件：`src/field/field_smoothing.h` + `.cpp`

```cpp
namespace cslc {

struct SmoothingParams {
    int passes = 1;            // 平滑次数；建议 1-3
    double center_weight = 1.0;
    double neighbor_weight = 1.0;
    bool preserve_bc = true;   // BC 体素是否保持（默认是）
};

VectorField smoothVectorField(const VoxelGrid& grid,
                                const VectorField& field,
                                const LaplacianVectorBC& bc,
                                const SmoothingParams& params);

}
```

### §7.4 pipeline.cpp vector_kuka 流水线（更新）

```
read STL → voxelize → buildSDF
        → generateVectorBC
        → solveLaplacianVector
        → projectToHemisphere
        → smoothVectorField              ← 【v4 §7 新增】
        → solvePoisson
        → gaugeShiftPhi
        → planIsoLevels → MC → metrics
```

config 新增节：
```toml
[algorithm.field.smoothing]
passes          = 1
center_weight   = 1.0
neighbor_weight = 1.0
preserve_bc     = true
```

默认 passes=1 兼顾性能与效果。如果 mao 仍不收敛，CLI #1 决策提到 2 或 3。

### §7.5 验收

按以下顺序测试：

1. **mao_rescaled passes=1**：Poisson 应收敛到 < 1e-6（500 迭代内，即至少前 200 次的 66% 下降速率延续到 100% 收敛）
2. 如果 passes=1 仍不收敛（残差 > 0.005）：升级到 passes=2 重测
3. 如果 passes=2 也不收敛：CLI #1 决策升级到 passes=3 或考虑其他方案
4. **passes 确定后跑 mao 原版**：验证规模+几何叠加场景
5. **回归测试 armadillo + bunny**：确认 passes=1 不破坏已收敛的两个模型（M1 / per-model cc / per-layer cc 不劣化超 5%）

### §7.6 论文叙事

§7 增加的 G 平滑步骤可以在论文里写成"low-pass pre-conditioner for Poisson convergence on geometrically high-frequency models"。这是工程优化，**不是核心算法贡献**——核心算法仍是 Laplacian + Hemisphere + Poisson 三步。

---

*v4 §7 修订：在 §6 基础上加一步轻量 G 场平滑（6 邻居加权平均，1-3 次），解决 mao 类高频几何模型的 Poisson 不收敛问题。BC 体素不动，平滑量限制在 1-2 voxel 范围，不损伤算法精度。*

### §7.7 v4 §7 实施结果：方案证伪（2026-04-29）

CLI #2 实施 v4 §7 后，对 mao_rescaled 做 passes ∈ {1, 4, 8, 16} 扫描实验。结果：

| iter | passes=0 (无平滑) | passes=1 | passes=4 | passes=8 | passes=16 |
|---:|---:|---:|---:|---:|---:|
| 50  | 0.34404 | 0.34476 | 0.34595 | 0.34699 | 0.34842 |
| 200 | 0.02547 | 0.02549 | 0.02555 | 0.02562 | 0.02571 |
| 500 | 0.02044 | 0.02046 | 0.02052 | 0.02059 | 0.02069 |

**结论**：G 场平滑对 Poisson 收敛**完全无效**，passes=1 到 16 误差几乎不变（甚至每 doubling 微升 0.1%）。所有 5 组的收敛轨迹形状完全相同——前 200 次迭代清掉低频（每段 -50%~-66%），后 300 次卡在高频残差（每段 -5%）。

**根因重新认定**：问题**不在 G 场高频内容**，**在 Poisson 离散矩阵本身的条件数**。
- 微升原因：smoothing 减小 ||∇·G||，相对残差 ||A·x - b|| / ||b|| 中分母变小，比值反而上升，绝对残差几乎不变
- 矩阵条件数恶劣的来源：稀疏域（mao 195K occupied 在 1.3M 总体素中）+ 单 anchor Dirichlet（仅 1 个体素钉死）+ 自由 Neumann 边界 → IC-CG 预条件子无法处理

**v4 §7 方案废止**。`field_smoothing.{h,cpp}` 模块代码保留（实现正确，未来如果要用做正则化可以复用），但**默认 passes=0**（关闭），pipeline 也加个 if 在 passes=0 时跳过调用。

**剩余路径**：
- v4 §8（下面）：放宽 Poisson tolerance，工程妥协
- 长期路径（Phase 5+）：引入 AMG（Algebraic Multigrid）或 Hypre 作为 Poisson 预条件子，能根治高频残差

---

## §8 Poisson tolerance 务实放宽（2026-04-29 五次修订）

### §8.1 决策背景

v4 §7 证伪后，确认 mao 类高频几何模型在 IC-CG + 单 anchor Dirichlet 下**理论上**无法收敛到 1e-6。剩余两条路：

| 路径 | 工作量 | 收益 | 适合时机 |
|---|---|---|---|
| **A. 引入 AMG 预条件子** | 高（需要 AMGCL / Hypre 第三方依赖 + 较复杂的 setup） | 根治 | Phase 5 论文实验时如有余力再做 |
| **B. 放宽 tolerance**（本节） | 极低（一个 config 数字改动） | 工程可用 | **当前阶段必选**，让 Phase 3 三模型收尾 |

选 B。

### §8.2 数值依据

mao_rescaled 数据：
- IC-CG 在 iter=200 附近达到 error ≈ 0.025
- 之后 error 在 0.020-0.027 之间振荡，几乎不下降
- iter=500 终态 error = 0.0204

新 tolerance 选 **0.03**：
- 略高于当前最优 0.02，给求解器留一点余量保证健壮收敛
- 对应 iter ≈ 200 即可达成（耗时 ~70s 而非 168s）
- 对 φ 场的影响：相对残差 0.03 = 平均误差 ≤ 3%，对等值面位置影响 < 1 个 voxel = 0.5mm，远小于 layer_thickness 0.8mm，不影响后续切片

### §8.3 接口与配置

`PoissonParams.tolerance` 字段不变，仍由 toml 配置。修改：
- `config/batch_three_models.toml` 与 `config/mao_rescaled.toml`：vector_kuka 路径下 `[algorithm.field.poisson] tolerance = 0.03`
- scalar 路径**保持** tolerance=1e-6（标量 Laplacian 用纯 Dirichlet BC，矩阵条件数好，IC-CG 能严格收敛）
- 因此 toml 实际上需要分文件设置，或者通过 pipeline 名字自动选择 tolerance（约定俗成 vector_kuka 用 0.03，scalar 用 1e-6）

简化方案：直接把 toml 的 `tolerance` 字段改成 0.03，scalar 与 vector_kuka 共用（scalar 反正能收敛到 1e-6 也能更早收敛到 0.03，无害）。

### §8.4 Phase 3 验收（最终）

修改 tolerance 后跑：
- mao_rescaled vector_kuka：应在 iter ~200 内收敛
- mao 原版 vector_kuka：可能 iter ~300 收敛（规模更大，但容差宽，应能跑通）
- armadillo + bunny vector_kuka：原本就严格收敛，0.03 容差更宽松，应秒收敛
- 所有指标（per-model cc、M1、M2、layer_count、face_count）回归对照 Phase 2 + 之前 Phase 3 数据，不应有明显劣化

**Phase 3 收尾验收清单**（CLI #1 钦定）：
- [ ] 三模型 vector_kuka 全部跑完不崩
- [ ] mao 原版完整 metrics.json 出炉（含 per-model cc、M1、M2、layer_count、face_count_total）
- [ ] armadillo 与 bunny 在 tolerance=0.03 下跑通且各项指标不劣化超 5%（vs tolerance=1e-6 之前结果）
- [ ] docs/Phase3.md 正式收尾文档落地
- [ ] git commit 拆分清晰（[phase3-D21-smoothing-disable]、[phase3-D22-tolerance-relax]、[phase3-summary]）

### §8.5 论文叙事

§8 在论文中的写法："our IC-CG-based Poisson solver achieves a relative residual ≤ 3% on geometrically complex models with high-frequency surface features (e.g., the Mao bust). Tighter convergence requires multigrid preconditioning, which is left as future work."

这是诚实且可信的限制说明。3% 残差在物理打印中完全可接受（layer thickness 0.8mm × 3% = 0.024mm = 24 微米，远小于喷头分辨率）。

### §8.6 v4 §7 模块的处理

field_smoothing.{h,cpp} 模块代码保留（实现正确）但默认 passes=0 关闭：

- `pipeline.cpp` 在 passes <= 0 时跳过调用 `smoothVectorField`，跳过 stdout 日志，smoothing_ms = 0
- `config/*.toml` 默认 `passes = 0`（v4 §7 实验完后默认关闭）
- 模块仍编译进二进制（CMakeLists 不改），方便后续如有正则化需求复用

---

*v4 §8 修订：v4 §7 G 平滑方案证伪后，工程务实选择放宽 Poisson tolerance 0.03，让三模型 vector_kuka 全部收尾。AMG 升级留给 Phase 5。论文叙事保持诚实，残差物理意义明确。*

### §8.7 v4 §8 实施结果：方案再次证伪（2026-04-29）

CLI #2 实施 tolerance=0.03 后实测 mao_rescaled：

| 指标 | 数值 | 评估 |
|---|---|---|
| Poisson 收敛 | iter=169, error=0.0297221 | ✅ 算"收敛" |
| **phi_min** | **-20.7007** | ❌ |
| phi_max_pre_shift | ~20.8 | ✅ |
| phi_range | 41.5016 | ✅ 量级合理 |
| **负值漂移** | **49.9% > 10% 闸门** | ❌ D10' 不通过 |

**关键发现**：tolerance=0.03 的"收敛"是**假收敛**。CG 残差 0.03 在条件数差的 Poisson 矩阵上对应 solution error 量级 ~10mm（条件数 κ ≈ 10³ × tolerance × ||b|| 量级）。phi anchor 仍 = 0（hard-coded 在矩阵 row 里强制），但其他地方 φ 跌到 -20mm，等值面会从"伪深井"区域抽出无物理意义的层。

**v4 §8 整节废止**。tolerance 放宽不是工程妥协，是数据上行不通的方案。

**根因升级到第三层**：
- 第一层（v4 §6 §6.6）：以为是规模问题 → mao_rescaled 195K 也不收敛，证伪
- 第二层（v4 §7）：以为是 G 高频问题 → 平滑 16 pass 无效果，证伪
- **第三层（v4 §9 下面）**：是 Poisson **离散系统的条件数**——单 Dirichlet anchor + 自由 Neumann 边界在大型稀疏域上条件数 κ ~ N（系统规模），IC-CG 预条件子无法处理

---

## §9 多 anchor Dirichlet 重构 Poisson 边界条件（2026-04-29 六次修订）

### §9.1 决策背景

v4 §8 证伪后，确认 Poisson 矩阵条件数本身是瓶颈。三个 plan B 候选：

| 方案 | 工作量 | 收益 | 风险 |
|---|---|---|---|
| **A. 多 anchor Dirichlet（本节）** | 中（API 改动） | 条件数下降 ~N 倍，CG 严格收敛 | 微弱过约束，但语义自洽 |
| B. AMG 多重网格 | 高（第三方依赖 + setup） | 根治 | Phase 5 时机更合适 |
| C. mao 用 scalar pipeline shipping | 极低 | 工程出货 | mao 失去 vector_kuka 算法演示 |

选 A。

### §9.2 算法依据

当前 Poisson 离散：
```
矩阵 A: 195K x 195K, 7-point Laplacian, 单 anchor row = identity
RHS:    b = -∫∇·G_clamped, anchor row = 0
```
条件数 κ(A) ~ N （证明：单 Dirichlet 的 Poisson 在 N 维有效自由度下，最小特征值 ~ 1/N²，最大特征值 ~ O(1)）。

改为多 anchor：
```
矩阵 A': anchor 集合 D（数千体素）每行 = identity，其他行不变
RHS:     b' = -∫∇·G_clamped, D 中每行 = 0
```
条件数 κ(A') ~ N / |D|（证明：|D| 个 Dirichlet 把有效自由度降到 N - |D|，最小特征值上升到 1/(N-|D|)² 量级）。
对 mao_rescaled：|D| ≈ 1500 底面体素，κ 从 ~195K 降到 ~130，**降 1500 倍**。

### §9.3 anchor 集合的来源

`generateVectorBC` 已经标记了底面体素（`fixed_vectors[i] == +print_direction`，即统一指向 +Z 的那一批）。直接复用：

```cpp
// pipeline.cpp vector_kuka 分支：
LaplacianVectorBC bc = generateVectorBC(grid, sdf, config.field_boundary);
// ... Laplacian + clamp ...

// 提取底面 BC 集合给 Poisson 作 Dirichlet anchor
PoissonParams pp = config.algorithm.field.poisson;
const Vec3 print_dir = printDirection(config.field_boundary);
for (std::size_t i = 0; i < bc.fixed_indices.size(); ++i) {
    // 底面 voxel 的 fixed_vector == print_direction（容差 1e-6）
    if (norm(bc.fixed_vectors[i] - print_dir) < 1e-6) {
        pp.anchor_voxels.push_back(bc.fixed_indices[i]);
        pp.anchor_values.push_back(0.0);
    }
}
// 顶面 BC voxel **不**作 Poisson anchor，只在 Laplacian 阶段约束 G

ScalarField phi = solvePoisson(grid, smoothed, pp);
```

### §9.4 接口改动

`PoissonParams`（src/field/poisson.h）：

```cpp
struct PoissonParams {
    int max_iterations = 500;
    double tolerance = 1e-6;          // ← 恢复严格容差（多 anchor 后矩阵条件数大幅改善）
    bool use_precondition = true;
    
    // v4 §9：多 anchor Dirichlet 集合
    std::vector<VoxelIndex> anchor_voxels;     // 替代旧的单字段 anchor_voxel
    std::vector<double>     anchor_values;     // 与 anchor_voxels 一一对应（通常全 0）
    
    // 兼容字段（已弃用，仅在 anchor_voxels 为空时回退）
    VoxelIndex anchor_voxel{0, 0, 0};
    
    bool log_iterations = false;
    std::string progress_label;
    int* out_iterations = nullptr;
    double* out_error = nullptr;
};
```

`solvePoisson`（src/field/poisson.cpp）：
```cpp
// 构造 anchor 行集合
std::unordered_map<int, double> anchor_rows;  // row -> phi value
if (!params.anchor_voxels.empty()) {
    for (std::size_t i = 0; i < params.anchor_voxels.size(); ++i) {
        const std::size_t key = grid.index(...);
        if (auto it = rows.find(key); it != rows.end()) {
            anchor_rows[it->second] = params.anchor_values[i];
        }
    }
} else {
    // 回退到单 anchor 模式
    const std::size_t key = grid.index(params.anchor_voxel...);
    if (auto it = rows.find(key); it != rows.end()) {
        anchor_rows[it->second] = 0.0;
    }
}

// 装配矩阵：anchor_rows 中的 row 用 identity；其他用 7-point Laplacian
for (int row = 0; row < n; ++row) {
    if (auto it = anchor_rows.find(row); it != anchor_rows.end()) {
        triplets.emplace_back(row, row, 1.0);
        rhs[row] = it->second;
        continue;
    }
    // ... 7-point Laplacian + RHS = -∫∇·G ...
    // 注意：邻居是 anchor 时，rhs 累加 -coefficient * anchor_value（已经被移到右侧）
}
```

### §9.5 验收标准

修改后跑：
- mao_rescaled vector_kuka tolerance=1e-6（恢复严格）：iter ≤ 200 严格收敛，phi_min ≥ -1e-3，gauge 三闸门全过
- mao 原版 vector_kuka tolerance=1e-6：iter ≤ 400 严格收敛，gauge 三闸门全过
- armadillo + bunny vector_kuka tolerance=1e-6：秒收敛（< 50 iter），指标不劣化超 5%
- scalar 路径不受影响（标量 Laplacian 本来就用多 Dirichlet）

如果 mao 原版仍不收敛：CLI #1 做 plan B 决策（AMG 第三方库引入 / mao 单独 spacing=1.0mm / mao 用 scalar 路径 shipping）。

### §9.6 论文叙事

§9 在论文中可以不单独提（属于工程实现细节），或者在 implementation 章节用一句话："we anchor the entire bottom boundary as Dirichlet rather than a single voxel, dramatically improving conditioning of the Poisson system."

---

*v4 §9 修订：彻底改造 Poisson 边界条件，从单 anchor 升级到底面整体 Dirichlet。条件数从 ~N 降到 ~N/|D|（约 1500 倍），让 IC-CG 在严格容差下收敛。这是 §6→§7→§8 三轮证伪之后的真正解。*
