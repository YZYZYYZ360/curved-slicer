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

---

## §10 BC 选择重构：几何 z 坐标取代 SDF normal（2026-05-07 七次修订）

### §10.1 问题：mao 等值面碎裂的根因

§9 多 anchor Dirichlet 让 Poisson 在严格容差下收敛，但 mao 模型 iso_045 仍然碎裂成多个不连通片段。`runs/phase3_mao` 体素抽样后发现：BC 选择把**下颌底、鼻底、耳底**等"下向曲面"全部判为底面，phi=0 钉死了多个独立区域 → 多个零势能盆 → 等值面在中层撕裂成碎片。

碎裂根因不是求解器，而是 BC 选择算法。`generateVectorBC` 用 SDF gradient 与 print_direction 点积判断（`bottom_dot_threshold=-0.5`），凡是法向朝下的体素都钉成 phi=0。这对 bunny / armadillo 影响不大（它们底部是单一平面），但对 mao 这种有多个下向曲面的人像几何就崩了。

### §10.2 方案：基于绝对 z 坐标

由用户**预先把模型 z=0 对齐工作平台底面**（这是 KUKA 打印的物理前提，不是 v4 新增约束），然后：

- **底面 BC**: `voxel.z_world ∈ [0, bottom_band_mm]` → phi = 0（Dirichlet）
- **顶面 BC**: `voxel.z_world ∈ [z_max - top_band_mm, z_max]` → phi = 1（Dirichlet，**取代 §9 顶面 free 方案**，2026-05-10 拍板）
  - 选 Dirichlet 而不是 Neumann free 的理由：用户目标是"层厚尽可能均匀"。Neumann free 让顶面 phi 自由，φ 场梯度在不同位置不一致 → 等值面物理间距不均；Dirichlet 强制 φ ∈ [0, 1] → 等值面间距更均匀，再配合 `iso_spacing = "uniform"`（按测地距离自适应选 phi 值），层厚误差可压到 ±5%
- **bottom_band_mm / top_band_mm 自动推算**：`bbox.z * 5%`，clamp 到 `[0.5, 5.0]` mm
  - 例：mao bbox.z = 70mm → band = 3.5mm
  - 例：mao_rescaled bbox.z = 49mm → band = 2.45mm
  - 这避免用户手填一个不合规模的固定值

### §10.3 实现

```cpp
// src/field/laplacian.cpp generateVectorBC 重写
// 旧逻辑（删除）：
// for each voxel: if dot(grad_sdf, print_direction) < bottom_dot_threshold → bottom
// 新逻辑：
double band_mm = std::clamp(grid.bbox_size.z() * 0.05, 0.5, 5.0);
for (auto& voxel : grid.active_voxels) {
    double z_world = grid.toWorld(voxel.idx).z();
    if (z_world < band_mm) bc.bottom_indices.push_back(voxel.linear_idx);
    if (z_world > grid.bbox.zmax - band_mm) bc.top_indices.push_back(voxel.linear_idx);
}
```

### §10.4 配置变更（向后兼容）

`config/*.toml` 里 `[field.boundary]` 新增 `strategy` 字段（之前只有隐式行为）：

```toml
[field.boundary]
strategy             = "geometric_z"     # 新默认，§10
# strategy           = "sdf_normal"      # 旧行为，仅向后兼容用
print_direction      = [0.0, 0.0, 1.0]
# bottom_band_mm / top_band_mm 留空 → 按 bbox.z * 5% 自动推算
```

### §10.5 验收

- mao 原版 vector_kuka tolerance=1e-6：iso_045 单一连通片，不再碎裂；per-layer cc=1 且 max-per-layer cc 也 = 1
- mao_rescaled / armadillo / bunny：与 §9 结果不劣化超 5%（这些模型底部规整，sdf_normal 和 geometric_z 应当给出几乎一样的 BC）
- visualizations 必须输出（`bottom_band.ply`, `top_band.ply`, `iso_NNN.stl`）并人眼审过 mao 的 iso_005/045/095 三层

### §10.6 工程量

~80 LOC 改动 + 配置项加 1。0.5 天。

---

## §11 笛卡尔位姿构造 + 解析 6 轴 IK 模块（2026-05-07 七次修订）

### §11.1 触发：MATLAB 链路彻底删除

v4 §6 把 IK 委托给 MATLAB 脚本，C++ 只输出笛卡尔位姿。本次修订**彻底取消 MATLAB 依赖**，所有 IK 在 C++ 内闭环。理由：
1. MATLAB 不在生产部署链路（KUKA 现场没装），原方案是脱离实际的
2. 跨语言桥接（mat 文件读写）是无谓复杂度
3. KR4 R600 是球腕，解析 IK 闭式解只需 ~300 LOC，工程上比维护 MATLAB 桥便宜

### §11.2 笛卡尔位姿构造

输入：曲面路径点 `p ∈ R^3` + 路径切线 `t ∈ R^3`（来自 §13 等值线）+ G 场方向 `g ∈ R^3`（来自 §7 平滑后 G 场，每点的工件竖直方向）。

构造工具坐标系（喷头朝向工件、沿路径切线移动）：
```
tool_z = -g                                  // 喷头沿 G 反向（指向工件内部）
tool_x = normalize(t - dot(t, tool_z)*tool_z) // 路径切线投影到 ⊥tool_z 平面
tool_y = cross(tool_z, tool_x)                // 正交补全
R_tool = [tool_x | tool_y | tool_z]           // 3x3 旋转矩阵
(A, B, C) = ZYX_Euler(R_tool)                 // 转 KUKA RPY 欧拉角
```

输出：`CartPose { X, Y, Z, A, B, C }`（KUKA 约定：X/Y/Z mm, A/B/C deg）。

**路径切线方向约定（2026-05-10 拍板）**：等值线 polyline 的方向（顺/逆时针）由 §13 决定后，§11 检查 `tool_y = tool_z × tool_x` 是否指向工件**外侧**（即 dot(tool_y, outward_normal) > 0）。如果指向内侧，反转切线方向重算。理由：避免喷头侧面碰到已打印部分。

### §11.3 KR4 R600 DH 参数（来自 `mstraj0110.m`）

| 关节 | a (mm) | α (rad) | d (mm) | θ offset | qlim (deg)      | speed (deg/s) |
|------|--------|---------|--------|----------|-----------------|---------------|
| L1   | 0      | π/2     | 330    | 0        | [-170, 170]     | 336           |
| L2   | 290    | π       | 0      | 0        | [-195, 40]      | 336           |
| L3   | 20     | -π/2    | 0      | 0        | [-115, 150]     | 488           |
| L4   | 0      | π/2     | 310    | 0        | [-185, 185]     | 600           |
| L5   | 0      | -π/2    | 0      | 0        | [-120, 120]     | 529           |
| L6   | 0      | 0       | flange + 12 | 0   | [-350, 350]     | 800           |

法兰 z 偏移 12mm（来自 `mstraj0110.m` 的 `Tz(0.012)`），喷头 TCP 在法兰再加用户标定值（默认 0.28mm，来自 `[kuka.robot] z_offset = 0.28`）。

### §11.4 解析 IK 算法（球腕闭式解）

```
1. wrist_center = TCP_position - (d6 + tool_offset) * tool_z   // 球腕中心 = TCP 沿 tool_z 反推
2. A1 = atan2(wc.y, wc.x)                                      // 第一轴解 (主臂态: front/back)
3. r = sqrt(wc.x² + wc.y²) - a3                                // 平面内径向距离
4. s = wc.z - d1                                                // 平面内高度
5. cos(A3) = (r² + s² - a2² - d4²) / (2 * a2 * d4)             // 余弦定理
   A3 = ±acos(...)                                              // 主臂态: elbow up/down
6. A2 = atan2(s, r) - atan2(d4*sin(A3), a2 + d4*cos(A3))
7. R0_3 = forwardKin(A1, A2, A3).rotation                      // 前 3 轴累积旋转
8. R3_6 = R0_3.transpose() * R_tool                            // 球腕需要的相对旋转
9. (A4, A5, A6) = ZYZ_Euler(R3_6)                              // 球腕 ZYZ 解（A5 ≥ 0 / A5 ≤ 0 → 腕翻转两解）
```

**多解处理**：理论上有 2(肩) × 2(肘) × 2(腕) = 8 解。挑选规则：
- 排除越限（关节超 qlim）
- 排除奇异邻域（|A5| < 5°）
- 在剩余解中选**与上一帧关节角加权 L2 距离最小**的（保证轨迹连续）
- 第一帧用 home 关节角（§15.5）作为参考

### §11.5 接口

```cpp
// src/kinematics/dh_params.h
struct KR4DHParams {
    std::array<double, 6> a    = {0,    290,  20,    0,    0, 0};
    std::array<double, 6> alpha= {M_PI/2, M_PI, -M_PI/2, M_PI/2, -M_PI/2, 0};
    std::array<double, 6> d    = {330,  0,    0,    310,  0, 0};
    double flange_z   = 12.0;
    double tool_z     = 0.28;
    std::array<std::array<double, 2>, 6> qlim_deg{{
        {-170, 170}, {-195, 40}, {-115, 150}, {-185, 185}, {-120, 120}, {-350, 350}}};
};

// src/kinematics/ik_solver.h
struct CartPose { double X, Y, Z, A, B, C; };  // KUKA 约定 mm + deg ZYX
struct JointConfig { std::array<double, 6> q_deg; };

enum class IKStatus { OK, OutOfWorkspace, JointLimit, NearSingularity };

struct IKResult {
    IKStatus status;
    JointConfig solution;
    std::array<JointConfig, 8> all_solutions;  // 调试用
};

IKResult solveAnalyticalIK(
    const CartPose& target,
    const JointConfig& reference,         // 上一帧或 home
    const KR4DHParams& dh);

CartPose forwardKin(const JointConfig& q, const KR4DHParams& dh);
```

### §11.6 验收

- 单元测试：`testIKRoundTrip` —— 随机生成 1000 组关节角 → forwardKin → solveAnalyticalIK → 解的关节角差 < 1e-6 deg
- 与 KUKA 示教器实测：现场任选 5 个示教点位姿，C++ IK 解的关节角与示教器读数差 < 0.05°
- 与 home 位姿对照：home `(A2, A3, A5) = (9.95, -62.26, -37.69)` 对应 forwardKin 给出的 TCP 位姿，与示教器 RIst 实测差 < 0.5mm + 0.05°

### §11.7 工程量

- DH 参数 + 正向运动学：~80 LOC
- 解析 IK + 多解选择：~250 LOC
- 单元测试：~150 LOC
- **总 ~480 LOC，4 天**

---

## §12 双层可达性 + 奇异性过滤（2026-05-07 七次修订）

### §12.1 双层结构

| 层 | 数据对象 | 时机 | 用途 |
|----|---------|------|------|
| Position B | G 场体素（每个体素一个 G 方向）| §11 IK 之前 | 提早剔除"该体素附近无可行姿态"，让 §13 等值线生成跳过 |
| Position C | 路径采样点（来自 §13 等值线）| §13 之后、§14 平滑之前 | 精确判断每个 TCP 点位是否可达 |

B 层是粗剔除（按体素，~1 个 IK 每体素），C 层是精剔除（按路径点，密度高得多但已被 B 层过滤剩下的部分）。

### §12.2 检查项

每次调用 `checkReachability(pose, reference)`：
1. **工作空间**：`||TCP|| ∈ [r_min, r_max]`，r_min/r_max 由 KR4 R600 标定（默认 r_min = 100mm, r_max = 600mm）
2. **关节限位**：`solveAnalyticalIK` 返回的所有解都越限 → 不可达
3. **奇异性**：
   - 腕奇异: `|A5| < 5°`
   - 肩奇异: `|A2 + A3| < 5°` 或 `|A2 + A3 - 180°| < 5°`（手腕直接位于 A1 旋转轴线）
4. **臂态突变**：与 reference 比较，主臂态（front/back, elbow up/down）发生翻转 → 标记为不连续点

### §12.3 整层放弃阈值

- 单层路径点不可达率 > 30% → 整层标记 SKIPPED，不参与后续 §14
- 单层路径点不可达率 ∈ [10%, 30%] → 警告，可视化高亮，由用户判定
- 单层路径点不可达率 < 10% → 局部跳过这些点，断点连续段独立平滑

### §12.4 接口

```cpp
// src/kinematics/reachability.h
struct ReachabilityParams {
    double workspace_r_min_mm = 100.0;
    double workspace_r_max_mm = 600.0;
    double singularity_a5_deg = 5.0;
    double singularity_shoulder_deg = 5.0;
    double layer_skip_threshold = 0.30;       // 整层放弃阈值
    double layer_warn_threshold = 0.10;       // 警告阈值
};

struct ReachabilityResult {
    IKStatus status;
    JointConfig solution;        // 仅当 status == OK
    bool arm_status_flip;        // 臂态翻转标记
};

ReachabilityResult checkReachability(
    const CartPose& pose,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params);

// Position B 层入口（体素级）
std::vector<bool> filterReachableVoxels(
    const VoxelGrid& grid,
    const VectorField& G,
    const KR4DHParams& dh,
    const ReachabilityParams& params);

// Position C 层入口（路径点级）
struct PathReachabilityResult {
    std::vector<JointConfig> joint_solutions;  // size = path.size, 不可达点为 nullopt
    std::vector<int> unreachable_indices;
    bool layer_skipped;
};
PathReachabilityResult filterReachablePathPoints(
    const PathPolyline& path,
    const std::vector<CartPose>& poses,
    const JointConfig& reference,
    const KR4DHParams& dh,
    const ReachabilityParams& params);
```

### §12.5 验收

- mao 模型：可达体素 ≥ 80%（B 层），可达路径点 ≥ 90%（C 层）
- armadillo / bunny：≥ 95%（B + C 层）
- 没有任何一层被整层放弃
- 臂态翻转点数 < 10% 路径总点数

### §12.6 工程量

- B 层（体素级）：~120 LOC
- C 层（路径点级）：~150 LOC
- 单元测试：~80 LOC
- **总 ~350 LOC，3 天**

---

## §13 几何距离等值线路径生成（libigl，2026-05-07 七次修订）

### §13.1 取代 XY scanline 投影

之前 §6 路径策略用 XY 投影 + scanline，这在 overhang 区域（曲面切线接近水平）会丢点或扭曲间距。改用**曲面测地距离 + 等值线提取**：

- 在每层 iso-surface mesh 上选种子点 → 计算每顶点到种子的测地距离 d(v)
- 提取 isoline at d = k * line_spacing_mm 得到一组闭合等值线
- 等值线本身**就是路径**，间距均匀（曲面测地意义下），自然处理 overhang

### §13.2 算法

```
输入: 单层 iso-surface (V ∈ R^Nx3, F ∈ Z^Mx3)
1. 选种子点：bbox 中心投影到最近顶点
2. d = igl::exact_geodesic(V, F, seeds={selected})         // O(N log N) Dijkstra-like
3. for k = 1, 2, ..., until d_max:
     iso_d = k * line_spacing_mm
     polylines_k = igl::isolines(V, F, d, iso_d)            // 提取等值线
4. 每条 polyline 做起点选择 + 方向规范化（见下方 §13.2.1）
5. 每条 polyline 做 Frenet 切线计算（前向差分）
6. 输出 PathPolyline { points, tangents, is_closed }
```

#### §13.2.1 闭合等值线起点 + 方向约定（2026-05-10 拍板）

`igl::isolines` 输出的闭合环没有"起点"且方向任意。规范化规则：
- **起点**：环上 z 坐标最小的顶点（最靠近工作平台底）。若多个顶点 z 相同（环水平），取 x 最小的。理由：起点选最低点让喷头从"贴近平台"开始打，初始落点更稳定。
- **方向**：先任取一个方向算切线 → 调用 §11.2 构造工具坐标系 → 检查 `tool_y` 是否指向工件外侧。若指向内侧 → 反转 polyline 方向重算。这样保证 §11 不会"喷头侧面朝向已打印部分"。

### §13.3 老项目代码评估结论

| 老项目文件 | LOC | 决策 |
|-----------|-----|------|
| `exact_geodesic.cpp/h` | 1607+191 | **不直接迁移**。功能用 `igl::exact_geodesic` 替代（libigl header-only，~30 行调用代码）。老代码包含 Kimmel-Sethian 自实现 + 大量调试 logging，与 libigl 重复。 |
| `MarchingTriangles.cpp/h` | 1927+210 | **不迁移**。包含 `gmpxx.h`（GMP 精确算术依赖）。等值线提取用 `igl::isolines`（基于 Shewchuk robust predicates，已足够鲁棒）。 |
| `pathPostprocessing.cpp/h` | 335+41 | **简化迁移 ~250 LOC**。保留 `wlsFilter1d` + `LinearSmooth31/51/52`，删除冗余调试代码。 |

GMP 不引入：libigl 用 Shewchuk robust predicates 处理几何谓词的退化情况，不需要任意精度算术。GMP 是 binary 库，Windows 上集成复杂；libigl 是 header-only，与 Eigen 无缝集成。

### §13.4 接口

```cpp
// src/path/geodesic_paths.h
struct GeodesicPathParams {
    double line_spacing_mm = 0.4;          // 等值线间距 = 路径间距
    double resample_step_mm = 0.2;         // 沿等值线重采样步长
    int    seed_strategy = 0;              // 0=bbox center, 1=user-specified
    Eigen::Vector3d user_seed_point;       // 仅当 seed_strategy=1
    int    smooth_window = 5;              // 平滑窗口
};

struct PathPolyline {
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> tangents;
    bool   is_closed;
    double total_length_mm;
};

std::vector<PathPolyline> generateGeodesicPaths(
    const Eigen::MatrixXd& V,            // iso-surface 顶点
    const Eigen::MatrixXi& F,            // iso-surface 三角面
    const GeodesicPathParams& params);
```

### §13.5 验收

- 路径间距均匀性：测量相邻路径的最近邻距离，均值 ± 5% 范围内
- 路径连续性：单条 polyline 内相邻点间距方差 < 0.05 * resample_step_mm
- overhang 测试：用 armadillo 后腿（典型 overhang）验证不丢点
- 与 XY scanline 对比：在 mao 头顶（接近水平）路径数量与曲面面积比一致

### §13.6 工程量

- libigl 集成 + 等值线提取：~150 LOC
- 路径切线计算 + 重采样：~100 LOC
- 路径后处理（迁移自 pathPostprocessing 简化版）：~250 LOC
- 单元测试：~80 LOC
- **总 ~580 LOC，5 天**

---

## §14 5 次多项式关节角轨迹平滑（2026-05-07 七次修订）

### §14.1 目标

`§12 输出 vector<JointConfig>`（离散关节角序列，间距由 §13 路径采样决定）→ `§14 输出 vector<TrajectoryPoint>`（密采样、时间戳对齐 RSI 4ms 周期、位置/速度/加速度连续）。

### §14.2 算法：分段 5 次多项式

每两个相邻关节角点之间，对每个关节独立做 5 次多项式插值：
```
q(t) = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
```

边界条件（端点零速度版，`zero_endpoint_velocity = true`）：
- `q(0) = q_start`, `q(T) = q_end`
- `q'(0) = 0`, `q'(T) = 0`
- `q''(0) = 0`, `q''(T) = 0`

→ 6 个未知数 6 个方程，闭式解。位置 + 速度 + 加速度连续，jerk 在段交界处可能间断（可接受）。

### §14.3 双时间约束

每段段长 T 由两个约束的 max 决定：

```
Δt_cartesian = d_cartesian / target_line_speed_mm_per_s     // TCP 直线距离 / 期望线速度
Δt_joint     = max_i(|Δθ_i|) / max_joint_velocity_deg_per_s  // 最大关节转角 / 关节速度限
T = max(Δt_cartesian, Δt_joint)
```

举例（target_line_speed = 5mm/s, max_joint_velocity = 200°/s）：
- 段 A：TCP 移 2mm，最大关节转 1° → Δt_cart = 0.4s, Δt_joint = 0.005s → T = 0.4s（线速度主导）
- 段 B：TCP 几乎不动（0.01mm），但 A1 翻转 30°（接近奇异） → Δt_cart = 0.002s, Δt_joint = 0.15s → T = 0.15s（关节速度主导）

### §14.4 周期对齐 RSI 4ms

`sample_period_ms = 4`，与 RSI IPO_FAST 严格对齐。每段 T 内插值出 `ceil(T / 4ms)` 个采样点，第 i 个点：
```
t_i = i * 4ms
q_i = polynomial(t_i)
```

最后一个点的 t 可能 ≠ T（被向上取整），用线性插值修正。

### §14.5 关键参数（保守初值）

| 参数 | 默认值 | 上限 | 说明 |
|------|--------|------|------|
| target_line_speed_mm_per_s | 5.0 | 100 | 喷头打印线速度 |
| sample_period_ms | 4.0 | 12 | 必须匹配 RSI 周期 |
| max_joint_velocity_deg_per_s | 200 | 800（A6 物理上限）| 保守，远低于 KR4 R600 实际 336/336/488/600/529/800 |
| max_joint_accel_deg_per_s2 | 500 | 2000 | 保守 |
| max_joint_delta_per_cycle_deg | 3.0 | 5.0（RSI AXISCORR 默认安全限）| 每 4ms 单关节最大增量 |
| zero_endpoint_velocity | true | — | 段端点零速度（简单但有短暂停顿）|

### §14.6 接口

```cpp
// src/trajectory/poly5_smoother.h
struct TrajectoryParams {
    double target_line_speed_mm_per_s    = 5.0;
    double sample_period_ms              = 4.0;
    double max_joint_velocity_deg_per_s  = 200.0;
    double max_joint_accel_deg_per_s2    = 500.0;
    double max_joint_delta_per_cycle_deg = 3.0;
    bool   zero_endpoint_velocity        = true;
};

struct TrajectoryPoint {
    double timestamp_ms;
    std::array<double, 6> joint_deg;
    std::array<double, 6> joint_velocity_deg_per_s;
    std::array<double, 6> joint_accel_deg_per_s2;
    bool   wire_on;
    double plc_mode  = 1.0;       // 0=stop, 1=fwd, 2=rev
    double plc_speed = 5.0;
    double plc_ratio = 1000.0;
};

struct PathPointWithJoints {
    Eigen::Vector3d cart_pos;
    JointConfig     joint;
    bool            wire_on;
};

std::vector<TrajectoryPoint> smoothTrajectoryPoly5(
    const std::vector<PathPointWithJoints>& path,
    const TrajectoryParams& params);
```

### §14.7 验收

- 速度上限：`max_i max_t |joint_velocity[i](t)| ≤ max_joint_velocity` 严格成立
- 加速度上限：同上
- 单步增量：相邻两个 TrajectoryPoint 的关节角差 < `max_joint_delta_per_cycle_deg`
- 段间连续：位置连续到 1e-9，速度连续到 1e-6
- 时间戳：`timestamp_ms[i+1] - timestamp_ms[i] == sample_period_ms` 严格成立

### §14.8 工程量

- 5 次多项式系数闭式解：~80 LOC
- 双时间约束 + 段长决定：~60 LOC
- 周期重采样：~50 LOC
- 上限校验 + 错误处理：~60 LOC
- 单元测试：~150 LOC
- **总 ~400 LOC，3 天**

---

## §15 双格式输出：KRL 离线 + RSI 在线（2026-05-07 七次修订）

### §15.1 共享数据源

`§14 输出的 vector<TrajectoryPoint>` 是 §15a / §15b 的共同起点，两条下游分支独立：

```
vector<TrajectoryPoint>
       │
       ├──→ §15a writeKRL()           → job.src + job.dat 写盘（离线）
       │
       └──→ §15b RSISender::stream()  → 4ms UDP XML 双向通信（在线）
```

### §15.2 §15a 离线 KRL 输出

#### §15.2.1 文件对：`.src` 程序文件 + `.dat` 数据文件

**`.dat`（数据文件）**：每个轨迹点一条 `DECL E6AXIS XPi` 记录。
```
&ACCESS RVO
&REL 1
DEFDAT  curved_print
DECL E6AXIS XHOME = {A1 0.0,A2 9.95,A3 -62.26,A4 0.0,A5 -37.69,A6 0.0,
                     E1 0.0,E2 0.0,E3 0.0,E4 0.0,E5 0.0,E6 0.0}
DECL E6AXIS XP1 = {A1 0.123, A2 9.876, ..., A6 0.0}
DECL E6AXIS XP2 = ...
...
ENDDAT
```

**`.src`（程序文件）**：主流程。
```
&ACCESS RVO
&REL 1
DEF curved_print()
  INI
  $TOOL = TOOL_DATA[8]
  $BASE = BASE_DATA[6]
  $VEL_AXIS[1] = 80
  $APO.CPTP = 0                          ; 禁用 PTP blending（见 §15.2.2）
  ...
  EXTRUDER_OFFLINE_ENABLE = TRUE
  PTP XHOME                              ; 起点
  EXTRUDER_OFFLINE_MODE  = 1.0           ; 1=正转送丝（整层启动）
  EXTRUDER_OFFLINE_SPEED = 5.0
  EXTRUDER_OFFLINE_RATIO = 1000.0
  PTP XP1
  PTP XP2
  PTP XP3
  ...
  EXTRUDER_OFFLINE_MODE  = 0.0           ; 整层结束停止
  PTP XHOME
  EXTRUDER_OFFLINE_ENABLE = FALSE
END
```

**运动指令选择（2026-05-10 拍板）**：全程用 **PTP** 不用 LIN。理由：
1. §14 已经做了 4ms 密采样，相邻两点间距极小（线速度 5mm/s × 4ms = 0.02mm），PTP 走出来的 TCP 轨迹与 LIN 直线差异可忽略
2. PTP 指定关节角，KUKA 不会内部重做 IK，避免**臂态跳变**风险
3. LIN 在曲面打印的某些段（接近奇异）会被 KUKA 自动降速，反而不如 PTP 稳定

**送丝触发时机（2026-05-10 拍板，方案 X）**：每层开头 `EXTRUDER_OFFLINE_MODE = 1.0`，层尾 `= 0.0`。层内不切换。简单可用即可，预启动/延迟关停等精细策略推迟到 §16 或 §17 brainstorm。

#### §15.2.2 关键参数（与现役 KUKA 配置对齐）

| 字段 | 值 | 来源 |
|------|----|------|
| `$TOOL` | `TOOL_DATA[8]` | `[kuka.robot] tool_no = 8`（来自老 Offline.cpp） |
| `$BASE` | `BASE_DATA[6]` | 同上 base_no |
| Status (E6POS 用，本设计用 E6AXIS 不用) | 4 | 老 Offline.cpp |
| Turn (同上) | 28 | 同上 |
| 平台 z_offset | 0.28 mm | 同上 |
| `$VEL_AXIS[*]` | 80（保守） | 联调期可调 |
| `$APO.CPTP` | 0（禁用 PTP blending） | §14 已平滑，不希望 KUKA 再插入近似过渡改变轨迹 |
| `EXTRUDER_OFFLINE_ENABLE/MODE/SPEED/RATIO` | 见 sps.sub | 离线模式 PLC 信号 |

#### §15.2.3 接口

```cpp
// src/io/krl_writer.h
struct KRLOutputParams {
    int tool_no       = 8;
    int base_no       = 6;
    double vel_axis_pct = 80.0;
    JointConfig home_pose;     // 见 §15.5
    std::string job_name = "curved_print";
};

void writeKRL(
    const std::vector<TrajectoryPoint>& trajectory,
    const KRLOutputParams& params,
    const std::filesystem::path& output_dir);
// 输出: output_dir/<job_name>.src + output_dir/<job_name>.dat
```

#### §15.2.4 工程量

- `.dat` writer: ~80 LOC
- `.src` writer (含 PLC 信号 + WAIT 处理): ~150 LOC
- KRL 模板比对老 Offline.cpp 校验: ~50 LOC
- 单元测试 (输出文本对比 fixture): ~100 LOC
- **总 ~380 LOC，3 天**

### §15.3 §15b 在线 RSI 输出

#### §15.3.1 协议确认（来自 KUKA Ethernet RSI XML 1.1 官方文档）

| RSI XML 标签 | 含义 | 单位 |
|-------------|------|------|
| `<RKorr X Y Z A B C>` | 笛卡尔位姿增量（X/Y/Z 平移 + A/B/C 转角） | X/Y/Z=米, A/B/C=度 |
| `<AKorr A1..A6>` 或 `<AK A1..A6>` | 6 关节角增量 | 度 |
| `<EKorr E1..E6>` 或 `<EK E1..E6>` | 6 外部轴角增量（不用） | 度 |
| `<DiO>` | 数字输出（位掩码） | LONG |
| `<ZENG_OUT1/2/3>` | 用户自定义字段（PLC 信号） | DOUBLE |
| `<IPOC>` | 周期序号（必须原样回） | LONG |

**两组校正字段独立**，不可混用。本设计用 `<AK A1..A6>`（关节角路径）。

#### §15.3.2 必须的 RSI 配置改造（OPEN-1 解决方案）

现役 `RSI_Ethernet.rsi.xml` 是 POSCORR 配置（`RKorr.X/Y/Z/A/B/C`），无法直接发关节角。改造步骤：

1. **改 `RSI_Ethernet.rsi.xml`**：把 `POSCORR1` 块替换为 `AXISCORR1`（KUKA RSI 标准对象）
2. **改 `RSI_EthernetConfig.xml` `<RECEIVE>` 段**：
   ```xml
   <!-- 删除 -->
   <ELEMENT TAG="RKorr.X" TYPE="DOUBLE" INDX="1" UNIT="1" HOLDON="1" />
   ... RKorr.Y/Z/A/B/C ...
   <!-- 新增 -->
   <ELEMENT TAG="AK.A1" TYPE="DOUBLE" INDX="1" UNIT="0" HOLDON="1" />
   <ELEMENT TAG="AK.A2" TYPE="DOUBLE" INDX="2" UNIT="0" HOLDON="1" />
   <ELEMENT TAG="AK.A3" TYPE="DOUBLE" INDX="3" UNIT="0" HOLDON="1" />
   <ELEMENT TAG="AK.A4" TYPE="DOUBLE" INDX="4" UNIT="0" HOLDON="1" />
   <ELEMENT TAG="AK.A5" TYPE="DOUBLE" INDX="5" UNIT="0" HOLDON="1" />
   <ELEMENT TAG="AK.A6" TYPE="DOUBLE" INDX="6" UNIT="0" HOLDON="1" />
   <!-- 保留 -->
   <ELEMENT TAG="ZENG_OUT1" TYPE="DOUBLE" INDX="9" HOLDON="1" />
   <ELEMENT TAG="ZENG_OUT2" TYPE="DOUBLE" INDX="10" HOLDON="1" />
   <ELEMENT TAG="ZENG_OUT3" TYPE="DOUBLE" INDX="11" HOLDON="1" />
   ```
3. **用 RSIVisual 重编译** `.rsi.xml` → `.rsi` 二进制（覆盖 `RSI_Ethernet.dat`）
4. **部署到 KUKA** `KRC:\R1\Program\`：`RSI_Ethernet.dat` + `RSI_EthernetConfig.xml`
5. `RSI_Ethernet.src` **不需要改**（`RSI_CREATE / RSI_ON(#RELATIVE) / RSI_MOVECORR` 不依赖 POSCORR vs AXISCORR）

新配置文件以文本形式纳入仓库：`docs/kuka/rsi_axiscorr/RSI_Ethernet.rsi.xml` + `RSI_EthernetConfig.xml`，由 CLI #2 在 §15b 实现时生成。

#### §15.3.3 周期 = 4ms（IPO_FAST 默认）

来源：`RSI.src` 第 168-170 行 + `RSI_Ethernet.src` 第 50 行 `RSI_ON(#RELATIVE)` 不传 sensorMode → 默认 `#IPO_FAST`。

**OPEN-3 待 PDF 终验**：用户从 `KST_RSI_50_en.pdf` 确认即可。

#### §15.3.4 PLC 信号映射（来自 sps.sub）

| 上层语义 | RSI XML 字段 | KUKA 内部 | 物理输出 |
|---------|-------------|----------|---------|
| 模式 (0=停, 1=正转, 2=反转) | `<ZENG_OUT1>` | `ZENG_OUT1` 直接赋值 | `$OUT[1/2/3]` 三选一 |
| 送丝速度 | `<ZENG_OUT2>` | `ZENG_OUT2` → 内部 `Speed` | analog out |
| 流量倍率 | `<ZENG_OUT3>` | `ZENG_OUT3` → 内部 `Ratio` | analog out |

> 注：旧版 sps.sub 第 63-65 行原本通过 `$SEN_PREA[6/7/8]` 间接路由，新版 RSI 配置直接用 `<ZENG_OUT1/2/3>`，不再需要 SEN_PREA。

#### §15.3.5 收发包格式

KUKA → PC（每 4ms 一包）：
```xml
<Sen Type="ImFree">
  <RIst X="461.89" Y="5.64" Z="706.63" A="0" B="0" C="-90"/>
  <RSol X="461.89" Y="5.64" Z="706.63" A="0" B="0" C="-90"/>
  <Delay D="0"/>
  <Tech.C1 .../>
  <DiL>0</DiL>
  <Digout o1="0" o2="1" o3="0"/>
  <Source1>...</Source1>
  <IPOC>1234567890</IPOC>
</Sen>
```

PC → KUKA（必须 < 4ms 内回，否则 RSI 超时停机）：
```xml
<Sen Type="ImFree">
  <EStr></EStr>
  <Tech.T2 .../>
  <AK A1="0.10" A2="-0.05" A3="0.08" A4="0.0" A5="-0.02" A6="0.0"/>
  <FREE>0</FREE>
  <DiO>0</DiO>
  <ZENG_OUT1>1.0</ZENG_OUT1>
  <ZENG_OUT2>5.0</ZENG_OUT2>
  <ZENG_OUT3>1000.0</ZENG_OUT3>
  <IPOC>1234567890</IPOC>
</Sen>
```

#### §15.3.6 实时性约束

Windows 非 RT OS，4ms 硬实时回包要求：
- 主回包线程优先级：`SetThreadPriority(THREAD_PRIORITY_TIME_CRITICAL)`
- 关键路径**禁止**：堆分配、互斥锁、文件 IO、日志格式化
- 预分配所有缓冲区（XML 解析 / 序列化用 fixed-size buffer）
- XML 解析：用 `pugixml` header-only 库，禁用 DOM tree 持久化（每包 parse 后立即丢弃）
- IPOC 同步：第一帧等 KUKA 包到达，记下起始 IPOC；之后每包原样回 IPOC

#### §15.3.7 接口

```cpp
// src/io/rsi_sender.h
struct RSISenderParams {
    std::string kuka_host  = "192.168.2.128";    // 来自 RSI_EthernetConfig.xml
    int         port       = 59152;              // 同上
    int         cycle_ms   = 4;                  // IPO_FAST
    int         max_late_packets = 100;          // 与 KUKA 端 Timeout=100 对齐
    std::string sentype    = "ImFree";
};

class RSISender {
public:
    explicit RSISender(const RSISenderParams& params);
    ~RSISender();

    // 阻塞执行整段轨迹：每 4ms recv KUKA 包 → 取下一帧 delta → format → send
    // 返回值: 完成的轨迹点数 (期望 == trajectory.size())
    size_t streamTrajectory(const std::vector<TrajectoryPoint>& trajectory);

    void disconnect();

private:
    RSISenderParams params_;
    boost::asio::io_context io_ctx_;
    std::unique_ptr<boost::asio::ip::udp::socket> socket_;
    int64_t initial_ipoc_ = -1;
};
```

#### §15.3.8 工程量

- UDP socket + boost::asio 集成：~100 LOC
- pugixml 解析 + 序列化：~200 LOC
- 实时线程 + 优先级控制：~80 LOC
- IPOC 同步 + 增量差分（从绝对关节角差分到 delta）：~80 LOC
- 错误处理（包丢失、超时、KUKA 端 EStr）：~80 LOC
- 单元测试（loopback UDP，模拟 KUKA 端）：~150 LOC
- **总 ~700 LOC，6 天**

### §15.4 HOME 关节角（现场实测，2026-05-07）

```
A1 = 0.00°
A2 = +9.95°
A3 = -62.26°
A4 = 0.00°
A5 = -37.69°
A6 = 0.00°
```

A2+A3+A5 ≈ -90° → 工具姿态相对 base 翻 90°，喷头水平指向工件（与现场布局一致）。

配置写入：
```toml
[kuka.home]
joint_deg = [0.0, 9.95, -62.26, 0.0, -37.69, 0.0]
```

KRL 端使用：`§15a` 写 `DECL E6AXIS XHOME = {A1 0.0, A2 9.95, ...}` + `.src` 起始/结束 `PTP XHOME`。RSI 端 `RSI_Ethernet.src` 已有 `SPTP Xhome`，§15b 不需要发 home 指令但首帧前会校验当前位姿与 home 一致（差 > 1° 报警）。

### §15.5 验收

- §15a：KUKA 示教器导入 `.src + .dat` 无语法错误，离线模拟运行（无机器人）走完整段路径无 fault
- §15b loopback：本机模拟 KUKA 端 4ms 周期回包，1 万包丢包率 < 0.1%
- §15b 现场：与真机连接 3 分钟，丢包率 < 1%，机器人无超时停机
- HOME 校验：§15b 启动时若示教器实际位姿与 home_pose 差 > 1° → 报警拒绝启动

---

## §16 层间衔接：⊓ 形 PTP 转移（2026-05-10 八次修订）

### §16.1 背景

§14 输出的 `vector<TrajectoryPoint>` 是**单层内**的连续轨迹。实际打印是多层叠加，相邻两层之间需要**过渡路径**：从层 N 末点抬升、水平移动到层 N+1 入口上方、下降到层 N+1 起点。此过程不打印，需保证：
- 不撞已打印部分（§17 校验）
- 速度可比打印段快（节省时间）
- 送丝信号正确切换（开→关→开）

### §16.2 路径形状：⊓ 形 4 keypoint（Q1 选 B2）

每层间转移产生 4 个关键点，依次连接成 ⊓ 形（KRL 用 PTP 串接，§14 5 次多项式平滑）：

| 编号 | 点名 | 坐标 |
|------|------|------|
| 1 | `last_pt_N` | 本层 §13 路径最后一个点 `(xy_N_end, z_N_end)` |
| 2 | `lift_corner_1` | `(xy_N_end, lift_z)` — xy 不变，z 抬到 lift_z |
| 3 | `lift_corner_2` | `(xy_N+1_start, lift_z)` — z 不变，xy 移到下层入口 |
| 4 | `first_pt_N+1` | 下一层 §13 路径第一个点 `(xy_N+1_start, z_N+1_start)` |

工具姿态 (A/B/C) 的处理：
- 点 1 / 点 4：用 §11 算出的姿态（来自各自所属的打印路径段）
- 点 2 / 点 3：用与点 1 相同的姿态（lift + 水平段不旋转工具，避免姿态抖动）
- 点 3 → 点 4 之间：§14 5 次多项式自然把姿态从点 1 的姿态平滑插值到点 4 的姿态（在 4ms/帧的密采样里逐步旋转）

### §16.3 抬升高度（Q2 选 B）

```
lift_z = max(layer_N_max_z, layer_N+1_max_z) + safety_margin_mm
safety_margin_mm = 5.0 (默认)
```

理由：考虑下一层 z 可能比当前层高，水平段必须高于两层最高点。

### §16.4 转移速度（Q3：用户指定 25 mm/s）

```
transition_speed_mm_per_s = 25.0  (默认，比打印段快 5×)
target_line_speed_mm_per_s = 5.0  (打印段，§14)
```

§14 trajectory smoothing 在 transition 段（点 1→2→3→4）用 `transition_speed_mm_per_s` 替代 `target_line_speed_mm_per_s`。两段交界（last_pt → lift_corner_1）瞬时切速度 → §14 `zero_endpoint_velocity = true` 自然处理（每段端点零速度，从打印段的 5mm/s 减到 0，再加速到 transition 的 25mm/s）。

### §16.5 送丝时机（方案 X，§15.2.1 已定）

```krl
PTP last_pt_N
EXTRUDER_OFFLINE_MODE = 0.0      ; 整层结束，关送丝
PTP lift_corner_1                ; 抬升 (transition speed)
PTP lift_corner_2                ; 水平
PTP first_pt_N+1                 ; 下降
EXTRUDER_OFFLINE_MODE = 1.0      ; 下层开始，开送丝
```

### §16.6 接口

注意：§16 在 pipeline 里**早于 §14**（见 §17.5 数据流），所以接口用 §14 的输入类型 `PathPointWithJoints`（不是 §14 输出的 `TrajectoryPoint`）。`transition_speed` 作为 hint 传给 §14，让它在该段用 transition speed 替代 print speed。

```cpp
// src/trajectory/layer_transition.h
struct LayerTransitionParams {
    double safety_margin_mm           = 5.0;
    double transition_speed_mm_per_s  = 25.0;
};

struct LayerTransitionKeypoints {
    std::array<PathPointWithJoints, 4> keypoints;   // 1-4 同上表
    double lift_z;
    double segment_speed_mm_per_s;                   // = transition_speed_mm_per_s, §14 用
    bool   collision_safe;                           // §17 校验结果
};

LayerTransitionKeypoints buildLayerTransition(
    const PathPointWithJoints& last_pt_N,
    const PathPointWithJoints& first_pt_Np1,
    double layer_N_max_z,
    double layer_Np1_max_z,
    const LayerTransitionParams& params,
    const SDFGrid& sdf_full,                         // §17 用
    const NozzleCylinderParams& nozzle);             // §17 用
```

§14 的 `smoothTrajectoryPoly5` 接口需要扩展：接收一个 `vector<SegmentSpeedHint>` 让每个 segment 可以独立设速度（打印段用 5mm/s，transition 段用 25mm/s）。这是 §14 的小改动，约 +20 LOC。

### §16.7 失败处理

如果 §17 校验某层间 transition 的 4 个 keypoint 中任一撞墙：
- 标记该层间过渡失败
- abort 后续层（不再继续打印）
- 上报 `failure_layer_index` + 撞点详细信息让用户决策

不做 fallback（如尝试更高 `lift_z`）：违反 "simplest"，且如果 `max(N, N+1) + 5mm` 都撞，说明已打印几何已经异常，不应继续。

### §16.8 验收

- mao / armadillo / bunny 三模型每层间 transition 都生成成功
- transition keypoints 经 §17 校验通过率 100%
- KRL 模拟器（KUKA OfficeLite）走完 5 层路径，每层间过渡无 fault

### §16.9 工程量

- 4 keypoint 构造：~80 LOC
- 与 §14 集成（变速度处理）：~50 LOC
- 失败处理 + 报告：~50 LOC
- 单元测试：~80 LOC
- **总 ~260 LOC，2 天**

---

## §17 喷头圆柱体碰撞检查（2026-05-10 八次修订）

### §17.1 圆柱体模型

把喷头近似为**直立圆柱**（沿 -tool_z 方向，从 TCP 向"工件外"延伸，即喷头本体 + 加热块部分，不含喷嘴尖端）：

```
        ┌──┐  ← 圆柱顶 (TCP - h * tool_z)
        │  │
        │  │ h
        │  │
        │  │
        ●  ← 圆柱底 (TCP)
        ↓ tool_z (指向工件)
       工件
```

参数：
- `r`: 圆柱半径 (mm) — 喷头本体最粗处的半径
- `h`: 圆柱高度 (mm) — 从 TCP 到喷头本体顶部的距离

**OPEN-10**: r 和 h 用户尚未实测。临时默认 `r = 10mm`, `h = 30mm`（典型 FDM 喷头 + 加热块尺寸），待用户测量后改 toml。

### §17.2 SDF 源：动态裁剪（Q4 选 C）

不维护增量 SDF。直接用原始输入 STL 的 SDF（§2 voxelize 阶段已生成），但检查时**只考虑当前打印层 z 以下的部分**：

```cpp
double maskedSDF(const Eigen::Vector3d& p,
                 double current_layer_max_z,
                 const SDFGrid& sdf_full) {
    if (p.z() > current_layer_max_z) {
        return std::numeric_limits<double>::infinity();   // 上方未打印，视为无穷远
    }
    return sdf_full.query(p);
}
```

注：`current_layer_max_z` 在每层检查时变化，是当前打印层的最高 z 值。SDF 内部 (sdf < 0) 才算撞。

### §17.3 检查算法

对每个待检查的 `(TCP_pose, R_tool)` 点：
1. 从 R_tool 提取 tool_x, tool_y, tool_z 方向
2. 沿圆柱轴线 (-tool_z) 等间距采样 K 个点（默认 K=10）：
   ```
   sample_i = TCP - (i / K) * h * tool_z    for i = 1..K
   ```
3. 对每个采样点 sample_i，圆柱"截面"是半径 r 的圆。在 ⊥tool_z 平面内绕 sample_i 取 M 个圆周点（默认 M=8）：
   ```
   circle_pt_j = sample_i + r * (cos(θ_j) * tool_x + sin(θ_j) * tool_y)
                 for θ_j = j * 2π / M, j = 0..M-1
   ```
4. 对每个圆周点 circle_pt，查 `maskedSDF(circle_pt) < 0` → 撞
5. 任意一点撞 → 整个 pose 标记 `COLLISION`

总采样点数：`K × M = 80` 个/pose。SDF 查询 O(1)（grid lookup）。每 pose 检查开销 ~80 SDF queries。

### §17.4 碰撞响应（Q5 选 B 两级，与 §12 一致）

| 单层碰撞率 | 处理 |
|-----------|------|
| 0% | 通过 |
| 0% < 率 ≤ 30% | 单点跳过（§14 平滑时跳过这些点的 segment）|
| > 30% | 整层 SKIPPED，不参与 §14 + §15a/b |

阈值参数：`collision_layer_skip_threshold = 0.30`（与 §12 `layer_skip_threshold` 同值，但语义独立）。

### §17.5 检查时机（Q6 选 A 预平滑）

更新 §15.1 的 pipeline 数据流：

```
§13 geodesic isolines
       │
       ▼
§11 IK (per-path-point)
       │
       ▼
§12 reachability filter (per-layer 30% threshold)
       │
       ▼
§17 collision check (path-level, per-layer 30% threshold)  ← 新增
       │
       ▼
§16 insert layer transitions (4 keypoints / 转移)
       │
       ▼
§17 collision check (transition keypoints only)            ← 新增
       │
       ▼
§14 5-poly smooth
       │
       ▼
§15a writeKRL  /  §15b RSI sender
```

§14 平滑后**不再 §17 二查**（理由：5 次多项式不会大幅偏移 XY，多检几乎冗余）。

### §17.6 接口

```cpp
// src/safety/nozzle_collision.h
struct NozzleCylinderParams {
    double radius_mm                        = 10.0;     // OPEN-10
    double height_mm                        = 30.0;     // OPEN-10
    int    axial_samples                    = 10;       // K
    int    circumferential_samples          = 8;        // M
    double collision_layer_skip_threshold   = 0.30;
};

enum class CollisionStatus { OK, COLLISION };

struct CollisionResult {
    CollisionStatus status;
    int             colliding_sample_idx;    // 第几个采样点撞 (调试用)
    double          min_distance_mm;         // 最近距离 (调试用，所有 80 点中最小)
};

// 单点检查
CollisionResult checkNozzleCollision(
    const CartPose& pose,
    const Eigen::Matrix3d& R_tool,           // tool_x, tool_y, tool_z 由列提取
    const SDFGrid& sdf_full,
    double current_layer_max_z,
    const NozzleCylinderParams& params);

// 整层检查 + 两级阈值
struct LayerCollisionReport {
    int    total_points;
    int    colliding_points;
    double collision_rate;
    bool   layer_skipped;
    std::vector<int> skipped_indices;        // 单点跳过的 idx 列表
};

LayerCollisionReport checkLayerCollision(
    const std::vector<CartPose>& layer_poses,
    const std::vector<Eigen::Matrix3d>& layer_R_tools,
    const SDFGrid& sdf_full,
    double current_layer_max_z,
    const NozzleCylinderParams& params);

// transition keypoint 单独检查（§16 调用）
bool checkTransitionCollision(
    const std::array<TrajectoryPoint, 4>& keypoints,
    const SDFGrid& sdf_full,
    double layer_max_z,                      // = max(layer_N, layer_N+1) max z
    const NozzleCylinderParams& params);
```

### §17.7 验收

- **单元测试**（已知碰撞 case）：构造 cylinder + 已知 SDF，验证算法返回正确 status + min_distance
- **三模型预检**：mao / armadillo / bunny 各层碰撞率应当 < 5%（典型几何下喷头本体不应频繁撞）
- **极端 case**：构造一个高度悬出的 STL（如人像伸出的手臂），确认 §17 能识别"喷头本体撞悬挂部分"
- **transition 验收**：5 层 + 4 transitions 的 mao 子集，所有 transition keypoints 通过率 100%

### §17.8 工程量

- 圆柱采样 + SDF 查询：~120 LOC
- 整层检查 + 两级阈值：~80 LOC
- transition keypoint 单独检查：~40 LOC
- 单元测试（含 fixture）：~120 LOC
- **总 ~360 LOC，3 天**

## §18 测试策略（部分已 brainstorm，2026-05-10）

设计目标：单元测试（每个模块）+ 三模型 phase3 集成测试（mao / armadillo / bunny）+ KRL 模拟器验证 + RSI 协议双端一致性验证 + 现场联调。完整 brainstorm 待续。

### §18.1 RSI 协议双端一致性测试（OPEN-8 验收，2026-05-10 拍板）

由于现役 RSI 配置是 POSCORR（笛卡尔），改造为 AXISCORR（关节角）后必须严格验证双端协议一致：

| 测试 | 内容 | 通过标准 |
|------|------|---------|
| **T1 loopback** | 本机起 mock KUKA 服务器（Python pugixml 解析回包），PC 发 1 万周期 | 每个 `<AK A1..A6>` 字段顺序、数值（精度 1e-6）、IPOC 与 mock 接收一致；零包丢失 |
| **T2 KUKA 单向收** | KUKA 端只接收不动作（用 RSI logging），PC 发 100 周期 | 导出 KUKA `RSI.log`，每帧 `AK.A1..A6` 数值与 PC 发出值一致到 1e-4 |
| **T3 KUKA 闭环移动** | 发送已知小幅度增量序列（每帧 A1 += 0.01°，1000 周期） | KUKA A1 实际转 10° ± 0.05°（误差含齿隙 + 编码器精度） |
| **T4 超时容忍** | 故意丢包 5%（PC 端不回某些帧） | KUKA 不停机（在 Timeout=100 容忍范围内）；丢包率达 10% 时 KUKA 应报警 |
| **T5 IPOC 同步** | PC 故意回错的 IPOC（如 +1 偏移） | KUKA 应报错并停机，**而不是**默默执行错误增量 |

T1 由 CLI #2 实现（纯本机，可自动化）。T2-T5 必须现场实测（用户 + CLI #1 协调）。

### §18.2 其他测试目标（待 §18 完整 brainstorm）

- 单元测试：每个 §11/§12/§13/§14 模块的关键函数，覆盖 OK/边界/异常路径
- 集成测试：mao / armadillo / bunny 三模型走完整 pipeline，输出 KRL + 可视化
- KRL 模拟器：用 KUKA OfficeLite 或 KUKAVARPROXY 离线跑生成的 .src，确认无 fault
- 性能基准：每个模型从 STL 输入到 KRL 输出的总时间 < 30 min

---

## §19 遗留问题清单（OPEN-* 登记）

| ID | 问题 | 当前默认 | 状态 | 解决时机 |
|----|------|---------|------|---------|
| OPEN-1 | 在线协议走笛卡尔 RKorr 还是关节角 AKorr | AKorr (关节角) | ✅ **已解 2026-05-07** | — |
| OPEN-2 | RSI `<EStr>` 字段语义 | 空字符串 | 待联调 | §15b 联调期 |
| OPEN-3 | RSI 周期 4ms 还是 12ms | 4ms (IPO_FAST 默认) | ✅ **已解 2026-05-10**（RSI.src 第 168-170 行 + RSI_Ethernet.src 第 50 行 RSI_ON(#RELATIVE) 默认 sensorMode=#IPO_FAST 已确证） | — |
| OPEN-4 | `$SEN_PREA[1..5]` 用途 | 未用 | 仅扩展时相关，新版 RSI 不需要 | — |
| OPEN-5 | AXISCORR 默认 ±5° 安全限是否需调高 | 不调，按默认 | 联调期 | §15b 联调期 |
| OPEN-6 | KUKA 端是否残留旧 UDP listener | 无残留 (现场仅 RSI) | ✅ **已解 2026-05-07** | — |
| OPEN-7 | `$VEL_AXIS[*]` 最优值 | 80% | 联调期 | §15a 联调期 |
| OPEN-8 | RSI 配置改造 (POSCORR → AXISCORR) 部署方 | CLI #2 写文本 + 用户用 RSIVisual 部署，配套 §18.1 五项测试验收 | 待启动 | §15b 实现前 |
| OPEN-9 | AXISCORR 模式下 `integrationSystem` 是否被忽略 | 假设忽略 (关节空间无坐标系概念)；CLI #2 实现时直接不传该参数试 | 待 KUKA 实测验证 | §15b 联调期 |
| OPEN-10 | 喷头几何参数 r/h | 用户待测 | 待用户测量 | §17 启动前 |
| OPEN-11 | 现役 KUKA 控制器 KSS 版本 (影响 RSI 1.0 vs 1.1 vs 5.0 语法差异) | 假设 KSS 8.6/8.7 (RSI 5.0 兼容 1.1 语法) | 待用户现场示教器查（HMI → Help → Info → System info） | §15b 联调前 |
| **OPEN-12（新 2026-05-10）** | `$TOOL` / `$BASE` 编号 + `tool_z_offset` + `platform_z_offset` 等标定相关参数应当 UI 可调 | 当前在 toml 硬编码（tool_no=8, base_no=6, z_offset=0.28），每次重新标定需手改 toml | 设计待办 | UI 阶段（v4 之后） |

---

*v4 §10-§15 修订（2026-05-07 七次）：完成 KUKA 6 轴 FDM 曲面打印的算法重构 + 硬件输出对接。BC 选择推翻 SDF normal 方案，改用几何 z 坐标 (§10)。MATLAB 链路彻底删除，6 轴解析 IK 移入 C++ (§11)。可达性检查升级为双层结构 (§12)。路径策略从 XY scanline 改为曲面测地等值线 (§13)。轨迹平滑用 5 次多项式 + 双时间约束 (§14)。输出双格式：离线 KRL + 在线 RSI 关节角 AKorr 协议 (§15)。RSI 配置需要从 POSCORR 改造为 AXISCORR，由 CLI #2 自动生成新配置文件，用户用 RSIVisual 部署。**§16-§17 brainstorm 待续，§18 测试策略已部分定稿（§18.1 RSI 双端一致性 5 项测试）。***

*2026-05-10 §10-§18 细节补丁（用户 review 第一轮反馈）：
- §10 顶面 BC 改为 Dirichlet phi=1（取代 free Neumann），追求层厚均匀
- §11.2 路径切线方向规范化：让 tool_y 指向工件外侧
- §13.2.1 闭合等值线起点 + 方向约定（z 最小点 + 工件外侧导向）
- §15.2.1 KRL 全程 PTP 不用 LIN；送丝按层级开关（方案 X）；禁用 $APO.CPTP blending
- §18.1 完成 RSI 协议双端一致性 5 项测试设计（OPEN-8 验收依据）
- OPEN-3 关闭（4ms 已确证）；OPEN-12 新增（$TOOL/$BASE UI 可调）；其他 OPEN 进度更新*

*2026-05-10 §16-§17 brainstorm 完成（八次修订）：
- §16 层间衔接定型：⊓ 形 4-keypoint，自适应 lift_z = max(N, N+1) + 5mm，转移速度 25 mm/s（打印 5 mm/s 的 5×）
- §17 喷头碰撞检查定型：圆柱体 (r=10mm, h=30mm 默认，OPEN-10 待测) + 动态裁剪 SDF（只查当前层 z 以下）+ 两级阈值（30% 跳层，与 §12 一致）+ 预平滑时机（§14 之前一次主筛 + transitions 单独）
- §17.5 更新 pipeline 数据流图：§12 → §17 path-level → §16 insert → §17 transition-level → §14 → §15
- §16 + §17 总工程量 ~620 LOC, 5 天
- v4 §10-§17 至此**主体设计完整**，§18 测试策略 §18.1 已定，§18.2-§18.X 待补*
