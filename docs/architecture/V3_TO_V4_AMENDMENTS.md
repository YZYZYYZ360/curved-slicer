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
