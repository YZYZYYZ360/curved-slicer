# 架构修订：v3 → v4 增补与废止

> 修订日期：2026-04-27
> 触发原因：Phase 1 实施审阅发现 `src/field/wavefront.cpp` 是 Dijkstra 重写而非老算法迁移；继续维护新仓库内的"基线"无意义。
> 决定：采用**方案 C**——基线由老项目可执行体提供，新项目专注新算法。
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

---

## §5 修订后的全局架构（一览）

```
┌─────────────────────────────────┐
│     curved-slicer (新仓库)       │
│   只实现新算法（Plan-C 基线之外）   │
│  ─────────────────────────────  │
│   io / geometry / field /        │
│   surface / metrics / path / app │
│   field = Laplacian + Poisson    │
│   + KUKA 投影                    │
└─────────────────────────────────┘
              │ 输出
              ▼
   runs/phase*_field/<model>/
       phi.npy / iso_*.ply / metrics.json
              │
              ▼
        ┌─────────────────┐
        │ compare_baselines│
        │   (Phase 2 末)   │
        └─────────────────┘
              ▲
              │ 输出
   runs/phase*_baseline/<model>/
       phi.npy / iso_*.ply / metrics.json
              ▲
              │ 输出
┌─────────────────────────────────┐
│   旧项目可执行体（只读参考）        │
│   tests/benchmarks/old_project/  │
│   编译 SFF + OpenVDB             │
│   = 论文基线                      │
└─────────────────────────────────┘
```

---

## §6 与 v3 自检清单对应

v3 §9 自检清单中**与 wavefront 相关**的条目：

- ~~v3 §6 映射表与 Phase 1 矛盾解决：把 `SFF::getDistanceFiled` 拆成"纯波前"+ "补丁逻辑"两行~~ → **作废**：方案 C 下两者都废弃，不在新仓库迁移
- ~~v3 §2.10 给出 `WavefrontParams` 与 `solveWavefront` 接口~~ → **作废**：模块已删除
- ~~`[algorithm] mode` 切换~~ → **作废**：单一算法路径

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

v3 中**与 wavefront 无关**的所有条目（KUKA 非对称、batch 模式、generateBC 接口、Phase 0-5 路线图、子目录数 ≤ 7、依赖 ≤ 3、headers < 100 行、中文路径处理 等）继续生效。

---

*v4 修订是局部增量，不替代 v3。两份文件并存：v3 是"原始设计"，v4 是"实施中的现实修正"。*
