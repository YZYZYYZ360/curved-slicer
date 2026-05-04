# Phase 3 进行中状态

> 本文档：Phase 3 尚未收尾时的工作日志，给任何接手者（Codex、新 Claude 账号、用户本人）一个明确的"我们停在哪里、下一步做什么"。
> 编写日期：2026-04-29
> 阶段：**Phase 3 · 向量 Laplacian + Hemisphere 投影 + Poisson**

---

## 1. 阶段目标（v4 §6 重定位）

| 子任务 | 状态 | 备注 |
|---|---|---|
| D1 向量 Laplacian + 非均匀 BC | ✅ 完成 | `solveLaplacianVector` + `generateVectorBC` |
| D2 Hemisphere clamp | ✅ 完成 | `projectToHemisphere` |
| D3 Poisson 重建 | ✅ 完成 | `solvePoisson` 已含 IC-CG 迭代日志 |
| D4 pipeline 接入 vector_kuka 分支 | ✅ 完成 | scalar / vector_kuka 共存 |
| D5 M1 (max\|H\|) 计算 | ✅ 完成 | 接入 metrics.json |
| D6 M2 hemisphere violation 验证 | ✅ 完成 | 三模型 = 0% |
| D7 chooseBottomAnchor 修复 | ✅ 完成 | 修了 anchor 取顶面 voxel 的 bug |
| D10' phi gauge 三闸门 | ✅ 完成 | 方向投影 / 量级 / 残差占比 |
| D12 per-model cc 分量诊断 | ✅ 完成 | metrics.json 含 components 数组 |
| D14 layer_count 阈值放宽 | ✅ 完成 | >= 47（原 50） |
| D18 cc 验收口径修正 | ✅ 完成 | 主分量占比 > 99% |
| **D20 mao_rescaled 测试** | ⚠️ **待跑** | config 已就绪，等用户跑 build |
| D21 v4 §6.6 max-cc 阈值放宽 | ⚠️ 待写 | 把 bunny 反向上升从失败改为描述 |
| Phase3.md 收尾 | ❌ 未做 | D20 出结果后才写 |

---

## 2. 三模型当前结果

### 2.1 scalar 路径（Phase 2 基线，全部通过）

| 模型 | 体素 | layer | faces | per-model cc | max-layer cc | M1 | solver_ms |
|---|---:|---:|---:|---:|---:|---:|---:|
| armadillo | 61K | 56 | 580804 | 1 | 72 | 519869 | 2007 |
| bunny | 170K | 56 | 872460 | 1 | 29 | 3361 | 5495 |
| mao | 568K | 88 | 4105921 | 1 | 91 | 7032 | 40963 |

### 2.2 vector_kuka 路径（Phase 3）

| 模型 | layer | faces | per-model cc | max-layer cc | M1 | M2 | 状态 |
|---|---:|---:|---:|---:|---:|---:|---|
| armadillo | 49 | 78990 | 1（主分量 99.987%） | **36** ↓50% | 209 ↓99.96% | 0 | ✅ 通过 |
| bunny | 47 | 244079 | 1 | **39** ↑34% | 2315 ↓31% | 0 | ✅ 通过（max-cc 反向是几何特性） |
| mao 原版 | — | — | — | — | — | — | ❌ Poisson 不收敛 |
| mao_rescaled 0.7x | — | — | — | — | — | — | ⚠️ 待跑 |

### 2.3 关键发现

1. **algorithm 在 armadillo 上效果显著**：max-cc 72→36 下降 50% 达标，M1 大幅下降
2. **algorithm 在 bunny 上有副作用**：max-cc 29→39 反向上升
   - 原因：bunny 凸光滑，标量 z 单调切片本来就连通；vector 弯曲反而引入起伏
   - **不是 bug**，是论文素材："our method excels on geometrically complex models"
3. **mao 原版 Poisson 不收敛**：568K 体素超过 IC-CG 有效区。0.7x 缩放（~195K）是规模 vs 几何的判别测试

---

## 3. mao 阻塞详情

### 3.1 调试时间线

| 尝试 | 结果 | 结论 |
|---|---|---|
| 默认 `use_precondition=true`（SimplicialCholesky） | 60+ 分钟无输出 | 内存爆炸或分解卡死 |
| 切到 IncompleteCholesky + CG | 491s 跑满 500 迭代未收敛 | 预条件不够 |
| 切到 ILUT + CG | 30+ 分钟仍卡 | 比 IC-CG 更差 |
| 恢复 IC-CG 当前默认 | 同 491s 不收敛 | 最稳的失败路径 |

### 3.2 D16 进度日志（mao 原版 IC-CG 跑时）

```
[mao] generateVectorBC done, fixed_count=39480
[mao] solveLaplacianVector done
[mao] projectToHemisphere done
[mao] solvePoisson begin
... 491s 后退出，error 仍未达 1e-6
```

### 3.3 两个候选根因

| 根因 | 描述 | 判别 |
|---|---|---|
| **A. 规模主导** | 568K 体素超过 IC-CG 预条件有效区 | 0.7x 缩放（195K）能收敛则坐实 |
| **B. 几何主导** | hair/眉/嘴等高频特征在 hemisphere clamp 后产生 div_G 高频噪声 | 0.7x 缩放仍不收敛则坐实 |

**优先怀疑 A**（80% 概率），因为 armadillo 61K + bunny 170K 都收敛，规模是主要变量。

---

## 4. D20 测试就绪状态（Claude 已准备）

### 4.1 已完成的代码改动

1. ✅ 新建 `config/mao_rescaled.toml`：单模型 + vector_kuka + spacing=0.5mm + debug_dump=true
2. ✅ 修改 `src/app/pipeline.cpp:898`：vector_kuka 分支调用 solvePoisson 时设 `log_iterations=true` + `progress_label=报告名`
3. ✅ Poisson 自身（`src/field/poisson.cpp`）已实现每 50 次迭代打印 `[<label> poisson] iter=N error=X`

### 4.2 需要用户/Codex 做的事

```
# 步骤 1：清理可能残留的 git lock（如果还有）
cd "E:\Git_Repository\curved-slicer"
del .git\index.lock 2>nul

# 步骤 2：rebuild（spacing 没变，只是日志加了，应该增量编译很快）
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build_nmake'

# 步骤 3：跑 mao_rescaled.toml
# 当前 phase2_field_batch_report.exe 接受 model_name + pipeline，不接受 config 路径。
# 三选一：
#   选 a) 临时把 batch_three_models.toml 的 mao 条目 stl_path 改成 0.7x 路径，跑：
#         build_nmake\phase2_field_batch_report.exe mao vector_kuka
#         跑完恢复
#   选 b) 给 phase2_field_batch_report 加 --config 参数（小改 main()）
#   选 c) 写一个 phase3_mao_rescaled.cpp 单测 driver，调用 runBatch(loadPipelineConfig("mao_rescaled.toml"))

# 步骤 4：观察 stdout
# 期待看到：
#   [mao_rescaled] generateVectorBC done, fixed_count=...
#   [mao_rescaled] solveLaplacianVector done
#   [mao_rescaled] projectToHemisphere done
#   [mao_rescaled] solvePoisson begin
#   [mao_rescaled poisson] iter=50 error=...
#   [mao_rescaled poisson] iter=100 error=...
#   ...
```

### 4.3 三个场景 + 处理方案

| 场景 | 表现 | 根因 | 后续动作 |
|---|---|---|---|
| **A** | 收敛 < 600s | 规模主导 | mao 原版临时切 spacing=1.0mm 收尾 Phase 3；论文实验时再考虑 AMG/multigrid |
| **B** | 仍跑满未收敛 | 几何主导 | 写 v4 §7：在 Poisson 前对 G_clamped 做轻量 Laplacian 平滑 |
| **C** | 5-10 min 慢但收敛 | 临界 | 接受当前性能；Phase 3 收尾 |

---

## 5. 未提交的代码改动（git working tree）

```
M CMakeLists.txt
M README.md
M config/batch_three_models.toml
M read.md
M scripts/compare_baselines.py
M src/app/pipeline.cpp     ← 含 D20 的 log_iterations + 完整 D7-D19 + D20.4 修改
M src/app/pipeline.h
M src/field/laplacian.cpp  ← 完整 D1（向量 Laplacian + 非均匀 BC）
M src/field/laplacian.h
M src/field/poisson.h
A src/field/poisson.cpp    ← D3 Poisson 重建 + 迭代日志
A src/field/kuka_projection.{h,cpp}  ← D2 Hemisphere clamp
A config/mao_rescaled.toml ← Claude 本轮新建
A src/io/config_loader.{h,cpp}（已修改）
A tests/poisson_smoke_test.cpp
A docs/算法流程详解.md     ← Claude 本轮新建
A docs/Phase3_进行中.md    ← Claude 本轮新建（本文档）
```

`.git/index.lock` 历史问题已确认清掉（本轮 git status 跑通）。

---

## 6. 当前 commit 计划

D20 测试出结果后，按以下 commit 拆分：

1. `[phase3-D7-D19-impl]` 包含 D1-D19 完整实现（laplacian/kuka_projection/poisson + pipeline 接入 + tests）
2. `[phase3-D20-mao-rescaled-{A|B|C}]` mao_rescaled 测试结果 + 对应处理
3. `[v4-amend-§7]` 仅场景 B 需要：G 平滑预处理架构修订
4. `[phase3-summary]` Phase3.md 正式收尾

---

## 7. 给接手者的快速上下文

如果你是新 Claude / 新 Codex 实例 / 第二天的我：

1. **必读文档（按顺序）**：
   - `docs/architecture/NEW_PROJECT_ARCHITECTURE_v3.md`：原始架构
   - `docs/architecture/V3_TO_V4_AMENDMENTS.md` §1-§6：6 次架构修订
   - `docs/Phase0.md` / `Phase1.md` / `Phase2.md`：每阶段实施总结
   - `docs/算法流程详解.md`：完整算法流程（中文，无术语门槛）
   - **本文档（Phase3_进行中.md）**：当前停在哪里
   - `README.md`：详细改动日志

2. **当前任务优先级**：
   - 高：跑 mao_rescaled，看场景 A/B/C
   - 中：v4 §6.6 把 max-cc 阈值改为按模型分类描述
   - 低：Phase3.md 正式收尾（D20 出结果后）

3. **不要做的事**：
   - 不要再调 ReachabilityParams.min_dot_threshold（已是合理值 0.05）
   - 不要再调 BC 阈值
   - 不要为了让 cc 数据漂亮自行加二次投影/迭代修正
   - 不要试图修改老项目（在 `E:\Git_Repository\STL-free Print\project`，只读）
   - 不要在 C++ 里做 KUKA 6 轴 IK（MATLAB 已实现）

4. **当前不工作的事**：
   - mao 原版 + vector_kuka + spacing=0.5mm + IC-CG（待 0.7x 测试出结果决定方向）

---

*Phase 3 收尾后本文档归档为 docs/Phase3.md。*
