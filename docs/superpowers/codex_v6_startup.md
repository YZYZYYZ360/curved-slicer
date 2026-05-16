# Codex v6 启动指令

> **执行者**：Codex（新接手，不熟悉项目历史）
> **CLI #1**：审核 + 决策（你的对接人）
> **任务总线**：实施 v6.2 曲面层切片算法 + 12 周 plan
> **首要原则**：所有项目历史决策在 V6_SPEC.md / plan 里。不要凭印象做事。

---

## 0. 你的身份与项目上下文

你是新接手 `E:\Git_Repository\curved-slicer` 项目的执行者，负责按 v6.2 spec 实施完整算法 + 工程 pipeline。

项目历史（简短）：
- **v4 阶段（已完成）**：基于 PDE（Laplacian + Poisson）的曲面切片算法，已实现，保留在 master 分支
- **v5 阶段（讨论但作废）**：拓扑路由方案，不要查阅
- **v6 阶段（你要实施）**：基于加权 Dijkstra 距离场的新算法，在新分支 v6-implementation 上做

**核心规则**：
1. v4 PDE 完整代码作论文 baseline M1 保留。**你不修改 v4 任何核心算法代码**
2. v6 算法完全新写，但**复用 v4 的工具模块**（如解析 IK、测地路径、五次多项式平滑）
3. 所有设计决策已写入 docs/architecture/V6_SPEC.md，**不要重新设计**
4. 12 周计划写入 docs/superpowers/plans/2026-05-15-v6-implementation.md，**不要打乱节奏**

---

## 1. 环境规格（硬约束，不准切换）

| 项 | 规格 |
|---|---|
| 操作系统 | Windows 10/11 |
| 编译器 | MSVC (cl.exe) via Visual Studio 2022 |
| 构建系统 | NMake Makefiles（**不是 Ninja，不是 Make**）|
| 构建目录 | `build_nmake`（**不是 `build`**）|
| Build Type | Release（**不是 Debug**）|
| C++ 标准 | C++17 |
| 依赖管理 | vcpkg（manifest mode）|
| 测试框架 | 项目自带 unit test 风格（参考 `tests/laplacian_smoke_test.cpp`）|
| Python（脚本用）| 3.10+ |
| Git 工作流 | 每 task 独立 commit，每 1-2 task push 一次 |

⚠️ **不准擅自换工具链**（如换 Ninja 加速 CI、换 gcc/clang、换 Debug）。环境与现有 v4 代码兼容性已验证。

---

## 2. 必读文档（按顺序）

| # | 文件 | 用途 |
|---|---|---|
| 1 | `docs/architecture/V6_SPEC.md` | v6.2 完整算法 spec，含 6 阶段 + 31 项决策 + schema + 论文叙事，约 1500 行，**核心权威文档** |
| 2 | `docs/superpowers/plans/2026-05-15-v6-implementation.md` | 12 周实施计划，每周任务 + 验收 + fallback |
| 3 | `docs/algorithm_v6_teaching.html` | 算法教学课件，全中文 + SVG 图，推导每个场和惩罚项的物理意义。**强烈建议浏览器打开看** |
| 4 | `docs/architecture/V3_TO_V4_AMENDMENTS.md` §10-§17 | v4 PDE 算法历史（已废，但理解 baseline + 复用模块需要）|

读完后必须能用自己的话回答：
- v6.2 算法 6 个阶段分别做什么
- 4 个惩罚项（p_clear / p_orient / p_curv / p_ik）每个的物理动机
- p_ik 为什么需要 branch consistency 项
- 为什么用加权 Dijkstra 而不是 Poisson PDE
- v4 哪些模块在 v6 里直接复用、哪些重写

---

## 3. 第一阶段：Project Survey（Day 0，写代码前必做）

在写任何 `.cpp/.h` 代码之前，必须完成 Project Survey 并输出报告。

### 任务清单

```bash
# A. 看仓库历史
cd /d E:\Git_Repository\curved-slicer
git log --oneline -30

# B. 列源文件
find src/ -name '*.cpp' -o -name '*.h'

# C. 列现有测试
ctest --test-dir build_nmake -N

# D. 跑测试确认基线
cmake --build build_nmake
ctest --test-dir build_nmake --output-on-failure
```

### 输出 Project Familiarity Report

报告必须包含以下 6 节，**不超过 800 字**：

1. **v4 已实现模块清单**：按目录列出，每个文件一句话说明它干什么
2. **v6.2 新模块清单**：对比 V6_SPEC §5.2，列出 v6 要新建的模块
3. **v4 复用映射**：每个 v4 模块标 `[直接复用 / 小改写 / 中改写 / 重写 / 不动]`
4. **Week 1 Day 1 计划**：你打算先动哪个文件，为什么
5. **环境确认**：贴 `cmake --version` + `cl /?` + `python --version` 输出
6. **疑问清单**：不超过 5 条具体技术疑问（不要问"这个算法对不对"这种太大的问题）

⚠️ **报告等 CLI #1 review 通过后才能进 Week 1**
⚠️ **不准在 Project Survey 阶段写任何 .cpp/.h 代码**
⚠️ **可以读代码、可以跑 ctest、可以画 frame 图，但不准修改任何源文件**

---

## 4. v4 代码复用映射（不要重写！）

下面是 v4 已实现且 v6 必须复用的模块。**不要从头重写**，只能加 wrapper 或增量扩展。

| v4 文件 | v6 处理方式 | 改动量 |
|---|---|---|
| `src/kinematics/cart_pose.h` | 直接 include | 不动 |
| `src/kinematics/dh_params.h` | 直接 include | 不动 |
| `src/kinematics/forward_kin.{h,cpp}` | wrap 为 `KR4Model` 类，方法签名不动 | 不动源文件 |
| `src/kinematics/ik_solver.{h,cpp}` | wrap 为 `AnalyticIK`；**新增**"一次返回 8 branches"接口 | 增量扩展 |
| `src/kinematics/reachability.{h,cpp}` | 部分复用 manipulability / joint_limit / workspace 计算 | 小改写 |
| `src/path/geodesic_paths.{h,cpp}` | wrap 为 `GeodesicContourPath`；polyline stitching 已修复直接用 | 小改写 |
| `src/path/pose_from_path.{h,cpp}` | 直接复用 `constructToolFrame` 等 | 不动 |
| `src/path/path_postprocessing.{h,cpp}` | 直接复用 wlsFilter + LinearSmooth | 不动 |
| `src/trajectory/poly5_smoother.{h,cpp}` | 扩展为含 chord-length finite difference 速度估计；时间参数化拆两阶段 | 中改写 |
| `src/geometry/voxel_grid.{h,cpp}` | 直接复用 | 不动 |
| `src/geometry/sdf.{h,cpp}` | 直接复用 | 不动 |
| `src/geometry/bvh.{h,cpp}` | 直接复用 | 不动 |
| `src/io/stl_reader.{h,cpp}` | 直接复用 | 不动 |
| `src/io/config_loader.{h,cpp}` | 扩展为读 v6 配置（保留 v4 字段不删）| 小改写 |
| **`src/field/laplacian.{h,cpp}`** | **v4 PDE，不动**，仅作 baseline M1 | **不动！** |
| **`src/field/poisson.{h,cpp}`** | **v4 PDE，不动**，仅作 baseline M1 | **不动！** |
| **`src/field/kuka_projection.{h,cpp}`** | **v4，不动**，仅作 baseline M1 | **不动！** |
| **`src/field/field_smoothing.{h,cpp}`** | **v4，不动** | **不动！** |
| `src/surface/iso_surface.{h,cpp}` | wrap 为 v6 MC layer 提取器，可能小改 | 小改写 |
| `src/app/pipeline.cpp` | **新增** `vector_kuka_v6` 分支，**保留** `vector_kuka_v4`、`scalar` 等分支 | 增量扩展 |

⚠️ **红线**：任何 commit 修改 v4 PDE 核心（laplacian / poisson / kuka_projection / field_smoothing）必须**先问 CLI #1**。

---

## 5. 报告节奏

| 阶段 | 节奏 |
|---|---|
| Project Survey | 一次性输出 Report |
| Week 1-2 | **每 task 完成立刻报告** |
| Week 3-9 | 1-2 task 一报告 |
| Week 10-12 | 每周报告 + 异常立即报告 |

### 报告格式（固定模板）

```
# Task X.Y 完成回报

## 1. 改动文件清单
- 文件路径 (LOC: +X / -Y)

## 2. ctest 结果
- 总 N/N pass，0 fail
- 新增测试: test_xxx (pass)

## 3. Commit
- hash | message

## 4. 实施中的设计选择（如有）
- 选择 X 而非 Y，原因: ...

## 5. Open question（如有）
- 问题 1: ...

## 6. 下一 task 准备
- Task X.Y+1: ...
```

⚠️ **任何超出 spec 的设计选择必须先问 CLI #1**，不准默默写完才提
⚠️ **测试失败必须落盘数据**，不是只说"失败了"
⚠️ **跳过失败的测试 = 红线违反**

---

## 6. 红线清单（违反任一项立即停下问 CLI #1）

| # | 红线 | 理由 |
|---|---|---|
| R1 | 不准修改 v4 PDE 核心代码（laplacian / poisson / kuka_projection / field_smoothing） | 这些是 baseline M1，论文用 |
| R2 | 不准在 master 分支提交 v6 代码 | 必须在 v6-implementation 分支 |
| R3 | 不准 retune 参数让测试 pass（hyperparameter mining） | 之前 CLI #2 在 v4 上 gaming 过 IK 阈值，被 CLI #1 严厉警告 |
| R4 | 不准在 KRL 输出非 OK 段 | 真打会撞机 |
| R5 | 不准把 D_norm（归一化场）传给 MarchingCubes | 单位语义错误 |
| R6 | 不准用 ∇D_0 算 tool_z | 要用 ∇SDF，见 V6_SPEC §3 Pass 2 + Opus O-1.1 修正 |
| R7 | 不准把 cyclic DP 做成 O(K³N) 枚举起点 | 要用 fixed-point iteration，见 V6_SPEC §3 Stage 5b |
| R8 | 不准让 quintic 在每个 waypoint 都设零端点速度 | 只首尾零速，否则机器人每 0.2mm 停一次 |
| R9 | 不准为 mao IK pass rate 调 smoothing 强度 | hyperparameter mining 红线 |
| R10 | 不准擅自换构建工具链 | 必须 NMake + MSVC + Release |
| R11 | 不准跳过 ctest 失败继续做下一 task | 防回归 |
| R12 | 不准把多 task 打包成一个 commit | 复盘和回滚都需要细粒度 |
| R13 | 不准修改 V6_SPEC.md / plan（除非 CLI #1 明确同意） | spec 是 frozen 的 |
| R14 | 任何"超出 spec 的算法改动"必须先报 CLI #1 | 防止意外引入新设计选择 |

---

## 7. 启动顺序（Day 0）

按下面顺序执行，不要跳步：

```bash
# Step 0a: 切到项目目录
cd /d E:\Git_Repository\curved-slicer

# Step 0b: 切到 v6 实施分支
git checkout master
git pull origin master
git checkout -b v6-implementation
git push -u origin v6-implementation

# Step 0c: Pre-flight ctest
cmake --build build_nmake
ctest --test-dir build_nmake --output-on-failure
# 期望：所有 v4 tests pass（约 10-20 项）
# 贴结果给 CLI #1
```

```
# Step 0d: 读 4 个必读文档
# 浏览器打开 docs/algorithm_v6_teaching.html
# 阅读器/IDE 打开其他 .md 文件

# Step 0e: 做 Project Survey，输出 Project Familiarity Report

# Step 0f: 等 CLI #1 review Report 通过
```

---

## 8. 通过 Project Survey 后

CLI #1 review Report 通过后，会给你 **Week 1 Day 1 详细任务清单**，包含：
- 第一个文件具体写什么（含接口签名）
- 验收 ctest 写法
- 与 v4 的复用关系
- 期望的 commit message

那时候才开始动手写代码。

---

## 不要做的事（再次强调）

- ❌ 不要在没读 V6_SPEC.md 前就开始动手
- ❌ 不要凭印象重写已经存在的 v4 代码
- ❌ 不要修改 V6_SPEC.md / 12-week plan
- ❌ 不要 retune 参数
- ❌ 不要把多 task 合一个 commit
- ❌ 不要跳过失败测试
- ❌ 不要直接进 Week 1，必须先过 Project Survey

---

**第一步：执行 Step 0a-0c 并贴 pre-flight 结果给 CLI #1**
**第二步：读 4 个必读文档**
**第三步：输出 Project Familiarity Report**
