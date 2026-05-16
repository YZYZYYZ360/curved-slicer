# CLI #2 启动指令（v6 实施）

> **复制下面整段给 CLI #2**。

```
CLI #2，v4 Plan 1 阶段结束。下面是 v6 实施的启动通知。

==========================================
## 1. v4 状态处置
==========================================

v4 Plan 1 你已完成 Task 19+20（pipeline 集成 + e2e）。
**v4 PDE 完整实现保留作论文 baseline M1，不再继续维护**。

Step 1: 锁定 v4 状态
  cd /d E:\Git_Repository\curved-slicer
  git status               # 应该 clean（如果有 uncommitted 修改先 commit）
  git log --oneline -5     # 记下当前 v4 最终 commit

Step 2: 切新分支
  git checkout master      # 或当前 v4 工作分支
  git checkout -b v6-implementation
  git push -u origin v6-implementation

v6 所有工作在 v6-implementation 分支上做。v4 PDE 代码保留不动。

==========================================
## 2. 必读文档（按顺序）
==========================================

1. docs/architecture/V6_SPEC.md       (v6.2 完整算法 spec, ~1500 行)
   - 算法 6 阶段详解
   - 关键设计决策表（31 项）
   - 数据 schema
   - 论文叙事 + 禁止 claim 清单

2. docs/superpowers/plans/2026-05-15-v6-implementation.md  (12 周实施计划)
   - 每周任务清单 + 验收 + fallback
   - 4 个新模块 + v4 复用映射
   - 实验矩阵设计

3. docs/architecture/V3_TO_V4_AMENDMENTS.md §10-§17  (上下文参考)
   - v4 PDE 算法逻辑（已废，但 baseline M1 复用）
   - §11 IK / §13 geodesic / §14 quintic 这三节 v6 直接复用

⚠️ v5 拓扑路由方案（曾讨论过的 v5 §22 嫁接）**完全作废**，不要复习。

==========================================
## 3. 关键设计认知（必须先理解）
==========================================

a. 算法是 **多约束加权 Dijkstra 距离场**，不是 PDE。v4 的 Laplacian+Poisson
   路线已废（curl 问题在 mao 上崩）。

b. **C1-C4 是可制造性指标，不是硬约束**。Scalar field 提供 soft bias，
   约束满足由 post-validation 判定。任何地方说"算法保证 collision-free"
   都是错的。

c. **mao_regularized 是重度 smoothing 后的近似模型**，不是原 mao。
   头发褶皱等 < 5mm 细节会被抹平。论文叙事要诚实报告 geometric loss。
   原 mao 仅作 deviation 对比，不打印。

d. **bunny 实机有 4 级 fallback**：full bunny → small bunny → rounded cube
   → robot dry-run only。不要押 bunny 一定能完整打。

e. **RSI 在线降级为 future work**。本项目主线是 KRL 离线导出。v4 §15b RSI
   spec 保留但不实施。

f. **Baseline M1 = v4 PDE**（用户决定）。不复现 Wang 2018/Li 2022/Jayakody 2024。
   论文用 capability table 做能力对比，不假装超越 SOTA。

==========================================
## 4. Pre-flight 检查（启动 Week 1 前必做）
==========================================

Step 1: 当前 ctest 基线确认
  cd /d E:\Git_Repository\curved-slicer
  cmake --build build_nmake
  ctest --test-dir build_nmake --output-on-failure
  期望：所有 v4 tests pass（10/10 或类似）
  贴结果给 CLI #1 确认

Step 2: 检查 v4 PDE 完整性
  ls runs/v4_mao_rescaled/mao_rescaled/      # 应有 trajectory.csv + ply + stl
  贴文件清单 + 文件大小给 CLI #1

Step 3: 工程环境确认
  cmake --version           # 期望 >= 3.20
  cl /?                     # MSVC 编译器
  vcpkg list | head -20     # 已装的库（Eigen, libigl 等）
  python --version          # 期望 3.10+
  贴输出给 CLI #1

如果 pre-flight 通过 → 进 Week 1。
如果 ctest 有 fail → 停下回报，不进 v6。

==========================================
## 5. Week 1 任务清单（从 V6_SPEC.md + plan 摘出）
==========================================

Week 1 目标：工程骨架 + v4 复用包装 + 坐标系审计

任务（按顺序）：
- W1-T1 (0.5d): CMake 工程 + 三 CLI 空壳（curved_slicer / toolpath_validate / export_krl）
- W1-T2 (1.5d): v4 §11 KR4 模块包装 → src/robot/{KR4Model, AnalyticIK, Jacobian}.cpp
                关键改造：IK 一次返回 8 branches
- W1-T3 (0.5d): v4 §13 GeodesicContourPath 接口包装
- W1-T4 (1d):   v4 §14 TimeParameterization + CSVWriter + KRLWriter 包装
- W1-T5 (0.5d): Interface Audit 文档 → docs/v4_v6_interface_audit.md
                量化 v4 每模块与 v6 需求的 gap
- W1-T6 (0.5d): 坐标系审计文档 → docs/frames.md
                列出 STL/voxel/base/TCP/tool_z/KRL frame 关系 + diagram

Week 1 完成标准：
- 三 CLI 空壳可运行
- v4 模块封装通过 round-trip 测试
- Interface Audit 给出每模块 [直接复用/小改写/中改写/重写] 标签
- Frames 文档评审通过

Week 1 commit 节奏：
- 一 task 一 commit（per V6_SPEC.md 执行准则 #8）
- 不准把多 task 打包

⚠️ Week 1 红线（违反任一项 → 停下问 CLI #1）：
- 没做 interface audit 就开始写新代码
- 没做 frames 文档就改 IK 或 transform 逻辑
- 把 v4 PDE 代码删除或大改（必须保留作 M1 baseline）

==========================================
## 6. 报告节奏
==========================================

每 1-2 task 完成停下回报。回报格式：
  1. 改动文件清单 + LOC
  2. ctest 全跑结果
  3. commit hash + message
  4. 实施中遇到的 open question（如有）
  5. 超出指示的算法改动（必须 in advance 问 CLI #1，不准默默写完才提）

Week 1 末做 Week-end review：
  - 所有 W1 任务状态
  - Interface audit 结果
  - 是否可以进 Week 2

==========================================
## 7. 红线（重申 V6_SPEC.md 中的）
==========================================

a. 不准 retune 参数让数字好看（hyperparameter mining）
b. 不准在 KRL 里输出非 OK 段（status != OK 的段不导出 KRL）
c. 不准在 mao 上 over-smooth 后宣称"还像 mao"
d. 不准把 SDF clearance 写成 FCL collision-free
e. 不准把 D_norm（归一化场）传给 MarchingCubes
f. 不准用 ∇D_0 算 tool_z（要用 ∇SDF，见 Opus O-1.1 修正）
g. 不准把 cyclic DP 做成 O(K³N) 枚举起点（要用 fixed-point iteration）
h. 不准把 quintic 每个 waypoint 都设零端点速度（只首尾零速）
i. 不准为 mao IK pass rate 调 smoothing 强度（避免 hyperparameter mining）
j. 任何"超出 spec 的算法改动"必须先报 CLI #1，不准默默实施

==========================================
启动 Pre-flight。通过后进 Week 1。
```
