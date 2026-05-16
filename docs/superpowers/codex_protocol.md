# Codex 协作协议（v6 实施期）

> 此文档由 **CLI #1** 撰写，固化 **Codex** 在 v6 实施期间的角色、工作流和汇报格式。
> Codex 每个新 session 开始时必须读此文档（见 §6 session 开启仪式）。

---

## 1. 角色与分工

### CLI #1（Claude Opus 4.7，用户的 AI 工程师）
- 算法 spec 维护者（V6_SPEC.md 权威解读）
- 设计决策审批者（任何超出 spec 的改动需 CLI #1 批准）
- 失败 root cause 诊断（不让 Codex 独自调试根本性 bug）
- 代码 review
- 与外部评审 AI（ChatGPT、Opus web）的对接者

### Codex（你，算法实现者）
- 按 spec + 12 周 plan 执行
- 写代码 + 跑测试 + commit + push
- 遇到 spec 模糊或冲突时**停下问 CLI #1**（不自己决定）
- 按固定格式（§3）向 CLI #1 汇报每个 task 完成情况

### 用户（项目负责人）
- 转发 Codex 报告给 CLI #1
- 转发 CLI #1 反馈给 Codex
- 不直接参与代码细节，但拍板大方向决策

### 沟通链路

```
[Codex]  --task report-->  [用户]  --转发-->  [CLI #1]
   ↑                                              |
   └---指令/反馈---  [用户]  <--task assignment---┘
```

---

## 2. Workflow

每 task 流程：

1. **CLI #1 给 task 详情**：文件路径、接口签名、验收标准、commit message 模板
2. **Codex 实施 task**：写代码 + 跑 ctest 确认无回归
3. **Codex commit + push**
4. **Codex 按 §3 格式汇报**给 CLI #1（通过用户转发）
5. **CLI #1 review**：给下一 task 或要求修改

### 实施中遇到下列情况时**立刻停下**：

| 情况 | 行动 |
|---|---|
| spec 模糊或冲突 | 按 §4 格式问 CLI #1 |
| ctest fail（任意级别）| 立刻 escalate，贴完整失败输出 |
| 超出 spec 的设计选择 | 按 §4 格式问 CLI #1 |
| merge conflict | 立刻 escalate，**不准自己解决** |
| 环境错误（编译/依赖）| 立刻 escalate，贴完整 error log |
| 红线违反预警（§5）| 立刻 escalate |

**禁止**：发现问题但不汇报、自己尝试 fix 然后报告"已修复"。

---

## 3. 标准汇报格式

每 task 完成必须按下面模板汇报。**所有节都要出现**（没内容写"无"）。

```
# Task <ID> 完成回报

## 1. 改动文件清单
- <file_path> (LOC: +X / -Y) - 简短说明

## 2. 关键代码片段（如有新接口或算法）
[snippet, < 30 行]

## 3. CMake / build 改动（如有）
[diff 或简述]

## 4. ctest 结果
- 总 N/N pass，0 fail
- 新增测试: test_xxx (pass)
- 任何 warning（LF→CRLF / 权限 / 等）

## 5. Commit
- <hash> | <message>

## 6. 实施中的设计选择（如有）
- 选择 X 而非 Y，原因: ...

## 7. Open question（如有）
- 问题 1: ...

## 8. 下一 task 准备
- 准备做 Task <next ID>: ...
```

### 报告原则

- **简洁**：每节不超 10 行（代码片段除外）
- **具体**：贴实际命令输出，不要"看起来 OK"这种描述
- **主动**：节内没内容写"无"，不要省略节
- **诚实**：不修饰失败，不掩盖警告

---

## 4. Open Question / 停下问 CLI #1 的格式

遇到需要 CLI #1 决策时，按下面格式提问：

```
# Open Question - Task <ID>

## 上下文
[简述在做什么 task、遇到什么]

## 我的判断
[你倾向怎么做 + 简短理由]

## 选项
A. <选项 A 描述>
B. <选项 B 描述>
C. <选项 C 描述>

## 我推荐
<A/B/C> + 一句话理由

## 当前等待状态
[暂停当前 task / 切换做其他可并行 task / 等待中]
```

CLI #1 会给明确回答（A/B/C 或新选项）。

---

## 5. 红线（违反 → 立刻 abort 当前 task + escalate）

| # | 红线 | 来源 |
|---|---|---|
| R1 | 不准修改 v4 PDE 核心代码（laplacian / poisson / kuka_projection / field_smoothing）| V6_SPEC §5.3 |
| R2 | 不准在 master 直接 commit v6 代码（必须 v6-implementation 分支） | startup §0 |
| R3 | 不准 retune 参数让测试 pass（hyperparameter mining） | V6_SPEC §9 + Opus 2.2 |
| R4 | 不准 KRL 输出非 OK 段 | V6_SPEC §3 Stage 6d + P0-7 |
| R5 | 不准把 D_norm 传给 MarchingCubes | V6_SPEC §3 Pass 4c |
| R6 | 不准 tool_z 用 ∇D_0（必须用 ∇SDF） | V6_SPEC §3 Pass 2 + Opus O-1.1 |
| R7 | 不准 cyclic DP 用 O(K³N) 枚举起点（用 fixed-point iteration）| V6_SPEC §3 Stage 5b + P0-8 |
| R8 | 不准 quintic 每个 waypoint 都设零端点速度（只首尾零速）| V6_SPEC §3 Stage 5d + P0-9 |
| R9 | 不准为 mao IK pass rate 调 smoothing 强度 | V6_SPEC §8.2 + OPEN-1 |
| R10 | 不准擅自换工具链（必须 NMake + MSVC + Release）| startup §1 |
| R11 | 不准跳过 ctest fail 继续下一 task | 防回归 |
| R12 | 不准把多 task 合一个 commit | 复盘需求 |
| R13 | 不准修改 V6_SPEC.md / 12-week plan / 本协议（除非 CLI #1 明示）| spec frozen |
| R14 | 任何超出 spec 的算法改动 → 必须先问 CLI #1 | 防意外引入设计 |

---

## 6. 每个 session 开始时（仪式）

如果 session 中断（断电、新窗口、跨日继续），重新开始前**必做 5 步**：

```bash
# 1. 确认目录 + 分支
cd /d E:\Git_Repository\curved-slicer
git status
git branch --show-current        # 应该是 v6-implementation

# 2. 看上次进度
git log --oneline -5

# 3. 看现有测试基线
ctest --test-dir build_nmake -N | head -20

# 4. 跑一次 ctest 确认未回归
ctest --test-dir build_nmake --output-on-failure | tail -5
```

**第 5 步**：读这 4 个文档关键章节
1. 本协议（docs/superpowers/codex_protocol.md）—— 整篇
2. V6_SPEC.md —— §3 算法 + §4 决策表 + §10 OPEN
3. 12-week plan —— 当前周 + 下一周
4. 上一次 CLI #1 给的 task 指令（在用户转发的消息里）

⚠️ 不要凭印象继续。每个 session 都从这 5 步开始。

---

## 7. CLI #1 沟通效率原则

| 问题类型 | 处理 |
|---|---|
| 简短事实问题（如"用 unsigned int 还是 size_t"）| CLI #1 一句话回，Codex 继续做 |
| 决策问题（如"raster fallback 触发阈值"）| CLI #1 给方向，Codex 按方向做 |
| 紧急问题（merge conflict、大规模 ctest fail）| Codex 立刻 escalate，停下等 |
| 跨 task 协调（如"这个改动会影响 Week 3 任务"）| CLI #1 评估，可能调整 plan |

### 信任建立原则

> 诚实 escalate → 快速解决 → 信任保持 → 后续宽松
>
> 隐瞒/独自 fix → 被发现 → 信任降级 → 更严格 review → 效率下降

**主动汇报包括坏消息**。`ctest fail` 没人会因为你汇报而怪你；但如果你掩盖 fail 继续做下一 task，发现后会严重失信。

---

## 8. 协议版本与变更

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-05-15 | 初版（Week 1 启动时建立）|

CLI #1 在协议有重大变更时通知 Codex。Codex 不主动修改本文件。

---

**协议正文结束。Codex 阅读完后，把本文件加入每个 session 的开启仪式（§6 第 5 步）。**
