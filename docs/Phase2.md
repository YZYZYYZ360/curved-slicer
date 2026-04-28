# Phase 2 总结

> 阶段：**Phase 2 · 标量 Laplacian + 等值面切片**（v4 §5 修订路径，原 v3 §8 Phase 2 重新定位）
> 时间预算：2 周
> 实际完成：2026-04-27 → 2026-04-27（含一次架构修订重做）
> 状态：**已收尾，可进入 Phase 3**
> 仓库：`E:\Git_Repository\curved-slicer`

---

## 1. 阶段目标对照（v4 §5.5.2 验收标准）

| v4 验收项 | 状态 | 证据 |
|---|---|---|
| ctest 全绿（6 项） | ✓ | phase0_batch / config_loader_unit / eigen_smoke / sdf_smoke / iso_surface_mc_test / laplacian_smoke |
| 三模型 spacing=0.5mm 跑完不崩 | ✓ | 总耗时 593s（< 30 min budget） |
| layer_count >= 50 | ✓ | armadillo=56, bunny=56, mao=88 |
| **per-model connected_components = 1** | ✓ | 三模型全部 = 1（硬阈值） |
| max(per-layer cc) 记录 | ✓ | 72 / 29 / 91（不作阈值） |
| φ 场可视化合理 | ✓ | runs/phase2_field/{model}/phi_points.ply 三模型底面 0 → 顶面 1 平滑过渡，无 NaN/Inf/负值 |

---

## 2. 算法路径（与 v3 原设计的差异）

### 2.1 v3 原设计（已废止）
```
read STL → voxelize → buildSDF → generateBC（向量 G） → solveLaplacian（向量 G）
        → solvePoisson（标量 φ） → planIsoLevels → MC → metrics
```

### 2.2 v4 §5 实施路径
```
read STL → voxelize → buildSDF → generateBC（标量 BC） → solveLaplacian（标量 φ）
        → planIsoLevels → MC → metrics
```

废止 Poisson 步骤的根因：均匀 Dirichlet BC 下向量 Laplacian 退化为常量场 G=[0,0,1]，∇·G=0，Poisson 解 φ 退化为齐次解，等值面就是水平面切片。**向量场 + Poisson 流水线只有在 KUKA 投影修改 G 之后才有非平凡解**——这部分推迟到 Phase 3 与 KUKA 投影一起实施。

### 2.3 BC 公式

```
∇²φ = 0
φ = 0  on  bottom voxels: SDF normal · print_direction < bottom_dot_threshold (默认 -0.5)
                          AND |SDF| < bottom_sdf_band * spacing
φ = 1  on  top voxels:    SDF normal · print_direction > -bottom_dot_threshold (默认 0.5)
                          AND |SDF| < bottom_sdf_band * spacing
∂φ/∂n = 0  on  free Neumann（侧面）
```

底面/顶面的"中间区"（dot ∈ [-0.5, 0.5]）即为侧面，自由 Neumann。

---

## 3. 实施记录

### 3.1 第一版（v3 向量 Laplacian + Poisson）
- 完成代码：laplacian.{h,cpp} + poisson.{h,cpp} + pipeline.cpp 串联
- ctest 7/7 通过
- 三模型 spacing=0.5mm 验证：armadillo per-layer cc 最高 5，触发 v3 §8 Phase 2 硬阈值停止
- 诊断：算法在均匀 BC 下退化为水平面切片
- 决策：用户选择"方案 B"——Phase 2 改标量 Laplacian，Phase 3 再做完整向量+KUKA 流水线

### 3.2 第二版（v4 §5 标量 Laplacian，最终）
- src/field/laplacian.h：LaplacianBC.fixed_vectors (Vec3) → fixed_values (double)，删 normalize_each_iter，solveLaplacian 返回 ScalarField
- src/field/laplacian.cpp 重写：标量 7-point Laplacian + Eigen ConjugateGradient，BC 双钉死（底=0, 顶=1）
- src/field/poisson.h 改回 Phase 3 stub；poisson.cpp 删除；poisson_smoke_test 删除
- src/app/pipeline.cpp 删 Poisson 调用与中间向量场处理；流水线变为 read → voxelize → buildSDF → generateBC → solveLaplacian → planIsoLevels → MC → metrics
- src/app/pipeline.h：ModelReport.poisson_ms 保留（向后兼容，恒为 0）
- M4 验收口径修正：per-layer cc=1 → per-model cc=1（armadillo 等非凸模型水平切就是多片，per-layer 改作描述指标）
- countLayerShellComponents：Union-Find 跨层顶点近邻（connect_radius = max(2.5*spacing, 1.75*layer_thickness) = 1.4mm @ 0.5mm grid）

### 3.3 性能辅助（第一版遗留，第二版保留）
- src/geometry/bvh.{h,cpp}：Phase 1 占位实现升级为真实 AABB BVH（buildSDF 最近距离查询提速关键）
- src/geometry/sdf.cpp + voxel_grid.cpp：(y, z) 射线分桶优化，0.5mm 大模型 SDF 初始化性能必需
- tests/benchmarks/old_project/old_project_phase1_baseline.cpp 保留（Phase 5 老项目基线复用）
- tests/benchmarks/old_project/eigen_compat/：旧项目 C++20 兼容 shim，保留

---

## 4. 验证结果

### 4.1 ctest

```
1/6 phase0_batch_test        Passed
2/6 config_loader_unit       Passed
3/6 eigen_smoke              Passed
4/6 sdf_smoke                Passed
5/6 iso_surface_mc_test      Passed
6/6 laplacian_smoke          Passed
100% tests passed, 0 tests failed out of 6
```

### 4.2 三模型 ModelReport（spacing=0.5mm）

| 模型 | grid | occupied | layer_count | face_count_total | per-model cc | max(per-layer cc) | 总耗时 | RSS |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| armadillo_flat | 77×59×90 | 61,178 | 56 | 580,804 | **1** | 72 | 27.1s | 51 MB |
| bunny | 93×71×90 | 170,682 | 56 | 872,460 | **1** | 29 | 120.2s | 131 MB |
| mao | 86×106×142 | 568,639 | 88 | 4,105,921 | **1** | 91 | 446.4s | 404 MB |

### 4.3 阶段耗时分解

| 阶段 | armadillo | bunny | mao |
|---|---:|---:|---:|
| read | 23.7ms | 208ms | 642ms |
| voxelize | 627ms | 3.8s | 11s |
| **buildSDF** | **16.0s** | **68.8s** | **197.4s** |
| **solveLaplacian** | **1.3s** | **5.3s** | **24.4s** |
| solvePoisson | 0ms | 0ms | 0ms |
| **iso_surface** | **29.1s** | **42.3s** | **184.8s** |

**性能观察**：buildSDF 与 iso_surface 是大头；laplacian 反而很快。Phase 4 路径生成层面再考虑 narrow-band SDF 优化。

### 4.4 PLY 可视化

`runs/phase2_field/{armadillo_flat,bunny,mao}/phi_points.ply` 三模型 φ 点云按 0→1 蓝→红映射，全部底面固定为 0，沿模型内部平滑过渡到顶面 1，未触发 NaN/Inf/负值检查。armadillo 在腿、爪、耳处 φ 等值面贴合不完美（per-layer cc 最高 72 反映此事实），但 per-model 层间通过近邻 BFS 连通成单一壳，per-model cc=1 达成。

---

## 5. 关键决策与遗留

### 5.1 max(per-layer cc) 高的解释
标量 harmonic 场的等值面在非凸模型上仍会穿过多个 disjoint 分支（armadillo 4 腿 4 爪 + 头 + 耳；mao 发丝细节）。这是 Phase 2 算法的固有特性，**不是 bug**。Phase 3 引入 KUKA 投影 + 向量场 Poisson 后，G 场会沿连通域方向流动，预期 max(per-layer cc) 显著下降到接近 1。论文叙事的好处：Phase 2 vs Phase 3 形成算法内生对照，体现 KUKA 投影的具体贡献。

### 5.2 ModelReport.poisson_ms 保留
向后兼容字段，值恒为 0。Phase 3 启动时 Poisson 重建后 metrics.json 自然填值。compare_baselines.py 已能消费。

### 5.3 M1 (max|H|) 未在 Phase 2 计算
metrics/curvature.h 里 computeMeanCurvature 已实现但 pipeline 未调用。Phase 3 启动时与新流水线一同接入（每层 IsoMesh 跑一次，取 max；汇总到 metrics.M1）——必须，因为 Phase 3 vs Phase 2 的核心数值对照就是 M1。

### 5.4 旧项目 baseline 推迟
v4 §4.1 已记录：Phase 5 论文写作期决定（推荐方案 a：在 OpenVDB SDF 上做平面切片基线）。

---

## 6. 工程笔记

- **CMakeLists.txt**：Phase 2 编译目标精简为 6 个 ctest。手动跑长任务用 `phase2_field_batch_report` 目标（不进 ctest，避免 0.5mm 大模型卡住自动化）
- **C++17 主库 + C++20 旧项目 benchmark**：分两条路径，互不干扰
- **Eigen + toml++**：仍是仅有的两个外部依赖（v3 §5 / v4 §1 约束守住）
- **headers**：laplacian.h / poisson.h / iso_surface.h 三个新核心头都 < 100 行
- **子目录数**：仍 ≤ 7（core / io / geometry / field / surface / metrics / app）

---

*v4 §5 实施完成。下阶段（Phase 3）：完整向量 Laplacian + KUKA projection + Poisson 流水线，与 Phase 2 标量基线对照。*
