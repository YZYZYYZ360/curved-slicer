# Phase 1 记录

> 阶段：**Phase 1 · 基线算法迁移（纯波前）**
> 依据：`docs/architecture/NEW_PROJECT_ARCHITECTURE_v3.md` §2.10、§6、§7、§8 Phase 1
> 老项目只读参考：`E:\Git_Repository\STL-free Print\project`，HEAD `2a60244`

## 启动前 sanity check

```text
1/4 Test #1: phase0_batch_test ................   Passed   14.37 sec
2/4 Test #2: config_loader_unit ...............   Passed    0.05 sec
3/4 Test #3: eigen_smoke ......................   Passed    0.02 sec
4/4 Test #4: sdf_smoke ........................   Passed    0.01 sec
100% tests passed, 0 tests failed out of 4
```

## 代码改动前 0.5mm 压力测试

命令：`build_nmake\phase0_io_voxel_benchmark.exe 0.5`

```text
MONITOR elapsed_sec=203.362 peak_rss_mb=88.820 timed_out=False
model,file_mb,triangles,new_stl_io_ms,old_style_stl_io_ms,phase0_voxel_ms,bbox_size_mm,grid,occupied
armadillo_flat,1.365,28620,23.770,19.436,2891.312,38.102x29.120x44.885,77x59x90,61178/408870
bunny,12.364,259298,212.469,178.382,28535.205,46.091x35.346x45.000,93x71x90,170682/594270
mao,38.078,798554,666.674,551.960,170194.269,42.652x52.943x70.981,86x106x142,568639/1294472
```

说明：实际 mao AABB 为约 `42.652x52.943x70.981mm`，`spacing=0.5mm` 对应 `86x106x142 = 1,294,472` 体素；不是 `850x1060x1420`。

## 本轮实现

- `src/io/gcode_writer.{h,cpp}` 移动为 `src/io/robot_writer.{h,cpp}`，并改名为 `RobotWriter` / `writePhase0PlaceholderKrl`。
- `src/geometry/curvature.{h,cpp}` 移动为 `src/metrics/curvature.{h,cpp}`。
- `src/app/run_batch.{h,cpp}` 移动为 `src/app/pipeline.{h,cpp}`。
- 新增 `src/field/wavefront.{h,cpp}`，实现 `solveWavefront`，`SFF.cpp:` 注释计数为 51。
- 新增 `src/field/laplacian.h` / `src/field/poisson.h`，Phase 2 stub 抛 `not_implemented`。
- 新增 `src/surface/iso_surface.{h,cpp}`、`src/surface/mc_lookup_table.h`。
- 新增 `src/geometry/bvh.{h,cpp}`，`buildSDF` 的最近距离查询改经 `TriangleBvh`。
- `VoxelGrid` 存储从 dense `vector<uint8_t>` 改为 sparse `unordered_set<size_t>`。
- 新增 `tests/wavefront_smoke_test.cpp` 与 `tests/phase1_wavefront_batch_report.cpp`。

## 验收证据

### ctest

```text
1/5 Test #1: phase0_batch_test ................   Passed   14.63 sec
2/5 Test #2: config_loader_unit ...............   Passed    0.01 sec
3/5 Test #3: eigen_smoke ......................   Passed    0.02 sec
4/5 Test #4: sdf_smoke ........................   Passed    0.01 sec
5/5 Test #5: wavefront_smoke ..................   Passed    0.01 sec
100% tests passed, 0 tests failed out of 5
```

### wavefront smoke

```text
wavefront_smoke vertices=16 triangles=18 components=1 iso=0.25
```

### 三模型 0.5mm wavefront ModelReport

命令：`build_nmake\phase1_wavefront_batch_report.exe 0.5`

```text
MONITOR elapsed_sec=213.491 peak_rss_mb=171.441 timed_out=False
Phase 0 Batch Report
success=3 failure=0 total_ms=213374.599
ModelReport name=armadillo_flat status=ok triangles=28620 read_ms=24.053 voxelize_ms=2924.612
  bbox=[(-19.051, -13.948, -22.191), (19.051, 15.172, 22.693)] size=(38.102, 29.120, 44.885)mm
  grid=77x59x90 occupied=61178/408870 ratio=0.1496
  wavefront_ms=76.347 iso_surface_ms=281.841 iso_vertices=5914 iso_triangles=11482 connected_components=1
ModelReport name=bunny status=ok triangles=259298 read_ms=214.423 voxelize_ms=29097.470
  bbox=[(123.097, 109.311, 15.000), (169.187, 144.657, 60.000)] size=(46.091, 35.346, 45.000)mm
  grid=93x71x90 occupied=170682/594270 ratio=0.2872
  wavefront_ms=218.826 iso_surface_ms=286.000 iso_vertices=3132 iso_triangles=6018 connected_components=1
ModelReport name=mao status=ok triangles=798554 read_ms=652.696 voxelize_ms=177661.084
  bbox=[(-21.267, -47.680, -17.597), (21.385, 5.263, 53.385)] size=(42.652, 52.943, 70.981)mm
  grid=86x106x142 occupied=568639/1294472 ratio=0.4393
  wavefront_ms=784.807 iso_surface_ms=1061.492 iso_vertices=25138 iso_triangles=49639 connected_components=1
```

PLY 输出：

```text
runs/phase1_wavefront/armadillo_flat/wavefront_mid_iso.ply
runs/phase1_wavefront/bunny/wavefront_mid_iso.ply
runs/phase1_wavefront/mao/wavefront_mid_iso.ply
```

metrics 输出：

```text
runs/phase1_wavefront/armadillo_flat/metrics.json
runs/phase1_wavefront/bunny/metrics.json
runs/phase1_wavefront/mao/metrics.json
```

## 遇到的问题 + 处理

1. 现象：`git mv` 失败，报 `.git/index.lock: Permission denied`。
   影响：无法通过 Git 记录 rename history；当前 Phase 0 代码也仍处于未跟踪状态。
   处理：用受控 `Move-Item` 完成路径对齐，并在本文档记录。若要真正保留 rename history，需要先修复 `.git` 写权限并把 Phase 0 代码纳入 Git 跟踪。

2. 现象：用户预估 mao `spacing=0.5mm` 会到约 1.3G 体素；实测为 `86x106x142`。
   影响：未触发 OOM，压力测试可直接完成。
   处理：仍将 `VoxelGrid` 改为 sparse occupied-set 存储，降低 dense vector 对大包围盒的风险。

3. 现象：老项目 `SFF.cpp` 函数实际起点为 `threadFun_first` 第 1060 行，用户点名的 1080 行位于其内部循环。
   影响：如果只按 1080 开始会丢失函数入口与边界检查语义。
   处理：`wavefront.cpp` 注释同时覆盖 1060 起点、1080-1431 点名范围、1432 后活动 next-seed 分支和 1583 函数结尾。

## 未完成项

- deferred to Phase 1.5 with reason: 尚未生成“老项目剥离补丁后版本”的 PLY，因此本轮只能确认新 baseline PLY 已落盘、M4 连通域为 1；新旧 PLY 肉眼对比截图尚缺老 baseline 参照。
- deferred to Phase 1.5 with reason: `extractIsoSurface` 当前是 cube cell 上的 tetra decomposition 实现，并已复制老项目 `MCLookUp_Table.h`；若必须完全复刻老项目 topological Marching Cubes 表驱动版本，需要继续迁移 `MarchingCubes.cpp` 的 table/ambiguity 逻辑。

---

## 后置审阅与修订（2026-04-27，方案 C）

Claude 对本阶段的实现做了二次审阅，发现 `src/field/wavefront.cpp` 名义上"迁移"老项目波前算法，**实际是 6-邻居 Dijkstra 测地距离重写**（51 条 SFF.cpp 注释全部是"X 已剥离 / X 改为 Y / X 收敛为 Y"的对照映射，没有一条是逐行复刻语义）。同时 `WavefrontParams` 多数字段（`isovalue_mode`、`height_interval_mm`、`serial_threshold`、`parallel_thread_num`）在实现里完全未被使用。

经用户决策采用**方案 C**：基线由老项目可执行体（`tests/benchmarks/old_project/`）提供，新仓库不再维护 wavefront 模块。详细的架构修订见 `docs/architecture/V3_TO_V4_AMENDMENTS.md`。

### 删除清单（已落地）

- `src/field/wavefront.h`
- `src/field/wavefront.cpp`
- `tests/wavefront_smoke_test.cpp`
- `tests/phase1_wavefront_batch_report.cpp`

### 编辑清单（已落地）

- `src/io/config_loader.h`：删除 `WavefrontParams` 引用、`algorithm.mode`、`algorithm.wavefront` 字段
- `src/io/config_loader.cpp`：删除 `[algorithm] mode` 与 `[algorithm.wavefront]` 解析块
- `src/app/pipeline.cpp`：删除 `if (mode == "wavefront")` 分支与 `toIsoExtractParams` / `writeMetricsJson` 辅助函数；保留 read + voxelize 主流程
- `src/app/pipeline.h`：删除 `ModelReport::wavefront_ms` 字段
- `config/batch_three_models.toml`：删除 `[algorithm]` 与 `[algorithm.wavefront]` 节
- `tests/config_loader_unit_test.cpp`：用 `algorithm.field.laplacian.tolerance` / `algorithm.field.poisson.max_iterations` 断言替换原 wavefront 断言
- `CMakeLists.txt`：从源列表删除 `wavefront.cpp`，删除 `wavefront_smoke_test` 与 `phase1_wavefront_batch_report` 目标

### 保留项

- `src/surface/mc_lookup_table.h`：迁移过来的 MC 查找表保留，**Phase 2 启动时把 `iso_surface.cpp` 从 tetra-decomposition 切换为表驱动 Marching Cubes**
- `src/field/laplacian.h` / `src/field/poisson.h`：Phase 2 stub，保留接口设计
- `src/geometry/bvh.{h,cpp}` / `src/geometry/voxel_grid` 的 sparse 存储改造：通用基础设施，Phase 2+ 受益
- `runs/phase1_wavefront/` 下已落盘的 PLY 文件保持原样作为历史记录（不在 git 跟踪范围）

### 后置 ctest 状态

修订后预期 ctest 4/4 通过（原 5/5 中减去 wavefront_smoke）。**待 Codex 在 Phase 2 启动前重跑 ctest 确认**。
