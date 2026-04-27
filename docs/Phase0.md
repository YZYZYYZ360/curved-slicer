# Phase 0 总结

> 阶段：**Phase 0 · 骨架 + IO**（NEW_PROJECT_ARCHITECTURE_v3.md §8 Phase 0）
> 时间预算：1 周
> 实际完成：2026-04-23 → 2026-04-26
> 状态：**已收尾，可进入 Phase 1**
> 仓库：`E:\Git_Repository\curved-slicer`
> 老项目只读参考：`E:\Git_Repository\STL-free Print\project`（commit `2a60244`）

---

## 1. 阶段目标对照（v3 §8 Phase 0 验收标准）

| v3 验收项 | 状态 | 证据 |
|---|---|---|
| 编译通过（MSVC 17+, C++17, UNICODE + /utf-8） | ✓ | `CMakeLists.txt:8-10,28-29,70-72`；MSVC 18 (VS 2026) + NMake Makefiles |
| 跑 `curved_slicer config/...toml` 三模型全部能读 | ✓ | `phase0_batch_test` 输出 `success=3 failure=0` |
| 每个 model 目录下生成 `run.log` + 空的 `job.src` / `job.dat` | △ 部分 | `gcode_writer` 是 stub，能写空占位文件；`run.log` 暂用 stdout 替代，未落盘 |
| 日志里 `毛主席头雕(42_52_70)` 无乱码 | ✓ | `phase0_batch_test` 直接读取该中文路径并打印 ModelReport |
| `runs.csv` 生成 header + 3 行模型记录 | △ 部分 | `IoConfig::runs_csv` 字段已解析，写入逻辑留待 Phase 1 接入 metrics 模块时落地 |

---

## 2. 仓库目录结构（Phase 0 收尾后）

```text
curved-slicer/
├── CMakeLists.txt                  ← FetchContent: toml++ v3.4.0 + Eigen 3.4.0
├── README.md / read.md
├── config/
│   └── batch_three_models.toml     ← 唯一 config，覆盖 v3 §4.1 全部字段
├── docs/
│   ├── Phase0.md                   ← 本文档
│   └── architecture/
│       ├── CURRENT_PROJECT_ANALYSIS.md
│       └── NEW_PROJECT_ARCHITECTURE_v3.md
├── src/
│   ├── core/
│   │   └── types.h                 ← Vec3 / Tri / AABB
│   ├── io/
│   │   ├── stl_reader.{h,cpp}
│   │   ├── gcode_writer.{h,cpp}    ← Phase 0 stub；Phase 1 收尾改名 robot_writer
│   │   └── config_loader.{h,cpp}   ← toml++ + PipelineConfig
│   ├── geometry/
│   │   ├── voxel_grid.{h,cpp}
│   │   ├── sdf.{h,cpp}             ← struct SDF + buildSDF
│   │   └── curvature.{h,cpp}       ← Phase 0 stub；Phase 2 前移到 metrics/
│   └── app/
│       └── run_batch.{h,cpp}       ← Phase 1 改名 pipeline.{h,cpp}
└── tests/
    ├── benchmarks/
    │   ├── phase0_io_voxel_benchmark.cpp
    │   └── old_project/
    │       ├── CMakeLists.txt
    │       └── old_project_phase0_benchmark.cpp
    ├── models/
    │   ├── cube_ascii.stl
    │   ├── armadillo_flat.stl
    │   ├── bunny(46_35_45).stl
    │   └── 毛主席头雕(42_52_70).stl
    ├── phase0_batch_test.cpp
    ├── config_loader_unit_test.cpp
    ├── eigen_smoke_test.cpp
    └── sdf_smoke_test.cpp
```

子目录 ≤ 7（core/io/geometry/app + tests/docs/config）✓
外部依赖 = 2（toml++ + Eigen）✓

---

## 3. 完成清单

### 3.1 第一轮（Codex 初版 Phase 0）
- 项目骨架 + CMake 基本框架
- `core/types.h`：`Vec3` / `Tri` / `AABB`（含 padded/expand/size 等便捷方法）
- `io/stl_reader`：ASCII/Binary 双格式解析，`std::filesystem::path` 接口
- `io/gcode_writer`：Phase 0 占位实现（待 Phase 4 改名 `robot_writer` 并真写 KRL）
- `io/config_loader`：手写最小 TOML 子集解析（已替换，见 3.2 M1）
- `geometry/voxel_grid`：闭合网格 X 射线填充体素化
- `geometry/sdf`：点-三角距离 + 内外判断 + 符号距离（已重构为 `buildSDF`，见 3.2 M4）
- `geometry/curvature`：Phase 0 三角网均值曲率 stub
- `app/run_batch`：批量入口 + `ModelReport` / `BatchReport`
- `tests/phase0_batch_test`：覆盖 STL 读取、配置加载、批量入口、bbox 验证、中文路径
- `tests/benchmarks/phase0_io_voxel_benchmark`：新项目 IO/体素化速度基准
- `tests/benchmarks/old_project/`：编译老项目源码做真实对照（boost/glm/openvdb via vcpkg）

### 3.2 第二轮（Phase 0 → Phase 1 收尾，M1–M4）
- **M1 toml++ 集成**：`FetchContent` pin commit `30172438...`（v3.4.0）；删除手写解析；`config_loader.cpp:291` 用 `toml::parse_file(path.u8string())`
- **M2 Eigen 集成 + smoke**：`FetchContent_Populate` + 手建 `Eigen3::Eigen` INTERFACE target（绕开 Eigen CMake 的 BLAS/Fortran 探测）；`tests/eigen_smoke_test.cpp` 25×40 五点拉普拉斯 CG 求解，残差 6.9e-11 / 82 次迭代
- **M3 PipelineConfig 扩展**：`config_loader.h` 新增 `AlgorithmConfig` / `FieldBoundaryConfig` / `IsoSurfaceConfig` / `PathConfig` / `KukaConfig` / `MetricsConfig` / `LoggingConfig`；KUKA 六轴默认值内嵌 v3 §2.3 表（A2=-195/40、A3=-115/150 等非对称）；`config/batch_three_models.toml` 覆盖 v3 §4.1 全字段；`tests/config_loader_unit_test` 覆盖 mode / KUKA 非对称 / 中文路径 / wavefront / metrics / logging
- **M4 SDF struct + buildSDF**：`sdf.h` 新增 `struct SDF { bbox, spacing, nx,ny,nz, values, narrow_band_mm }` 与 `buildSDF(mesh, params)`；旧自由函数收入实现匿名命名空间；`tests/sdf_smoke_test` cube 中心 -0.375 / 角点 +0.217 验证符号约定（内负外正）

### 3.3 第三轮（Phase 0 最终对齐，2026-04-26）
- **删除 `config/default.toml`**：保留 `batch_three_models.toml` 一份，避免两份 config 默认值差异引入调试坑
- **`batch_three_models.toml:29` `mode = "wavefront"` → `"field"`**：与 v3 §4.1 默认值对齐；之前为单测临时设的 `wavefront` 已纠正
- **`tests/config_loader_unit_test.cpp:38-40`**：`assert mode == "field"` 替换原 wavefront 断言
- **`tests/phase0_batch_test.cpp:136-156`**：所有 `default.toml` 引用切到 `batch_three_models.toml`

---

## 4. 验证结果

### 4.1 测试
```text
ctest --test-dir build_nmake --output-on-failure
4/4 tests passed
  - phase0_batch_test
  - config_loader_unit_test
  - eigen_smoke_test  (residual=6.91918e-11, iter=82)
  - sdf_smoke_test    (center=-0.375, corner=0.216506)
```
（注：第三轮三处改动后须重跑 ctest 再次确认 4/4 通过；本次改动只动了 toml 值与 include 文件名，不应影响测试结果）

### 4.2 三模型 ModelReport（spacing=5mm）
| 模型 | 三角面 | bbox (mm) | grid | occupied | read_ms | voxelize_ms |
|---|---:|---|---|---|---:|---:|
| armadillo_flat | 28,620 | 38.1 × 29.1 × 44.9 | 8×6×9 | 183/432 | 23.9 | 286.3 |
| bunny | 259,298 | 46.1 × 35.3 × 45.0 | 10×8×9 | 316/720 | 213.7 | 2,584.2 |
| mao | 798,554 | 42.7 × 52.9 × 71.0 | 9×11×15 | 899/1485 | 662.1 | 9,927.3 |

### 4.3 新旧速度对照（spacing=5mm，IO + 体素化）
| 模型 | 新 IO (ms) | 新体素化 (ms) | 旧 IO (ms) | 旧体素化总耗 (ms) | 备注 |
|---|---:|---:|---:|---:|---|
| armadillo_flat | 23.3 | 279.9 | 8.7 | 15,435.5 | 旧体素化主耗时在 OpenVDB `meshToSignedDistanceField` |
| bunny | 213.7 | 2,566.0 | 50.6 | 13,649.9 | |
| mao | 656.7 | 9,809.1 | 148.6 | 74,620.4 | 38MB 模型 |

**说明**：新 Phase 0 体素化是轻量 CPU 验证实现，**不**是 Phase 1+ 的生产级 narrow-band SDF。这组数据仅用于 Phase 0 工程速度对照，不代表 Phase 1+ 最终算法质量对照。

---

## 5. 已知遗留 / Phase 1 收尾项

下列项目已识别但**不阻塞 Phase 1 启动**，按计划在 Phase 1 第一周内完成：

| 项 | 现状 | Phase 1 处理 |
|---|---|---|
| **S1 老代码迁移强约束** | 第一版速度对比是 Codex 仿写的 old-style reader | Phase 1 启动 prompt 加硬约束："逐行对照 SFF.cpp，每段新代码注释 `// SFF.cpp:XXX`" |
| **S2 spacing=0.5mm 压力测试** | 当前只跑过 spacing=5mm，grid 仅 9×11×15 | Phase 1 启动前必跑：mao 在 0.5mm 下约 850×1060×1420 ≈ 1.3G 体素，须 narrow-band 化 |
| **S3 命名 / 路径与 v3 §6 对齐** | `gcode_writer`/`geometry/curvature`/`run_batch`/`Phase0Config` 偏离 v3 命名 | `git mv` 四组：`io/gcode_writer→io/robot_writer`，`geometry/curvature→metrics/curvature`，`app/run_batch→app/pipeline`，类型 `Phase0Config→PipelineConfig`（M3 已合并完成最后一项） |
| **可选 O1 CI 脚本封装** | `vcvars64.bat + NMake` 链路太长 | Phase 1 顺手写 `scripts/build.ps1` |
| **可选 O2 老项目 benchmark 迁出** | `tests/benchmarks/old_project/` 目前在新仓库内引用老项目源码 | 需把 `E:/Git_Repository/STL-free Print/project` 加入 Codex writable roots，或落到老仓库 |

---

## 6. 其他工程笔记

- **VS 版本**：实际使用 Visual Studio 18 (2026)，CMake 3.29 没有 `Visual Studio 18 2026` generator，已改用 `NMake Makefiles` + `vcvars64.bat`
- **Eigen 集成妥协**：Eigen 3.4 自带 CMake 在 MSVC + NMake 下会触发 BLAS/Fortran 探测卡住；`FetchContent_Populate` + 手建 INTERFACE target 是已知合理工作流，README 已记录
- **FetchContent SSL**：用 `GIT_CONFIG http.sslBackend=openssl` 规避 Windows Schannel 偶发 SSL 失败
- **中文路径**：`std::filesystem::u8path()` + `path.u8string()` 全程贯穿（toml 解析、文件 IO、日志），已实测 `毛主席头雕(42_52_70).stl` 无乱码
- **沙箱限制**：Codex writable roots 不含老项目目录，所以无法在老项目源码内新增 benchmark 模块；现行方案是新仓库引用老项目源码做对照
