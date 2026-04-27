# curved-slicer

新仓库路径：`E:\Git_Repository\curved-slicer`

旧项目只读参考路径：`E:\Git_Repository\STL-free Print\project`

架构基准：
- `docs\architecture\NEW_PROJECT_ARCHITECTURE_v3.md`
- `docs\architecture\CURRENT_PROJECT_ANALYSIS.md`

旧项目参考 commit：`2a60244`

## 开发环境

- Windows
- Visual Studio 2026
- MSVC
- CMake
- C++17

注意：Claude 的 Phase 0 文案中提到 Visual Studio 2022，但本机实际环境按 Visual Studio 2026 记录和实施。

## Phase 0 范围

Phase 0 目标：建立新项目骨架、IO、体素化、批量入口，验证测试模型能读入、体素化，并打印 `ModelReport`。

Phase 0 不涉及任何切片算法，不实现 Phase 1+ 内容。

本阶段实现：

```text
CurvedSlicer/
├── CMakeLists.txt
├── config/
│   ├── batch_three_models.toml
│   └── default.toml
├── src/
│   ├── core/
│   │   └── types.h
│   ├── io/
│   │   ├── stl_reader.h
│   │   ├── stl_reader.cpp
│   │   ├── gcode_writer.h
│   │   ├── gcode_writer.cpp
│   │   ├── config_loader.h
│   │   └── config_loader.cpp
│   ├── geometry/
│   │   ├── voxel_grid.h
│   │   ├── voxel_grid.cpp
│   │   ├── sdf.h
│   │   ├── sdf.cpp
│   │   ├── curvature.h
│   │   └── curvature.cpp
│   └── app/
│       ├── run_batch.h
│       └── run_batch.cpp
└── tests/
    ├── benchmarks/
    │   └── phase0_io_voxel_benchmark.cpp
    │   └── old_project/
    │       ├── CMakeLists.txt
    │       └── old_project_phase0_benchmark.cpp
    ├── config_loader_unit_test.cpp
    ├── eigen_smoke_test.cpp
    ├── phase0_batch_test.cpp
    ├── sdf_smoke_test.cpp
    └── models/
        ├── cube_ascii.stl
        ├── armadillo_flat.stl
        ├── bunny(46_35_45).stl
        └── 毛主席头雕(42_52_70).stl
```

## 当前项目结构（同步给 Claude）

截至 Phase 0 配置/SDF/Eigen smoke 更新后：

```text
curved-slicer/
├── .gitignore
├── CMakeLists.txt
├── config/
│   ├── batch_three_models.toml
│   └── default.toml
├── docs/
│   └── architecture/
│       ├── CURRENT_PROJECT_ANALYSIS.md
│       └── NEW_PROJECT_ARCHITECTURE_v3.md
├── read.md
├── src/
│   ├── app/
│   │   ├── run_batch.cpp
│   │   └── run_batch.h
│   ├── core/
│   │   └── types.h
│   ├── geometry/
│   │   ├── curvature.cpp
│   │   ├── curvature.h
│   │   ├── sdf.cpp
│   │   ├── sdf.h
│   │   ├── voxel_grid.cpp
│   │   └── voxel_grid.h
│   └── io/
│       ├── config_loader.cpp
│       ├── config_loader.h
│       ├── gcode_writer.cpp
│       ├── gcode_writer.h
│       ├── stl_reader.cpp
│       └── stl_reader.h
└── tests/
    ├── benchmarks/
    │   └── phase0_io_voxel_benchmark.cpp
    │   └── old_project/
    │       ├── CMakeLists.txt
    │       └── old_project_phase0_benchmark.cpp
    ├── models/
    │   ├── cube_ascii.stl
    │   ├── armadillo_flat.stl
    │   ├── bunny(46_35_45).stl
    │   └── 毛主席头雕(42_52_70).stl
    ├── config_loader_unit_test.cpp
    ├── eigen_smoke_test.cpp
    ├── phase0_batch_test.cpp
    └── sdf_smoke_test.cpp
```

## Phase 0 实施说明

- `stl_reader`：参考老项目 `basicDataType/STLReader.cpp/.h` 的职责，重新实现 ASCII/Binary STL 读取；接口使用 `std::filesystem::path`，为 Windows 中文路径保留路径对象。
- `voxel_grid`：实现 Phase 0 基础闭合网格体素化，采用 X 方向射线填充实体内部，并额外标记三角面附近的表面体素。
- `sdf`：公开 `SDF` 与 `buildSDF(const TriangleMesh&, const VoxelParams&)`；点到三角形距离、闭合网格内外判断和符号距离函数已收进 `sdf.cpp` 内部命名空间；不引入 OpenVDB。
- `config_loader`：已切到 `toml++ v3.4.0`，用 `toml::parse_file(path.u8string())` 解析；配置结构从 `Phase0Config` 重命名为 `PipelineConfig`，并提前解析 v3 §4.1 的 Phase 1+ 字段，Phase 0 batch 当前只消费 `io` 与 `voxel`。
- `run_batch`：批量读取模型、体素化，并生成/打印 `ModelReport`；现在输出 bbox min/max 和 bbox size。
- `config\batch_three_models.toml`：唯一配置文件，按 `NEW_PROJECT_ARCHITECTURE_v3.md` §4.1 补全字段，路径改为新仓库本地测试模型；`algorithm.mode` 默认为 `"field"`。
- `tests\phase0_batch_test.cpp`：覆盖 STL 读取、临时配置加载、仓库 `config/default.toml`、批量入口、体素化报告输出；额外验证 bunny AABB 接近 `46x35x45mm`，毛主席头雕 AABB 接近 `42x52x70mm`，并直接读取中文路径 `tests\models\毛主席头雕(42_52_70).stl`。
- `tests\config_loader_unit_test.cpp`：验证 `config/batch_three_models.toml` 完整解析不报错，并断言 `algorithm.mode == "wavefront"`、`kuka.limits.a2.min_deg == -195.0`、`kuka.limits.a3.max_deg == 150.0`。
- `tests\eigen_smoke_test.cpp`：使用 `Eigen3::Eigen` 构建 `1000x1000` 五点差分稀疏矩阵，`ConjugateGradient` 求解 `Delta u = 1`，验证相对残差 `< 1e-6`。
- `tests\sdf_smoke_test.cpp`：对 `cube_ascii.stl` 构建 SDF，验证中心体素为负、角落体素为正。
- `tests\benchmarks\phase0_io_voxel_benchmark.cpp`：Phase 0 IO/体素化速度测试。新 STL IO 走当前 `cslc::readStl`；旧 STL IO 对比为测试内实现的 old-style reader，按老项目 `basicDataType/STLReader` 的 binary/ascii pull pattern 建模，不修改也不链接老项目。
- `tests\benchmarks\old_project\old_project_phase0_benchmark.cpp`：真实旧项目 benchmark，编译旧项目 `basicDataType/STLReader.cpp`，并调用旧项目 `StlToSDF.h` 的 OpenVDB SDF 初始化与旧体素查询循环。该模块放在新仓库中引用旧项目源码，不修改旧项目。

## 修改中遇到的问题 / 给 Claude 的反馈

1. 原 `read.md` 首行包含 PowerShell here-string 标记 `@'`，且 Phase 0 目录代码块没有闭合。本次已整理为正常 Markdown，避免后续沟通时结构信息不完整。
2. Phase 0 清单没有列 `src/core/types.h`，但 `NEW_PROJECT_ARCHITECTURE_v3.md` 的后续接口明确依赖 `core/types.h`。本次只添加最小公共类型 `Vec3`、`Tri`、`AABB`，不引入 Phase 1+ 内容。
3. 架构文档提到第三方依赖 Eigen + toml++。本次已在顶层 `CMakeLists.txt` 用 `FetchContent` 接入：toml++ pin `30172438cee64926dc41fdd9c11fb3ba5b2ba9de`，Eigen pin `3147391d946bb4b6c68edd901f2add6ac1f31f8c`。
4. Phase 0 的体素化是轻量 CPU 实现，用于验证 IO 和批量入口，不等价于后续 Phase 1/2 的生产级 SDF 或 narrow-band 体素化。大模型若超过 50,000,000 个体素会主动报错，避免误把 Phase 0 简化实现当成最终算法。
5. 本机 CMake 3.29 的生成器列表没有 `Visual Studio 18 2026`，且 `Visual Studio 17 2022` 生成器找不到 VS 实例。实际可用路径为 `C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat`，通过 `vcvars64.bat` + `NMake Makefiles` 可以成功配置、编译和测试。`Ninja` 在 CMake ABI 检测阶段卡住，暂不作为推荐命令。
6. `phase0_batch_test` 已切到三个真实模型，中文文件名 `毛主席头雕(42_52_70).stl` 在 Windows + MSVC 下可以通过 `std::filesystem::u8path` 正常读取。
7. bunny 实测 AABB size 为 `46.091x35.346x45.000mm`，符合 `46x35x45mm`；mao 实测 AABB size 为 `42.652x52.943x70.981mm`，符合 `42x52x70mm`。
8. 第一版速度对比只做了新 STL IO vs old-style STL IO，没有调用真实旧项目体素化；原因是旧项目完整体素化依赖 OpenVDB + `SFF`/`StlToSDF`，直接塞进 Phase 0 主测试会引入旧项目依赖和长耗时。现在已新增独立 benchmark `tests\benchmarks\old_project`，真实编译旧项目 `STLReader.cpp` 与 `StlToSDF.h` 做对比。
9. 之前项目记录文件名沿用了仓库原有 `read.md`，不是标准 `README.md`，所以按 README 查找时会看不到速度表。本次同步生成标准 `README.md`，后续给 Claude 的项目记录以 `README.md` 为主，`read.md` 保留为历史兼容副本。
10. 已尝试按用户授权在旧项目 `E:\Git_Repository\STL-free Print\project` 新增独立 benchmark 模块，但旧项目不在当前 Codex writable roots 内，写入需要沙箱升级；升级审批连续两次自动超时，因此最终改为在新仓库 `tests\benchmarks\old_project` 中引用旧项目源码。若必须把测试模块落在旧项目目录内，需要明确允许本会话写入旧项目路径，或把 `E:\Git_Repository\STL-free Print\project` 加入 writable roots。
11. 旧项目 benchmark 编译需要 `D:\vcpkg` 中已安装的 `boost-headers`、`glm`、`openvdb`，本机已满足并编译通过。编译时旧项目 `TEACDef.h` / `TEACUtil.h` 有大量 C4828 编码警告，但不是错误。
12. FetchContent 首次拉取 toml++ 时 Git 默认走 Windows schannel，失败为 `SEC_E_NO_CREDENTIALS`；已在两个 FetchContent 声明里加入 `GIT_CONFIG http.sslBackend=openssl`。这是本机 Git/证书后端问题，不影响代码逻辑。
13. toml++ 拉取后默认 submodule update 会调用 Git 自带 `sh.exe`，在当前 Codex/Windows 沙箱里报 `couldn't create signal pipe, Win32 error 5`；toml++/Eigen 当前用法不需要子模块，已设置 `GIT_SUBMODULES ""` 跳过。
14. Eigen 3.4.0 仓库自身 CMake 会进入测试/BLAS/Fortran 探测，和本机 MSVC + NMake 不匹配。为满足“FetchContent 拉取 Eigen 并提供 `Eigen3::Eigen` target”的验收，本次只 FetchContent 拉取源码，然后手动创建 header-only `Eigen3::Eigen` interface target，避免引入 Eigen 自带构建树。

## Phase 0 验证命令

```powershell
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake -S . -B build_nmake -G "NMake Makefiles" --fresh'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build_nmake'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && ctest --test-dir build_nmake --output-on-failure'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && ctest --test-dir build_nmake -R config_loader_unit --output-on-failure'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && ctest --test-dir build_nmake -R eigen_smoke --output-on-failure'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && ctest --test-dir build_nmake -R sdf_smoke --output-on-failure'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && build_nmake\phase0_batch_test.exe'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && build_nmake\phase0_io_voxel_benchmark.exe 5.0'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake -S tests\benchmarks\old_project -B build_old_project_benchmark -G "NMake Makefiles" --fresh -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows -DOLD_PROJECT_ROOT="E:/Git_Repository/STL-free Print/project"'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build_old_project_benchmark'
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && build_old_project_benchmark\old_project_phase0_benchmark.exe "E:\Git_Repository\curved-slicer\tests\models" 5.0'
```

## Phase 0 验证结果

最后一次完整 `ctest` 验证结果：

```text
1/4 Test #1: phase0_batch_test ................   Passed   14.01 sec
2/4 Test #2: config_loader_unit ...............   Passed    0.02 sec
3/4 Test #3: eigen_smoke ......................   Passed    0.03 sec
4/4 Test #4: sdf_smoke ........................   Passed    0.03 sec
100% tests passed, 0 tests failed out of 4
```

单项验收补充：

```text
ctest -R config_loader_unit: Passed
ctest -R eigen_smoke: Passed
ctest -R sdf_smoke: Passed
eigen_smoke residual=6.91918e-11 iterations=82
sdf_smoke center=-0.375 corner=0.216506
```

手动运行 `build_nmake\phase0_batch_test.exe` 输出的 `ModelReport`：

```text
Phase 0 Batch Report
success=1 failure=0 total_ms=0.651
ModelReport name=cube_ascii status=ok triangles=12 read_ms=0.380 voxelize_ms=0.110
  bbox=[(0.000, 0.000, 0.000), (1.000, 1.000, 1.000)] size=(1.000, 1.000, 1.000)mm
  grid=4x4x4 occupied=64/64 ratio=1.0000
Phase 0 Batch Report
success=3 failure=0 total_ms=13705.863
ModelReport name=armadillo_flat status=ok triangles=28620 read_ms=23.945 voxelize_ms=286.330
  bbox=[(-19.051, -13.948, -22.191), (19.051, 15.172, 22.693)] size=(38.102, 29.120, 44.885)mm
  grid=8x6x9 occupied=183/432 ratio=0.4236
ModelReport name=bunny status=ok triangles=259298 read_ms=213.729 voxelize_ms=2584.226
  bbox=[(123.097, 109.311, 15.000), (169.187, 144.657, 60.000)] size=(46.091, 35.346, 45.000)mm
  grid=10x8x9 occupied=316/720 ratio=0.4389
ModelReport name=mao status=ok triangles=798554 read_ms=662.105 voxelize_ms=9927.260
  bbox=[(-21.267, -47.680, -17.597), (21.385, 5.263, 53.385)] size=(42.652, 52.943, 70.981)mm
  grid=9x11x15 occupied=899/1485 ratio=0.6054
```

## Phase 0 速度测试结果

### 新项目 Phase 0 benchmark

命令：

```powershell
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && build_nmake\phase0_io_voxel_benchmark.exe 5.0'
```

输出：

```text
Phase 0 IO/Voxel Benchmark
spacing_mm=5
Old-style STL IO is a test-local reader modeled on the old basicDataType/STLReader binary/ascii pull pattern.
Old project voxelization is not invoked here because it depends on the old OpenVDB/SFF pipeline.
model,file_mb,triangles,new_stl_io_ms,old_style_stl_io_ms,phase0_voxel_ms,bbox_size_mm,grid,occupied
armadillo_flat,1.365,28620,23.346,19.825,279.891,38.102x29.120x44.885,8x6x9,183/432
bunny,12.364,259298,213.675,179.643,2566.011,46.091x35.346x45.000,10x8x9,316/720
mao,38.078,798554,656.711,542.819,9809.057,42.652x52.943x70.981,9x11x15,899/1485
```

说明：这里的 `old_style_stl_io_ms` 是测试内 old-style reader，不是直接编译旧项目源码；真实旧项目 benchmark 见下一节。

### 旧项目真实 benchmark

命令：

```powershell
cmd /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && build_old_project_benchmark\old_project_phase0_benchmark.exe "E:\Git_Repository\curved-slicer\tests\models" 5.0'
```

输出：

```text
Old Project Phase 0 Benchmark
models_dir=E:\Git_Repository\curved-slicer\tests\models
spacing_mm=5
model,file_mb,triangles,old_stl_reader_ms,old_sdf_readstl_ms,old_sdf_init_ms,old_voxel_loop_ms,old_voxel_total_ms,bbox_size_mm,grid,occupied
armadillo_flat,1.365,28620,8.656,6.501,15424.224,4.726,15435.451,38.102x29.120x44.885,12x10x13,530/1560
bunny,12.364,259298,50.639,59.854,13583.163,6.891,13649.908,46.091x35.346x45.000,14x12x13,725/2184
mao,38.078,798554,148.643,188.726,74420.348,11.288,74620.361,42.652x52.943x70.981,13x15x19,1639/3705
```

### 新旧速度摘要

同样 `spacing_mm=5.0`、同样三模型：

```text
model,new_stl_io_ms,new_phase0_voxel_ms,old_stl_reader_ms,old_voxel_total_ms
armadillo_flat,23.346,279.891,8.656,15435.451
bunny,213.675,2566.011,50.639,13649.908
mao,656.711,9809.057,148.643,74620.361
```

注意：新项目 Phase 0 体素化是轻量 CPU 验证实现；旧项目 `old_voxel_total_ms` 包含 `SDF.ReadStl`、OpenVDB `meshToSignedDistanceField` 和旧体素查询循环，其中主要耗时是 `old_sdf_init_ms`。这组数据用于 Phase 0 工程速度对照，不代表 Phase 1+ 最终算法质量对照。

## Phase 1 当前状态

详见 `docs/Phase1.md`。当前 Phase 1 已完成 baseline wavefront smoke、三模型 `spacing=0.5mm` batch 报告、M4/M6 metrics 输出和 PLY dump。

新增/移动后的核心结构：

```text
src/
├── app/
│   └── pipeline.{h,cpp}
├── field/
│   ├── laplacian.h
│   ├── poisson.h
│   └── wavefront.{h,cpp}
├── geometry/
│   ├── bvh.{h,cpp}
│   ├── sdf.{h,cpp}
│   └── voxel_grid.{h,cpp}
├── io/
│   ├── config_loader.{h,cpp}
│   ├── robot_writer.{h,cpp}
│   └── stl_reader.{h,cpp}
├── metrics/
│   └── curvature.{h,cpp}
└── surface/
    ├── iso_surface.{h,cpp}
    └── mc_lookup_table.h
```

Phase 1 验证摘要：

```text
ctest: 5/5 passed
grep -c 'SFF.cpp:' src/field/wavefront.cpp = 51
phase1_wavefront_batch_report.exe 0.5:
  elapsed_sec=213.491 peak_rss_mb=171.441 timed_out=False
  armadillo_flat connected_components=1 iso_triangles=11482
  bunny connected_components=1 iso_triangles=6018
  mao connected_components=1 iso_triangles=49639
```
