# Week 2 Summary: Mesh Repair, Voxel/SDF, Weighted Dijkstra

Date: 2026-05-19
Branch: `v6-implementation`
Scope: Week 2 ended at W2-T4. W2-T5 layer extraction was rolled back because the current layer-field specification is not sufficient for production layer extraction.

## 1. Status

Week 2 established the geometry input and voxel-graph foundation for v6:

| Task | Status | Commit | Verification |
|---|---:|---|---|
| W2-T1 Mesh repair gate | done | `ce93572` | `ctest` reached 26/26 at task close; R15 visual gate passed |
| W2-T2 VoxelizationV6 | done | `4f3ec43` | `ctest` reached 27/27 at task close; R15 visual gate passed |
| W2-T2 OpenMP chunk parallelism | done | `bd45899` | deterministic occupancy check passed |
| W2-T2 OpenMP 24-thread cap | done | `1c60881` | `ctest` remained 27/27 |
| W2-T3 SDFV6 | done | `feebb99` | `ctest` reached 28/28 at task close; R15 visual gate passed |
| W2-T4 WeightedDijkstraV6 | done | `3538f1c` | `ctest` reached 29/29 at task close; R15 visual gate passed |
| W2-T5 layer extraction | rolled back | none | uncommitted code removed; build reconfigured; `ctest` returned to 29/29 |

Rollback verification after W2-T5 removal:

```text
ctest --test-dir build_nmake --output-on-failure
100% tests passed, 0 tests failed out of 29
Total Test time (real) = 5.27 sec
```

The working tree was clean before creating this summary.

## 2. Key Metrics

### W2-T1 MeshRepair

Demo input: `runs/w2_t1_repair/bunny_before.ply`

| Metric | Value |
|---|---:|
| input vertices | 777894 |
| input triangles | 259298 |
| bbox diagonal | 73.4755 mm |
| duplicate vertices detected/merged | 648243 |
| degenerate triangles removed | 6 |
| triangles flipped | 3 |
| holes filled | 5 |
| output vertices | 129656 |
| output triangles | 259308 |
| manifold after repair | true |
| orientable after repair | true |
| boundary edges after repair | 0 |

Additional test signal: reversed-normal testbed reported `n_triangles_flipped = 960`, matching the synthetic sphere triangle count.

### W2-T2 VoxelizationV6

R15 bunny demo stats at 1.0 mm:

| Metric | Value |
|---|---:|
| spacing | 1.0 mm |
| dims | `[51, 40, 49]` |
| inside voxels | 15635 |
| boundary voxels | 4008 |
| outside voxels | 80317 |

Performance measurements collected for the same bunny repair output before OpenMP optimization:

| spacing_mm | total_voxel | elapsed_serial | Notes |
|---:|---:|---:|---|
| 0.5 | 781942 | 10.289 s | demo-scale dense run |
| 0.3 | 3615216 | 46.674 s | used later for SDF/Dijkstra R15 demos |
| 0.2 | 12114515 | 154.733 s | too slow for repeated manual visual checks |

Follow-up perf commits added OpenMP chunk parallelism and capped default threads at 24 because measurements showed memory-bandwidth saturation beyond physical cores.

### W2-T3 SDFV6

R15 bunny demo stats at 0.3 mm:

| Metric | Value |
|---|---:|
| method | `LIBIGL_SIGNED_DISTANCE` |
| voxel count | 3615216 |
| min distance | -12.1118 mm |
| max distance | 27.3529 mm |
| mean absolute distance | 6.48002 mm |
| narrow band | -1.0, full field |
| voxelize elapsed | 13.3964 s |
| signed distance elapsed | 20.6153 s |
| marching cubes elapsed | 0.117896 s |
| total elapsed | 34.1304 s |

R15 outputs:

```text
runs/w2_t3_sdf/bunny_sdf_slice_z_mid.png
runs/w2_t3_sdf/bunny_sdf_isosurface_d0.ply
runs/w2_t3_sdf/bunny_sdf_stats.json
```

### W2-T4 WeightedDijkstraV6

R15 bunny demo stats at 0.3 mm:

| Metric | Value |
|---|---:|
| sources | 5 |
| visited voxels | 724613 |
| unreachable voxels | 0 |
| max distance | 84.1727 mm |
| traced path length | 59.6219 mm |
| traced path voxels | 151 |
| voxelize elapsed | 25.7787 s |
| SDF elapsed | 41.0855 s |
| Dijkstra elapsed | 0.274796 s |
| trace elapsed | 0.0000202 s |
| total elapsed | 67.1995 s |

R15 outputs:

```text
runs/w2_t4_dijkstra/bunny_path.ply
runs/w2_t4_dijkstra/bunny_distance_slice_z_mid.png
runs/w2_t4_dijkstra/bunny_dijkstra_stats.json
```

## 3. Interface Audit Increment

Week 2 added four v6-owned modules. These do not modify v4 baseline code.

| Module | Label | v6 role | Downstream use |
|---|---|---|---|
| `src/mesh/MeshRepair.{h,cpp}` | new v6 module | STL input gate, duplicate/degenerate cleanup, hole fill, orientation, unit gate | Stage 1 mesh ingestion |
| `src/voxel/VoxelizationV6.{h,cpp}` | new v6 wrapper/module | repaired mesh to occupancy grid plus boundary voxels | Stage 2 voxel/SDF and graph construction |
| `src/sdf/SDFV6.{h,cpp}` | new v6 wrapper/module | signed distance per voxel, negative inside convention | Stage 2 clearance, W3 penalties, W4 IK/collision priors |
| `src/path/WeightedDijkstraV6.{h,cpp}` | new v6 module | multi-source voxel distance field and predecessor trace | Stage 3/4 scalar field construction and path-field experiments |

Existing v4 modules remain frozen. Week 2 only consumes the Week 1 wrapper surface and new v6 modules.

## 4. W2-T5 Rollback Post-Mortem

W2-T5 attempted layer extraction from the current v6 Dijkstra distance field. The result was rejected before commit and the uncommitted changes were removed.

Root cause:

The current Dijkstra distance field `D` does not encode an SFF-style explicit layer-offset structure. Extracting layers by applying Marching Cubes to raw or smoothed `D` produces surfaces that are geometrically plausible at the scalar-field level but do not carry the explicit offset/accumulation semantics needed for usable curved layers. Smoothing cannot recover missing layer structure after the fact.

Observed implication:

Layer extraction is not a Marching Cubes quality issue alone. Replacing the meshing table or post-smoothing the iso-surface can improve visual artifacts, but it cannot fix an under-specified scalar field. The layer field must be redesigned before implementing production extraction.

Rollback actions completed:

```text
git checkout -- .
Remove-Item -Recurse runs\w2_t5_layers\
cmake -S . -B build_nmake -G "NMake Makefiles"
cmake --build build_nmake
ctest --test-dir build_nmake --output-on-failure
```

Result:

```text
100% tests passed, 0 tests failed out of 29
```

## 5. Spec Gap

Open design gap for Week 3:

The v6 scalar/path field must make layer offsets explicit enough that extraction can preserve manufacturable layer order and topology. Current `D` is useful as a graph distance and path-trace substrate, but not yet sufficient as the only scalar for layer extraction.

Required Week 3 decisions:

1. Define whether the next field is an accumulation/layer-offset field, a constrained distance field, or a coupled pair `(D_path, L_offset)`.
2. Define how support, clearance, IK, topology, and component penalties enter the field without destroying layer-offset semantics.
3. Define extraction invariants before implementing Marching Cubes or contour extraction again.
4. Decide how W2-T4 predecessor paths and W1-T3 `GeodesicContourPath` feed into the redesigned Stage 4.

## 6. Week 3 Preparation

Week 3 can start from a stable base:

- mesh input is repaired and audited;
- voxel occupancy and boundary markers exist;
- signed distance is available on the voxel grid;
- multi-source weighted Dijkstra exists with predecessor tracing;
- tests are green at 29/29.

Week 3 should focus on the full penalty set and path/layer-field design before any new layer extraction commit:

| Area | Week 3 starting point |
|---|---|
| penalty terms | add support, clearance, topology, curvature, IK feasibility, and branch-volatility terms as explicit field components |
| scalar/layer field | redesign field to preserve explicit layer offsets rather than relying on smoothed Dijkstra distance alone |
| topology/component policy | define component-aware handling before extraction, including split/merge diagnostics |
| extraction gate | do not re-enter layer extraction until the field invariants and acceptance metrics are written |
| validation | keep R15 gates for any PLY/PNG/HTML visual output |

## 7. Final Week 2 State

Week 2 is considered complete at W2-T4. W2-T5 is intentionally not part of the committed history.

Current test baseline:

```text
29/29 ctest pass
```

Ready for Week 3:

```text
yes, with blocker: layer extraction spec must be redesigned before implementation.
```
