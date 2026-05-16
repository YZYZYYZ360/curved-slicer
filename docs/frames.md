# Coordinate Frame Audit for v6.2

Status: Week 1 Task W1-T6 audit document.

Scope: document frame names, transform contracts, units, and current v4 code
locations before adding v6.2 geometry, IK-field, trajectory, collision, and KRL
modules.

Non-goal: this file does not change algorithms or prescribe new directory names.

## 1. Frame Inventory

### F0. STL frame

- Meaning: coordinates as stored in the input STL file.
- Units: assumed millimeters. STL has no unit metadata; all current fixtures and
  configs treat coordinates as mm.
- Axes: whatever the input model uses. No automatic reorientation is applied by
  the STL reader.
- Code owner: `src/io/stl_reader.cpp`, `src/io/stl_reader.h`.
- Data type: `TriangleMesh`, `StlTriangle`, `Vec3`, `AABB`.

### F1. World frame

- Meaning: algorithmic world before robot placement.
- Current v4 behavior: initially identical to the STL frame. If
  `kuka.world_to_base` is non-identity, `runBatch()` applies that matrix directly
  to mesh vertices and normals before voxelization. After that mutation, the
  downstream mesh coordinates are numerically in robot-base coordinates, even
  though many variable names still read as "world".
- Units: mm for positions, unitless normalized vectors for directions.
- Code owner: `src/app/pipeline.cpp`.

### F2. Voxel grid frame

- Meaning: integer index space `(i,j,k)` over a dense logical grid plus sparse
  occupied set.
- Origin convention: index `(0,0,0)` refers to the first cell. Its center is not
  the bbox minimum; the center is offset by half a voxel.
- Center mapping:
  `p_world(i,j,k) = bbox.min + spacing_mm * (i + 0.5, j + 0.5, k + 0.5)`.
- Units: indices are integer cells; spacing and mapped positions are mm.
- Code owner: `src/geometry/voxel_grid.cpp`, `src/geometry/sdf.cpp`.

### F3. Model frame

- Meaning: model coordinates after `model_transform` / current
  `kuka.world_to_base` placement.
- Current v4 behavior: no separate model-frame type exists. The transform is
  applied in place to `TriangleMesh`; downstream voxel/SDF/layer/path points are
  in the transformed model frame.
- Intended v6 contract: keep this as the model-to-robot placement frame. The
  model frame should be the frame used by slicing, SDF queries, IK sampling, and
  path generation after manual placement.
- Units: mm and normalized vectors.
- Code owner: current `src/app/pipeline.cpp`; future yaml reader may rename the
  config field to `model_transform` while preserving the same 4x4 contract.

### F4. Robot base frame

- Meaning: KUKA KR4 R600 base coordinate system used by FK, IK, reachability,
  `CartPose`, and eventually KRL output.
- Current v4 behavior: after `world_to_base` is applied, model/path coordinates
  fed into IK are expected to already be in robot base coordinates.
- Units: mm for `X,Y,Z`; degrees for joint angles and KUKA ABC orientation.
- Code owner: `src/kinematics/*`, `src/app/pipeline.cpp`, `src/io/config_loader.*`.

### F5. TCP frame

- Meaning: Tool Center Point pose of the nozzle tip.
- Position: `CartPose.X/Y/Z`, in robot base coordinates, mm.
- Orientation: `CartPose.A/B/C`, KUKA ZYX Euler angles in degrees.
- Current FK contract: joint angles plus KR4 DH parameters produce base-to-TCP.
- Current IK contract: base-frame TCP `CartPose` produces one or more joint
  solutions.
- Code owner: `src/kinematics/cart_pose.h`,
  `src/kinematics/forward_kin.*`, `src/kinematics/ik_solver.*`.

### F6. Tool axes

- Meaning: local nozzle/body axes expressed as columns of `R_tool`.
- Convention from v4 path code:
  - `tool_z = -normalize(G_outward)`.
  - `tool_x` is the path tangent projected onto the plane perpendicular to
    `tool_z`.
  - `tool_y = tool_z cross tool_x`.
  - `R_tool = [tool_x tool_y tool_z]`.
- v6 Stage 2 note: during IK field construction, `tool_z` must come from
  `-normalize(grad SDF)` and yaw remains a sampled state. Stage 5 can use path
  tangent once paths exist.
- Units: axes are unit vectors.
- Code owner: `src/path/pose_from_path.cpp`, later v6 IK-field module.

### F7. KRL frame

- Meaning: KUKA controller-side representation.
- Current v4 code: `robot_writer` is still a placeholder and does not emit real
  motion records.
- v4/v6 contract: KRL output should consume joint trajectory data in degrees and
  robot `$BASE` / `$TOOL` numbers from config. v6 KRL must export only valid OK
  motion segments and valid transitions.
- Units: `E6AXIS` joint angles in degrees; `E6POS`/`CartPose` positions in mm
  and ABC in degrees if Cartesian output is ever used.
- Code owner: current `src/io/robot_writer.*`; future W1/W9 KRL writer under
  actual v4 naming should live in `src/io/`.

## 2. Frame Diagram

```
Input STL
  F0 STL frame
      |
      | T_stl_world = I  (current default)
      v
  F1 World frame
      |
      | T_world_base = model_transform / current kuka.world_to_base
      | 4x4 homogeneous matrix, mm translation
      v
  F3 Model frame after placement
      |
      | Current v4 code stores this numerically in robot base coordinates
      v
  F4 Robot base frame
      |
      | voxelize/SDF/layers/path points use base-frame mm coordinates
      v
  F2 Voxel grid index frame --[bbox.min + (i+0.5)*spacing]--> F4 positions
      |
      | path tangent + normal or grad SDF
      v
  F6 Tool axes [tool_x tool_y tool_z]
      |
      | CartPose(X,Y,Z,A,B,C)
      v
  F5 TCP frame
      |
      | IK / FK, joint trajectory q[6] in deg
      v
  F7 KRL frame (.src/.dat, $BASE/$TOOL, E6AXIS/E6POS)
```

## 3. Transform Inventory

### T0. STL to World

- Source -> target: F0 STL frame -> F1 World frame.
- Math: `T_stl_world = I_4`.
- Current code: implicit in `readStl()`; no transform is applied in
  `src/io/stl_reader.cpp`.
- Units: STL vertex coordinates are treated as mm.
- Caveat: STL files do not carry units; fixtures and configs define the mm
  interpretation.

### T1. World to Robot Base / Model Transform

- Source -> target: F1 World frame -> F4 Robot base frame.
- Math:
  ```
  p_base_h = T_world_base * [x_world, y_world, z_world, 1]^T
  n_base   = R_world_base * n_world
  ```
  where `R_world_base` is the upper-left 3x3 block of the 4x4 matrix.
- Current config field: `[kuka.world_to_base].world_to_base`.
- Current code:
  - parsed in `src/io/config_loader.cpp`
  - stored in `KukaConfig::world_to_base`
  - applied in `src/app/pipeline.cpp` before voxelization and SDF.
- Units: translation in mm; rotation unitless.
- Current examples:
  - `config/batch_three_models.toml` uses identity.
  - `config/mao_rescaled_v4.toml` translates the model to `Y=421mm`,
    `Z=182mm`.
- Audit note: current v4 code mutates the mesh in place. v6 should document
  whether a future `model_transform.yaml` is an alias for this field or replaces
  it, but the mathematical contract should stay 4x4 homogeneous mm transform.

### T2. Voxel Index to Position

- Source -> target: F2 voxel index -> F1/F4 continuous position, depending on
  whether model placement has already been applied.
- Math:
  ```
  x = bbox.min.x + (i + 0.5) * spacing_mm
  y = bbox.min.y + (j + 0.5) * spacing_mm
  z = bbox.min.z + (k + 0.5) * spacing_mm
  ```
- Current code:
  - `VoxelGrid::center()` in `src/geometry/voxel_grid.cpp`
  - internal `voxelCenter()` in `src/geometry/sdf.cpp`
  - similar scalar/SDF field sampling helpers in `src/app/pipeline.cpp`.
- Units: `i,j,k` are integer cell indices; `bbox.min`, `spacing`, and output
  positions are mm.
- Audit note: `bbox.min` is the corner of the padded grid bbox. The center of
  voxel `(0,0,0)` is `bbox.min + 0.5 * spacing`, not `bbox.min`.

### T3. Voxel Grid to SDF Field Coordinates

- Source -> target: F2 voxel index -> SDF sample point in the same continuous
  model/base frame.
- Math: same center mapping as T2.
- Current code:
  - `SDF::bbox`, `SDF::spacing`, `SDF::nx/ny/nz`
  - `voxelCenter()` and `signedDistanceToMesh()` in `src/geometry/sdf.cpp`.
- Units: SDF values are signed distances in mm; inside is negative and outside
  is positive in current v4 convention.
- v6 red-line tie-in: Dijkstra distance fields may be normalized for
  visualization, but layer extraction must use mm-valued slicing fields, not a
  normalized field.

### T4. Path Point and Tool Axes to TCP CartPose

- Source -> target: path point plus F6 tool axes -> F5 TCP pose.
- Math:
  ```
  tool_z = -normalize(G_outward)
  tool_x = normalize(tangent - dot(tangent, tool_z) * tool_z)
  tool_y = tool_z cross tool_x
  R_tool = [tool_x tool_y tool_z]
  CartPose.XYZ = path point
  CartPose.ABC = ZYX_Euler(R_tool)
  ```
- Current code:
  - `constructToolFrame()` in `src/path/pose_from_path.cpp`
  - `toolFrameToCartPose()` in `src/path/pose_from_path.cpp`
  - `rotMatToKukaABC()` in `src/kinematics/forward_kin.cpp`.
- Units: point in mm; axes unitless; ABC in degrees.
- v6 Stage 2 distinction: before paths exist, `tool_z` is
  `-normalize(grad SDF)` and yaw is sampled. Do not use path tangent in Stage 2.

### T5. Forward Kinematics

- Source -> target: six joint angles in F4 robot base model -> F5 TCP pose.
- Math:
  ```
  A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
  T_base_tcp = A_1 * A_2 * ... * A_6 * Tz(flange_z_mm + tool_z_mm)
  XYZ = translation(T_base_tcp)
  ABC = ZYX_Euler(rotation(T_base_tcp))
  ```
- Current code:
  - DH parameters: `src/kinematics/dh_params.h`
  - matrices: `dhMatrix()`, `forwardKinMatrix()`
  - pose conversion: `forwardKin()` in `src/kinematics/forward_kin.cpp`.
- Units: joint angles in degrees, DH lengths in mm, ABC in degrees.
- Config hook: `dh.tool_z_mm` is set from `[kuka.robot].z_offset` in
  `src/app/pipeline.cpp` for v4 trajectory generation.

### T6. Inverse Kinematics

- Source -> target: F5 TCP `CartPose` in robot base -> one or more joint
  configurations.
- Math:
  ```
  R_tcp = Rz(A) * Ry(B) * Rx(C)
  wrist_center = TCP_position - tcp_offset * R_tcp[:,2]
  solve J1/J2/J3 from wrist_center
  R_456 = R_03^T * R_tcp
  solve J4/J5/J6 from R_456
  reject by joint limits, singularity, FK verification
  ```
- Current code:
  - `tcp_to_wrist_center()` and `solveIKAll()` in
    `src/kinematics/ik_solver.cpp`
  - `checkReachability()` in `src/kinematics/reachability.cpp`.
- Units: input XYZ in mm, ABC in degrees, returned q in degrees.
- Audit note: the current v4 `tcp_to_wrist_center()` subtracts a hard-coded
  `12.0 + 0.28mm` offset, while FK uses `dh.flange_z_mm + dh.tool_z_mm`. Keep
  this visible for future KR4 wrapper review; this task does not change it.

### T7. TCP / Joint Trajectory to KRL

- Source -> target: F5 TCP or joint trajectory -> F7 KRL controller records.
- Math:
  - Joint output: `E6AXIS {A1 q1, A2 q2, ..., A6 q6}` in degrees.
  - Cartesian output if used: `E6POS {X,Y,Z,A,B,C}` in mm and degrees.
  - Controller base/tool selection comes from `$BASE = BASE_DATA[base_no]` and
    `$TOOL = TOOL_DATA[tool_no]`.
- Current code: only placeholder comments exist in `src/io/robot_writer.cpp`.
- Config fields: `[kuka.robot].tool_no`, `base_no`, `status`, `turn`,
  `z_offset`, and `robot_ini_abc` in `src/io/config_loader.h` and config files.
- v6 contract: KRL export must include OK print/retract/travel/approach motion
  and exclude all `*_FAIL` segments.

### T8. World/Base to KRL Cartesian Convention

- Source -> target: F4 robot base Cartesian pose -> F7 KRL Cartesian record.
- Math:
  ```
  X_krl, Y_krl, Z_krl = X_base, Y_base, Z_base
  A_krl, B_krl, C_krl = ZYX_Euler(R_base_tcp)
  ```
  The selected `$BASE` is expected to make these coordinates meaningful to the
  controller. With the current contract, model/path points must already be in
  the same robot-base placement used by KRL.
- Current code:
  - `CartPose` field units in `src/kinematics/cart_pose.h`
  - ABC conversion in `src/kinematics/forward_kin.cpp`
  - KRL placeholder in `src/io/robot_writer.cpp`.
- Units: positions in mm, ABC in degrees.
- Audit note: v6 mainline is expected to export joint-space records for validated
  dense trajectories, but this Cartesian convention is still the bridge from
  world/base geometry to KUKA pose fields when constructing IK targets.

## 4. Unit Contract

- Mesh positions: mm.
- Voxel spacing, padding, SDF band, layer height, path spacing: mm.
- SDF values: signed mm.
- Dijkstra distances in v6: mm for `D_0_mm`, `D_final_mm`, `D_slice_mm`.
- Joint angles: degrees in public structs, configs, CSV, and KRL.
- Internal trigonometry: radians inside low-level math where explicitly named
  `*_rad`.
- KUKA ABC: degrees, ZYX convention.
- Tool axes, normals, tangents: normalized unit vectors.

## 5. Current Code Location Summary

| Concern | Current code |
|---|---|
| STL coordinates and bbox | `src/io/stl_reader.cpp`, `src/io/stl_reader.h` |
| Voxel grid origin and spacing | `src/geometry/voxel_grid.cpp`, `src/geometry/voxel_grid.h` |
| SDF sample coordinates | `src/geometry/sdf.cpp`, `src/geometry/sdf.h` |
| `world_to_base` config parse | `src/io/config_loader.cpp`, `src/io/config_loader.h` |
| `world_to_base` application | `src/app/pipeline.cpp` |
| KR4 FK and ABC conversion | `src/kinematics/forward_kin.cpp`, `src/kinematics/forward_kin.h` |
| KR4 IK | `src/kinematics/ik_solver.cpp`, `src/kinematics/ik_solver.h` |
| Reachability wrapper | `src/kinematics/reachability.cpp`, `src/kinematics/reachability.h` |
| Path tangent to tool frame | `src/path/pose_from_path.cpp`, `src/path/pose_from_path.h` |
| KRL output placeholder | `src/io/robot_writer.cpp`, `src/io/robot_writer.h` |

## 6. Implementation Rules for v6 Work

1. Treat transformed model coordinates as robot-base coordinates unless a future
   task introduces a dedicated frame type.
2. Keep voxel index coordinates out of geometry algorithms except at explicit
   T2/T3 conversion boundaries.
3. Keep `D_*_mm` fields in millimeters. `D_norm` is visualization-only.
4. Stage 2 IK field tool axis construction uses `tool_z = -normalize(grad SDF)`
   with yaw sampling. It must not call the Stage 5 path-tangent frame builder
   before paths exist.
5. Stage 5 path pose construction can use `constructToolFrame()` style logic:
   path tangent determines `tool_x`, normal determines `tool_z`.
6. KRL writer must consume final validated trajectory states, not raw paths.
7. Do not change v4 PDE code for frame cleanup. Any future frame refactor should
   wrap or adapt v4 modules without modifying the baseline PDE implementation.

## 7. Open Audit Items

- The v6 spec names `model_transform.yaml`; current configs use
  `[kuka.world_to_base].world_to_base`. Future config work should choose one
  public field name and document it as the same T1 matrix.
- The current IK wrist-center helper hard-codes `12.28mm`; FK uses DH fields.
  Future W1 KR4 wrapper work should decide whether to pass `KR4DHParams` into
  that helper.
- Current KRL writer is a placeholder, so F7 is a contract rather than an
  implemented transform today.
