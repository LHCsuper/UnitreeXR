# Work Log

## 2026-08-19

### 14:22 - UnitreeXR workspace initialized

**Action**

Initialized the long-term UnitreeXR workspace skeleton and documentation set.

**Result**

The repository now has a clean Phase 0 bootstrap structure for future XR experiments.

**Files Changed**

- README.md
- AGENTS.md
- .gitignore
- pyproject.toml
- configs/README.md
- docs/STATUS.md
- docs/PROJECT_PLAN.md
- docs/WORKLOG.md
- docs/DECISIONS.md
- docs/ENVIRONMENT.md
- docs/COORDINATE_SYSTEM.md
- docs/experiments/README.md
- docs/experiments/EXPERIMENT_TEMPLATE.md
- external/README.md
- src/unitree_xr/__init__.py
- src/unitree_xr/xr/__init__.py
- src/unitree_xr/calibration/__init__.py
- src/unitree_xr/adapters/__init__.py
- src/unitree_xr/common/__init__.py
- data/raw/.gitkeep
- data/processed/.gitkeep
- logs/.gitkeep

**Next**

Keep Phase 0 stable and wait for the first XR data pipeline validation task.

### 14:30 - Script directories prepared

**Action**

Created placeholder directories for diagnostics and calibration scripts.

**Result**

The workspace now includes dedicated script entry points for future Phase 1 and Phase 2 tooling.

**Files Changed**

- scripts/diagnostics/
- scripts/calibration/

**Next**

Leave the script directories empty until a verified use case appears.

### 14:58 - Phase 1 EXP-001 prepared

**Action**

Closed Phase 0, started Phase 1, and prepared EXP-001 for XR data pipeline validation.

**Result**

The project status now reflects Phase 1. EXP-001 documentation and a minimal diagnostic probe script are ready, but the probe has not been executed.

**Files Changed**

- AGENTS.md
- docs/STATUS.md
- docs/WORKLOG.md
- docs/experiments/EXP-001_XR_DATA_PIPELINE.md
- scripts/diagnostics/xr_stream_probe.py

**Next**

Run EXP-001 only when the XR runtime and PICO setup are ready for live observation.

### 15:16 - EXP-001 validated PASS

**Action**

Archived the EXP-001 live validation results.

**Result**

XR data transport is verified end-to-end. Coordinate semantics remain unverified.

**Files Changed**

- docs/experiments/EXP-001_XR_DATA_PIPELINE.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Defer coordinate semantics to Phase 2. Do not start EXP-002 in this step.

### 15:58 - EXP-002 position frame validated

**Action**

Archived EXP-002 results for the XR position coordinate convention.

**Result**

Position order, axis directions, handedness, and unit consistency are
recorded. Quaternion, tracking origin, controller local frame, recenter,
and Unitree mapping remain unverified.

**Files Changed**

- docs/experiments/EXP-002_XR_POSITION_FRAME.md
- docs/STATUS.md
- docs/COORDINATE_SYSTEM.md
- docs/WORKLOG.md

**Next**

Defer quaternion, tracking origin, controller local frame, recenter, and
Unitree mapping to later experiments. Do not start EXP-003 in this step.

### Phase 2 PICO coordinate calibration closed

**Action**

- Consolidated PICO/XRoboToolkit raw pose convention.
- Recorded quaternion order and transform semantics.
- Recorded controller local-frame observations.
- Recorded Device Tracking Origin behavior.
- Recorded Home recenter behavior.
- Recorded repeated Home vertical-axis validation.

**Result**

- Phase 2 closed.
- Raw XR pose convention is sufficiently defined for downstream Unitree coordinate mapping.
- Project moves to Phase 3 — Unitree Coordinate Mapping.

**Experimental evidence**

Home Y-axis deviation:

- 1.0307 deg
- 0.4118 deg
- 1.0421 deg

**Interpretation**

- Tracking Origin vertical direction remains effectively unchanged under tested HMD downward/lateral tilt during recenter.
- Home primarily redefines the horizontal forward reference.

**Hypothesis**

- Because PICO has inertial sensing capability and the vertical axis remains stable during recenter, gravity-related inertial information may be used as an important vertical reference.

**Known limitation**

- This does not prove that PICO directly sets +Y from the IMU gravity vector.
- Exact IMU / camera / SLAM fusion remains unknown.

**Next**

Study:

```text
XRoboToolkit controller pose
→ Unitree TeleData
→ Unitree wrist target frame
```

**Files Changed**

- AGENTS.md
- docs/COORDINATE_SYSTEM.md
- docs/STATUS.md
- docs/WORKLOG.md

### S0 — `wheelloong_m2` URDF/MJCF arm FK consistency validated

**Action**

Executed and archived EXP-003, comparing direct torso-relative arm end-link
forward kinematics from the checked-in URDF/Pinocchio and controlled
MJCF/MuJoCo models.

**Result**

For five deterministic, within-limit configurations, the maximum measured
errors were `9.254183423523e-14 m` and `4.132388077110e-13 rad`. The scoped
arm FK is directly consistent; no fixed rotation, mirror, axis-swap, or
compensation transform was used or indicated.

**Files Changed**

- experiments/validate_urdf_mujoco_fk.py
- docs/experiments/EXP-003_WHEELLOONG_M2_FK_CONSISTENCY.md
- docs/WORKLOG.md

**Next**

Keep any XR-to-robot coordinate mapping work separate from this validated
model-consistency result.

### S0.5 — `wheelloong_m2` end-effector frame candidates analyzed

**Action**

Extracted the left/right gripper kinematic structure from the URDF and MJCF,
built a runtime-only frame-inspection script, and derived a geometric
gripper-root center candidate. Recorded as EXP-004.

**Result**

- `arm_link_7` is the wrist-roll joint frame, not the gripper-root operational
  point.
- No explicit palm / gripper_base / tool_center link exists.
- The mean of the four direct gripper root joint origins is `0.220210 m`
  from each `arm_link_7` origin.
- Candidate gripper-root operational point:
  `^arm7 p = [-0.0365, ±0.21691, 0.0105]`.
- Orientation was not finalized in S0.5; its initial inference was corrected
  and superseded by S0.5b below.
- No final EE frame was selected; no IK, XR, model, or coordinate-mapping
  changes were made.

**Files Changed**

- experiments/inspect_wheelloong_m2_ee_frames.py
- docs/experiments/EXP-004_WHEELLOONG_M2_EE_FRAME.md
- docs/experiments/EXP-003_WHEELLOONG_M2_FK_CONSISTENCY.md
- docs/WORKLOG.md

**Next**

Only after a robot-side/downstream authority definition or visual
confirmation, finalize the EE frame and defer any IK work until then.

### S0.5b — Baseline logical teleoperation EE frames defined

**Action**

Corrected EXP-004 by transforming all four second-stage finger displacement
vectors through their direct gripper parent rotations into `arm_link_7`.
Derived and numerically validated logical operational frames `W_L` / `W_R`,
then added full XYZ MeshCat frames while retaining raw position-only
gripper-root center markers.

**Result**

- Left secondary vectors point primarily along `+L7 Y`; right secondary
  vectors point primarily along `-R7 Y` after applying the right-side fixed
  yaw.
- Frame semantics are now explicit: `+Y_W` = physical finger extension,
  `+Z_W` = positive direct hinge axis, `+X_W = y_W cross z_W`.
- `^L7 R_WL` is identity; `^R7 R_WR` is approximately
  `diag(-1, -1, +1)` with the small residual from the URDF's rounded
  `-1.5708 rad` yaw.
- Both rotations pass orthogonality, determinant `+1`, and right-handed
  cross-product checks.
- The ordinary inspection command and the `--visualize` command both ran to
  completion; MeshCat created the two named operational EE frames without an
  exception.
- `W_L` / `W_R` use fixed gripper-root/palm-center operational points. They
  are not calibrated fingertip TCPs.
- DEC-004 records these frames as the baseline targets for later arm IK.
- No IK, XR, URDF/MJCF, coordinate-adapter, or robot-control change was made.

**Files Changed**

- experiments/inspect_wheelloong_m2_ee_frames.py
- docs/experiments/EXP-004_WHEELLOONG_M2_EE_FRAME.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S0.5b. Any IK implementation belongs to a later explicitly scoped
step and must use the recorded `W_L` / `W_R` frame contract.

### S1.0 — `wheelloong_m2` Pinocchio FK/Jacobian backbone established

**Action**

Added a module-relative URDF loader, a single named 14-DOF arm interface,
and pure Pinocchio torso-relative FK/Jacobian interfaces for the existing
S0.5b logical operational frames `W_L` / `W_R`. Added EXP-005 with zero,
fixed-seed legal-random, and finite-difference checks.

**Result**

- `q_arm` has a fixed left-then-right ordering and name-resolved Pinocchio
  configuration/velocity addresses.
- FK returns `^torso T_WL` / `^torso T_WR`, never world-frame poses.
- Both operational Jacobians have shape `(6, 14)` with `[linear; angular]`
  rows explicitly expressed in torso axes.
- The finite-difference check for `left_arm_joint_4` ran at the random legal
  configuration; left residuals were `1.812425267363e-15 m` and
  `1.729217900636e-16 rad`.
- DEC-005 records the public arm-order and torso-frame kinematics contract.
- No IK, CasADi, IPOPT, XR, MuJoCo controller, URDF/MJCF edit, or robot
  control was added.

**Files Changed**

- src/wheelloong_m2/__init__.py
- src/wheelloong_m2/kinematics/__init__.py
- src/wheelloong_m2/kinematics/robot_model.py
- src/wheelloong_m2/kinematics/frames.py
- src/wheelloong_m2/kinematics/dual_arm_fk.py
- experiments/test_wheelloong_m2_kinematics.py
- docs/experiments/EXP-005_WHEELLOONG_M2_PINOCCHIO_KINEMATICS.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S1.0. Any IK design or implementation requires a separately scoped
task and must consume this named `q_arm` plus the unchanged `W_L` / `W_R`
contract.

### S1.1 — `wheelloong_m2` CasADi symbolic FK backbone established

**Action**

Verified the installed CasADi/Pinocchio symbolic environment, then added an
independent CasADi FK path derived from the same URDF-loaded Pinocchio model.
The symbolic full configuration starts at neutral and scatters the existing
14 named arm symbols. EXP-006 compares its outputs to S1.0 numeric FK.

**Result**

- CasADi `3.6.7`, Pinocchio `3.4.0`, and `pinocchio.casadi` are available.
- The public symbolic input is `SX q_arm(14)` in the unchanged S1.0 order;
  full symbolic Pinocchio configuration is `SX(42)`.
- Symbolic FK returns the existing `^torso T_WL` / `^torso T_WR` components
  via a `dual_arm_fk` CasADi function.
- Zero and fixed-seed legal-random cases had zero printed position error;
  maximum printed rotation residual was `5.334383746338e-16 rad`.
- DEC-006 records the independent numeric/symbolic FK architecture.
- No IK, cost, IPOPT, Opti, solver, XR, controller, URDF/MJCF edit, or robot
  control was added.

**Files Changed**

- src/wheelloong_m2/kinematics/casadi_fk.py
- experiments/test_wheelloong_m2_casadi_fk.py
- docs/experiments/EXP-006_WHEELLOONG_M2_CASADI_FK.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S1.1. Any symbolic Jacobian, IK formulation, or optimization work
requires a separately scoped task.

### S1.2.0 — `wheelloong_m2` solver-free SE(3) IK math foundation established

**Action**

Added numeric and CasADi symbolic SE(3) pose-error utilities plus a numeric
dual-arm cost decomposition. Added EXP-007 and a deterministic experiment
covering error sign, SO(3) direction, log/exp reconstruction, symbolic
agreement, and cost accounting.

**Result**

- The fixed S1.0/S1.1 `q_arm(14)` and S0.5b `W_L` / `W_R` contracts are
  consumed unchanged.
- Per-arm error in torso axes is explicitly `p_current - p_target` and
  `Log(R_current * R_target^T)`; `Rz(+30 deg)` versus identity produced the
  expected `+Z` `0.523598775598 rad` rotation vector.
- CasADi’s symbolic SO(3) log includes a small-angle branch and matched the
  numeric 30-degree result exactly in the recorded evaluation.
- `IKWeights` exposes pose, rotation, nominal, and smoothness weights; the
  cost function only evaluates and returns its scalar terms.
- DEC-007 records the convention. No iteration, CasADi Opti, NLP, IPOPT,
  solver, XR, controller, URDF/MJCF edit, or robot control was added.

**Files Changed**

- src/wheelloong_m2/ik/__init__.py
- src/wheelloong_m2/ik/se3_error.py
- src/wheelloong_m2/ik/casadi_se3_error.py
- src/wheelloong_m2/ik/cost.py
- experiments/test_wheelloong_m2_ik_math.py
- docs/experiments/EXP-007_WHEELLOONG_M2_SE3_IK_MATH.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S1.2.0. A future explicitly scoped stage may formulate a solver, but
this stage does not construct, configure, or invoke one.

### S1.2.1 — `wheelloong_m2` offline CasADi/IPOPT dual-arm IK baseline established

**Action**

Added a reusable CasADi Opti/IPOPT solver over the existing named
`q_arm(14)` interface. It calls the existing symbolic FK, reuses the S1.2.0
symbolic pose error, accepts torso-relative Pinocchio `SE3` targets as Opti
parameters, applies URDF position limits, and supports `q_init` warm starts.
Added EXP-008 and deterministic numeric-FK round-trip tests.

**Result**

- All zero, fixed-seed legal-random, and simultaneous left/right target test
  cases reported IPOPT success with URDF-limit-respecting solutions.
- Recorded solve times were 0.015417 s, 0.009763 s, and 0.005229 s for 5, 12,
  and 6 IPOPT iterations respectively.
- The zero target returned floating-point-scale FK residuals. Random target
  cases retained millimetre-scale position and approximately 0.02-rad
  orientation residuals because pose tracking is a soft cost alongside
  default nominal and smoothness regularization.
- DEC-008 records this explicitly as an offline baseline. No XR, adapter,
  controller, trajectory, velocity/acceleration, collision, torque,
  URDF/MJCF edit, or robot-control work was added.

**Files Changed**

- src/wheelloong_m2/ik/dual_arm_ik.py
- src/wheelloong_m2/ik/__init__.py
- experiments/test_wheelloong_m2_ik_solver.py
- docs/experiments/EXP-008_WHEELLOONG_M2_NLP_IK.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S1.2.1. This remains an offline pose solver; do not connect it to VR,
MuJoCo control, or a physical robot without separately scoped work.

### S2.0 — `wheelloong_m2` offline MuJoCo IK loop established

**Action**

Added a module-relative loader for the existing controlled MJCF, named
joint-to-qpos-to-position-actuator mapping, and a position-control stepping
loop. Added an IK-to-MuJoCo experiment that solves offline targets, writes
only `data.ctrl`, advances physics, reads final named qpos, and evaluates
Pinocchio FK.

**Result**

- MuJoCo `3.10.0` loaded `wheelloong_m2_controlled.xml` with `nq=32`,
  `nv=32`, `nu=17`, and `0.001 s` timestep.
- Loaded name mapping resolved left arm qpos/control addresses `6..12` /
  `2..8` and right addresses `19..25` / `9..15`.
- Neutral, left-only, and right-only targets all had IPOPT success. The
  measured simulated joint tracking norms were `1.284e-3`, `1.629e-3`, and
  `1.688e-3 rad` respectively.
- The unilateral cases showed approximately `9.8e-2 rad` simulated motion in
  the requested arm versus approximately `9.1e-4 rad` in the held-neutral arm.
- DEC-009 records that targets enter only through existing position-actuator
  controls. No XR, real robot, low-level motor controller, trajectory,
  velocity/acceleration limit, collision, torque, or model edit was added.

**Files Changed**

- src/wheelloong_m2/simulation/__init__.py
- src/wheelloong_m2/simulation/mujoco_model.py
- src/wheelloong_m2/simulation/mujoco_arm_controller.py
- experiments/test_wheelloong_m2_mujoco_ik_loop.py
- docs/experiments/EXP-009_WHEELLOONG_M2_MJOC0_IK_LOOP.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S2.0. The loop is offline MuJoCo simulation only; do not attach XR or
real robot control without separately scoped work.

### S2.1 — `wheelloong_m2` multi-rate teleoperation simulation runtime established

**Action**

Added shared runtime frequency configuration, a latest-value dual-arm target
buffer, and a simulation-time multi-rate scheduler. Added a fake SE(3)
sinusoidal target source experiment that runs target updates, warm-started
IK, named actuator writes, and MuJoCo physics at independent rates.

**Result**

- The deterministic two-second run recorded exactly 240 target updates,
  500 IK solves, and 2000 physics steps: 120 Hz / 250 Hz / 1000 Hz in
  simulation time.
- IPOPT solve latency samples had `4.168258 ms` mean, `4.546778 ms` p95, and
  `15.083930 ms` maximum wall-clock duration.
- Final joint tracking norm was `2.110591184150e-02 rad`; final left/right
  EE position residuals were `7.230049812038e-03 m` and
  `6.023004265867e-03 m` under the changing source target.
- DEC-010 records latest-value and simulation-time semantics. No XR/PICO,
  adapter, hand retargeting, filtering, real robot, motor controller,
  trajectory, velocity/acceleration, collision, torque, or model edit was
  added.

**Files Changed**

- src/wheelloong_m2/simulation/runtime/__init__.py
- src/wheelloong_m2/simulation/runtime/config.py
- src/wheelloong_m2/simulation/runtime/target_buffer.py
- src/wheelloong_m2/simulation/runtime/scheduler.py
- experiments/test_wheelloong_m2_multirate_loop.py
- docs/experiments/EXP-010_WHEELLOONG_M2_MULTIRATE_LOOP.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S2.1. The runtime remains a fake-source MuJoCo simulation; do not
connect XR or a robot without separately scoped work.

### S3.0 — `wheelloong_m2` fake XR adapter abstraction established

**Action**

Added an SDK-independent XR controller pose type, deterministic fake XR
source, and separate adapter interface. Integrated fake controller samples
through the existing latest-value buffer, warm-started IK, and MuJoCo physics
loop without changing robot-side runtime modules.

**Result**

- `XRControllerPose` explicitly represents `^xr T_controller`; its source and
  adapter have no PICO, XRoboToolkit, OpenXR, or device dependency.
- The fake source is called at the shared 120 Hz target ticks. In the recorded
  two-second integration test it produced 240 sample pairs (480 controller
  poses), which generated 240 target updates, 500 IK solves, and 2000 physics
  steps.
- The adapter uses an explicitly synthetic identity-copy convention only; it
  is not coordinate calibration. Final simulated EE tracking remained the
  S2.1 fake-signal baseline (`7.23 mm` left and `6.02 mm` right position
  errors).
- DEC-011 records the abstraction boundary. No real XR source, PICO SDK,
  calibration, retargeting, scale, offset, filtering, or robot control was
  added.

**Files Changed**

- src/wheelloong_m2/xr/__init__.py
- src/wheelloong_m2/xr/types.py
- src/wheelloong_m2/xr/source.py
- src/wheelloong_m2/xr/adapter.py
- experiments/test_wheelloong_m2_xr_adapter.py
- docs/experiments/EXP-011_WHEELLOONG_M2_XR_ADAPTER.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Stop at S3.0. The adapter path is fake-source only; do not connect PICO or
replace the identity test convention without separately scoped work.

### S3.1 — `wheelloong_m2` XRoboToolkit/PICO source layer established

**Action**

Added a lazy-import XRoboToolkit source wrapper with explicit SDK lifecycle,
raw controller-pose parsing, `xyzw` quaternion-to-matrix conversion, SDK
timestamp conversion, and raw quaternion logging support. Added a 10-second
pose logger with optional interactive Pose A/B capture.

**Result**

- The installed `xrobotoolkit_sdk` distribution reports version `1.0.2`.
- The wrapper's device-independent identity-quaternion conversion check
  passed: test position was copied, timestamp was converted from ns to s, and
  `[0, 0, 0, 1]` produced an identity rotation.
- Live SDK initialization connected to the local PC Service, but the SDK
  timestamp remained zero during a full 10-second wait. Therefore no current
  PICO pose, sample count, actual rate, or physical Pose A/B delta is claimed.
- DEC-012 records that the wrapper remains raw and adapter-independent. No
  coordinate mapping, calibration, controller offset, retargeting, IK,
  MuJoCo change, or robot control was added.

**Files Changed**

- src/wheelloong_m2/xr/robotoolkit_source.py
- src/wheelloong_m2/xr/__init__.py
- experiments/test_robotoolkit_pose.py
- docs/experiments/EXP-012_XROBOToolKIT_SOURCE.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

The source can be rerun after the PICO XRoboToolkit app produces a nonzero
timestamp. Do not use its raw output for robot targets until a separately
scoped coordinate-mapping stage.

### Documentation Track — Phase 3 teleoperation coordinate mapping refined

**Action**

Retained the historical absolute XR-to-wrist chain in the coordinate document
and marked it deprecated/incomplete. Added the Phase 3 initialized-relative
controller-motion formulation and recorded the architectural decision.

**Result**

- Raw PICO controller pose remains `^D T_C(t)`; Phase 3 targets use
  `inverse(^D T_C(0)) * ^D T_C(t)` rather than direct absolute-pose mapping.
  This body-relative form was later superseded by the implemented spatial
  delta formulation in DEC-014 / EXP-014.
- The current robot-side scope is fixed-torso dual-arm operational frames;
  future waist/mobile-base work can move from `^torso T_EE` to `^base T_EE`.
- `^torso T_D` (future `^base T_D`) and `^C T_EE` remain Unknown calibration
  relationships. No numeric calibration or coordinate-conversion code was
  added.

**Files Changed**

- docs/COORDINATE_SYSTEM.md
- docs/DECISIONS.md
- docs/STATUS.md
- docs/WORKLOG.md

**Next**

Treat calibration and offline validation as a separately scoped Phase 3 task;
do not infer numeric transforms from this formulation alone.

### 2026-08-21 — Phase 3 started

**Action**

Formally documented Phase 3 — Unitree Coordinate Mapping and created its
dedicated Unitree mapping research contract.

**Result**

- Phase 2 remains closed with its raw XR coordinate evidence preserved.
- Started Unitree coordinate-mapping research focused on the mathematical
  transform chain from XR controller pose to Unitree wrist target.
- No robot-control implementation, coordinate-conversion code, or unverified
  transform was added.

**Next**

Inspect Unitree `xr_teleoperate` source, beginning with `TeleData`,
`TeleVuerWrapper`, and the wrist target passed to `robot_arm_ik`.

### 2026-08-21 — EXP-013 Unitree source mapping and IK inspection completed

**Action**

Cloned the two official upstream repositories named by the project owner,
pinned their current commits, and inspected TeleVuer coordinate conversion,
`TeleData` wrist semantics, the teleoperation call site, and Unitree's
Pinocchio/CasADi/IPOPT arm IK.

**Result**

- Recorded TeleVuer commit
  `766de45e74373ae0ea66321d942ce538385655a5` and `xr_teleoperate` commit
  `845b25a32f7febedf220e830952a7134897adb9d`; the latter pins the former as its
  submodule.
- Established Unitree's OpenXR-to-robot basis rotation as Source Evidence.
- Confirmed that the existing `wheelloong_m2` solver already matches the
  transferable Unitree NLP structure and objective defaults.
- Classified Unitree waist/wrist offsets and controller-local convention as
  upstream robot/source-specific rather than values to copy.
- Added EXP-013 and updated the environment/mapping evidence documents.

**Next**

Implement a named initialized mapping that uses the evidenced basis rotation
without assuming XRoboToolkit/PICO controller-local identity.

### 2026-08-21 — Initialized spatial-relative XR adapter implemented

**Action**

Added `RelativeXRMapping` and `InitializedRelativeXRAdapter`. Superseded the
earlier body-relative DEC-013 formula with the spatial-delta mapping in
DEC-014, keeping raw SDK acquisition unchanged.

**Result**

- Zero motion maps exactly to the captured robot `W_L` / `W_R` anchors.
- OpenXR `+X/+Y/+Z` deltas map to robot `-Y/+Z/-X` through the pinned Unitree
  basis rotation.
- Spatial rotation cancels a fixed controller-local rotational extrinsic and
  absolute tracking-origin translation cancels through initialization.
- Seven deterministic pytest cases pass. The globally installed ROS
  `launch_testing` plugin is incompatible with the installed pytest version,
  so project tests are run with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

**Files Changed**

- `src/wheelloong_m2/xr/adapter.py`
- `src/wheelloong_m2/xr/__init__.py`
- `tests/test_relative_xr_adapter.py`
- `docs/COORDINATE_SYSTEM.md`
- `docs/DECISIONS.md`

**Next**

Connect the adapter to the existing named IK/MuJoCo path and validate a
nonzero synthetic trajectory.

### 2026-08-21 — EXP-014 relative XR-to-MuJoCo simulation validated

**Action**

Added a reusable simulation-only XR runtime, deterministic end-to-end
experiment, and fake/live CLI. Ran the two-second synthetic trajectory and a
separate live XRoboToolkit startup attempt.

**Result**

- The deterministic run completed 240 target updates, 500 IK solves, and 2000
  MuJoCo physics steps through named arm position actuators.
- Final target deltas were about `-30 mm` left robot Y and `+25 mm` right robot
  Y, matching the evidenced OpenXR `X -> -robot Y` map.
- Final left/right EE position errors were approximately `9.70 mm` and
  `4.21 mm`; rotation errors were `0.00672 rad` and `0.00530 rad`.
- A live three-second startup attempt connected to the local PC Service, but
  the SDK timestamp remained non-positive. The application timed out and
  disconnected; no fake sample replaced the unavailable device data.
- Added EXP-014, refreshed STATUS/README, and retained the no-real-robot
  boundary.

**Files Changed**

- `src/wheelloong_m2/simulation/xr_mujoco_runtime.py`
- `experiments/test_wheelloong_m2_relative_xr_mujoco.py`
- `experiments/run_wheelloong_m2_xr_mujoco.py`
- `docs/experiments/EXP-014_WHEELLOONG_M2_RELATIVE_XR_MUJOCO.md`
- `docs/STATUS.md`
- `README.md`

**Next**

Start the PICO XRoboToolkit app so the SDK timestamp becomes positive, then
run the live MuJoCo-only CLI. Do not begin real robot integration before
tracking-loss, collision, velocity/acceleration, and hardware safety work is
separately authorized and validated.

### 2026-08-21 — Phase 3 regression completed

**Action**

Ran the new pytest suite plus the established Pinocchio FK, CasADi FK, SE(3)
math, NLP IK, static MuJoCo IK, multi-rate runtime, legacy fake adapter, and
initialized relative XR-to-MuJoCo experiments.

**Result**

- All invoked commands exited successfully.
- `tests/test_relative_xr_adapter.py`: 7 passed.
- Existing EXP-005 through EXP-011 numerical results remained consistent.
- EXP-014 repeated the recorded target counts and final EE errors.
- `git diff --check` passed.
- The environment has no `black` or `ruff` executable; no formatter/linter
  result is claimed.

**Next**

The only immediate device-side blocker is the non-positive XRoboToolkit SDK
timestamp. Resume live MuJoCo validation after the PICO app provides a valid
stream.

### 2026-08-21 — EXP-015 user teleoperation initial posture integrated

**Action**

Recorded the owner's supplied left/right MoveJ arrays and rates, checked the
project's motion-index/feedback mapping, validated all angles against named
URDF limits, and integrated the posture into the MuJoCo/XR startup path.

**Result**

- Added one immutable posture definition in public left-then-right `q_arm`
  order and updated the ROS arm-state viewer's local HOME display values.
- All 14 requested values are within URDF limits.
- Added simulation prepositioning through named position actuators only; 3000
  settling steps reached `0.011899541117 rad` total joint tracking error.
- A preliminary run exposed about `0.05 rad` stationary EE orientation error
  from the old neutral regularizer. Parameterized `q_nom` and selected the
  requested posture only in the new runtime; existing solver callers still
  default to model neutral.
- The final one-second relative-XR run completed 120 target updates, 250 IK
  solves, and 1000 teleoperation physics steps. Left/right EE position errors
  were approximately `6.38 mm`; rotation errors were about `0.01213 rad`.
- The complete pytest suite reports 9 passed. Existing neutral IK and EXP-014
  results remain unchanged.
- No ROS service call or physical robot command was executed. Supplied MoveJ
  rates are recorded but are not simulated as trajectory constraints.

**Files Changed**

- `src/wheelloong_m2/kinematics/postures.py`
- `src/wheelloong_m2/kinematics/__init__.py`
- `src/wheelloong_m2/ik/dual_arm_ik.py`
- `src/wheelloong_m2/simulation/xr_mujoco_runtime.py`
- `src/description/wheelloong_m2/scripts/view_m2_mujoco_arm_driver_state.py`
- `experiments/run_wheelloong_m2_xr_mujoco.py`
- `experiments/test_wheelloong_m2_user_initial_posture.py`
- `tests/test_teleop_initial_posture.py`
- `docs/experiments/EXP-015_USER_TELEOP_INITIAL_POSTURE.md`
- `docs/DECISIONS.md`
- `docs/STATUS.md`
- `README.md`

**Next**

Use the default fake-source viewer to inspect the requested posture visually.
Live PICO-to-MuJoCo validation still requires a positive SDK timestamp. Real
driver commanding remains outside the current authorized scope.
