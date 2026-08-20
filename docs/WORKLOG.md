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
