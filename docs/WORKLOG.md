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
gripper-center candidate. Recorded as EXP-004.

**Result**

- `arm_link_7` is the wrist-roll joint frame, not the grasp center.
- No explicit palm / gripper_base / tool_center link exists.
- Four direct gripper joints mount distally to each `arm_link_7` (mean
  offset `0.220210 m`).
- Candidate grasp center: `^arm7 p = [-0.0365, ±0.21691, 0.0105]`.
- Candidate orientation axes (directions only): `z` = joint axis, `x` =
  closing, `y` = finger extension. Sign conventions remain Unknown.
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
