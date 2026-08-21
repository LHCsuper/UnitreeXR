# Unitree Coordinate Mapping Research

## Objective

Phase 3 studies the mathematical relationship between the established raw
XRoboToolkit controller pose and the target pose consumed by Unitree
`xr_teleoperate`:

```text
XRoboToolkit pose
    -> Unitree xr_teleoperate
    -> TeleData wrist pose
    -> robot_arm_ik target
```

This document records the Unitree source evidence and the parts transferred to
the simulation-only `wheelloong_m2` adapter. It defines no real robot control.

## Coordinate Chain

All transforms use robotics notation: `^a T_b` is the pose of frame `b`
expressed in frame `a`, and `^a R_b` is its rotation component.

For this research document, define:

```text
XR          = Unitree-side XR reference frame to be identified from source
controller  = XR controller local frame
wrist       = Unitree wrist target frame to be identified from source
base        = Unitree robot base reference frame to be identified from source
```

The original research decomposition was:

```text
^base T_wrist_target
=
^base T_XR
*
^XR T_controller
*
^controller T_wrist
```

Its corresponding rotation composition is:

```text
^base R_wrist_target
=
^base R_XR
*
^XR R_controller
*
^controller R_wrist
```

This is an **incomplete historical decomposition**, not the implemented
teleoperation mapping. The implemented initialized spatial-relative mapping
is recorded in `COORDINATE_SYSTEM.md` and EXP-014.

The Phase 2 raw pose is currently written as `^D T_C`, where `D` is the PICO
Device Tracking Origin and `C` is the observed controller frame. No evidence
yet establishes that the Unitree research placeholder `XR` equals `D`, or that
`controller` equals `C` without a fixed convention check.

The related pose names are therefore scoped as follows:

```text
^XR T_controller  = XR reference to controller pose; Unitree-side semantics Unknown
^controller T_wrist = controller to wrist fixed transform; Unknown
^base T_wrist      = robot base to wrist pose; target-frame semantics Unknown
```

## Unitree Source Evidence

Source inspection is pinned to:

```text
TeleVuer:       766de45e74373ae0ea66321d942ce538385655a5
xr_teleoperate: 845b25a32f7febedf220e830952a7134897adb9d
```

The current `xr_teleoperate` tree pins the same TeleVuer commit as a submodule.

TeleVuer defines OpenXR basis `(+X right, +Y up, +Z back)` and robot basis
`(+X front, +Y left, +Z up)`. Its proper basis rotation is:

```text
S =
[[ 0, 0,-1],
 [-1, 0, 0],
 [ 0, 1, 0]]
```

It changes an OpenXR pose basis with `R_robot = S R_openxr S^T` and
`p_robot = S p_openxr`. Its default `head_yaw` mode then expresses arm poses
relative to head position and yaw before adding Unitree-specific waist
offsets. `TeleData.left_wrist_pose` and `right_wrist_pose` are the resulting
4x4 targets passed directly to `robot_arm_ik.solve_ik`.

The G1 IK source builds robot-specific operational frames and a CasADi/IPOPT
problem with `p_current-p_target`, `Log(R_current R_target^T)`, joint bounds,
zero-pose regularization, and previous-solution smoothing. EXP-013 records the
full evidence and migration assessment.

## Migrated Mapping

This project reuses only the source-evidenced proper basis rotation `S`. It
does not copy Unitree waist offsets, wrist offsets, or the TeleVuer assertion
about controller-local initial axes.

For each arm it captures a raw PICO controller pose and the current robot
operational pose at initialization, then maps spatial relative motion:

```text
delta p_D = ^D p_C(t) - ^D p_C(0)
delta R_D = ^D R_C(t) * ^D R_C(0)^T

^torso p_W(t) = ^torso p_W(0) + scale * S * delta p_D
^torso R_W(t) = S * delta R_D * S^T * ^torso R_W(0)
```

This is implemented in `InitializedRelativeXRAdapter` and validated offline
in EXP-014. It is deliberately different from TeleVuer's current head-yaw
absolute target construction.

## Current Unknowns

The following items are **Unknown** and are the primary Phase 3 research
questions:

1. Physical PICO-controller-to-human-hand calibration.
2. Whether XRoboToolkit/PICO controller-local axes match TeleVuer/WebXR
   controller-local axes; the implemented spatial-relative mapping does not
   require that identity.
3. Exact live behavior during XR tracking loss or a PICO Home recenter.
4. A validated translation gain for a specific operator and task.
5. Collision, velocity, and acceleration policies for larger motions.
6. Real-device end-to-end motion validation; the last recorded SDK attempt
   had a zero timestamp.
7. Real robot integration, which remains prohibited in the current scope.

## Evidence Level

### Confirmed Fact

- `^a T_b` and `^a R_b` are the notation used in this document: frame `b` is
  expressed in frame `a`.
- This document does not add a coordinate conversion or robot-control path.

### Source Evidence

- Unitree basis conversion, head-yaw/waist target construction, `TeleData`
  wrist outputs, direct IK call site, operational-frame construction, and NLP
  objective structure are recorded in EXP-013 at pinned upstream commits.

### Experimental Evidence

- Phase 2 records the PICO/XRoboToolkit raw controller pose convention as
  `^D T_C`, including the observed quaternion order and controller-frame
  observations. See `COORDINATE_SYSTEM.md` and EXP-002.
- EXP-014 validates initialized zero motion, the three basis-axis mappings,
  fixed controller-local extrinsic cancellation, and the full
  XR-to-IK-to-MuJoCo synthetic path.

### Hypothesis

- A task/operator-specific translation gain may improve reachable workspace
  use, but no value other than the explicit neutral default `1.0` is validated.

### Unknown

- The physical and live-device items listed in **Current Unknowns** remain
  Unknown.

## Research Boundary

The raw XR acquisition layer remains unchanged. All conversion is named in
the adapter, and all execution is limited to offline MuJoCo. This work does
not authorize or add real robot control.
