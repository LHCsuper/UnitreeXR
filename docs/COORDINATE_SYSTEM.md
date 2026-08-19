# PICO / XRoboToolkit Coordinate System

This document is the canonical definition of the raw PICO 4 Ultra +
XRoboToolkit pose convention. It is the single source of truth for
coordinate-related claims in this repository.

## Evidence Levels

- **Confirmed Fact** — Stated by an authoritative specification, official
  documentation, or an unambiguous definitional fact.
- **Source Evidence** — Established by inspecting upstream source code or
  configuration.
- **Experimental Evidence** — Established by this project's live device
  experiments or observations.
- **Hypothesis** — A plausible engineering inference supported by an
  evidence chain but not yet directly confirmed.
- **Unknown** — No evidence yet, or not yet verified.

Do not promote a Hypothesis to Fact without direct confirmation.

## Raw Pose Format

XRoboToolkit Pose:

```text
[x, y, z, qx, qy, qz, qw]
```

- position order = `[x, y, z]` — Experimental Evidence (EXP-002).
- position unit = `meter` — Experimental Evidence. Consistent with meters;
  not a high-precision scale calibration.
- quaternion order = `[qx, qy, qz, qw]` — Experimental Evidence.

Tracking Origin axes (Experimental Evidence, EXP-002):

```text
+X = right
-X = left

+Y = up
-Y = down

+Z = backward
-Z = forward
```

The frame is right-handed.

## Pose Transform Semantics

Let:

```text
D = current PICO Device Tracking Origin
```

Device poses are expressed in `D`:

```text
Head:            ^D T_H
Left Controller: ^D T_L
Right Controller:^D T_R
```

The quaternion represents:

```text
^D R_device
```

meaning:

```text
v_D = ^D R_device * v_device
```

This is not the inverse direction.

The rotation matrix columns are:

```text
R[:,0] = device local +X expressed in D
R[:,1] = device local +Y expressed in D
R[:,2] = device local +Z expressed in D
```

`q` and `-q` represent the same orientation.

## Tracking Origin

Source Evidence:

- The XRoboToolkit Unity project configures XR Origin as
  **Device Tracking Origin**, not Floor Tracking Origin.

Evidence boundary:

- This comes from upstream source/configuration inspection.
- The currently installed APK has not been binary-hash-matched against that
  source. Do not overstate this as a confirmed runtime fact.

Current device observation:

- After entering the XRoboToolkit APK, the PICO Runtime has established a
  Tracking Origin.
- `xrt.init()` does not create this Tracking Origin.
- The Head / Left / Right poses read from Python are already expressed in
  that Tracking Origin.

The Device Tracking Origin must not be described as "the currently moving
Head frame".

## Home Recenter

Long-pressing the PICO Home button triggers a recenter.

Current working interpretation:

- Home rebuilds the Tracking Origin.
- Its primary effect is to redefine the horizontal forward reference.
- Subsequent Head / Left / Right poses are expressed relative to the new
  Tracking Origin.
- In the current Device mode experiment, Head position was observed to be
  close to `[0, 0, 0]` after Home — Experimental Evidence.

Do not state that "PICO officially guarantees Home zeroes all XYZ".

### Reference-frame interpretation

Let `D0` be the old Tracking Origin and `D1` the new Tracking Origin.

For a tracked object `C` that does not move in reality:

```text
^D0 T_C = ^D0 T_D1 * ^D1 T_C
```

Therefore:

```text
^D0 T_D1 = ^D0 T_C * inverse(^D1 T_C)
```

Recenter is a change of the shared reference frame; it is not an
independent zeroing of each Head / Left / Right pose.

## Home Recenter: Vertical Axis Experiment

Objective: verify whether a Home recenter makes the new Tracking Origin
inherit the HMD's downward/upward pitch or lateral roll, thereby tilting
the `+Y` axis.

Repeated experiment results:

```text
Trial 1: Y-axis tilt = 1.0307 deg
Trial 2: Y-axis tilt = 0.4118 deg
Trial 3: Y-axis tilt = 1.0421 deg
```

Conclusion (Experimental Evidence):

- Before and after Home recenter, the old and new Tracking Origin `+Y`
  axes remain approximately parallel.
- HMD downward/upward tilt does not substantially tilt the Tracking Origin
  `+Y`.
- HMD lateral roll does not substantially tilt the Tracking Origin `+Y`.

The measured old/new Tracking Origin `+Y` deviation remained around
1 degree or below in the repeated experiment.

Do not state this as an absolute mathematical "exactly unchanged".

Outlier note:

- An earlier `Trial 3` measured `13.1965 deg`, but it did not reappear in
  the repeated experiment.
- Because this experiment assumes the HMD physical pose stays fixed across
  the Home action, that outlier is more likely caused by manual head motion.
- Keep it as a motion-sensitive outlier, not as evidence of Tracking Origin
  `Y` tilt.

## Hypothesis: Source of the Vertical Direction

This section is a **Hypothesis**, not a Fact.

Evidence chain:

1. Experimental Evidence — During Home recenter, Tracking Origin `+Y`
   remains approximately unchanged and does not inherit HMD pitch or roll.
2. Device capability / engineering context — PICO is a 6DoF XR device with
   inertial sensing / IMU capability.
3. Engineering reasoning — For an inertial tracking system, the gravity
   direction naturally provides a real-world vertical reference.

Reasonable hypothesis:

- PICO Runtime may use gravity-related information as one important
  reference for maintaining the vertical direction of the Tracking Origin.

Approximate conceptual form:

```text
+Y_tracking ≈ opposite(gravity_direction)
```

This is a Hypothesis, not a Confirmed Fact or Experimental Evidence.

Do not state:

- "PICO +Y is determined by the gravity sensor", or
- "PICO directly uses the accelerometer gravity vector as +Y".

There is currently no official evidence for those claims.

Unknown:

- Whether PICO Runtime directly uses the gravity vector.
- Whether the vertical direction is determined solely by the IMU.
- How accelerometer / gyroscope / camera / SLAM are fused.
- Whether additional floor or spatial state estimation exists.
- The specific weight of each sensor in that estimate.

A reasonable engineering hypothesis is that the PICO runtime uses
gravity-related inertial information as one important reference for
maintaining the vertical direction of the Tracking Origin. The exact
internal estimation and fusion method remains unknown.

## Right Controller Local Frame

Experimental Evidence.

- `+X_controller` — roughly perpendicular to the palm plane, pointing into
  the right palm.
- `-X_controller` — from inside the palm outward.
- `-Z_controller` — along the main grip direction, roughly from the grip
  tail / little-finger side toward the controller top / thumb side.
- `+Z_controller` — the opposite direction, roughly from the controller
  top / thumb side toward the grip tail / little-finger side.
- `+Y_controller` — determined by the right-handed rule.

Notes:

- "Along the grip" is a physical approximation only.
- Do not assume the tracking frame is strictly collinear with the plastic
  shell.

## Left Controller Local Frame

Experimental Evidence.

Based on live MeshCat observation:

- In the current XRoboToolkit / PICO output, the Left and Right controller
  local-frame numerical convention is the same.

Engineering conclusion:

- In the raw XR pose layer, do not apply an extra reflection, mirror, or
  sign flip just because the data came from the Left Controller.

Limitation:

- This describes only the current actual XRoboToolkit / PICO output.
- Do not generalize it to "all OpenXR left/right controller local frames
  are identical".

## Head Local Frame

- Head local physical axes = not independently calibrated / Unknown.
- Head quaternion order is confirmed.
- Head transform direction is confirmed.
- Only the precise physical relationship between the Head local frame and
  the headset shell axes has not been independently calibrated.

## Phase 2 Boundary

Phase 2 confirms only the raw XR pose convention.

The current evidence is sufficient to proceed to Phase 3.

Still unresolved:

```text
^base T_xr

^controller T_wrist
```

The next-phase core relationship:

```text
^base T_wrist_target
=
^base T_xr
*
^xr T_controller
*
^controller T_wrist
```

These belong to Phase 3 — Unitree Coordinate Mapping.