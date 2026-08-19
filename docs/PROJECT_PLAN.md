# Project Plan

## Phase 0 — Workspace Bootstrap

Establish the repository structure, logging rules, and environment records.

## Phase 1 — XR Data Pipeline Validation

Validate the pipeline:

```text
PICO
→ XRoboToolkit APK
→ PC Service
→ PXREARobotSDK
→ xrobotoolkit_sdk
→ Python
```

## Phase 2 — PICO Coordinate Calibration

Validate:

- position unit
- world XYZ
- handedness
- tracking origin
- quaternion convention
- controller local frame
- recenter behavior

## Phase 3 — Unitree Coordinate Mapping

Study the mathematical mapping between XRoboToolkit Pose and Unitree `xr_teleoperate` / `TeleData`.

## Phase 4 — Offline Adapter Validation

Validate offline without connecting to a real robot:

```text
XR Pose
→ Adapter
→ Unitree wrist target
```

## Phase 5 — Robot Integration

Only begin after the earlier phases have been verified.