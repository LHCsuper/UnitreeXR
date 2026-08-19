# UnitreeXR Status

## Current Phase

Phase 0 — Workspace Bootstrap

## Hardware

VR: PICO 4 Ultra
PC OS: Ubuntu 22.04 x86_64

## XR Software

XRoboToolkit PC Service SDK
XRoboToolkit Python Pybind: `xrobotoolkit_sdk`
Runtime:
    `/usr/local/lib/libPXREARobotSDK.so`

## Confirmed

- Python module `xrobotoolkit_sdk` has been installed.
- PICO has XRoboToolkit APK installed.
- The APK can display Head / Controller / Hand tracking items.
- The workspace is being initialized for long-term experiments.

## Not Yet Verified

- Whether Python can continuously receive real-time PICO Pose.
- Pose position units.
- World coordinate XYZ definitions.
- Coordinate system handedness.
- Tracking origin.
- Quaternion array layout and rotation direction.
- Controller local frame.
- Recenter behavior.
- XRoboToolkit → Unitree mapping.