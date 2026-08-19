# UnitreeXR Status

## Current Phase

Phase 1 — XR Data Pipeline Validation

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
- The Phase 0 workspace skeleton has been initialized.

## Not Yet Verified

- Whether Python can continuously receive real-time PICO Pose.
- Position unit.
- XYZ directions.
- Coordinate system handedness.
- Tracking origin.
- Quaternion convention.
- Controller local frame.
- Recenter behavior.
- XRoboToolkit → Unitree mapping.