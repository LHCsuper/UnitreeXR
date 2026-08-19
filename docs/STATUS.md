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
- `xrt.init()` connects successfully to the local PC Service.
- The Python SDK connects to PC Service at `127.0.0.1:60061`.
- XR timestamp returns a non-zero value and keeps changing.
- Headset, left controller, and right controller poses are readable and change with physical movement.
- All three poses are arrays of length 7.
- `xrt.close()` executes normally.

## Not Yet Verified

- Position unit.
- XYZ directions.
- Coordinate system handedness.
- Tracking origin.
- Quaternion convention.
- Controller local frame.
- Recenter behavior.
- XRoboToolkit → Unitree mapping.