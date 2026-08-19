# UnitreeXR Status

## Current Phase

Phase 2 — PICO Coordinate Calibration

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
- XR position order is `[X, Y, Z]` for `pose[0]`, `pose[1]`, `pose[2]`.
- XR position axes: `+X = right`, `+Y = up`, `+Z = backward` (`-Z = forward`).
- XR position frame is right-handed.
- XR position values are consistent with meters; EXP-002 was not a precision scale calibration.

## Not Yet Verified

- Tracking origin.
- Quaternion component order.
- Quaternion rotation direction / transform semantics.
- Controller local frame.
- Recenter behavior.
- XRoboToolkit → Unitree mapping.