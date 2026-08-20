# UnitreeXR Status

## Current Phase

Phase 3 — Unitree Coordinate Mapping

Phase 2 is closed.

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

## Established — Raw XR Pose Convention

- XR raw pose format: `[x, y, z, qx, qy, qz, qw]`.
- Tracking-frame axis convention: `+X = right`, `+Y = up`, `+Z = backward` (`-Z = forward`).
- Right-handedness.
- Position unit consistency: values are consistent with meters; not a precision scale calibration.
- Device Tracking Origin configuration from source inspection.
- Quaternion component order: `[qx, qy, qz, qw]`.
- Quaternion transform direction: `v_D = ^D R_device * v_device`.
- Controller local-frame convention.
- Home recenter behavior.
- Home recenter vertical-axis experiment.

Home recenter vertical-axis experiment results:

```text
Trial 1: 1.0307 deg
Trial 2: 0.4118 deg
Trial 3: 1.0421 deg
```

## Established — `wheelloong_m2` Baseline Operational EE Frames

- EXP-004/S0.5b defines logical teleoperation operational EE frames `W_L`
  and `W_R` for later baseline arm IK.
- Their origins are fixed gripper-root/palm-center operational points formed
  from the mean of the four direct gripper root joint origins.
- `+Y_W` means physical finger extension, `+Z_W` follows the positive direct
  hinge axis, and `+X_W = y_W cross z_W`.
- The derived left extension direction is approximately `+L7 Y`; the right
  is approximately `-R7 Y`.
- The frames are not calibrated fingertip TCPs.
- Rotation orthogonality, determinant `+1`, and right-handed cross-product
  checks pass offline.
- No arm IK implementation, XR connection, model edit, or robot control has
  been introduced.

## Hypothesis

PICO Runtime may use gravity-related inertial information as an important
vertical reference.

## Unknown

- Exact PICO vertical-direction estimation.
- Exact IMU / vision / SLAM fusion.

## Not Yet Verified

- Precise Head local physical-frame calibration (if later required).
- XRoboToolkit → Unitree coordinate mapping.
- `^base T_xr`.
- `^controller T_wrist`.
- Offline Unitree wrist-target validation.
- Real robot integration.
