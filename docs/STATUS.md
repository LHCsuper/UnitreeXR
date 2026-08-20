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

## Established — `wheelloong_m2` Pinocchio Kinematics Backbone

- EXP-005 provides a 14-DOF named `q_arm` interface: seven left-arm joints
  followed by seven right-arm joints.
- The module resolves public arm indices to Pinocchio configuration and
  velocity indices by joint name rather than exposing the backend layout.
- FK returns the fixed S0.5b operational frames as `^torso T_WL` and
  `^torso T_WR`.
- Jacobians have shape `(6, 14)`, use rows `[linear; angular]`, and are
  explicitly expressed in torso axes at the operational EE origins.
- Fixed-seed legal-limit sampling and a finite-difference check run without
  exception; residuals are at double-precision scale for the checked joint.
- No IK, optimizer, XR path, MuJoCo controller, model edit, or robot control
  has been introduced.

## Established — `wheelloong_m2` CasADi Symbolic FK Backbone

- EXP-006 builds an independent CasADi/`pinocchio.casadi` symbolic FK path
  for the unchanged 14-DOF `q_arm` and S0.5b `W_L` / `W_R` contract.
- Symbolic full configuration has `nq=42`, starts at Pinocchio neutral, and
  scatters only the 14 named arm symbols into their resolved q indices.
- The CasADi function returns `^torso T_WL` / `^torso T_WR` components, not
  world-frame poses.
- Zero and fixed-seed legal-random cases agree with numeric Pinocchio FK at
  zero position error and floating-point-scale rotation error.
- No IK, cost, NLP, IPOPT, Opti, solver, XR path, controller, model edit, or
  robot control has been introduced.

## Established — `wheelloong_m2` SE(3) IK Math Foundation

- EXP-007 defines solver-free per-arm torso-frame errors as
  `p_current - p_target` and `Log(R_current * R_target^T)` for the unchanged
  logical operational frames `W_L` / `W_R`.
- Numeric errors use Pinocchio `log3`; CasADi has an independent symbolic
  SO(3)-log expression with a small-angle branch.
- The numeric dual-arm objective is decomposed into pose,
  nominal-configuration regularization, and previous-configuration smoothness
  terms with caller-provided `IKWeights`.
- Deterministic checks cover zero, translation, a positive-Z 30-degree
  rotation error, random `exp(log(R))` reconstruction, numeric-symbolic
  agreement, and cost decomposition.
- S1.2.1 consumes this math in an offline CasADi Opti/IPOPT baseline; no XR,
  controller, URDF/MJCF edit, or robot control is introduced.

## Established — `wheelloong_m2` Offline Dual-Arm NLP IK Baseline

- EXP-008 adds a reusable CasADi Opti/IPOPT solve over named `q_arm(14)` for
  the unchanged torso-relative operational targets `^torso T_WL` / `^torso T_WR`.
- It reuses the independent symbolic FK and S1.2.0 CasADi SE(3) error. Targets
  are Opti parameters set per call; `q_init` supports warm start and `q_prev`
  supplies the smoothness reference.
- The only constraints are finite URDF joint-position bounds in the public
  arm order. The objective is a soft pose, neutral-regularization, and
  smoothness cost.
- Zero, fixed-seed reachable random, and simultaneous dual-arm target cases
  reported IPOPT success and limit-respecting solutions in offline tests.
- S2.0 consumes the offline solver in a MuJoCo-only position-actuator loop;
  no XR/PICO, coordinate adapter, model edit, or robot control is introduced.

## Established — `wheelloong_m2` MuJoCo IK Simulation Loop

- EXP-009 loads the existing controlled MJCF and maps each named `q_arm`
  joint to its loaded qpos address and its single loaded position-actuator
  ctrl address; no numeric-index assumption is used.
- IK output is written only to `data.ctrl`, and the model advances by
  `mj_step`; the loop never writes arm `data.qpos` targets directly.
- Neutral and unilateral left/right target experiments loaded successfully,
  reported IPOPT success, tracked their arm targets within approximately
  `1.3e-3` to `1.7e-3 rad`, and produced the expected unilateral simulated
  arm motion.
- Final simulated named qpos values are evaluated by the existing Pinocchio
  FK for torso-relative `W_L` / `W_R` validation.
- This remains simulation-only: there is no XR/PICO, coordinate adapter,
  real robot, low-level motor controller, trajectory generator,
  velocity/acceleration limit, collision avoidance, torque policy, URDF/MJCF
  edit, or robot-control path.

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
