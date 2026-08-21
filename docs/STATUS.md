# UnitreeXR Status

## Current Phase

Phase 3 — Unitree Coordinate Mapping

Phase 2 is closed.

## Current Goal

Use the completed Unitree source inspection and initialized relative adapter
to validate live PICO controller motion in MuJoCo. No real robot integration
is authorized.

The core Phase 3 research quantity is:

```text
^base T_wrist_target
```

The following factorization is an **Unknown / Hypothesis** research
decomposition, not a confirmed transform chain or an implementation:

```text
^base T_wrist_target
=
^base T_XR
*
^XR T_controller
*
^controller T_wrist
```

This factorization is retained only as a historical absolute-pose research
decomposition. The implemented spatial-relative mapping and its evidence are
recorded in `UNITREE_COORDINATE_MAPPING.md`, `COORDINATE_SYSTEM.md`, EXP-013,
and EXP-014.

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

## Established — `wheelloong_m2` Multi-Rate Teleoperation Simulation Runtime

- EXP-010 adds a latest-value-only dual-arm SE(3) target buffer and a
  simulation-time scheduler using shared `120 Hz` target, `250 Hz` IK, and
  `1000 Hz` MuJoCo physics configuration.
- The deterministic 2-second fake-target run recorded exactly 240 target
  updates, 500 IPOPT solves, and 2000 physics steps. Target updates precede
  IK when both ticks are due; IK warm-starts from its last `q_arm` target.
- Recorded IK solve latency was `4.168258 ms` mean, `4.546778 ms` p95, and
  `15.083930 ms` maximum. These are offline solver wall-clock measurements,
  not end-to-end teleoperation latency claims.
- Final simulated joint tracking was `2.110591184150e-02 rad`; Pinocchio FK
  measured left/right EE position errors of approximately `7.23 mm` / `6.02
  mm` for the final changing-target state.
- The runtime is simulation-only: no XR/PICO, coordinate adapter, hand
  retargeting, filtering, real robot, motor controller, trajectory,
  velocity/acceleration limit, collision avoidance, torque policy, model
  edit, or robot-control path is added.

## Established — `wheelloong_m2` Fake XR Adapter Abstraction

- EXP-011 introduces an SDK-independent `XRControllerPose` contract for
  `^xr T_controller`, a fake source at the shared 120 Hz target rate, and an
  `XRAdapter` interface that emits Pinocchio robot target poses.
- The original `XRAdapter` remains deliberately identity-copy only for its
  synthetic regression test. It is not a PICO/XRoboToolkit/OpenXR convention, XR-to-torso
  calibration, controller-to-wrist transform, axis mapping, scale, or offset.
- A 2-second fake-source integration run delivered 240 XR sample pairs through
  the adapter and S2.1 buffer, followed by 500 IK solves and 2000 physics
  steps; final tracking matched the existing fake-signal baseline.
- S3.0 itself added no PICO SDK, XRoboToolkit, OpenXR, coordinate calibration,
  hand retargeting, filtering, real device, model edit, or robot-control path;
  the later S3.1 raw source wrapper is recorded separately below.

## Established — `wheelloong_m2` XRoboToolkit Source Wrapper

- EXP-012 adds `XRoboToolkitSource.connect/sample/disconnect` and converts
  raw SDK controller arrays `[x, y, z, qx, qy, qz, qw]` into the existing
  `XRControllerPose` with a seconds timestamp and a rotation matrix.
- SDK position values are copied without a robot-frame transform, scale,
  offset, axis conversion, or calibration. Raw `xyzw` quaternions are retained
  for source logging.
- The installed `xrobotoolkit_sdk` distribution is `1.0.2`. In the recorded
  10-second live attempt, SDK initialization connected to local PC Service but
  `get_time_stamp_ns()` remained zero, so current pose acquisition, sample
  rate, and interactive Pose A/B validation remain unavailable.
- No XRAdapter, coordinate transform, hand retargeting, IK, MuJoCo module,
  model file, or real robot path is modified.

## Established — Unitree TeleVuer / IK Source Mapping

- EXP-013 pins TeleVuer commit
  `766de45e74373ae0ea66321d942ce538385655a5` and `xr_teleoperate` commit
  `845b25a32f7febedf220e830952a7134897adb9d`.
- Unitree's OpenXR-to-robot proper basis rotation is established as Source
  Evidence: OpenXR `+X/+Y/+Z` maps to robot `-Y/+Z/-X`.
- `TeleData.left_wrist_pose` / `right_wrist_pose` are head-yaw/waist-relative
  4x4 targets passed directly to `robot_arm_ik.solve_ik`.
- The existing `wheelloong_m2` solver matches the transferable Unitree IK
  structure: robot-specific operational frames, independent CasADi FK,
  current-minus-target position and SO(3)-log errors, the same default
  objective weights, URDF bounds, IPOPT, and warm start.
- Unitree G1/H1/H2 joint names, wrist offsets, waist offsets, controller-local
  convention assumption, feed-forward torque, and real robot control are not
  copied.

## Established — Initialized Relative XR-to-MuJoCo Path

- `InitializedRelativeXRAdapter` anchors each raw controller pose to the
  current simulated `W_L` / `W_R` pose, then maps spatial relative translation
  and rotation through the source-evidenced proper basis rotation.
- Zero controller motion exactly preserves both robot operational anchors;
  arbitrary absolute Tracking Origin translation and fixed controller-local
  rotational extrinsics cancel in deterministic tests.
- Seven adapter tests pass, including all three axis maps and explicit
  translation scaling.
- EXP-014 completes the simulation-only chain from arbitrary fake raw XR
  poses through the relative adapter, 14-DOF IK, named MuJoCo position
  actuators, and Pinocchio FK validation.
- The two-second run produced 240 target updates, 500 IK solves, and 2000
  physics steps. Final left/right EE errors were approximately `9.70 mm` /
  `4.21 mm` and `0.00672 rad` / `0.00530 rad`.
- A runnable CLI supports fake input and live `XRoboToolkitSource` input to
  MuJoCo only. It never imports a robot SDK or sends a physical command.
- A current three-second live startup attempt connected to PC Service but the
  SDK timestamp remained non-positive, so the CLI timed out and disconnected
  without substituting fake data. Live PICO motion remains unverified.

## Implemented — Phase 3 Teleoperation Mapping Formulation

- Teleoperation targets are formulated from initialized spatial translation
  `p(t)-p(0)` and rotation `R(t)R(0)^T`, rather than an absolute XR pose.
- The current robot-side validation scope is fixed-torso dual-arm operation
  with `^torso T_WL` / `^torso T_WR`; a future waist/mobile-base scope can use
  `^base T_EE`.
- Absolute `^torso T_D` (future `^base T_D`) and physical `^C T_EE` remain
  Unknown calibration relationships. The initialized relative implementation
  does not require them and makes no absolute calibration claim.

## Established — User-Specified Teleoperation Initial Posture

- EXP-015 records the owner's supplied left/right seven-joint MoveJ arrays in
  the established public order and retains the supplied `0.5/0.5` left and
  `0.8/0.8` right velocity/acceleration values as provenance metadata.
- Project source maps driver feedback indices `0..6` directly to named arm
  joints `1..7` without a sign conversion. The real service-command-to-
  feedback path remains not independently exercised.
- All 14 requested joint values are inside named URDF limits.
- The MuJoCo CLI now defaults to this posture, commands it only through named
  position actuators, settles for 3000 physics steps, and anchors XR from the
  post-settling state. Model-neutral startup remains explicit via
  `--initial-posture neutral`.
- The IK solver retains model neutral by default but accepts a limit-validated
  `q_nom` override; the XR simulation supplies the requested posture so the
  soft regularizer does not pull stationary targets toward zero.
- The recorded three-second preposition reached `0.0118995 rad` total joint
  error. The following one-second XR/IK simulation ended at about `6.38 mm`
  position and `0.01213 rad` rotation error for each arm.
- Nine unit tests and the prior neutral-posture IK/EXP-014 regressions pass.
- No ROS service call or real robot command was executed.

## Hypothesis

PICO Runtime may use gravity-related inertial information as an important
vertical reference.

## Unknown

- Exact PICO vertical-direction estimation.
- Exact IMU / vision / SLAM fusion.

## Not Yet Verified

- Precise Head local physical-frame calibration (if later required).
- Absolute `^torso T_D` (future `^base T_D`).
- Physical `^C T_EE` calibration.
- Live PICO/XRoboToolkit → relative adapter → MuJoCo motion validation.
- Tracking-loss and Home-recenter handling beyond required reinitialization.
- Collision avoidance and task-specific translation gain validation.
- Real `/arm_driver/joint_move` command-to-feedback equivalence for the
  supplied arrays; only the checked-in feedback-to-model map is established.
- Real robot integration.
