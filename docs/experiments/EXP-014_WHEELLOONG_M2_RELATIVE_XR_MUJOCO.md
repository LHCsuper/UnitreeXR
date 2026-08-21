# EXP-014 — Initialized Relative XR Mapping and MuJoCo Validation

## Objective

Connect the established raw `XRControllerPose` contract to the existing
`wheelloong_m2` dual-arm IK and MuJoCo actuator loop without absolute XR pose
copying, an unverified controller-local extrinsic, or any real robot path.

## Mapping

At initialization, capture controller pose `(^D p_C0, ^D R_C0)` and the
current robot operational target `(^torso p_W0, ^torso R_W0)` for each arm.
For a current controller pose, compute spatial relative motion:

```text
delta p_D = ^D p_C(t) - ^D p_C(0)
delta R_D = ^D R_C(t) * ^D R_C(0)^T
```

With the Unitree-source-evidenced OpenXR-to-robot basis rotation `S`:

```text
^torso p_W(t) = ^torso p_W(0) + scale * S * delta p_D
^torso R_W(t) = S * delta R_D * S^T * ^torso R_W(0)
```

The explicit default translation scale is `1.0`. The spatial rotation form
cancels a fixed controller-local extrinsic present in both samples; it does
not assert that PICO controller axes equal the TeleVuer controller convention.

Initialization maps exactly to the current simulated `W_L` / `W_R` poses. A
PICO Home recenter or tracking-origin discontinuity requires reinitialization.

## Unit tests

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q tests/test_relative_xr_adapter.py
```

Results:

```text
7 passed
```

The tests cover:

- `S` orthogonality, determinant `+1`, and all three axis mappings;
- required explicit initialization;
- exact zero-motion preservation of both robot anchors;
- explicit translation scale;
- arbitrary absolute tracking-origin translation cancellation; and
- fixed controller-local rotational-extrinsic cancellation.

The environment variable disables incompatible globally installed ROS
`launch_testing` pytest plugin auto-loading; it does not skip project tests.

## End-to-end MuJoCo experiment

Run:

```bash
python3 experiments/test_wheelloong_m2_relative_xr_mujoco.py
```

The fake source starts from deliberately arbitrary absolute XR values rather
than numerically copying robot targets. Over two seconds it drives a slow
left/right raw XR `+X/-X` motion. The source-evidenced basis maps these to
robot `-Y/+Y` motion. This recorded EXP-014 run uses model-neutral robot
initialization. EXP-015 later adds the user-specified CLI default posture
without changing these baseline results.

Recorded results:

```text
target updates: 240 (120 Hz)
IK solves:       500 (250 Hz)
physics steps:  2000 (1000 Hz)

final left target delta:  [0, -0.029999407826, 0] m
final right target delta: [0,  0.024999506521, 0] m

joint tracking error: 0.004514583437 rad
left EE error:  0.009704246110 m / 0.006716358251 rad
right EE error: 0.004210479512 m / 0.005300970142 rad
```

Both EE position errors passed the experiment's `20 mm` bound and both
rotation errors passed its `0.03 rad` bound. These bounds validate this
moderate synthetic trajectory; they are not a general workspace guarantee.

## Runnable application

Headless synthetic simulation:

```bash
python3 experiments/run_wheelloong_m2_xr_mujoco.py --source fake --duration 5
```

Live PICO/XRoboToolkit input to MuJoCo only:

```bash
python3 experiments/run_wheelloong_m2_xr_mujoco.py \
  --source robotoolkit --duration 30 --visualize
```

`--translation-scale` is explicit and defaults to `1.0`. Live mode waits for
a positive SDK timestamp, anchors the first valid controller pair to the
current simulated operational frames, wall-clock paces the simulation, and
always disconnects the SDK on exit.

After EXP-015, the CLI defaults to the user-specified MoveJ posture and a
three-second actuator settling interval. Use `--initial-posture neutral` to
reproduce the model-neutral startup used by the recorded EXP-014 result.

A three-second live startup attempt in the current environment connected to
`127.0.0.1:60061` but again received only the non-positive SDK timestamp.
The CLI timed out and disconnected cleanly before loading or advancing the
MuJoCo teleoperation loop. Therefore the live PICO motion path remains
unverified; no fake data was substituted for the missing stream.

## Boundary

This experiment controls only the checked-in MuJoCo model through named
position actuators. It adds no Unitree SDK, ROS controller, CAN/serial path,
motor command, torque output, or real robot control. Collision avoidance,
velocity/acceleration limits, tracking-loss hold policy, physical
controller-to-hand calibration, and live-device motion validation remain
unverified.

## Related files

- `src/wheelloong_m2/xr/adapter.py`
- `src/wheelloong_m2/simulation/xr_mujoco_runtime.py`
- `experiments/test_wheelloong_m2_relative_xr_mujoco.py`
- `experiments/run_wheelloong_m2_xr_mujoco.py`
- `tests/test_relative_xr_adapter.py`
- `docs/experiments/EXP-013_UNITREE_SOURCE_MAPPING_AND_IK.md`
