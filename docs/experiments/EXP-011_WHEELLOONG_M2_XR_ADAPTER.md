# EXP-011 — `wheelloong_m2` XR Adapter Abstraction

## XR pose contract

`XRControllerPose` is a device-SDK-independent dataclass:

```text
timestamp: float
position: ndarray(shape=(3,))
rotation: ndarray(shape=(3, 3))
```

It represents exactly:

```text
^xr T_controller
```

The type copies and validates finite arrays, but it does not assert a PICO,
XRoboToolkit, OpenXR, device, scale, offset, or coordinate-calibration
convention.

## Frame definition

The robot-side S2.1 runtime consumes fixed operational EE targets:

```text
^torso T_WL
^torso T_WR
```

For this interface-only experiment, `XRAdapter` applies an explicit temporary
identity convention:

```text
^torso T_WL := ^xr T_left_controller
^torso T_WR := ^xr T_right_controller
```

This is a fake-test design convention, not a confirmed physical relationship
between an XR tracking frame and `torso_link`, and not a real controller-to-W
transform. It must be replaced by separately validated conversion work before
any device source is used.

## Adapter interface

```python
XRAdapter.convert(left_controller, right_controller)
```

accepts two `XRControllerPose` instances and returns plain Pinocchio poses:

```text
{
  "left_target_pose":  ^torso T_WL,
  "right_target_pose": ^torso T_WR,
}
```

The present implementation copies position and rotation unchanged. It
contains no calibration, axis conversion, hand retargeting, scale, or offset
logic.

## Fake source

`FakeXRSource` has no PICO SDK, XRoboToolkit, OpenXR, or device dependency.
Its configured `sample_rate_hz` imports the shared S2.1 `TARGET_HZ = 120`.
`sample(time_s)` returns left/right synthetic `^xr T_controller` poses with
fixed rotations and sinusoidal X translations.

The experiment injects reference XR values numerically equal to neutral robot
operational targets solely to construct reachable values under the temporary
identity convention. That injected equality is synthetic test setup, not a
measurement or calibration result.

## Integration test

Run:

```bash
python3 experiments/test_wheelloong_m2_xr_adapter.py
```

The 2-second simulation-time loop is:

```text
FakeXRSource (120 Hz)
  -> XRAdapter identity interface
  -> DualArmTargetBuffer latest value
  -> WheelloongM2DualArmIK (250 Hz)
  -> named MuJoCo position actuators / mj_step (1000 Hz)
```

The source/adapter are called only at target ticks. The test records XR sample
pairs, buffer updates, IK solves, physics steps, and final Pinocchio FK error
from simulated qpos.

## Results

The recorded run completed with no device SDK use:

```text
XR sample pairs: 240 (120.000000 Hz)
XR controller poses: 480
target updates: 240
IK solves: 500
physics steps: 2000

final joint tracking ||qpos-q_target||: 2.110591184150e-02 rad
left EE position / rotation error:  7.230049812038e-03 m / 1.186173159313e-03 rad
right EE position / rotation error: 6.023004265867e-03 m / 1.070827212223e-03 rad
```

This confirms fake XR pose data can enter the existing S2.1 runtime through
the adapter abstraction. It does not validate a real XR coordinate mapping.

## Current limitations

- Fake XR only: no PICO SDK, XRoboToolkit, OpenXR, device discovery, or real
  device sample is used.
- No coordinate calibration, axis mapping, controller-to-wrist transform,
  scale, offset, hand retargeting, filtering, or source-time synchronization
  exists.
- The robot path remains offline MuJoCo simulation; there is no real robot,
  motor controller, trajectory generator, collision avoidance, torque policy,
  or URDF/MJCF modification.

## Related files

- `src/wheelloong_m2/xr/types.py`
- `src/wheelloong_m2/xr/source.py`
- `src/wheelloong_m2/xr/adapter.py`
- `experiments/test_wheelloong_m2_xr_adapter.py`
- `docs/experiments/EXP-010_WHEELLOONG_M2_MULTIRATE_LOOP.md`
