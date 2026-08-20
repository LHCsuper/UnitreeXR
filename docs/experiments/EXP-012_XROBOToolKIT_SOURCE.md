# EXP-012 — XRoboToolkit PICO Pose Source Layer

## Objective

Add a read-only XRoboToolkit/PICO source wrapper that acquires SDK controller
poses and converts their representation into the existing SDK-independent
`XRControllerPose` contract. This experiment does not connect that source to
the robot adapter, IK, or MuJoCo runtime.

## SDK version

The installed Python distribution reported:

```text
xrobotoolkit_sdk: 1.0.2
```

The module itself does not expose a `__version__` attribute. The wrapper lazily
imports the module and calls only:

```text
init()
get_time_stamp_ns()
get_left_controller_pose()
get_right_controller_pose()
close()
```

## Pose contract

The raw SDK pose format already established by the preceding XR experiments
is:

```text
[x, y, z, qx, qy, qz, qw]
```

`XRoboToolkitSource.sample()` reads one SDK timestamp and both controller
arrays, then returns the existing source-compatible pair:

```text
(left_controller_pose, right_controller_pose)
```

Each value is:

```text
XRControllerPose(
  timestamp = get_time_stamp_ns() * 1e-9,
  position  = [x, y, z],
  rotation  = R(qx, qy, qz, qw),
)
```

`timestamp` therefore has seconds as its `XRControllerPose` unit, while its
source is the SDK's nanosecond `get_time_stamp_ns()` value. The source also
retains the unmodified `xyzw` quaternion arrays and raw nanosecond timestamp
for logging.

## Quaternion order

The source parses the SDK quaternion strictly as:

```text
[qx, qy, qz, qw]
```

It normalizes that quaternion and converts only its representation to a 3x3
rotation matrix using the established raw-pose transform direction. It does
not invert the rotation or change its axes. A device-independent conversion
self-check with `[0, 0, 0, 1]` produced an identity rotation and preserved a
test position/timestamp exactly.

## Position unit

The source copies raw `[x, y, z]` values unchanged. EXP-002 provides
experimental evidence that SDK positions are consistent with metres, but not
a high-precision scale calibration. This source applies no scale, offset,
axis swap, mirror, or robot-frame transform.

## Update rate

Run:

```bash
python3 experiments/test_robotoolkit_pose.py
```

The 10-second logger waits up to 10 seconds for a nonzero timestamp, records
unique timestamp samples, prints first/last left/right position and raw
quaternion, and reports dt mean, standard deviation, maximum, and observed
timestamp rate.

Recorded live attempt in the current environment:

```text
xrt.init(): connection to local PC Service succeeded
get_time_stamp_ns(): remained 0 for the full 10-second startup wait
sample count / dt / actual rate: unavailable
```

Consequently, this run did not confirm a currently active PICO pose stream.
The test exits with a clear timeout after closing the SDK connection; it does
not manufacture fake device samples or infer an update rate from polling.

## Static Pose A/B check

When run from an interactive terminal, the logger defaults to this sequence:

1. press Enter to save Pose A;
2. physically change controller orientation;
3. press Enter to save Pose B; and
4. print left/right delta position plus `Log(R_B R_A^T)` rotation vectors and
   magnitudes.

This live static check was not executed in the recorded attempt because no
nonzero SDK timestamp was available. It is retained for future source-frame
investigation only; it does not apply a coordinate transform.

## Coordinate unknowns

This source layer confirms neither robot mapping nor calibration. In
particular it does not define or change:

```text
^torso T_xr
^base T_xr
^controller T_wrist
```

No `XRAdapter` implementation was modified. There is no controller offset,
hand retargeting, scale policy, axis mapping, calibration, IK call, MuJoCo
call, or real robot operation in this stage.

## Related files

- `src/wheelloong_m2/xr/robotoolkit_source.py`
- `src/wheelloong_m2/xr/types.py`
- `experiments/test_robotoolkit_pose.py`
- `docs/experiments/EXP-001_XR_DATA_PIPELINE.md`
- `docs/experiments/EXP-002_XR_POSITION_FRAME.md`
