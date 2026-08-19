# EXP-002 — XR Position Coordinate Convention Validation

## Objective

Validate the XR position coordinate convention: position component order,
axis directions, handedness, and unit consistency.

## Question

For an XR pose array of length 7, what do the first three components
(`pose[0]`, `pose[1]`, `pose[2]`) represent physically?

## Setup

- VR device: PICO 4 Ultra
- PC OS: Ubuntu 22.04 x86_64
- Python module: `xrobotoolkit_sdk`
- Runtime library: `/usr/local/lib/libPXREARobotSDK.so`
- Controller used: right controller
- Reference motions: manually estimated ~20 cm translations

## Procedure

1. Start the XR runtime and confirm pose streaming works (EXP-001).
2. Move the right controller approximately 20 cm to the physical right.
3. Record the pose delta and its norm.
4. Move the right controller approximately 20 cm physically upward.
5. Record the pose delta and its norm.
6. Move the right controller approximately 20 cm physically forward.
7. Record the pose delta and its norm.
8. Compare the dominant delta components against the physical direction.

The ~20 cm distance is a manual, approximate estimate. It is used only to
determine direction and order of magnitude, not for millimeter-level
calibration, scale error estimation, or precise yaw estimation.

## Raw / Representative Measurements

`pose = [pose[0], pose[1], pose[2], ...]`

This experiment studies only the first three components.

### 1. Reality-right motion, approximate 20 cm

Trial 1:

```text
DELTA = [+0.1724806166, -0.0006605942, +0.0875115063]
NORM  = 0.1934121589
```

Trial 2:

```text
DELTA = [+0.1795879522, +0.00028483985, +0.08453248025]
NORM  = 0.1984884227
```

Observation: horizontal motion primarily appears in `pose[0]`/`pose[2]`,
while `pose[1]` stays approximately unchanged.

Do not conclude a precise tracking-frame yaw angle from these data, because
the physical motion direction was only manually estimated.

### 2. Reality-up motion, approximate 20 cm

Trial 1:

```text
DELTA = [+0.00026682405, +0.1841965737, +0.00144896135]
NORM  = 0.1842024659
```

Trial 2:

```text
DELTA = [-0.0014517484, +0.18471259895, +0.0078484054]
NORM  = 0.1848849622
```

Observation: upward motion maps overwhelmingly to positive `pose[1]`.

This provides repeatable experimental evidence that:

```text
pose[1] = Y
+Y = up
```

### 3. Reality-forward motion, approximate 20 cm

```text
DELTA = [+0.077248166, -0.0018097017, -0.15376699755]
NORM  = 0.1720896386
```

Observation: forward motion produces a negative `pose[2]` component,
consistent with:

```text
-Z = forward
+Z = backward
```

Combined with the right-motion experiment:

```text
+X = right
```

## Observations

- Horizontal motion primarily appears in `pose[0]` and `pose[2]`.
- Upward motion maps overwhelmingly to positive `pose[1]`.
- Forward motion produces a negative `pose[2]` component.
- Rightward motion produces a positive `pose[0]` component.
- Measured displacement norms for ~20 cm motions were approximately
  0.19 m, 0.20 m, 0.18 m, 0.18 m, and 0.17 m.

## Analysis

### Position component order

Physical motion direction maps to pose components as follows:

```text
pose[0] = X
pose[1] = Y
pose[2] = Z
```

### Axis directions

```text
+X = right
-X = left

+Y = up
-Y = down

-Z = forward
+Z = backward
```

### Handedness

With `+X = right`, `+Y = up`, and `+Z = backward`, the frame is
right-handed.

### Position unit

The measured displacement magnitudes for approximate 20 cm physical motions
were approximately:

```text
0.193 m
0.198 m
0.184 m
0.185 m
0.172 m
```

These values are consistent with XR position values using meters. Because
the commanded physical distance was manually estimated, this is not a
precision scale calibration.

## Conclusion

Confirmed (Experimental Evidence):

- Position order is `[X, Y, Z]` for `pose[0]`, `pose[1]`, `pose[2]`.
- `+X = right`, `+Y = up`, `+Z = backward`, `-Z = forward`.
- The position frame is right-handed.

Strong experimental evidence (not a precision calibration):

- Position unit is consistent with meters.

The `right` / `up` / `forward` labels describe the PICO/XR tracking frame,
not the user's instantaneous body or table orientation. The XR
tracking/world frame does not necessarily remain aligned with an arbitrary
table edge or the user's current body yaw.

## Confidence

- Position component order: Confirmed by directional motion mapping.
- Axis directions: Confirmed by repeatable directional motion experiments.
- Handedness: Confirmed by the validated axis directions.
- Position unit: Strong experimental evidence, but not a precision scale
  calibration.

## Limitations

- The ~20 cm motion distance was manually estimated.
- Only the right controller was used.
- Only position components were analyzed; `pose[3:7]` were not interpreted.
- No precise tracking-frame yaw was derived from the manual motions.

## Open Questions

The following remain Unknown and are out of scope for EXP-002:

- `pose[3:7]` quaternion component order.
- Quaternion rotation direction / transform semantics.
- Controller local coordinate frame.
- Tracking origin semantics.
- Recenter behavior.
- XRoboToolkit → Unitree mapping.

## Related Files

- `docs/COORDINATE_SYSTEM.md`
- `docs/STATUS.md`
- `docs/experiments/EXP-001_XR_DATA_PIPELINE.md`