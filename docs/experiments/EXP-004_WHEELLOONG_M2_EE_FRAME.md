# EXP-004 — `wheelloong_m2` End-Effector Frame Analysis

## Objective

Determine, from the checked-in model only, what end-effector (EE) frame the
`wheelloong_m2` dual-arm teleoperation / IK should use. This experiment only
studies the model; it does not implement IK, connect XR, modify coordinate
mapping, or control a real robot.

## Why arm_link_7 is not automatically the teleop EE

- `arm_link_7` is the child frame of the last arm revolute joint
  (`arm_joint_7`). Its origin coincides with that joint axis (the wrist-roll
  joint).
- Four direct gripper root joints attach *distally* to `arm_link_7`; their
  mean origin is about `0.220 m` from the wrist-roll frame.
- Therefore `arm_link_7` is a wrist-roll frame, not the gripper-root/palm
  operational point used below.

This is a Source Evidence fact, not a naming guess: it follows from the
joint-parent/child topology and the finger mounting offsets.

## Robot-side kinematic structure

Both the URDF and the MuJoCo MJCF contain the same gripper topology. The
URDF values below are quoted from the source; the MJCF reproduces the same
parent/child graph (with quaternions instead of rpy for the right arm, and
hinge joints instead of `continuous`).

Notation: `^a p_b` is the position of frame `b` in frame `a`.

### Left gripper

```text
left_arm_link_7
├── left_gripper_joint_1 -> left_gripper_link_1
│     origin=[-0.065316, 0.21003, 0.0105], rpy=[0,0,0], axis=[0,0,1], revolute
│     └── left_gripper_joint_2 -> left_gripper_link_2
│           origin=[-0.000692, 0.049995, -0.005], axis=[0,0,-1],
│           continuous, mimic(left_gripper_joint_1, mult=+1)
├── left_gripper_joint_3 -> left_gripper_link_3
│     origin=[-0.050808, 0.22379, 0.0105], axis=[0,0,1],
│     continuous, mimic(left_gripper_joint_1, mult=+1)
├── left_gripper_joint_4 -> left_gripper_link_4
│     origin=[-0.007684, 0.21003, 0.0105], axis=[0,0,1],
│     continuous, mimic(left_gripper_joint_1, mult=-1)
│     └── left_gripper_joint_5 -> left_gripper_link_5
│           origin=[0.000692, 0.049995, -0.005], axis=[0,0,-1],
│           continuous, mimic(left_gripper_joint_2, mult=-1)
└── left_gripper_joint_6 -> left_gripper_link_6
      origin=[-0.022192, 0.22379, 0.0105], axis=[0,0,1],
      continuous, mimic(left_gripper_joint_3, mult=-1)
```

### Right gripper

```text
right_arm_link_7
├── right_gripper_joint_1 -> right_gripper_link_1
│     origin=[-0.065316, -0.21003, 0.0105], rpy=[0,0,-1.5708], axis=[0,0,1], revolute
│     └── right_gripper_joint_2 -> right_gripper_link_2
│           origin=[0.049995, -0.000692, -0.005], axis=[0,0,-1],
│           continuous, mimic(right_gripper_joint_1, mult=+1)
├── right_gripper_joint_3 -> right_gripper_link_3
│     origin=[-0.050808, -0.22379, 0.0105], rpy=[0,0,-1.5708], axis=[0,0,1],
│     continuous, mimic(right_gripper_joint_1, mult=+1)
├── right_gripper_joint_4 -> right_gripper_link_4
│     origin=[-0.007684, -0.21003, 0.0105], rpy=[0,0,-1.5708], axis=[0,0,1],
│     continuous, mimic(right_gripper_joint_1, mult=-1)
│     └── right_gripper_joint_5 -> right_gripper_link_5
│           origin=[0.049995, 0.000692, -0.005], axis=[0,0,-1],
│           continuous, mimic(right_gripper_joint_2, mult=-1)
└── right_gripper_joint_6 -> right_gripper_link_6
      origin=[-0.022192, -0.22379, 0.0105], rpy=[0,0,-1.5708], axis=[0,0,1],
      continuous, mimic(right_gripper_joint_3, mult=-1)
```

Key source facts:

- `left_gripper_joint_1` is the only revolute (bounded) joint and is the
  mimic master. Left master limits are `[0, 1]`; right master limits are
  `[-1, 0]` (the MJCF matches this as `range="0 1"` / `range="-1 0"`).
- Joints 3, 4, 6 are `continuous` in the URDF and all mimic joint 1; joints
  2 and 5 are `continuous` second-stage fingers.
- Mimic multipliers pair the four direct joints into two assemblies that
  rotate in opposite directions: `(+1, +1)` for joints 1/3 and `(-1, -1)`
  for joints 4/6.

## Left gripper geometry

The four direct mounting points (relative to `left_arm_link_7`) are:

```text
^arm7 p_joint_1 = [-0.065316, 0.21003, 0.0105]
^arm7 p_joint_3 = [-0.050808, 0.22379, 0.0105]
^arm7 p_joint_4 = [-0.007684, 0.21003, 0.0105]
^arm7 p_joint_6 = [-0.022192, 0.22379, 0.0105]
```

They form a 2×2 grid: two columns separated along `x` and two rows separated
along `y`. All four share the same `z = +0.0105` offset. These mounting-point
spans alone do not set the logical EE axis signs.

## Right gripper geometry

The right side is the mirror image (negated `y`, plus an `rpy` yaw of
`-1.5708 rad` on the direct joints):

```text
^arm7 p_joint_1 = [-0.065316, -0.21003, 0.0105]
^arm7 p_joint_3 = [-0.050808, -0.22379, 0.0105]
^arm7 p_joint_4 = [-0.007684, -0.21003, 0.0105]
^arm7 p_joint_6 = [-0.022192, -0.22379, 0.0105]
```

## Gripper-root operational-point candidates

There is no dedicated `palm`, `gripper_base`, or `tool_center` link in either
model. The points below are the means of the four direct gripper root joint
origins. Each is a fixed robot-side **gripper-root center candidate**, also
called a **palm-center operational-point candidate** in this experiment.

These points are not fingertip centers, grasp centers, calibrated tool
centers, or true physical TCPs.

```text
^L7 p_WL = [-0.0365, +0.21691, +0.0105]
^R7 p_WR = [-0.0365, -0.21691, +0.0105]

||^L7 p_WL|| = ||^R7 p_WR|| = 0.220210 m
```

The symmetry is consistent with a simple parallel-gripper midpoint:

```text
left pairwise root midpoints:
  (j1 + j4)/2 = [-0.0365, 0.21003, 0.0105]
  (j3 + j6)/2 = [-0.0365, 0.22379, 0.0105]
```

The mounting-point spread is:

```text
x span = 0.057632 m
y span = 0.013760 m
z span = 0.000000 m
```

## Secondary finger vectors in `arm_link_7`

The earlier EXP-004 orientation inference treated each second-stage joint
origin as though it were already expressed in `arm_link_7`. That is incorrect
for the right gripper because its direct gripper child frames have a fixed
yaw of approximately `-90 deg`.

For each second-stage joint, the script now computes:

```text
^arm7 v_secondary = ^arm7 R_parent_gripper * ^parent_gripper v_secondary
```

The source vectors and transformed results at the zero gripper configuration
are:

```text
left_gripper_joint_2:
  ^left_gripper_link_1 v = [-0.000692, +0.049995, -0.005000]
  ^L7 v                     = [-0.000692, +0.049995, -0.005000]

left_gripper_joint_5:
  ^left_gripper_link_4 v = [+0.000692, +0.049995, -0.005000]
  ^L7 v                     = [+0.000692, +0.049995, -0.005000]

right_gripper_joint_2:
  ^right_gripper_link_1 v = [+0.049995, -0.000692, -0.005000]
  ^R7 v                      = [-0.000692, -0.049995, -0.005000]

right_gripper_joint_5:
  ^right_gripper_link_4 v = [+0.049995, +0.000692, -0.005000]
  ^R7 v                      = [+0.000691, -0.049995, -0.005000]
```

The two transformed vectors on each side are averaged. Their component along
the direct hinge axis is removed so the extension direction lies in the
hinge-normal plane; the small opposing lateral components cancel. The result
is:

```text
left physical finger extension:  approximately +L7 Y
right physical finger extension: approximately -R7 Y
```

This is a geometry calculation from the URDF transforms, not a side-specific
hard-coded sign choice.

## Baseline logical teleoperation EE orientation

Define `W_L` and `W_R` as the left and right logical teleoperation operational
EE frames. Their axis semantics are a **design convention**:

```text
+Y_W: physical extension direction from wrist/palm toward the fingers
+Z_W: positive axis direction of the direct gripper hinge joints
+X_W: y_W cross z_W, completing a right-handed frame
```

The program obtains `+Y_W` from the projected mean secondary vector, obtains
`+Z_W` by expressing and averaging all direct hinge axes in `arm_link_7`, and
then derives `+X_W`; it does not guess the `x` sign independently.

The resulting rotations are:

```text
^L7 R_WL =
[[1, 0, 0],
 [0, 1, 0],
 [0, 0, 1]]

^R7 R_WR approximately =
[[-1, 0, 0],
 [ 0,-1, 0],
 [ 0, 0, 1]]
```

The right result retains a roughly `3.7e-6` off-diagonal term because the URDF
uses the rounded source angle `-1.5708` rather than an exact `-pi/2`.

## Complete baseline transforms

Combining the gripper-root center positions with the derived orientations:

```text
^L7 T_WL =
[[1, 0, 0, -0.03650],
 [0, 1, 0, +0.21691],
 [0, 0, 1, +0.01050],
 [0, 0, 0,  1.00000]]

^R7 T_WR approximately =
[[-1.000000, -0.000004, 0, -0.03650],
 [+0.000004, -1.000000, 0, -0.21691],
 [ 0.000000,  0.000000, 1, +0.01050],
 [ 0.000000,  0.000000, 0,  1.00000]]
```

Numerical checks produced:

```text
W_L: det(R) = 1.000000000000
     ||R^T R - I||_F = 0.000000000000e+00
     ||x_W cross y_W - z_W|| = 0.000000000000e+00

W_R: det(R) = 1.000000000000
     ||R^T R - I||_F = 2.482534153247e-16
     ||x_W cross y_W - z_W|| = 0.000000000000e+00
```

## Visualization observations

Run with:

```bash
python3 experiments/inspect_wheelloong_m2_ee_frames.py --visualize
```

The viewer shows the robot neutral-pose meshes plus named frames for
`torso_link`, `left_arm_link_7`, `right_arm_link_7`, and the eight direct
gripper joint origins. Each raw gripper-root center remains a magenta
**position-only** marker.

It additionally draws complete XYZ frames named:

- `left_teleop_ee_candidate`
- `right_teleop_ee_candidate`

Their `+Y_W` axes follow the physical finger-extension direction, and their
`+Z_W` axes follow the positive direct hinge direction.

No XR frames are drawn. No model file is modified; all frames are created
dynamically at runtime.

## Confirmed source facts

- The checked-in URDF and MJCF contain the same gripper parent/child
  topology (source evidence, cross-checked against both files).
- `arm_link_7` origin is the wrist-roll joint axis (`arm_joint_7`), not a
  point at the fingertips.
- There is no explicit `palm`, `gripper_base`, or `tool_center` link.
- Four direct gripper joints attach to each `arm_link_7`; all declare axis
  `[0,0,1]` in their child frame.
- `left_gripper_joint_1` / `right_gripper_joint_1` are the bounded revolute
  masters; the other five gripper joints are continuous mimics.
- The right direct gripper child frames have a fixed yaw of approximately
  `-90 deg` relative to `right_arm_link_7`.

## Derived geometry

- The four direct mounting points form a symmetric 2×2 grid, so their mean
  defines a reproducible gripper-root/palm-center operational-point candidate.
- The operational-point candidates are `[-0.0365, +0.21691, 0.0105]` for
  the left and `[-0.0365, -0.21691, 0.0105]` for the right, both expressed in
  their respective `arm_link_7` frames.
- Full transform calculation shows that the main physical finger-extension
  direction is approximately `+L7 Y` on the left and `-R7 Y` on the right.
- The positive direct hinge direction is approximately `+arm7 Z` on both
  sides.

## Design convention

- `+Y_W` means the physical finger-extension direction from wrist/palm toward
  the fingers.
- `+Z_W` means the positive direct gripper hinge-axis direction.
- `+X_W` is derived as `y_W cross z_W`; its sign is not guessed separately.
- `W_L` and `W_R` are logical robot-side operational frames. This convention
  does not claim that either origin is a physical fingertip TCP.

## Baseline decision

Baseline arm IK will target the logical operational EE frames `W_L` and
`W_R`. They are fixed palm/gripper-root operational frames constructed from
the checked-in robot geometry, not calibrated fingertip TCPs. This decision
defines the robot-side frame contract for later offline IK work; no IK is
implemented in this experiment.

## Unknowns

- The calibrated physical fingertip/grasp TCP, which may vary with gripper
  configuration and is not supplied by this model analysis.
- Any task-specific tool frame that may later be mounted or calibrated.
- The downstream IK numerical formulation and cost design.

## Decision Status

`W_L` and `W_R` are accepted as the **baseline logical teleoperation
operational EE frames**:

```text
position: mean of the four direct gripper root joint origins
+Y_W: physical finger extension
+Z_W: positive direct hinge axis
+X_W: y_W cross z_W
```

This is a baseline logical operational-frame decision, not a claim that the
origin is the final physical TCP. No IK, XR connection, coordinate adapter,
URDF/MJCF edit, or robot control was introduced.

## Related Files

- `experiments/inspect_wheelloong_m2_ee_frames.py`
- `src/description/wheelloong_m2/urdf/wheelloong_m2.urdf`
- `src/description/wheelloong_m2/mujoco/wheelloong_m2_controlled.xml`
- `docs/experiments/EXP-003_WHEELLOONG_M2_FK_CONSISTENCY.md`
