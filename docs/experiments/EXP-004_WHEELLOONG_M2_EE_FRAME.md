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
- The four gripper fingers attach *distally* to `arm_link_7`, at a mean
  distance of about `0.220 m` forward along the arm.
- Therefore `arm_link_7` is a wrist-roll frame, not a point between the
  fingertips. A teleoperation target placed at `arm_link_7` would be offset
  from the grasp center by the finger length.

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

They form a 2×2 grid: two columns separated along `x` (closing direction)
and two rows separated along `y` (finger extension direction). All four
share the same `z = +0.0105` offset.

## Right gripper geometry

The right side is the mirror image (negated `y`, plus an `rpy` yaw of
`-1.5708 rad` on the direct joints):

```text
^arm7 p_joint_1 = [-0.065316, -0.21003, 0.0105]
^arm7 p_joint_3 = [-0.050808, -0.22379, 0.0105]
^arm7 p_joint_4 = [-0.007684, -0.21003, 0.0105]
^arm7 p_joint_6 = [-0.022192, -0.22379, 0.0105]
```

## Candidate EE positions

There is no dedicated `palm`, `gripper_base`, or `tool_center` link in either
model. The candidate below is a **geometric inference only** — it is the
mean of the four direct gripper joint origins and is **not** a confirmed
teleoperation TCP.

```text
^arm7 p_candidate = [-0.0365, +0.21691, +0.0105]   (left)
^arm7 p_candidate = [-0.0365, -0.21691, +0.0105]   (right)

||^arm7 p_candidate|| = 0.220210 m  (both arms)
```

The symmetry is consistent with a simple parallel-gripper midpoint:

```text
left pairwise jaw midpoints (closing axis):
  (j1 + j4)/2 = [-0.0365, 0.21003, 0.0105]
  (j3 + j6)/2 = [-0.0365, 0.22379, 0.0105]
```

The mounting-point spread is:

```text
x span = 0.057632 m   (closing / opening direction)
y span = 0.013760 m   (front/back row offset)
z span = 0.000000 m
```

## Candidate EE orientations

Evidence priority: URDF joint/mesh structure first, then MJCF, then visual
inspection.

- **Gripper joint axis (normal to the closing plane):** all four direct
  joints of each hand declare `axis=[0,0,1]` in their child frame, and their
  fixed `^arm7 R` keeps `z` aligned with the arm7 `z` axis. This is the best
  supported candidate axis direction.
- **Closing / opening direction:** the four mounting points spread along
  `x` (span `0.0576 m`), and the mimic multipliers `+1/-1` drive the two
  jaw assemblies in opposite senses. This makes `x` the closing-axis
  candidate.
- **Forward / extension direction:** the second-stage finger joints
  (`joint_2`, `joint_5`) are offset by `+0.049995` along the child `y`
  axis, i.e. fingers extend along `+y`. This makes `y` the extension-axis
  candidate.
- **Right-handed completion:** with `z` = joint axis and `x` = closing axis,
  `y` = `z × x` completes the frame.

Only the **direction** of the `z` (joint) axis is strongly supported. The
signs of the closing (`x`) and forward (`y`) axes are not fixed by the model
text alone; they require visual confirmation. Therefore the orientation is a
**candidate**, and its sign conventions remain Unknown until a visual check
or a robot-side authority definition is available.

## Visualization observations

Run with:

```bash
python3 experiments/inspect_wheelloong_m2_ee_frames.py --visualize
```

The viewer shows the robot neutral-pose meshes plus named frames for
`torso_link`, `left_arm_link_7`, `right_arm_link_7`, the eight direct
gripper joint origins, and the two candidate center points. The candidate
center is drawn as a **position-only** marker; no orientation is drawn,
because the candidate orientation is not yet mechanically confirmed.

No XR frames are drawn. No model file is modified; all frames are created
dynamically at runtime.

## Confirmed Facts

- The checked-in URDF and MJCF contain the same gripper parent/child
  topology (source evidence, cross-checked against both files).
- `arm_link_7` origin is the wrist-roll joint axis (`arm_joint_7`), not a
  point at the fingertips.
- There is no explicit `palm`, `gripper_base`, or `tool_center` link.
- Four direct gripper joints attach to each `arm_link_7`; all declare axis
  `[0,0,1]` in their child frame.
- `left_gripper_joint_1` / `right_gripper_joint_1` are the bounded revolute
  masters; the other five gripper joints are continuous mimics.

## Geometric Inferences

- The four direct mounting points form a symmetric 2×2 grid, so their mean
  is a reasonable geometric grasp-center candidate.
- The closing/opening axis is `x`; the finger-extension axis is `y`; the
  joint (normal) axis is `z`.
- The candidate center is `^arm7 p = [∓0.0365, ±0.21691, 0.0105]` with norm
  `0.220210 m`.

## Unknowns

- Whether the mean of the joint origins is the *true* grasp/TCP point (it is
  a model-geometry candidate, not a calibration).
- The sign conventions of the closing (`x`) and forward (`y`) axes.
- The intended EE orientation authority (robot-side control convention).
- Whether the teleoperation target should use a palm frame, a midpoint
  frame, or another frame defined by the downstream `TeleData` / IK contract.

## Decision Status

No final EE frame is selected yet. The model evidence supports the following
as a **Recommended candidate** for further study:

```text
position: ^arm7 p_candidate = [-0.0365, ±0.21691, 0.0105]
orientation axes (directions only):
  z ~ gripper joint axis (normal to closing plane)
  x ~ closing/opening axis
  y ~ finger extension axis (completes right-handed frame)
```

This is a candidate, **not** a `Confirmed final teleop frame`. The final
decision must be tied to the downstream IK/teleoperation contract and
confirmed by visual inspection or a robot-side definition.

## Related Files

- `experiments/inspect_wheelloong_m2_ee_frames.py`
- `src/description/wheelloong_m2/urdf/wheelloong_m2.urdf`
- `src/description/wheelloong_m2/mujoco/wheelloong_m2_controlled.xml`
- `docs/experiments/EXP-003_WHEELLOONG_M2_FK_CONSISTENCY.md`