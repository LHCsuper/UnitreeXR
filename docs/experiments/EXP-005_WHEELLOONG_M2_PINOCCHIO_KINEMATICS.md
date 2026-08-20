# EXP-005 — `wheelloong_m2` Pinocchio Kinematics Backbone

## Objective

Establish a small, explicit, offline Pinocchio interface for `wheelloong_m2`
dual-arm forward kinematics and Jacobians. This experiment consumes the
validated S0 arm model and the S0.5b logical operational frames `W_L` / `W_R`.
It does not implement IK or change any model or EE-frame definition.

## q_arm definition

The public arm configuration is the sole 14-DOF ordering exposed by the new
module:

```text
q_arm = [
  left_arm_joint_1, left_arm_joint_2, left_arm_joint_3,
  left_arm_joint_4, left_arm_joint_5, left_arm_joint_6,
  left_arm_joint_7,
  right_arm_joint_1, right_arm_joint_2, right_arm_joint_3,
  right_arm_joint_4, right_arm_joint_5, right_arm_joint_6,
  right_arm_joint_7,
]
```

The module resolves every named joint into both Pinocchio configuration and
velocity indices, then maps to the public `q_arm` index. No FK/Jacobian code
uses handwritten Pinocchio array indices.

At this model version, the resolved addresses are:

```text
joint name          | pin joint id | pin q index | pin v index | q_arm index
left_arm_joint_1    | 5            | 4           | 4           | 0
left_arm_joint_2    | 6            | 5           | 5           | 1
left_arm_joint_3    | 7            | 6           | 6           | 2
left_arm_joint_4    | 8            | 7           | 7           | 3
left_arm_joint_5    | 9            | 8           | 8           | 4
left_arm_joint_6    | 10           | 9           | 9           | 5
left_arm_joint_7    | 11           | 10          | 10          | 6
right_arm_joint_1   | 20           | 24          | 19          | 7
right_arm_joint_2   | 21           | 25          | 20          | 8
right_arm_joint_3   | 22           | 26          | 21          | 9
right_arm_joint_4   | 23           | 27          | 22          | 10
right_arm_joint_5   | 24           | 28          | 23          | 11
right_arm_joint_6   | 25           | 29          | 24          | 12
right_arm_joint_7   | 26           | 30          | 25          | 13
```

## EE frame contract

The interfaces use the accepted S0.5b operational frames without modifying
their definitions:

```text
^left_arm_link_7 T_WL =
[[1, 0, 0, -0.03650],
 [0, 1, 0, +0.21691],
 [0, 0, 1, +0.01050],
 [0, 0, 0,  1.00000]]

^right_arm_link_7 T_WR approximately =
[[-1.000000, -0.000004, 0, -0.03650],
 [+0.000004, -1.000000, 0, -0.21691],
 [ 0.000000,  0.000000, 1, +0.01050],
 [ 0.000000,  0.000000, 0,  1.00000]]
```

`W_L` and `W_R` remain logical palm/gripper-root operational frames, not
URDF links, virtual joints, or calibrated fingertip TCPs.

## FK definition

`WheelloongM2Kinematics.forward_kinematics(q_arm)` starts with
`pin.neutral(model)`, sets only the 14 named arm joints, and leaves every
other model configuration at neutral. It evaluates Pinocchio FK and returns:

```text
{
  "left_ee_pose":  ^torso T_WL,
  "right_ee_pose": ^torso T_WR,
}
```

The conversion is explicit:

```text
^torso T_W = inverse(^world T_torso) * ^world T_arm_link_7 * ^arm_link_7 T_W
```

No world-frame pose is exposed as the FK result.

## Jacobian convention

`compute_jacobians(q_arm)` returns:

```text
{
  "left_J":  ndarray(shape=(6, 14)),
  "right_J": ndarray(shape=(6, 14)),
}
```

Columns use the public `q_arm` order. Rows are spatial twist
`[linear; angular]`, expressed in `torso_link`; the twist is the operational
frame's motion relative to the torso for those 14 arm velocity columns.

Pinocchio first provides the parent `arm_link_7` Jacobian in
`LOCAL_WORLD_ALIGNED`, i.e. world-aligned coordinates at the parent-frame
origin. The implementation shifts the linear part to the fixed operational
point,

```text
v_W = v_arm_link_7 + omega_arm_link_7 x p_arm_link_7_to_W,
```

then rotates both linear and angular rows into torso axes. It does not assume
that Pinocchio's Jacobian output is already torso-expressed.

## Tests

Run:

```bash
python3 experiments/test_wheelloong_m2_kinematics.py
```

The script runs:

1. Case 0: `q_arm = zeros(14)`.
2. Case 1: a fixed-seed (`20260821`) random configuration sampled from the
   14 finite URDF joint-limit intervals.
3. A finite-difference sanity check of `left_arm_joint_4` at Case 1, with
   `epsilon = 1e-7 rad`.

No pass/fail threshold is applied to finite-difference residuals.

## Results

The module loaded the checked-in URDF through a module-relative repository
root search, so execution does not depend on the shell working directory.

Case 0 produced:

```text
^torso p_WL = [-2.4000e-08, +1.0900100000, +0.3152550000]
^torso p_WR = [+7.2285e-09, -1.0900100000, +0.3149870000]
left_J shape  = (6, 14)
right_J shape = (6, 14)
```

At the fixed-seed legal random case, both FK poses were finite and both
Jacobians had shape `(6, 14)`.

Finite-difference residuals for `left_arm_joint_4` were:

```text
Left translation error:  1.812425267363e-15 m
Left rotation error:     1.729217900636e-16 rad
Right translation error: 0.000000000000e+00 m
Right rotation error:    5.039980817457e-16 rad
```

These values are recorded as an offline sanity check, not as an IK or
controller validation threshold.

## Known limitations

- No IK, CasADi, IPOPT, optimization cost, or solver is implemented.
- No XR device, XR adapter, coordinate conversion, MuJoCo controller, or
  real robot is connected.
- No URDF/MJCF file, virtual joint, or model link is modified or added.
- `W_L` / `W_R` are unchanged S0.5b operational frames, not physical
  fingertip TCP calibration results.
- This covers only arm-joint kinematics with every non-arm model joint held at
  its Pinocchio neutral configuration.

## Related files

- `src/wheelloong_m2/kinematics/robot_model.py`
- `src/wheelloong_m2/kinematics/frames.py`
- `src/wheelloong_m2/kinematics/dual_arm_fk.py`
- `experiments/test_wheelloong_m2_kinematics.py`
- `docs/experiments/EXP-003_WHEELLOONG_M2_FK_CONSISTENCY.md`
- `docs/experiments/EXP-004_WHEELLOONG_M2_EE_FRAME.md`
