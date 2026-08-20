# EXP-006 — `wheelloong_m2` CasADi Symbolic FK

## Objective

Build an offline CasADi symbolic FK expression for the existing
`wheelloong_m2` dual-arm kinematics contract and compare it directly with the
independent numeric Pinocchio FK implementation from EXP-005.

This experiment is limited to Pinocchio FK plus CasADi expressions. It does
not implement IK, a cost function, NLP, IPOPT, Opti, any solver, XR path,
MuJoCo controller, or robot control.

## Runtime environment

```text
CasADi version:    3.6.7
Pinocchio version: 3.4.0
pinocchio.casadi:  importable
```

## Symbolic q definition

The symbolic public input is:

```text
q_arm = SX.sym("q_arm", 14)
```

Its order is exactly the EXP-005 public contract:

```text
[
  left_arm_joint_1, left_arm_joint_2, left_arm_joint_3,
  left_arm_joint_4, left_arm_joint_5, left_arm_joint_6,
  left_arm_joint_7,
  right_arm_joint_1, right_arm_joint_2, right_arm_joint_3,
  right_arm_joint_4, right_arm_joint_5, right_arm_joint_6,
  right_arm_joint_7,
]
```

The symbolic module obtains the same named Pinocchio addresses as
`WheelloongM2Kinematics` and verifies the 14-name order in its comparison
script.

## Full q mapping

The URDF-backed Pinocchio model has `nq=42`; `q_arm` is not used as a full
Pinocchio configuration. The module converts the model loaded by the existing
module-relative loader into `pinocchio.casadi.Model`, creates CasADi data, and
then constructs:

```text
full_q_symbolic = SX(pin.neutral(numeric_model))
full_q_symbolic[pinocchio_q_index(named_arm_joint)] = q_arm[q_arm_index]
```

All base, torso, neck, gripper, and other non-arm entries therefore remain at
their Pinocchio neutral configuration. The URDF is neither copied nor edited.

## EE frame usage

The symbolic FK consumes the unchanged S0.5b fixed transforms:

```text
^left_arm_link_7 T_WL
^right_arm_link_7 T_WR
```

`W_L` / `W_R` are still logical palm/gripper-root operational frames. They
are not added to the URDF, are not virtual joints, and are not calibrated
fingertip TCPs.

## Symbolic FK

`WheelloongM2CasadiKinematics.compute_symbolic_fk()` runs CasADi Pinocchio
forward kinematics and returns SX expressions:

```text
{
  "left_position":  ^torso p_WL,
  "left_rotation":  ^torso R_WL,
  "right_position": ^torso p_WR,
  "right_rotation": ^torso R_WR,
}
```

The explicit transform is:

```text
^torso T_W = inverse(^world T_torso) * ^world T_arm_link_7 * ^arm_link_7 T_W
```

The module packages these four expressions into a CasADi function:

```text
dual_arm_fk(q_arm) -> left_position, left_rotation, right_position, right_rotation
```

No world-frame pose is exposed as the symbolic result.

## Numeric comparison

Run:

```bash
python3 experiments/test_wheelloong_m2_casadi_fk.py
```

For each case, the experiment evaluates both:

```text
numeric:  WheelloongM2Kinematics.forward_kinematics(q_arm)
symbolic: dual_arm_fk(q_arm)
```

and measures:

```text
position error = ||p_numeric - p_symbolic||
rotation error = ||log3(R_numeric * R_symbolic^T)||
```

No pass/fail threshold is applied.

## Results

Case 0 (`q_arm=zeros`):

```text
Left position error:  0.000000000000e+00 m
Left rotation error:  0.000000000000e+00 rad
Right position error: 0.000000000000e+00 m
Right rotation error: 1.068584563117e-21 rad
```

Case 1 (fixed-seed `20260821` legal random `q_arm`):

```text
Left position error:  0.000000000000e+00 m
Left rotation error:  1.053601299251e-16 rad
Right position error: 0.000000000000e+00 m
Right rotation error: 5.334383746338e-16 rad
```

The observed residuals are at floating-point precision and verify this scoped
numeric-vs-symbolic FK agreement for the two deterministic test cases.

## Limitations

- No IK, cost function, nonlinear program, IPOPT, Opti, or solver exists.
- No symbolic Jacobian, optimization objective, or constraint is created in
  this stage.
- No XR, coordinate adapter, MuJoCo controller, real robot, URDF edit, or
  MJCF edit is involved.
- This validation covers fixed-neutral non-arm joints and the two tested
  `q_arm` configurations only.

## Related files

- `src/wheelloong_m2/kinematics/casadi_fk.py`
- `src/wheelloong_m2/kinematics/dual_arm_fk.py`
- `src/wheelloong_m2/kinematics/robot_model.py`
- `src/wheelloong_m2/kinematics/frames.py`
- `experiments/test_wheelloong_m2_casadi_fk.py`
- `docs/experiments/EXP-005_WHEELLOONG_M2_PINOCCHIO_KINEMATICS.md`
