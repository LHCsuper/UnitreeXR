# EXP-008 — `wheelloong_m2` Offline CasADi/IPOPT Dual-Arm IK

## Solver architecture

`WheelloongM2DualArmIK` is the first `wheelloong_m2` component that solves a
14-DOF arm configuration. It owns one reusable, parameterized CasADi `Opti`
problem and configures it with IPOPT. It reuses, without copying FK logic:

- `WheelloongM2CasadiKinematics` for symbolic `^torso T_WL` and
  `^torso T_WR`;
- `casadi_se3_error.compute_pose_error` for the established pose error; and
- `ARM_JOINT_NAMES` and the URDF-derived named joint-limit mapping.

This is an offline IK baseline only. It has no XR, PICO, coordinate adapter,
MuJoCo controller, trajectory generator, velocity/acceleration limit,
collision avoidance, torque term, or robot-control path.

## Optimization variable

The Opti variable is:

```text
q = opti.variable(14)
```

Its order is the established `ARM_JOINT_NAMES` contract: left arm joints 1–7,
then right arm joints 1–7. No Pinocchio backend index is handwritten or
exposed as the solver interface.

## Target contract

`solve(left_target_pose, right_target_pose, q_init=None, q_prev=None)` takes
two Pinocchio `SE3` targets:

```text
left_target_pose  = ^torso T_WL
right_target_pose = ^torso T_WR
```

Each call copies their position and rotation arrays into Opti parameters;
targets are therefore not fixed when the solver is constructed. `W_L` / `W_R`
remain the unchanged S0.5b palm/gripper-root operational frames, not
calibrated fingertip TCPs.

`q_init` sets the IPOPT warm-start seed. `q_prev` is the smoothness reference;
when it is omitted, it uses the neutral named arm configuration. If `q_init`
is also omitted, that same `q_prev` becomes the warm-start seed.

The plain-data return contract is:

```text
{
  "q_arm":     ndarray(14),
  "success":   bool,
  "cost":      float,
  "solve_time": float,
  "iterations": int,
}
```

No CasADi object is returned.

## Cost

For each arm, the established torso-axis error is:

```text
e_p = p_current - p_target
e_R = Log(R_current * R_target^T)
```

The Opti objective reuses the S1.2.0 `IKWeights` values and is:

```text
J_pose   = sum_side (wp * ||e_p,side||^2 + wr * ||e_R,side||^2)
J_reg    = wq * ||q - q_nom||^2
J_smooth = ws * ||q - q_prev||^2
J_total  = J_pose + J_reg + J_smooth
```

By default, `q_nom` is extracted from Pinocchio's neutral full configuration
through the existing named arm mapping. EXP-015 later adds an optional,
limit-validated constructor override so a simulation runtime can explicitly
select a task posture; the default and the recorded EXP-008 results remain
unchanged. The pose objective is intentionally a soft cost, not an equality
constraint; regularization and smoothness can therefore trade small pose
residuals for a preferred arm configuration.

## Constraints

For every named arm joint, lower and upper position bounds come from the
checked-in URDF through `arm_joint_limits`. The only constraints are:

```text
q_lower <= q <= q_upper
```

No velocity, acceleration, collision, torque, trajectory, or actuator
constraint is constructed.

## IPOPT configuration

The initial configuration is deliberately minimal:

```text
ipopt.print_level = 0
print_time = False
```

The solver result records wall-clock solve time and IPOPT's `iter_count`. No
additional tuning parameters are introduced in this stage.

## Tests

Run:

```bash
python3 experiments/test_wheelloong_m2_ik_solver.py
```

The deterministic offline test uses numeric Pinocchio FK to synthesize
reachable targets, then solves and performs numeric FK round trips:

1. zero `q_arm` FK target;
2. fixed-seed (`20260823`) legal random target, with zero warm-start seed and
   the reference configuration as `q_prev`; and
3. simultaneous distinct left/right target changes, warm-started from the
   prior solve result.

For every case it prints IPOPT success, solve time, iterations, final cost,
solution/reference `q` difference, joint-limit status, and left/right pose
round-trip errors.

## Results

The recorded run had IPOPT success for all three cases, and every returned
configuration was within the URDF position limits.

```text
Case                                      time (s)  iterations  final cost
zero q FK target                          0.015417       5      4.374166941641e-17
fixed-seed legal random q target          0.009763      12      6.670271030808e-02
simultaneous target change, warm start    0.005229       6      6.194859251334e-02
```

Numeric FK round-trip residuals were:

```text
Case 1
Left:  position 5.140384083324e-12 m, rotation 5.163561466511e-10 rad
Right: position 5.136485812550e-12 m, rotation 5.163960522852e-10 rad

Case 2
Left:  position 1.402894678653e-03 m, rotation 1.726603444384e-02 rad
Right: position 1.203194694500e-03 m, rotation 1.921005492770e-02 rad

Case 3
Left:  position 1.538917532556e-03 m, rotation 2.510674542777e-02 rad
Right: position 1.789245917205e-03 m, rotation 1.923136671654e-02 rad
```

The nonzero random-case residuals are expected for this first soft-cost
baseline: the default nominal and smoothness penalties are active alongside
the pose objective. They do not indicate a coordinate-frame change or a
failure of the FK round-trip measurement.

## Limitations

- This is an offline baseline, not a trajectory, controller, or robot command
  interface.
- Pose tracking is a weighted soft objective rather than a hard pose equality
  constraint, so its residual depends on `IKWeights`, `q_nom`, and `q_prev`.
- It includes only URDF position constraints; it has no velocity,
  acceleration, collision, torque, or self/environment constraint.
- It does not add XR, PICO, coordinate conversion, MuJoCo control, URDF/MJCF
  edits, or real-robot integration.

## Related files

- `src/wheelloong_m2/ik/dual_arm_ik.py`
- `src/wheelloong_m2/ik/casadi_se3_error.py`
- `src/wheelloong_m2/kinematics/casadi_fk.py`
- `src/wheelloong_m2/kinematics/robot_model.py`
- `experiments/test_wheelloong_m2_ik_solver.py`
- `docs/experiments/EXP-007_WHEELLOONG_M2_SE3_IK_MATH.md`
