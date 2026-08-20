# EXP-007 — `wheelloong_m2` SE(3) IK Mathematics

## Objective

Establish the solver-free mathematical layer required by later offline
dual-arm IK work: numeric and symbolic SE(3) pose errors plus a numeric
dual-arm objective decomposition. This experiment consumes the existing
14-DOF `q_arm` contract and the unchanged S0.5b logical operational frames
`W_L` / `W_R`.

It does not implement IK iteration, a cost optimizer, CasADi Opti, NLP,
IPOPT, any solver, XR, a MuJoCo controller, or robot control.

## Pose error definition

For a current operational-EE pose and its target, both expressed in the same
coordinate frame:

```text
e_p = p_current - p_target
```

The numeric API is:

```python
compute_pose_error(current_pose, target_pose)
```

where both arguments are Pinocchio `SE3` values. It returns a dictionary with
three-vector `position_error` and `rotation_error` entries. The dual-arm
wrapper applies the same definition independently to `left` and `right`.

## Rotation error definition

The orientation error is the principal SO(3) logarithm:

```text
e_R = Log(R_current * R_target^T)
```

The numeric implementation uses `pin.log3`; it does not use Euler angles.
For example, `R_current = Rz(+30 deg)` and `R_target = I` yields a rotation
error in the `+Z` direction with magnitude `0.523598775598 rad`.

`casadi_se3_error.py` provides the equivalent symbolic expression. It clips
the trace-derived cosine to `[-1, 1]`, uses the standard
`theta / (2 sin(theta)) * vee(R - R^T)` expression away from zero, and uses
its `theta -> 0` limit for small angles. This keeps the identity case free of
the otherwise undefined division by zero.

## Frame convention

S1.0 FK and S1.1 symbolic FK return:

```text
^torso T_WL
^torso T_WR
```

Therefore the error definitions above compare current and target poses in
torso axes. `W_L` and `W_R` retain their S0.5b meaning: fixed
palm/gripper-root operational frames, not calibrated fingertip TCPs. Their
axis semantics and fixed transforms are unchanged by this experiment.

## Cost definition

`compute_dual_arm_cost(errors, q, q_nom, q_prev, weights)` evaluates only the
numeric scalar decomposition:

```text
J_pose   = sum_side (wp * ||e_p,side||^2 + wr * ||e_R,side||^2)
J_reg    = wq * ||q - q_nom||^2
J_smooth = ws * ||q - q_prev||^2
J_total  = J_pose + J_reg + J_smooth
```

It returns each term rather than selecting a configuration or constructing an
optimization problem. `q`, `q_nom`, and `q_prev` must each follow the existing
14-element named `q_arm` order.

## Weight choice

The explicit `IKWeights` dataclass defaults are:

```text
wp = 50.0
wr = 1.0
wq = 0.02
ws = 0.1
```

They are caller-provided weights, not an implemented IK policy or tuned robot
control law.

## Tests

Run:

```bash
python3 experiments/test_wheelloong_m2_ik_math.py
```

The deterministic script evaluates:

1. identical current and target poses;
2. a `+0.1 m` target X translation;
3. `R_current = Rz(+30 deg)`, `R_target = I`;
4. fixed-seed random `exp(log(R))` reconstruction;
5. the `+Z` direction of the 30-degree orientation error;
6. CasADi symbolic-vs-numeric pose error for the same rotation; and
7. the dual-arm numeric-cost decomposition.

## Results

The recorded run produced:

```text
Case 1 position magnitude:                 0.000000000000e+00 m
Case 1 rotation magnitude:                 0.000000000000e+00 rad
Case 2 position magnitude:                 1.000000000000e-01 m
Case 3 rotation magnitude:                 5.235987755983e-01 rad
Case 4 exp(log(R)) rotation error:         1.261485851689e-17 rad
Case 5 orientation-error direction:        [0. 0. 1.]
Case 6 numeric-symbolic rotation diff:     0.000000000000e+00 rad
Case 7 cost decomposition residual:        5.898059818321e-17
```

## Limitations

- No IK iteration, optimization problem, CasADi Opti, NLP, IPOPT, or solver
  is present.
- The cost function is numeric evaluation only; no symbolic objective or
  constraints are constructed.
- The symbolic SO(3) logarithm includes a small-angle branch but does not
  introduce a separate near-pi disambiguation policy.
- No XR, controller, MuJoCo control path, URDF edit, MJCF edit, or real robot
  integration is involved.

## Related files

- `src/wheelloong_m2/ik/se3_error.py`
- `src/wheelloong_m2/ik/casadi_se3_error.py`
- `src/wheelloong_m2/ik/cost.py`
- `experiments/test_wheelloong_m2_ik_math.py`
- `docs/experiments/EXP-005_WHEELLOONG_M2_PINOCCHIO_KINEMATICS.md`
- `docs/experiments/EXP-006_WHEELLOONG_M2_CASADI_FK.md`
