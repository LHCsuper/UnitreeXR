# EXP-009 — `wheelloong_m2` MuJoCo IK Loop

## Objective

Establish the minimum offline closed loop from the S1.2.1 dual-arm IK output
to the existing `wheelloong_m2` MuJoCo position actuators, then validate the
result by reading simulated joint positions and running the existing
Pinocchio FK.

This is MuJoCo simulation only. It does not connect XR, PICO, a coordinate
adapter, a real robot, a low-level motor controller, or a trajectory
generator.

## MJCF model

The loader consumes the existing checked-in model without copying or editing
it:

```text
src/description/wheelloong_m2/mujoco/wheelloong_m2_controlled.xml
```

`WheelloongM2MuJoCo.load()` creates `mujoco.MjModel` and `mujoco.MjData` and
resets with `mj_resetData` followed by `mj_forward`. The recorded runtime was
MuJoCo `3.10.0`, with `nq=32`, `nv=32`, `nu=17`, and a `0.001 s` physics
timestep.

## Joint mapping

The 14-element public order is still `ARM_JOINT_NAMES`: left arm joints 1–7,
then right arm joints 1–7. The loader does not assume an MJCF numeric index.
For every required joint it:

1. resolves the MJCF joint ID by name;
2. reads its scalar `jnt_qposadr`; and
3. scans loaded actuators for exactly one joint-transmission position actuator
   attached to that joint, then uses its ctrl index.

The confirmed loaded mapping is:

```text
left_arm_joint_1 .. left_arm_joint_7:  qpos adr 6..12,  ctrl adr 2..8
right_arm_joint_1 .. right_arm_joint_7: qpos adr 19..25, ctrl adr 9..15
```

The module prints every name, qpos address, ctrl address, and actuator name at
startup. The discovered actuator names are `left_arm_joint_N_pos` and
`right_arm_joint_N_pos`; they are verified from the loaded transmission and
position-actuator structure rather than inferred from an array order.

## Controller interface

`MujocoArmPositionController.set_arm_position_target(q_arm)` accepts
`numpy.ndarray(shape=(14,))` in the existing public order. It writes only the
resolved actuator values to `data.ctrl`.

It deliberately never writes `data.qpos`. Motion is produced by the existing
MJCF position actuators and `mj_step`, preserving the intended simulation
dynamics/controller path.

## Loop frequency

`run_for_seconds(duration)` holds the most recently supplied arm position
target and advances one `mj_step` per loop iteration. With the loaded MJCF
time step of `0.001 s`, the offline physics loop advances at 1 kHz simulation
time. This stage has no trajectory generator, timing scheduler, or real-time
guarantee.

## Tests

Run:

```bash
python3 experiments/test_wheelloong_m2_mujoco_ik_loop.py
```

For each test, the script resets MuJoCo, synthesizes `^torso T_WL/WR` targets
with the existing numeric Pinocchio FK, calls `WheelloongM2DualArmIK`, applies
the returned `q_arm` through the position-actuator interface, advances 2.0 s
of MuJoCo simulation, then reads named arm qpos values and runs Pinocchio FK
again.

The cases are:

1. neutral arm pose;
2. left operational target translated `+X` by `0.05 m`, right target neutral;
   and
3. right operational target translated `-X` by `0.05 m`, left target neutral.

Each output includes IK success and solution, simulated joint tracking norm
`||qpos - q_target||`, and torso-frame EE position/rotation error after
Pinocchio FK. The unilateral cases also compare simulated left/right motion
norms.

## Results

MJCF loading, actuator mapping, and IPOPT all succeeded. Every case advanced
to `2.001 s` and used actuator control rather than direct qpos assignment.

```text
Case                                q tracking error (rad)
neutral pose                        1.284211597768e-03
left target +X 0.05 m               1.629371150726e-03
right target -X 0.05 m              1.688287922383e-03
```

Pinocchio FK round-trip errors from the final simulated qpos were:

```text
Case 1 — neutral
Left:  position 1.966553847386e-04 m, rotation 9.094708310348e-04 rad
Right: position 1.966775238558e-04 m, rotation 9.096571491269e-04 rad

Case 2 — left target +X 0.05 m
Left:  position 1.925958027111e-03 m, rotation 1.498533506921e-02 rad
Right: position 1.966967920182e-04 m, rotation 9.096657473287e-04 rad
Left/right simulated motion norms: 9.798156493143e-02 / 9.081511287270e-04 rad

Case 3 — right target -X 0.05 m
Left:  position 1.966490132671e-04 m, rotation 9.094707678238e-04 rad
Right: position 1.922354527371e-03 m, rotation 1.499818186250e-02 rad
Left/right simulated motion norms: 9.080023612243e-04 / 9.798796602569e-02 rad
```

The unilateral motion checks confirm that the requested arm moves much more
than the neutral arm. The active-arm EE residual includes both the S1.2.1
soft-cost IK residual and the measured MuJoCo actuator tracking difference;
it is not evidence of an EE-frame redefinition.

## Limitations

- This is an offline MuJoCo position-actuator loop, not a real robot or motor
  controller.
- There is no XR/PICO input, coordinate adapter, trajectory generator,
  velocity/acceleration bound, collision avoidance, torque policy, or contact
  validation.
- Only existing S0.5b operational frames `W_L` / `W_R` are used; they remain
  fixed palm/gripper-root operational frames, not calibrated fingertip TCPs.
- Target tracking is limited by the baseline soft IK objective and position
  actuator dynamics. No hard task-space constraint or controller tuning is
  introduced.
- No URDF or MJCF file is modified.

## Related files

- `src/wheelloong_m2/simulation/mujoco_model.py`
- `src/wheelloong_m2/simulation/mujoco_arm_controller.py`
- `src/wheelloong_m2/ik/dual_arm_ik.py`
- `src/wheelloong_m2/kinematics/dual_arm_fk.py`
- `experiments/test_wheelloong_m2_mujoco_ik_loop.py`
- `docs/experiments/EXP-008_WHEELLOONG_M2_NLP_IK.md`
