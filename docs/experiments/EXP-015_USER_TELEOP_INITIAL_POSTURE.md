# EXP-015 — User-Specified Teleoperation Initial Posture

## Objective

Make the owner's left/right ROS 2 `MoveJ` posture the explicit default
initial pose for Wheelloong M2 XR-to-MuJoCo simulation, without calling the
ROS service or controlling the physical robot.

## Input — User Instruction

The owner supplied the following seven-joint arrays in driver order:

```text
left  = [-1.5707963, 1.2217305,  1.5707963, -1.5707963,
          1.5707963, 0.0,        0.0]

right = [ 1.5707963, 1.2217305, -1.5707963, -1.5707963,
         -1.5707963, 0.0,        0.0]
```

The supplied MoveJ velocity/acceleration values are:

```text
left:  velocity=0.5, acceleration=0.5
right: velocity=0.8, acceleration=0.8
```

These rates are retained as provenance metadata. They are not presented as
MuJoCo trajectory limits: the current simulation uses its existing position
actuator dynamics and still has no velocity/acceleration trajectory generator.

## Joint-order evidence

Project Source Evidence:

- `mujoco_joint_map.yaml` maps each side's `motion_index: 0..6` directly to
  `left/right_arm_joint_1..7`.
- `view_m2_mujoco_arm_driver_state.py` copies
  `ArmInfo.left_joint/right_joint` directly to those named MuJoCo joints,
  without a sign conversion.

The service-command-to-feedback path was not exercised in this experiment.
The direct command-array interpretation is therefore based on the owner's
explicit requested posture plus the checked-in feedback/model mapping, not a
new claim about undocumented driver internals.

The public 14-vector is left seven joints followed by right seven joints:

```text
q_arm = [left..., right...]
```

## Implementation

`USER_REQUESTED_TELEOP_INITIAL_POSTURE` stores the named arrays and supplied
MoveJ rate metadata. The simulation CLI selects it by default; model-neutral
startup remains available with `--initial-posture neutral`.

Initialization is simulation-only:

1. load and reset the checked-in MuJoCo model;
2. validate all 14 values against named URDF limits;
3. command the values through the existing named position actuators;
4. advance 3000 physics steps (`3.0 s`) without writing arm qpos directly;
5. use the requested posture as the robot operational-pose anchor and IK
   nominal configuration; and
6. sample/anchor XR only after actuator settling.

Using the requested pose as `q_nom` is necessary because the previous neutral
regularization otherwise pulled a valid stationary target toward zero and
produced about `0.05 rad` of EE orientation residual in the preliminary run.
The solver default remains model neutral for all existing callers.

## Validation

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
python3 experiments/test_wheelloong_m2_user_initial_posture.py
```

All 14 requested values are within the named URDF limits. The full test suite
reported:

```text
9 passed
```

The one-second initialized relative-XR simulation reported:

```text
initialization physics steps: 3000
initial actuator tracking error: 0.011899541117 rad

target updates / IK solves / physics steps: 120 / 250 / 1000
left target delta:  [0, -0.010539749547, 0] m
right target delta: [0,  0.008431799638, 0] m

final joint tracking error: 0.011959054460 rad
left EE error:  0.006378833297 m / 0.012125772001 rad
right EE error: 0.006380665350 m / 0.012132350243 rad
```

The preliminary actuator-only three-second check also ended with finite qpos
and qvel, the same approximately `0.0119 rad` joint error, and zero contacts at
the final settled sample. Zero final contacts is Experimental Evidence for
that sample only, not a collision-free workspace guarantee.

## Usage

The supplied posture is now the default:

```bash
python3 experiments/run_wheelloong_m2_xr_mujoco.py \
  --source fake --duration 5 --visualize
```

Explicit neutral comparison:

```bash
python3 experiments/run_wheelloong_m2_xr_mujoco.py \
  --source fake --duration 5 --initial-posture neutral
```

## Boundary

No `ros2 service call` was executed. No `/arm_driver/joint_move` client,
physical motor command, velocity/acceleration controller, or real robot path
was added.

## Related files

- `src/wheelloong_m2/kinematics/postures.py`
- `src/wheelloong_m2/ik/dual_arm_ik.py`
- `src/wheelloong_m2/simulation/xr_mujoco_runtime.py`
- `experiments/run_wheelloong_m2_xr_mujoco.py`
- `experiments/test_wheelloong_m2_user_initial_posture.py`
- `tests/test_teleop_initial_posture.py`
