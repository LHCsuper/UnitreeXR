# EXP-010 — `wheelloong_m2` Multi-Rate Teleoperation Simulation Loop

## Frequency design

S2.1 establishes a simulation-only multi-rate architecture with one shared
configuration source:

```text
TARGET_HZ      = 120
IK_HZ          = 250
SIMULATION_HZ  = 1000
```

The 2.0-second experiment executes 2000 MuJoCo physics steps. At each
simulation-time tick, target work occurs before IK work when both are due, so
the IK solve consumes the newest target at that timestamp.

These rates describe scheduled simulation time. They do not claim that the
Python process achieves wall-clock real-time operation.

## Buffer semantics

`DualArmTargetBuffer` stores one timestamped pair of torso-relative Pinocchio
`SE3` operational targets:

```text
^torso T_WL
^torso T_WR
```

`update(timestamp, left_pose, right_pose)` replaces that single pair.
`get_latest()` returns a copy of the current value or `None` before the first
update. There is deliberately no queue and no historical target replay:
teleoperation consumers use the latest source value only.

The fake producer is simulation-only, not XR. It samples at the target ticks
and generates fixed-orientation SE(3) targets with left/right sinusoidal X
translation amplitudes of `0.03 m` and `0.025 m` at `0.5 Hz`.

## Scheduler

`MultiRateScheduler` uses a simulation-time accumulator, not `sleep`, with
all periods imported from `simulation.runtime.config`. It emits a target due
flag, an IK due flag, and a physics tick on every `next_tick()` call.

If an external caller advances it late, overdue periodic work is coalesced to
one current-time tick rather than replaying stale events. In this deterministic
experiment, all calls occur at the configured 1 kHz simulation rate.

The loop ordering is:

```text
if target tick: update latest target buffer
if IK tick:     solve latest target with previous q as q_init and q_prev
every sim tick: write current q_target to named MuJoCo position actuators
every sim tick: mj_step
```

No arm qpos target is assigned directly.

## Latency

Each S1.2.1 IPOPT solve returns wall-clock `solve_time`. EXP-010 records all
500 samples and reports mean, p95, and maximum solver latency. These values
measure solver execution only; they are not XR, transport, motor-controller,
or real-robot latency.

## Tracking results

Run:

```bash
python3 experiments/test_wheelloong_m2_multirate_loop.py
```

The recorded 2.0-second run produced:

```text
Counts / simulation-time frequencies
target updates: 240  (120.000000 Hz)
IK solves:       500  (250.000000 Hz)
physics steps:  2000 (1000.000000 Hz)

IK latency
mean: 4.168258 ms
p95:  4.546778 ms
max: 15.083930 ms

Final tracking
||qpos - q_target||:      2.110591184150e-02 rad
left EE position error:   7.230049812038e-03 m
left EE rotation error:   1.186173159313e-03 rad
right EE position error:  6.023004265867e-03 m
right EE rotation error:  1.070827212223e-03 rad
```

The tracking error combines the S1.2.1 soft IK objective, 250 Hz target
updates, and existing MuJoCo position-actuator dynamics under a changing
target. It does not change the `W_L` / `W_R` operational frame definitions.

## Limitations

- This is simulation only; no XR, PICO, coordinate adapter, hand retargeting,
  real robot, or motor controller is present.
- The source is a deterministic fake signal, not a reusable trajectory
  generator. There is no filtering, target transport, packet timestamp
  synchronization, or target queue.
- There is no trajectory generator, velocity/acceleration limit, collision
  avoidance, torque policy, or real-time deadline enforcement.
- The loop uses the existing soft-cost IK and existing MuJoCo position
  actuators; it does not introduce a hard task-space tracking constraint or
  controller tuning.
- No URDF or MJCF file is modified.

## Related files

- `src/wheelloong_m2/simulation/runtime/config.py`
- `src/wheelloong_m2/simulation/runtime/target_buffer.py`
- `src/wheelloong_m2/simulation/runtime/scheduler.py`
- `src/wheelloong_m2/simulation/mujoco_arm_controller.py`
- `src/wheelloong_m2/ik/dual_arm_ik.py`
- `experiments/test_wheelloong_m2_multirate_loop.py`
- `docs/experiments/EXP-009_WHEELLOONG_M2_MJOC0_IK_LOOP.md`
