# UnitreeXR

UnitreeXR is a PICO 4 Ultra teleoperation and simulation workspace for the
`wheelloong_m2` robot model.

The implemented simulation pipeline is:

```text
PICO 4 Ultra / deterministic fake source
→ XRoboToolkit raw controller pose
→ initialized OpenXR-to-robot relative-motion adapter
→ 14-DOF Pinocchio/CasADi/IPOPT dual-arm IK
→ named MuJoCo position actuators
```

Current status:

```text
Phase 3 — Unitree Coordinate Mapping
```

The Unitree TeleVuer/IK source structure has been inspected at pinned commits,
the robot-specific IK has been migrated to the checked-in Wheelloong M2 URDF,
and the end-to-end synthetic MuJoCo path is validated. See
`docs/STATUS.md`, EXP-013, and EXP-014 for evidence and limitations.

Run the deterministic validation:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q tests/test_relative_xr_adapter.py
python3 experiments/test_wheelloong_m2_relative_xr_mujoco.py
```

Run a simulation application:

```bash
# Headless synthetic input
python3 experiments/run_wheelloong_m2_xr_mujoco.py --source fake --duration 5

# Live PICO/XRoboToolkit input, still MuJoCo-only
python3 experiments/run_wheelloong_m2_xr_mujoco.py \
  --source robotoolkit --duration 30 --visualize
```

Both commands default to the user-specified MoveJ initial posture, positioned
through MuJoCo actuators for three simulated seconds before XR anchoring. For
the earlier model-neutral baseline, add `--initial-posture neutral`.

There is no real robot control path in this repository. Closing this gap would
require separate collision, velocity/acceleration, tracking-loss, and hardware
safety work plus explicit authorization.
