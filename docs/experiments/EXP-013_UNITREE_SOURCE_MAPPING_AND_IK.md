# EXP-013 — Unitree TeleVuer Mapping and Arm IK Source Inspection

## Objective

Inspect the two official repositories named by the project owner and identify
which parts can be migrated to `wheelloong_m2` without guessing robot-specific
frames or controlling hardware.

## Upstream versions

Source inspection used shallow clones of the official repositories at:

```text
unitreerobotics/televuer
766de45e74373ae0ea66321d942ce538385655a5

unitreerobotics/xr_teleoperate
845b25a32f7febedf220e830952a7134897adb9d
```

The `xr_teleoperate` tree pins its `teleop/televuer` submodule to the same
TeleVuer commit above.

## TeleVuer coordinate path — Source Evidence

`src/televuer/tv_wrapper.py` states the basis conventions:

```text
OpenXR: +X right, +Y up, +Z back
Robot:  +X front, +Y left, +Z up
```

It defines the proper basis rotation:

```text
S = R_robot_openxr =
[[ 0, 0,-1],
 [-1, 0, 0],
 [ 0, 1, 0]]
```

Thus OpenXR vector components map as:

```text
+X_openxr -> -Y_robot
+Y_openxr -> +Z_robot
+Z_openxr -> -X_robot
```

For an OpenXR pose it applies the similarity transform:

```text
R_robot = S R_openxr S^T
p_robot = S p_openxr
```

In hand-tracking mode TeleVuer additionally right-multiplies different fixed
left/right arm initial-pose rotations. In controller mode the upstream source
states that controller pose data already follows its Unitree humanoid arm
initial-pose convention, so those fixed rotations are not applied.

That controller-local claim is Source Evidence for the TeleVuer/WebXR source,
not evidence that this project's XRoboToolkit/PICO controller local axes are
identical. The project therefore does not copy that assumption.

## TeleData target semantics — Source Evidence

The current default `arm_reference_mode="head_yaw"`:

1. extracts a yaw-only head rotation in robot basis;
2. subtracts the head position from each arm position;
3. left-multiplies pose rotation and position by inverse head yaw; and
4. adds fixed Unitree waist offsets `+0.15 m` in robot X and `+0.45 m` in
   robot Z.

The resulting 4x4 matrices are returned as:

```text
TeleData.left_wrist_pose
TeleData.right_wrist_pose
```

`teleop/teleop_hand_and_arm.py` passes those matrices directly to
`arm_ik.solve_ik(...)`.

The fixed `0.15/0.45 m` offsets are Unitree model-specific and are not
transferred to `wheelloong_m2`.

## Unitree IK structure — Source Evidence

For `G1_29_ArmIK`, `robot_arm_ik.py`:

- builds a reduced Pinocchio model;
- adds left/right operational frames `0.05 m` along wrist-yaw local X;
- constructs independent CasADi symbolic FK;
- uses translation error `p_current - p_target`;
- uses rotation error `Log(R_current R_target^T)`;
- constrains joint positions to model limits;
- minimizes translation, rotation, zero-configuration regularization, and
  previous-solution smoothness terms with weights `50`, `1`, `0.02`, `0.1`;
- warm-starts IPOPT from current/previous arm state; and
- applies a four-sample weighted moving filter after solving.

## Migration assessment

The existing `wheelloong_m2` implementation already contains the transferable
IK core:

- named 14-DOF reduced arm interface;
- robot-specific `W_L` / `W_R` operational frames derived from its own URDF;
- independent Pinocchio and CasADi FK;
- the same translation and SO(3)-log error direction;
- the same default four objective weights;
- URDF position bounds and IPOPT warm start; and
- MuJoCo position-actuator validation.

The following Unitree-specific details are deliberately not copied:

- G1/H1/H2 joint names or models;
- Unitree wrist `+0.05 m` operational-frame offsets;
- Unitree head-to-waist `0.15/0.45 m` offsets;
- feed-forward torque output or real robot controller;
- the assertion that PICO/XRoboToolkit controller axes already equal the
  Unitree arm initial-pose convention; and
- post-solve moving-filter behavior without a separate latency/tracking study.

## Conclusion

The Unitree IK algorithmic structure is migrated and robot-model-specific
parts are supplied by the checked-in `wheelloong_m2` URDF. The remaining
implementation work is the explicit PICO/OpenXR relative-motion adapter and
simulation-only end-to-end validation recorded by EXP-014.

## Upstream source links

- https://github.com/unitreerobotics/televuer/blob/766de45e74373ae0ea66321d942ce538385655a5/src/televuer/tv_wrapper.py
- https://github.com/unitreerobotics/xr_teleoperate/blob/845b25a32f7febedf220e830952a7134897adb9d/teleop/robot_control/robot_arm_ik.py
- https://github.com/unitreerobotics/xr_teleoperate/blob/845b25a32f7febedf220e830952a7134897adb9d/teleop/teleop_hand_and_arm.py
