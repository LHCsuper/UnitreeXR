# EXP-003 — `wheelloong_m2` URDF/MJCF Arm FK Consistency

## Objective

Validate whether the checked-in `wheelloong_m2` URDF and controlled MJCF
produce the same torso-relative end-link forward kinematics for both
seven-joint arms.

## Question

When identical named arm-joint values are applied directly to both models,
do the poses of `left_arm_link_7` and `right_arm_link_7`, relative to
`torso_link`, agree between Pinocchio and MuJoCo?

## Hypothesis

The two model descriptions represent the same arm kinematic chains and can
be compared directly, without a fixed rotation, axis permutation, mirror, or
other compensation transform.

## Setup

- URDF: `src/description/wheelloong_m2/urdf/wheelloong_m2.urdf`
- MJCF: `src/description/wheelloong_m2/mujoco/wheelloong_m2_controlled.xml`
- Pinocchio model loaded from the URDF.
- MuJoCo model loaded from the MJCF.
- Script: `experiments/validate_urdf_mujoco_fk.py`
- Target bodies/frames: `torso_link`, `left_arm_link_7`, `right_arm_link_7`
- Tested joints: `left_arm_joint_1` through `left_arm_joint_7`, and
  `right_arm_joint_1` through `right_arm_joint_7`.

## Procedure

1. Parse both source files and require the 14 named arm joints and the three
   target link/body names to exist.
2. Compare every identically named source joint's axis, finite limits, and
   parent-to-child topology. The script prints raw local origin pose values
   from both source representations for inspection.
3. Resolve each tested joint independently by name in the loaded Pinocchio
   and MuJoCo models. Do not assume their configuration indices are shared.
4. For every case, start from each backend's own neutral configuration
   (`pin.neutral(model)` and `model.qpos0`), then write the same scalar
   values to the 14 named arm joints.
5. Run Pinocchio `forwardKinematics` and `updateFramePlacements`; run MuJoCo
   `mj_forward` only. No dynamics step is performed.
6. Compute direct relative transforms:

```text
^torso T_target = inverse(^world T_torso) × ^world T_target
```

7. Measure translation as the Euclidean norm in metres, and rotation as
   `||log3(R_pinocchio × R_mujoco^T)||` in radians. No acceptance threshold
   is applied by the script.

The cases are deterministic and all generated values are within the common
source joint limits:

- Case 0: both arms at limit-derived neutral values.
- Case 1: selected left-arm joints changed.
- Case 2: selected right-arm joints changed.
- Case 3: both arms changed.
- Case 4: both arms changed using fixed-seed (`20260820`) safe-random limit
  fractions.

## Raw Data

Command:

```bash
python3 experiments/validate_urdf_mujoco_fk.py
```

Source-structure result:

```text
Required 14 arm joint names: present in both source files and loaded models.
Required targets: torso_link, left_arm_link_7, right_arm_link_7 present.
Axis, finite limits, and parent-child topology discrepancies: none.
```

Measured torso-relative target-pose errors:

| Case | Arm | Position error (m) | Rotation error (rad) | Rotation error (deg) |
| --- | --- | ---: | ---: | ---: |
| 0 — limit-derived neutral | Left | 0.000000000000e+00 | 0.000000000000e+00 | 0.000000000000e+00 |
| 0 — limit-derived neutral | Right | 1.521375868448e-14 | 4.132388077110e-13 | 2.367683961286e-11 |
| 1 — selected left joints changed | Left | 5.551115123126e-17 | 2.361223718672e-16 | 1.352881535661e-14 |
| 1 — selected left joints changed | Right | 1.521375868448e-14 | 4.132388077110e-13 | 2.367683961286e-11 |
| 2 — selected right joints changed | Left | 0.000000000000e+00 | 0.000000000000e+00 | 0.000000000000e+00 |
| 2 — selected right joints changed | Right | 1.514482067861e-14 | 4.132379844371e-13 | 2.367679244274e-11 |
| 3 — both arms changed | Left | 1.241267076624e-16 | 2.770277955903e-16 | 1.587252349513e-14 |
| 3 — both arms changed | Right | 7.137309436525e-14 | 3.859093612430e-13 | 2.211097767381e-11 |
| 4 — fixed-seed safe random | Left | 1.570092458684e-16 | 2.094534847290e-16 | 1.200080067928e-14 |
| 4 — fixed-seed safe random | Right | 9.254183423523e-14 | 3.673196057318e-13 | 2.104586314084e-11 |

Maximum measured errors:

```text
left position:  1.570092458684e-16 m
left rotation:  2.770277955903e-16 rad (1.587252349513e-14 deg)
right position: 9.254183423523e-14 m
right rotation: 4.132388077110e-13 rad (2.367683961286e-11 deg)
```

## Observations

- Pinocchio and MuJoCo use different configuration addresses; assignment by
  shared numeric index would be invalid.
- Pinocchio represents continuous gripper joints with a non-scalar
  configuration, but the 14 tested arm joints each resolve to scalar
  configuration entries in both backends.
- All tested poses were computed directly from the same named arm-joint
  values. No alignment or compensation transform was introduced.
- The largest measured translation difference was
  `9.254183423523e-14 m`; the largest rotation difference was
  `4.132388077110e-13 rad`.

## Analysis

The observed differences are at double-precision floating-point scale. They
are consistent with numerical evaluation and source-representation rounding,
not with a fixed frame mismatch: the comparison remains near zero across
neutral, unilateral, bilateral, and deterministic random configurations.

This result validates only the scoped torso-relative FK comparison for the
two arm end links and the five deterministic configurations above. It does
not validate IK, controller/XR mapping, simulation dynamics, contact
behavior, real-robot behavior, or arbitrary links outside this test.

## Conclusion

Confirmed: for the tested `wheelloong_m2` arm chains, the URDF evaluated by
Pinocchio and the controlled MJCF evaluated by MuJoCo produce directly
compatible torso-relative end-link FK. No fixed rotation, mirror,
axis-swap, or compensation transform is supported or required by this
experiment.

## Confidence

High for direct arm FK consistency within the exercised source model and
configuration set. Confidence is based on source-name/axis/limit/topology
checks and five deterministic end-to-end FK cases covering both unilateral
and bilateral arm changes.

## Open Questions

- Whether the same direct consistency holds for links not covered by this
  experiment.
- Whether later XR-to-robot mapping experiments require a separate,
  physically defined transform. This is outside the scope of EXP-003.

## Related Files

- `experiments/validate_urdf_mujoco_fk.py`
- `src/description/wheelloong_m2/urdf/wheelloong_m2.urdf`
- `src/description/wheelloong_m2/mujoco/wheelloong_m2_controlled.xml`
- `docs/experiments/EXPERIMENT_TEMPLATE.md`