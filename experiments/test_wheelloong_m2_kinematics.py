#!/usr/bin/env python3
"""EXP-005: validate wheelloong_m2 Pinocchio FK/Jacobian interfaces only.

This is an offline kinematics experiment. It does not implement IK, connect
XR, run a MuJoCo controller, edit model files, or control hardware.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pinocchio as pin


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SRC_PATH = REPOSITORY_ROOT / "src"
if str(SRC_PATH) not in sys.path:
    sys.path.insert(0, str(SRC_PATH))

from wheelloong_m2.kinematics import ARM_JOINT_NAMES, WheelloongM2Kinematics
from wheelloong_m2.kinematics.robot_model import (
    ARM_Q_INDEX_BY_NAME,
    arm_joint_limits,
    print_arm_joint_index_table,
)


RANDOM_SEED = 20260821
FINITE_DIFFERENCE_JOINT_NAME = "left_arm_joint_4"
FINITE_DIFFERENCE_EPSILON = 1e-7


def format_transform(pose: pin.SE3) -> str:
    return np.array2string(pose.homogeneous, precision=10, suppress_small=False)


def print_case(
    name: str,
    kinematics: WheelloongM2Kinematics,
    q_arm: np.ndarray,
) -> None:
    poses = kinematics.forward_kinematics(q_arm)
    jacobians = kinematics.compute_jacobians(q_arm)
    print(f"=== {name} ===")
    print(f"q_arm = {np.array2string(q_arm, precision=10, suppress_small=False)}")
    print("^torso T_WL =")
    print(format_transform(poses["left_ee_pose"]))
    print("^torso T_WR =")
    print(format_transform(poses["right_ee_pose"]))
    print(f"left_J shape:  {jacobians['left_J'].shape}")
    print(f"right_J shape: {jacobians['right_J'].shape}")
    print()


def finite_difference_check(
    kinematics: WheelloongM2Kinematics,
    q_arm: np.ndarray,
    q_arm_index: int,
    epsilon: float,
) -> None:
    """Compare one-column spatial-Jacobian predictions to pose differences."""
    poses_before = kinematics.forward_kinematics(q_arm)
    jacobians = kinematics.compute_jacobians(q_arm)

    perturbed_q_arm = q_arm.copy()
    perturbed_q_arm[q_arm_index] += epsilon
    poses_after = kinematics.forward_kinematics(perturbed_q_arm)

    joint_name = ARM_JOINT_NAMES[q_arm_index]
    print("=== Finite-difference Jacobian sanity check ===")
    print(f"joint: {joint_name} (q_arm index {q_arm_index})")
    print(f"epsilon: {epsilon:.1e} rad")
    for side, pose_key, jacobian_key in (
        ("Left", "left_ee_pose", "left_J"),
        ("Right", "right_ee_pose", "right_J"),
    ):
        pose_before = poses_before[pose_key]
        pose_after = poses_after[pose_key]
        jacobian_prediction = epsilon * jacobians[jacobian_key][:, q_arm_index]

        observed_translation = pose_after.translation - pose_before.translation
        observed_rotation = pin.log3(pose_after.rotation @ pose_before.rotation.T)
        translation_error = float(
            np.linalg.norm(observed_translation - jacobian_prediction[:3])
        )
        rotation_error = float(
            np.linalg.norm(observed_rotation - jacobian_prediction[3:])
        )

        print(f"{side} translation error: {translation_error:.12e} m")
        print(f"{side} rotation error:    {rotation_error:.12e} rad")
    print("No PASS threshold is applied; this is a finite-difference sanity check.")


def main() -> None:
    kinematics = WheelloongM2Kinematics()
    limits = arm_joint_limits(kinematics.model)
    random_generator = np.random.default_rng(RANDOM_SEED)
    q_arm_random = random_generator.uniform(limits[:, 0], limits[:, 1])

    print("EXP-005 — wheelloong_m2 Pinocchio kinematics")
    print(f"URDF: {kinematics.urdf_path}")
    print(f"Pinocchio version: {pin.__version__}")
    print(f"q_arm DOF: {len(ARM_JOINT_NAMES)}")
    print_arm_joint_index_table(kinematics.model)
    print()

    print_case("Case 0 — q_arm=zeros", kinematics, np.zeros(len(ARM_JOINT_NAMES)))
    print_case(
        f"Case 1 — fixed-seed legal random q_arm (seed={RANDOM_SEED})",
        kinematics,
        q_arm_random,
    )
    finite_difference_check(
        kinematics,
        q_arm_random,
        ARM_Q_INDEX_BY_NAME[FINITE_DIFFERENCE_JOINT_NAME],
        FINITE_DIFFERENCE_EPSILON,
    )


if __name__ == "__main__":
    main()
