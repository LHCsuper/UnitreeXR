#!/usr/bin/env python3
"""EXP-006: compare wheelloong_m2 numeric Pinocchio FK to CasADi symbolic FK.

This offline experiment constructs no IK, cost function, NLP, IPOPT/Opti
problem, controller, XR connection, or robot-control path.
"""

from __future__ import annotations

from pathlib import Path
import sys

import casadi as ca
import numpy as np
import pinocchio as pin
import pinocchio.casadi as cpin


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SRC_PATH = REPOSITORY_ROOT / "src"
if str(SRC_PATH) not in sys.path:
    sys.path.insert(0, str(SRC_PATH))

from wheelloong_m2.kinematics.casadi_fk import WheelloongM2CasadiKinematics
from wheelloong_m2.kinematics.dual_arm_fk import WheelloongM2Kinematics
from wheelloong_m2.kinematics.robot_model import ARM_JOINT_NAMES, arm_joint_limits


RANDOM_SEED = 20260821


def evaluate_symbolic_fk(
    kinematics: WheelloongM2CasadiKinematics,
    q_arm: np.ndarray,
) -> dict[str, np.ndarray]:
    left_position, left_rotation, right_position, right_rotation = kinematics.FK_function(
        q_arm
    )
    return {
        "left_position": np.asarray(left_position, dtype=float).reshape(3),
        "left_rotation": np.asarray(left_rotation, dtype=float).reshape(3, 3),
        "right_position": np.asarray(right_position, dtype=float).reshape(3),
        "right_rotation": np.asarray(right_rotation, dtype=float).reshape(3, 3),
    }


def print_comparison(
    case_name: str,
    numeric_kinematics: WheelloongM2Kinematics,
    symbolic_kinematics: WheelloongM2CasadiKinematics,
    q_arm: np.ndarray,
) -> None:
    numeric_poses = numeric_kinematics.forward_kinematics(q_arm)
    symbolic_poses = evaluate_symbolic_fk(symbolic_kinematics, q_arm)

    print(f"=== {case_name} ===")
    print(f"q_arm = {np.array2string(q_arm, precision=10, suppress_small=False)}")
    for side, pose_key in (("Left", "left_ee_pose"), ("Right", "right_ee_pose")):
        key_prefix = side.lower()
        numeric_pose = numeric_poses[pose_key]
        position_error = float(
            np.linalg.norm(numeric_pose.translation - symbolic_poses[f"{key_prefix}_position"])
        )
        rotation_error = float(
            np.linalg.norm(
                pin.log3(
                    numeric_pose.rotation @ symbolic_poses[f"{key_prefix}_rotation"].T
                )
            )
        )
        print(f"{side} position error: {position_error:.12e} m")
        print(f"{side} rotation error: {rotation_error:.12e} rad")
    print("No PASS threshold is applied; this is a numeric-symbolic FK comparison.")
    print()


def main() -> None:
    numeric_kinematics = WheelloongM2Kinematics()
    symbolic_kinematics = WheelloongM2CasadiKinematics()
    if tuple(address.name for address in symbolic_kinematics.arm_joint_addresses) != ARM_JOINT_NAMES:
        raise AssertionError("Symbolic q_arm order does not match the numeric kinematics contract")

    limits = arm_joint_limits(numeric_kinematics.model)
    random_generator = np.random.default_rng(RANDOM_SEED)
    q_arm_random = random_generator.uniform(limits[:, 0], limits[:, 1])

    print("EXP-006 — wheelloong_m2 CasADi symbolic FK")
    print(f"CasADi version: {ca.__version__}")
    print(f"Pinocchio version: {pin.__version__}")
    print(f"pinocchio.casadi module: {cpin.__name__}")
    print(f"Symbolic q_arm shape: {symbolic_kinematics.q_arm.shape}")
    print(f"Symbolic full q shape: {symbolic_kinematics.full_q_symbolic.shape}")
    print()

    print_comparison(
        "Case 0 — q_arm=zeros",
        numeric_kinematics,
        symbolic_kinematics,
        np.zeros(len(ARM_JOINT_NAMES)),
    )
    print_comparison(
        f"Case 1 — fixed-seed legal random q_arm (seed={RANDOM_SEED})",
        numeric_kinematics,
        symbolic_kinematics,
        q_arm_random,
    )


if __name__ == "__main__":
    main()
