"""Named arm postures with explicit provenance and public joint ordering."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .robot_model import ARM_JOINT_NAMES


@dataclass(frozen=True)
class TeleoperationInitialPosture:
    """One user-specified dual-arm MoveJ posture in public ``q_arm`` order."""

    left_q_rad: tuple[float, ...]
    right_q_rad: tuple[float, ...]
    left_move_j_vel: float
    left_move_j_acc: float
    right_move_j_vel: float
    right_move_j_acc: float

    def __post_init__(self) -> None:
        if len(self.left_q_rad) != 7 or len(self.right_q_rad) != 7:
            raise ValueError("left_q_rad and right_q_rad must each contain seven joints")
        values = np.asarray((*self.left_q_rad, *self.right_q_rad), dtype=float)
        if values.shape != (len(ARM_JOINT_NAMES),) or not np.all(np.isfinite(values)):
            raise ValueError("initial posture joints must be 14 finite values")
        rates = np.asarray(
            (
                self.left_move_j_vel,
                self.left_move_j_acc,
                self.right_move_j_vel,
                self.right_move_j_acc,
            ),
            dtype=float,
        )
        if not np.all(np.isfinite(rates)) or np.any(rates <= 0.0):
            raise ValueError("MoveJ velocity and acceleration values must be positive")

    def q_arm(self) -> np.ndarray:
        """Return a new 14-vector: seven left joints followed by seven right."""
        return np.asarray((*self.left_q_rad, *self.right_q_rad), dtype=float)


# User Instruction, 2026-08-21: values supplied as two ROS 2 MoveJ requests.
# Project Source Evidence: ``mujoco_joint_map.yaml`` maps motion indices 0..6
# directly to the same named left/right arm joints, and the arm-driver feedback
# viewer copies ``ArmInfo.left_joint/right_joint`` into those names with no sign
# conversion. The real MoveJ-command-to-feedback path is not independently
# exercised here; this constant is used for simulation initialization only.
USER_REQUESTED_TELEOP_INITIAL_POSTURE = TeleoperationInitialPosture(
    left_q_rad=(
        -1.5707963,
        1.2217305,
        1.5707963,
        -1.5707963,
        1.5707963,
        0.0,
        0.0,
    ),
    right_q_rad=(
        1.5707963,
        1.2217305,
        -1.5707963,
        -1.5707963,
        -1.5707963,
        0.0,
        0.0,
    ),
    left_move_j_vel=0.5,
    left_move_j_acc=0.5,
    right_move_j_vel=0.8,
    right_move_j_acc=0.8,
)
