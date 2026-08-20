"""Position-actuator loop for the named wheelloong_m2 MuJoCo arm interface."""

from __future__ import annotations

import numpy as np

from wheelloong_m2.kinematics.robot_model import ARM_JOINT_NAMES

from .mujoco_model import WheelloongM2MuJoCo


class MujocoArmPositionController:
    """Write q_arm position targets through existing MuJoCo actuators only."""

    def __init__(self, simulation: WheelloongM2MuJoCo) -> None:
        if simulation.model is None or simulation.data is None:
            raise RuntimeError("simulation must be loaded before creating its arm controller")
        self.simulation = simulation
        self._target_q_arm: np.ndarray | None = None

    @staticmethod
    def _validate_q_arm(q_arm: np.ndarray) -> np.ndarray:
        values = np.asarray(q_arm, dtype=float)
        if values.shape != (len(ARM_JOINT_NAMES),):
            raise ValueError(
                f"q_arm must have shape ({len(ARM_JOINT_NAMES)},), got {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("q_arm must contain only finite values")
        return values

    def set_arm_position_target(self, q_arm: np.ndarray) -> None:
        """Map public q_arm values to existing position actuators in data.ctrl.

        This deliberately does not assign ``data.qpos``: MuJoCo's existing
        position actuators and physics stepping remain responsible for motion.
        """
        values = self._validate_q_arm(q_arm)
        if len(self.simulation.arm_joint_addresses) != len(ARM_JOINT_NAMES):
            raise RuntimeError("Named arm mapping is unavailable; call simulation.load() first")
        assert self.simulation.data is not None
        for address in self.simulation.arm_joint_addresses:
            self.simulation.data.ctrl[address.ctrl_index] = values[address.q_arm_index]
        self._target_q_arm = values.copy()

    def run_for_seconds(self, duration: float) -> None:
        """Hold the latest position target while advancing the physics simulation."""
        if not np.isfinite(duration) or duration <= 0.0:
            raise ValueError("duration must be a finite positive number of seconds")
        if self._target_q_arm is None:
            raise RuntimeError("set_arm_position_target(q_arm) must be called before running")
        assert self.simulation.data is not None
        end_time = self.simulation.data.time + duration
        while self.simulation.data.time < end_time:
            self.simulation.step()
