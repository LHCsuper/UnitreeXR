"""CasADi symbolic FK for the existing wheelloong_m2 operational EE frames.

This module is intentionally independent of ``dual_arm_fk.py``: numeric
Pinocchio FK and symbolic CasADi expressions can be compared directly.
"""

from __future__ import annotations

import casadi as ca
import numpy as np
import pinocchio as pin
import pinocchio.casadi as cpin

from .frames import LEFT_EE_FRAME, RIGHT_EE_FRAME, TORSO_FRAME_NAME, OperationalEEFrame
from .robot_model import ArmJointAddress, arm_joint_addresses, load_wheelloong_m2_model


def _casadi_se3(transform: pin.SE3) -> cpin.SE3:
    """Convert an existing fixed numeric SE3 without redefining its values."""
    return cpin.SE3(ca.SX(transform.rotation), ca.SX(transform.translation))


class WheelloongM2CasadiKinematics:
    """Symbolic torso-relative FK for the fixed S0.5b frames ``W_L`` / ``W_R``."""

    def __init__(self) -> None:
        loaded = load_wheelloong_m2_model()
        self.numeric_model = loaded.model
        self.urdf_path = loaded.urdf_path
        self.model = cpin.Model(self.numeric_model)
        self.data = self.model.createData()
        self._arm_addresses = arm_joint_addresses(self.numeric_model)
        self._torso_frame_id = self._required_frame_id(TORSO_FRAME_NAME)
        self._ee_frames = (LEFT_EE_FRAME, RIGHT_EE_FRAME)
        self._ee_parent_frame_ids = {
            frame.name: self._required_frame_id(frame.parent_frame_name)
            for frame in self._ee_frames
        }

        self.q_arm = ca.SX.sym("q_arm", len(self._arm_addresses))
        self.full_q_symbolic = self._build_full_q_symbolic()
        self._symbolic_fk = self.compute_symbolic_fk()
        self.FK_function = ca.Function(
            "dual_arm_fk",
            [self.q_arm],
            [
                self._symbolic_fk["left_position"],
                self._symbolic_fk["left_rotation"],
                self._symbolic_fk["right_position"],
                self._symbolic_fk["right_rotation"],
            ],
            ["q_arm"],
            ["left_position", "left_rotation", "right_position", "right_rotation"],
        )

    @property
    def arm_joint_addresses(self) -> tuple[ArmJointAddress, ...]:
        """Named mapping shared with ``WheelloongM2Kinematics``."""
        return self._arm_addresses

    def _required_frame_id(self, frame_name: str) -> int:
        frame_id = int(self.model.getFrameId(frame_name, cpin.FrameType.BODY))
        if frame_id >= len(self.model.frames):
            raise KeyError(f"CasADi model is missing required frame {frame_name}")
        return frame_id

    def _build_full_q_symbolic(self) -> ca.SX:
        """Start from Pinocchio neutral and scatter the public 14-DOF symbols."""
        full_q = ca.SX(pin.neutral(self.numeric_model))
        for address in self._arm_addresses:
            full_q[address.pinocchio_q_index] = self.q_arm[address.q_arm_index]
        return full_q

    def _torso_T_operational(self, frame: OperationalEEFrame) -> cpin.SE3:
        world_T_torso = self.data.oMf[self._torso_frame_id]
        world_T_parent = self.data.oMf[self._ee_parent_frame_ids[frame.name]]
        return world_T_torso.inverse() * world_T_parent * _casadi_se3(
            frame.parent_T_operational
        )

    def compute_symbolic_fk(self) -> dict[str, ca.SX]:
        """Return symbolic components of ``^torso T_WL`` and ``^torso T_WR``.

        The returned dictionary has ``left/right_position`` and
        ``left/right_rotation`` SX expressions. It does not return a world pose.
        """
        cpin.forwardKinematics(self.model, self.data, self.full_q_symbolic)
        cpin.updateFramePlacements(self.model, self.data)

        torso_T_left = self._torso_T_operational(LEFT_EE_FRAME)
        torso_T_right = self._torso_T_operational(RIGHT_EE_FRAME)
        return {
            "left_position": torso_T_left.translation,
            "left_rotation": torso_T_left.rotation,
            "right_position": torso_T_right.translation,
            "right_rotation": torso_T_right.rotation,
        }
