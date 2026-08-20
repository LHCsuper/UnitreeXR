"""Torso-relative FK and Jacobians for wheelloong_m2 logical EE frames."""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from .frames import LEFT_EE_FRAME, RIGHT_EE_FRAME, TORSO_FRAME_NAME, OperationalEEFrame
from .robot_model import ArmJointAddress, arm_joint_addresses, load_wheelloong_m2_model


def _copy_se3(transform: pin.SE3) -> pin.SE3:
    return pin.SE3(transform.rotation.copy(), transform.translation.copy())


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


class WheelloongM2Kinematics:
    """Pure Pinocchio kinematics for the 14-DOF dual-arm ``q_arm`` interface."""

    def __init__(self) -> None:
        loaded = load_wheelloong_m2_model()
        self.model = loaded.model
        self.data = loaded.data
        self.urdf_path = loaded.urdf_path
        self._arm_addresses = arm_joint_addresses(self.model)
        self._torso_frame_id = self._required_frame_id(TORSO_FRAME_NAME)
        self._ee_frames = (LEFT_EE_FRAME, RIGHT_EE_FRAME)
        self._ee_parent_frame_ids = {
            frame.name: self._required_frame_id(frame.parent_frame_name)
            for frame in self._ee_frames
        }

    @property
    def arm_joint_addresses(self) -> tuple[ArmJointAddress, ...]:
        """Resolved mappings for the sole public ``q_arm`` ordering."""
        return self._arm_addresses

    def _required_frame_id(self, frame_name: str) -> int:
        frame_id = int(self.model.getFrameId(frame_name, pin.FrameType.BODY))
        if frame_id >= len(self.model.frames):
            raise KeyError(f"URDF/Pinocchio model is missing required frame {frame_name}")
        return frame_id

    def _validate_q_arm(self, q_arm: np.ndarray) -> np.ndarray:
        values = np.asarray(q_arm, dtype=float)
        if values.shape != (len(self._arm_addresses),):
            raise ValueError(
                f"q_arm must have shape ({len(self._arm_addresses)},), got {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("q_arm must contain only finite values")
        return values

    def _full_configuration(self, q_arm: np.ndarray) -> np.ndarray:
        values = self._validate_q_arm(q_arm)
        q = pin.neutral(self.model).copy()
        for address in self._arm_addresses:
            q[address.pinocchio_q_index] = values[address.q_arm_index]
        return q

    def _update_placements(self, q_arm: np.ndarray) -> np.ndarray:
        q = self._full_configuration(q_arm)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return q

    def _torso_T_operational(self, frame: OperationalEEFrame) -> pin.SE3:
        world_T_torso = self.data.oMf[self._torso_frame_id]
        parent_frame_id = self._ee_parent_frame_ids[frame.name]
        world_T_parent = self.data.oMf[parent_frame_id]
        return world_T_torso.inverse() * world_T_parent * frame.parent_T_operational

    def forward_kinematics(self, q_arm: np.ndarray) -> dict[str, pin.SE3]:
        """Return ``^torso T_WL`` and ``^torso T_WR`` for a 14-DOF ``q_arm``."""
        self._update_placements(q_arm)
        return {
            "left_ee_pose": _copy_se3(self._torso_T_operational(LEFT_EE_FRAME)),
            "right_ee_pose": _copy_se3(self._torso_T_operational(RIGHT_EE_FRAME)),
        }

    def _operational_jacobian_torso(self, q: np.ndarray, frame: OperationalEEFrame) -> np.ndarray:
        """Compute a torso-expressed spatial Jacobian for one logical EE frame."""
        parent_frame_id = self._ee_parent_frame_ids[frame.name]
        parent_J_world = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            parent_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        world_T_torso = self.data.oMf[self._torso_frame_id]
        world_T_parent = self.data.oMf[parent_frame_id]
        parent_p_operational_world = world_T_parent.rotation @ frame.parent_T_operational.translation

        operational_J_world = parent_J_world.copy()
        operational_J_world[:3, :] += (
            -_skew(parent_p_operational_world) @ parent_J_world[3:, :]
        )

        torso_R_world = world_T_torso.rotation.T
        operational_J_torso = operational_J_world.copy()
        operational_J_torso[:3, :] = torso_R_world @ operational_J_world[:3, :]
        operational_J_torso[3:, :] = torso_R_world @ operational_J_world[3:, :]
        return operational_J_torso

    def compute_jacobians(self, q_arm: np.ndarray) -> dict[str, np.ndarray]:
        """Return 6x14 torso-expressed spatial Jacobians in public ``q_arm`` order.

        Rows are ``[linear; angular]`` and map public arm velocities to the
        twist of the logical operational frame relative to ``torso_link``.
        """
        q = self._update_placements(q_arm)
        pin.computeJointJacobians(self.model, self.data, q)

        arm_velocity_indices = [address.pinocchio_v_index for address in self._arm_addresses]
        left_full = self._operational_jacobian_torso(q, LEFT_EE_FRAME)
        right_full = self._operational_jacobian_torso(q, RIGHT_EE_FRAME)
        return {
            "left_J": left_full[:, arm_velocity_indices].copy(),
            "right_J": right_full[:, arm_velocity_indices].copy(),
        }
