"""Numeric SE(3) pose-error definitions for the wheelloong_m2 IK math layer."""

from __future__ import annotations

import numpy as np
import pinocchio as pin


def compute_pose_error(
    current_pose: pin.SE3,
    target_pose: pin.SE3,
) -> dict[str, np.ndarray]:
    """Return pose errors expressed in the common pose coordinate frame.

    ``position_error`` is ``p_current - p_target``. ``rotation_error`` is
    ``log3(R_current * R_target.T)``: an axis-angle vector with no Euler-angle
    conversion. For the S1 kinematics interfaces, both poses are
    torso-relative operational EE poses.
    """
    position_error = current_pose.translation - target_pose.translation
    rotation_error = pin.log3(current_pose.rotation @ target_pose.rotation.T)
    return {
        "position_error": np.asarray(position_error, dtype=float).copy(),
        "rotation_error": np.asarray(rotation_error, dtype=float).copy(),
    }


def compute_dual_arm_error(
    current_left: pin.SE3,
    current_right: pin.SE3,
    target_left: pin.SE3,
    target_right: pin.SE3,
) -> dict[str, dict[str, np.ndarray]]:
    """Return the numeric SE(3) error dictionaries for both operational EEs."""
    return {
        "left": compute_pose_error(current_left, target_left),
        "right": compute_pose_error(current_right, target_right),
    }
