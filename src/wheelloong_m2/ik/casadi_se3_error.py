"""CasADi-differentiable SE(3) pose errors without numeric Pinocchio log3."""

from __future__ import annotations

import casadi as ca


SMALL_ANGLE_RAD = 1e-6


def so3_log(rotation: ca.SX) -> ca.SX:
    """Return a CasADi SO(3) logarithm vector from a 3x3 rotation expression.

    The principal angle uses ``acos((trace(R)-1)/2)`` with clipping. The
    nonzero-angle branch is the usual ``theta/(2 sin(theta)) * vee(R-R.T)``.
    Its theta-to-zero limit is ``0.5 * vee(R-R.T)``, which avoids division by
    zero and remains the first-order differentiable rotation-vector form.
    """
    if rotation.shape != (3, 3):
        raise ValueError(f"rotation must have shape (3, 3), got {rotation.shape}")

    cosine = (ca.trace(rotation) - 1.0) / 2.0
    cosine = ca.fmin(1.0, ca.fmax(-1.0, cosine))
    theta = ca.acos(cosine)
    skew_vee = ca.vertcat(
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    )
    nonzero_scale = theta / (2.0 * ca.sin(theta))
    scale = ca.if_else(theta < SMALL_ANGLE_RAD, 0.5, nonzero_scale)
    return scale * skew_vee


def compute_pose_error(
    p_current: ca.SX,
    R_current: ca.SX,
    p_target: ca.SX,
    R_target: ca.SX,
) -> dict[str, ca.SX]:
    """Return symbolic ``p_current-p_target`` and ``Log(R_current R_target.T)``."""
    if p_current.shape != (3, 1) or p_target.shape != (3, 1):
        raise ValueError(
            "p_current and p_target must each have CasADi shape (3, 1); "
            f"got {p_current.shape} and {p_target.shape}"
        )
    return {
        "position_error": p_current - p_target,
        "rotation_error": so3_log(R_current @ R_target.T),
    }
