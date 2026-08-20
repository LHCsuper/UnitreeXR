"""Numeric dual-arm IK objective terms, deliberately independent of a solver."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class IKWeights:
    """Explicit weights for pose, rotation, nominal and smoothness terms."""

    wp: float = 50.0
    wr: float = 1.0
    wq: float = 0.02
    ws: float = 0.1


def _squared_norm(vector: np.ndarray) -> float:
    values = np.asarray(vector, dtype=float)
    return float(values @ values)


def _require_vector(name: str, value: np.ndarray, expected_shape: tuple[int, ...]) -> np.ndarray:
    values = np.asarray(value, dtype=float)
    if values.shape != expected_shape:
        raise ValueError(f"{name} must have shape {expected_shape}, got {values.shape}")
    return values


def compute_dual_arm_cost(
    errors: dict[str, dict[str, np.ndarray]],
    q: np.ndarray,
    q_nom: np.ndarray,
    q_prev: np.ndarray,
    weights: IKWeights,
) -> dict[str, float]:
    """Evaluate the numeric dual-arm objective decomposition.

    ``pose_cost`` weights translational and rotational SE(3) errors for both
    arms. ``regularization_cost`` penalizes deviation from ``q_nom`` and
    ``smooth_cost`` penalizes deviation from ``q_prev``. This only evaluates
    mathematics; it does not construct or call a solver.
    """
    q_values = _require_vector("q", q, (14,))
    q_nom_values = _require_vector("q_nom", q_nom, (14,))
    q_prev_values = _require_vector("q_prev", q_prev, (14,))

    pose_cost = 0.0
    for side in ("left", "right"):
        if side not in errors:
            raise KeyError(f"errors is missing {side!r} arm error data")
        arm_error = errors[side]
        position_error = _require_vector(
            f"errors[{side!r}]['position_error']",
            arm_error["position_error"],
            (3,),
        )
        rotation_error = _require_vector(
            f"errors[{side!r}]['rotation_error']",
            arm_error["rotation_error"],
            (3,),
        )
        pose_cost += weights.wp * _squared_norm(position_error)
        pose_cost += weights.wr * _squared_norm(rotation_error)

    regularization_cost = weights.wq * _squared_norm(q_values - q_nom_values)
    smooth_cost = weights.ws * _squared_norm(q_values - q_prev_values)
    return {
        "pose_cost": pose_cost,
        "regularization_cost": regularization_cost,
        "smooth_cost": smooth_cost,
        "total_cost": pose_cost + regularization_cost + smooth_cost,
    }
