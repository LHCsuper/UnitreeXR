#!/usr/bin/env python3
"""Numerical checks for the solver-free wheelloong_m2 SE(3) IK math layer."""

from __future__ import annotations

import sys
from pathlib import Path

import casadi as ca
import numpy as np
import pinocchio as pin


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheelloong_m2.ik.casadi_se3_error import compute_pose_error as compute_symbolic_pose_error
from wheelloong_m2.ik.cost import IKWeights, compute_dual_arm_cost
from wheelloong_m2.ik.se3_error import compute_dual_arm_error, compute_pose_error


def rotation_z(angle_rad: float) -> np.ndarray:
    """Return an active right-handed rotation about the positive Z axis."""
    cosine = np.cos(angle_rad)
    sine = np.sin(angle_rad)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def pose(rotation: np.ndarray | None = None, translation: np.ndarray | None = None) -> pin.SE3:
    """Construct a Pinocchio SE3 with explicit numeric arrays."""
    return pin.SE3(
        np.eye(3) if rotation is None else np.asarray(rotation, dtype=float),
        np.zeros(3) if translation is None else np.asarray(translation, dtype=float),
    )


def format_vector(vector: np.ndarray) -> str:
    return np.array2string(np.asarray(vector), precision=12, suppress_small=False)


def main() -> None:
    zero_pose = pose()

    print("Case 1 — current == target")
    zero_error = compute_pose_error(zero_pose, zero_pose)
    print("  position error:", format_vector(zero_error["position_error"]))
    print("  rotation error:", format_vector(zero_error["rotation_error"]))
    print("  position magnitude: %.12e m" % np.linalg.norm(zero_error["position_error"]))
    print("  rotation magnitude: %.12e rad" % np.linalg.norm(zero_error["rotation_error"]))

    print("\nCase 2 — pure target translation +X by 0.1 m")
    translation_error = compute_pose_error(zero_pose, pose(translation=np.array([0.1, 0.0, 0.0])))
    print("  position error:", format_vector(translation_error["position_error"]))
    print("  position magnitude: %.12e m" % np.linalg.norm(translation_error["position_error"]))

    angle_rad = np.deg2rad(30.0)
    current_z30 = pose(rotation=rotation_z(angle_rad))
    rotation_error = compute_pose_error(current_z30, zero_pose)
    print("\nCase 3 — current Rz(+30 deg), target identity")
    print("  rotation error:", format_vector(rotation_error["rotation_error"]))
    print("  rotation magnitude: %.12e rad" % np.linalg.norm(rotation_error["rotation_error"]))
    print("  expected magnitude: %.12e rad" % angle_rad)

    print("\nCase 4 — fixed-seed random SO(3) log/exp reconstruction")
    generator = np.random.default_rng(20260822)
    random_axis = generator.normal(size=3)
    random_axis /= np.linalg.norm(random_axis)
    random_rotation_vector = random_axis * 1.2
    random_rotation = pin.exp3(random_rotation_vector)
    recovered_rotation_vector = pin.log3(random_rotation)
    reconstructed_rotation = pin.exp3(recovered_rotation_vector)
    reconstruction_error = pin.log3(reconstructed_rotation @ random_rotation.T)
    print("  source rotation vector:", format_vector(random_rotation_vector))
    print("  recovered rotation vector:", format_vector(recovered_rotation_vector))
    print("  exp(log(R)) rotation error: %.12e rad" % np.linalg.norm(reconstruction_error))

    print("\nCase 5 — orientation-error direction")
    print("  current Rz(+30 deg), target identity")
    print("  rotation error direction:", format_vector(rotation_error["rotation_error"] / np.linalg.norm(rotation_error["rotation_error"])))
    print("  expected direction: [0. 0. 1.] (+Z)")

    print("\nCase 6 — CasADi symbolic pose error evaluation")
    p_current = ca.SX.sym("p_current", 3)
    R_current = ca.SX.sym("R_current", 3, 3)
    p_target = ca.SX.sym("p_target", 3)
    R_target = ca.SX.sym("R_target", 3, 3)
    symbolic_error = compute_symbolic_pose_error(p_current, R_current, p_target, R_target)
    symbolic_function = ca.Function(
        "se3_pose_error",
        [p_current, R_current, p_target, R_target],
        [symbolic_error["position_error"], symbolic_error["rotation_error"]],
    )
    symbolic_position, symbolic_rotation = symbolic_function(
        ca.DM.zeros(3), ca.DM(rotation_z(angle_rad)), ca.DM.zeros(3), ca.DM.eye(3)
    )
    print("  symbolic position error:", format_vector(np.asarray(symbolic_position).reshape(3)))
    print("  symbolic rotation error:", format_vector(np.asarray(symbolic_rotation).reshape(3)))
    print(
        "  numeric-symbolic rotation difference: %.12e rad"
        % np.linalg.norm(np.asarray(symbolic_rotation).reshape(3) - rotation_error["rotation_error"])
    )

    print("\nCase 7 — dual-arm numeric cost decomposition")
    errors = compute_dual_arm_error(
        zero_pose,
        current_z30,
        pose(translation=np.array([0.1, 0.0, 0.0])),
        zero_pose,
    )
    q = np.linspace(-0.2, 0.2, 14)
    q_nom = np.zeros(14)
    q_prev = np.full(14, 0.05)
    cost = compute_dual_arm_cost(errors, q, q_nom, q_prev, IKWeights())
    print("  pose cost: %.12e" % cost["pose_cost"])
    print("  regularization cost: %.12e" % cost["regularization_cost"])
    print("  smooth cost: %.12e" % cost["smooth_cost"])
    print("  total cost: %.12e" % cost["total_cost"])
    print(
        "  decomposition residual: %.12e"
        % (cost["total_cost"] - cost["pose_cost"] - cost["regularization_cost"] - cost["smooth_cost"])
    )


if __name__ == "__main__":
    main()
