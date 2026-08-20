#!/usr/bin/env python3
"""Offline validation of wheelloong_m2 CasADi Opti/IPOPT dual-arm IK."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheelloong_m2.ik.dual_arm_ik import WheelloongM2DualArmIK
from wheelloong_m2.ik.se3_error import compute_dual_arm_error
from wheelloong_m2.kinematics.dual_arm_fk import WheelloongM2Kinematics


def format_vector(vector: np.ndarray) -> str:
    return np.array2string(np.asarray(vector), precision=8, suppress_small=False)


def print_result(
    label: str,
    result: dict[str, object],
    reference_q: np.ndarray,
    targets: dict[str, object],
    kinematics: WheelloongM2Kinematics,
    limits: np.ndarray,
) -> None:
    """Print solver diagnostics and numeric FK round-trip errors."""
    solution_q = np.asarray(result["q_arm"], dtype=float)
    solved_poses = kinematics.forward_kinematics(solution_q)
    errors = compute_dual_arm_error(
        solved_poses["left_ee_pose"],
        solved_poses["right_ee_pose"],
        targets["left_ee_pose"],
        targets["right_ee_pose"],
    )
    in_limits = bool(
        np.all(solution_q >= limits[:, 0]) and np.all(solution_q <= limits[:, 1])
    )

    print(f"{label}")
    print("  IPOPT success:", result["success"])
    print("  solve time: %.6f s" % float(result["solve_time"]))
    print("  iterations:", result["iterations"])
    print("  final cost: %.12e" % float(result["cost"]))
    print("  q solution:", format_vector(solution_q))
    print("  q reference:", format_vector(reference_q))
    print("  q solution-reference norm: %.12e rad" % np.linalg.norm(solution_q - reference_q))
    print("  q within URDF limits:", in_limits)
    print("  Left EE position error: %.12e m" % np.linalg.norm(errors["left"]["position_error"]))
    print("  Left EE rotation error: %.12e rad" % np.linalg.norm(errors["left"]["rotation_error"]))
    print("  Right EE position error: %.12e m" % np.linalg.norm(errors["right"]["position_error"]))
    print("  Right EE rotation error: %.12e rad" % np.linalg.norm(errors["right"]["rotation_error"]))
    if not result["success"]:
        raise RuntimeError(f"{label} IPOPT did not report success")
    if not in_limits:
        raise RuntimeError(f"{label} solution violates an URDF joint limit")


def sample_interior_configuration(
    generator: np.random.Generator,
    limits: np.ndarray,
) -> np.ndarray:
    """Sample a legal, moderate-amplitude configuration around zero/neutral."""
    neutral = np.clip(np.zeros(limits.shape[0]), limits[:, 0], limits[:, 1])
    negative_extent = neutral - limits[:, 0]
    positive_extent = limits[:, 1] - neutral
    amplitudes = np.minimum(negative_extent, positive_extent) * 0.30
    return neutral + generator.uniform(-amplitudes, amplitudes)


def main() -> None:
    kinematics = WheelloongM2Kinematics()
    solver = WheelloongM2DualArmIK()
    limits = solver.q_limits

    zero_q = np.zeros(14)
    zero_targets = kinematics.forward_kinematics(zero_q)
    zero_result = solver.solve(
        zero_targets["left_ee_pose"],
        zero_targets["right_ee_pose"],
    )
    print_result("Case 1 — zero q FK target", zero_result, zero_q, zero_targets, kinematics, limits)

    generator = np.random.default_rng(20260823)
    random_q = sample_interior_configuration(generator, limits)
    random_targets = kinematics.forward_kinematics(random_q)
    random_result = solver.solve(
        random_targets["left_ee_pose"],
        random_targets["right_ee_pose"],
        q_init=zero_q,
        q_prev=random_q,
    )
    print()
    print_result(
        "Case 2 — fixed-seed legal random q target",
        random_result,
        random_q,
        random_targets,
        kinematics,
        limits,
    )

    dual_motion_q = random_q.copy()
    dual_motion_q[:7] = np.clip(
        dual_motion_q[:7] + np.array([0.08, -0.06, 0.05, -0.04, 0.03, -0.02, 0.01]),
        limits[:7, 0],
        limits[:7, 1],
    )
    dual_motion_q[7:] = np.clip(
        dual_motion_q[7:] + np.array([-0.07, 0.05, -0.04, 0.06, -0.03, 0.02, -0.01]),
        limits[7:, 0],
        limits[7:, 1],
    )
    dual_motion_targets = kinematics.forward_kinematics(dual_motion_q)
    dual_motion_result = solver.solve(
        dual_motion_targets["left_ee_pose"],
        dual_motion_targets["right_ee_pose"],
        q_init=np.asarray(random_result["q_arm"], dtype=float),
        q_prev=np.asarray(random_result["q_arm"], dtype=float),
    )
    print()
    print_result(
        "Case 3 — simultaneous left/right target change with warm start",
        dual_motion_result,
        dual_motion_q,
        dual_motion_targets,
        kinematics,
        limits,
    )
    solution_delta = np.asarray(dual_motion_result["q_arm"]) - np.asarray(random_result["q_arm"])
    print("  left-arm solution delta norm: %.12e rad" % np.linalg.norm(solution_delta[:7]))
    print("  right-arm solution delta norm: %.12e rad" % np.linalg.norm(solution_delta[7:]))


if __name__ == "__main__":
    main()
