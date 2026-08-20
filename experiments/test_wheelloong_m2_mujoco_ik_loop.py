#!/usr/bin/env python3
"""Offline wheelloong_m2 IK-to-MuJoCo position-actuator closed-loop checks."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pinocchio as pin


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheelloong_m2.ik import WheelloongM2DualArmIK
from wheelloong_m2.ik.se3_error import compute_dual_arm_error
from wheelloong_m2.kinematics import WheelloongM2Kinematics
from wheelloong_m2.simulation import MujocoArmPositionController, WheelloongM2MuJoCo


SIMULATION_DURATION_S = 2.0


def format_vector(vector: np.ndarray) -> str:
    return np.array2string(np.asarray(vector), precision=8, suppress_small=False)


def translated_pose(pose: pin.SE3, translation_delta: np.ndarray) -> pin.SE3:
    """Return a torso-frame target with unchanged orientation and explicit translation."""
    return pin.SE3(
        pose.rotation.copy(),
        pose.translation + np.asarray(translation_delta, dtype=float),
    )


def run_case(
    label: str,
    left_target: pin.SE3,
    right_target: pin.SE3,
    solver: WheelloongM2DualArmIK,
    kinematics: WheelloongM2Kinematics,
    simulation: WheelloongM2MuJoCo,
    controller: MujocoArmPositionController,
) -> tuple[np.ndarray, np.ndarray]:
    """Solve, actuate, step, and evaluate one target pair without qpos writes."""
    simulation.reset()
    initial_qpos = simulation.arm_qpos()
    ik_result = solver.solve(
        left_target,
        right_target,
        q_init=initial_qpos,
        q_prev=initial_qpos,
    )
    if not ik_result["success"]:
        raise RuntimeError(f"{label}: IPOPT did not report success")

    target_q = np.asarray(ik_result["q_arm"], dtype=float)
    controller.set_arm_position_target(target_q)
    controller.run_for_seconds(SIMULATION_DURATION_S)
    final_qpos = simulation.arm_qpos()
    tracked_poses = kinematics.forward_kinematics(final_qpos)
    errors = compute_dual_arm_error(
        tracked_poses["left_ee_pose"],
        tracked_poses["right_ee_pose"],
        left_target,
        right_target,
    )

    print(f"{label}")
    print("  IK success:", ik_result["success"])
    print("  IK q_solution:", format_vector(target_q))
    print("  IK solve time: %.6f s" % float(ik_result["solve_time"]))
    print("  IK iterations:", ik_result["iterations"])
    print("  MuJoCo simulation time: %.6f s" % float(simulation.data.time))
    print("  MuJoCo qpos:", format_vector(final_qpos))
    print("  MuJoCo joint tracking error ||qpos-q_target||: %.12e rad" % np.linalg.norm(final_qpos - target_q))
    print("  Left EE position error: %.12e m" % np.linalg.norm(errors["left"]["position_error"]))
    print("  Left EE rotation error: %.12e rad" % np.linalg.norm(errors["left"]["rotation_error"]))
    print("  Right EE position error: %.12e m" % np.linalg.norm(errors["right"]["position_error"]))
    print("  Right EE rotation error: %.12e rad" % np.linalg.norm(errors["right"]["rotation_error"]))
    return final_qpos, initial_qpos


def main() -> None:
    simulation = WheelloongM2MuJoCo().load()
    controller = MujocoArmPositionController(simulation)
    kinematics = WheelloongM2Kinematics()
    solver = WheelloongM2DualArmIK()

    neutral_q = np.zeros(14)
    neutral_poses = kinematics.forward_kinematics(neutral_q)

    run_case(
        "Case 1 — neutral pose",
        neutral_poses["left_ee_pose"],
        neutral_poses["right_ee_pose"],
        solver,
        kinematics,
        simulation,
        controller,
    )

    left_qpos, neutral_qpos = run_case(
        "\nCase 2 — left arm target translation +X 0.05 m; right neutral",
        translated_pose(neutral_poses["left_ee_pose"], np.array([0.05, 0.0, 0.0])),
        neutral_poses["right_ee_pose"],
        solver,
        kinematics,
        simulation,
        controller,
    )
    left_motion = np.linalg.norm(left_qpos[:7] - neutral_qpos[:7])
    right_motion = np.linalg.norm(left_qpos[7:] - neutral_qpos[7:])
    print("  left-arm simulated motion norm: %.12e rad" % left_motion)
    print("  right-arm simulated motion norm: %.12e rad" % right_motion)
    if not left_motion > right_motion:
        raise RuntimeError("Case 2 did not produce greater left-arm than right-arm motion")

    right_qpos, neutral_qpos = run_case(
        "\nCase 3 — right arm target translation -X 0.05 m; left neutral",
        neutral_poses["left_ee_pose"],
        translated_pose(neutral_poses["right_ee_pose"], np.array([-0.05, 0.0, 0.0])),
        solver,
        kinematics,
        simulation,
        controller,
    )
    left_motion = np.linalg.norm(right_qpos[:7] - neutral_qpos[:7])
    right_motion = np.linalg.norm(right_qpos[7:] - neutral_qpos[7:])
    print("  left-arm simulated motion norm: %.12e rad" % left_motion)
    print("  right-arm simulated motion norm: %.12e rad" % right_motion)
    if not right_motion > left_motion:
        raise RuntimeError("Case 3 did not produce greater right-arm than left-arm motion")


if __name__ == "__main__":
    main()
