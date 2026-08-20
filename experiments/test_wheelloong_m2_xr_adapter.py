#!/usr/bin/env python3
"""Fake-XR adapter integration with the existing offline multirate runtime."""

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
from wheelloong_m2.simulation.runtime import (
    IK_HZ,
    SIMULATION_HZ,
    TARGET_HZ,
    DualArmTargetBuffer,
    MultiRateScheduler,
)
from wheelloong_m2.simulation.runtime.config import SIMULATION_PERIOD_S
from wheelloong_m2.xr import FakeXRSource, XRAdapter, XRControllerPose


SIMULATION_DURATION_S = 2.0


def xr_pose_from_robot_target(timestamp: float, pose: pin.SE3) -> XRControllerPose:
    """Build an artificial XR pose for the identity-adapter interface test only."""
    return XRControllerPose(
        timestamp=timestamp,
        position=pose.translation,
        rotation=pose.rotation,
    )


def main() -> None:
    simulation = WheelloongM2MuJoCo().load()
    if not np.isclose(simulation.model.opt.timestep, SIMULATION_PERIOD_S):
        raise RuntimeError("Loaded MuJoCo timestep does not match SIMULATION_HZ")
    simulation.reset()
    controller = MujocoArmPositionController(simulation)
    kinematics = WheelloongM2Kinematics()
    solver = WheelloongM2DualArmIK()
    scheduler = MultiRateScheduler()
    target_buffer = DualArmTargetBuffer()
    adapter = XRAdapter()

    neutral_targets = kinematics.forward_kinematics(np.zeros(14))
    fake_source = FakeXRSource(
        xr_pose_from_robot_target(0.0, neutral_targets["left_ee_pose"]),
        xr_pose_from_robot_target(0.0, neutral_targets["right_ee_pose"]),
    )
    if fake_source.sample_rate_hz != TARGET_HZ:
        raise RuntimeError("Fake XR source does not use the shared TARGET_HZ configuration")

    q_target = simulation.arm_qpos()
    xr_sample_pair_count = 0
    target_update_count = 0
    ik_solve_count = 0
    physics_step_count = 0
    simulation_step_total = round(SIMULATION_DURATION_S * SIMULATION_HZ)

    for _ in range(simulation_step_total):
        tick = scheduler.next_tick()
        if tick.target_due:
            left_controller, right_controller = fake_source.sample(tick.time_s)
            target_poses = adapter.convert(left_controller, right_controller)
            target_buffer.update(
                left_controller.timestamp,
                target_poses["left_target_pose"],
                target_poses["right_target_pose"],
            )
            xr_sample_pair_count += 1
            target_update_count += 1

        if tick.ik_due:
            latest_target = target_buffer.get_latest()
            if latest_target is None:
                raise RuntimeError("IK tick occurred before the fake XR source updated")
            ik_result = solver.solve(
                latest_target.left_target_pose,
                latest_target.right_target_pose,
                q_init=q_target,
                q_prev=q_target,
            )
            if not ik_result["success"]:
                raise RuntimeError(f"IPOPT did not report success at t={tick.time_s:.6f} s")
            q_target = np.asarray(ik_result["q_arm"], dtype=float)
            ik_solve_count += 1

        controller.set_arm_position_target(q_target)
        simulation.step()
        physics_step_count += 1

    expected_target_count = round(SIMULATION_DURATION_S * TARGET_HZ)
    expected_ik_count = round(SIMULATION_DURATION_S * IK_HZ)
    if xr_sample_pair_count != expected_target_count or target_update_count != expected_target_count:
        raise RuntimeError("XR sample or target update count does not match TARGET_HZ")
    if ik_solve_count != expected_ik_count:
        raise RuntimeError("IK solve count does not match IK_HZ")
    if physics_step_count != simulation_step_total:
        raise RuntimeError("Physics step count does not match SIMULATION_HZ")

    latest_target = target_buffer.get_latest()
    if latest_target is None:
        raise RuntimeError("Target buffer is unexpectedly empty after the simulation")
    final_qpos = simulation.arm_qpos()
    final_poses = kinematics.forward_kinematics(final_qpos)
    ee_errors = compute_dual_arm_error(
        final_poses["left_ee_pose"],
        final_poses["right_ee_pose"],
        latest_target.left_target_pose,
        latest_target.right_target_pose,
    )

    print("Fake XR adapter integration")
    print("  XR sample pairs: %d (%.6f Hz)" % (xr_sample_pair_count, xr_sample_pair_count / SIMULATION_DURATION_S))
    print("  XR controller poses: %d" % (2 * xr_sample_pair_count))
    print("  target updates: %d" % target_update_count)
    print("  IK solves: %d" % ik_solve_count)
    print("  physics steps: %d" % physics_step_count)
    print("  MuJoCo final simulation time: %.12f s" % simulation.data.time)
    print("  final joint tracking ||qpos-q_target||: %.12e rad" % np.linalg.norm(final_qpos - q_target))
    print("  Left EE position error: %.12e m" % np.linalg.norm(ee_errors["left"]["position_error"]))
    print("  Left EE rotation error: %.12e rad" % np.linalg.norm(ee_errors["left"]["rotation_error"]))
    print("  Right EE position error: %.12e m" % np.linalg.norm(ee_errors["right"]["position_error"]))
    print("  Right EE rotation error: %.12e rad" % np.linalg.norm(ee_errors["right"]["rotation_error"]))


if __name__ == "__main__":
    main()
