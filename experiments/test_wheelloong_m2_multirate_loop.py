#!/usr/bin/env python3
"""Offline 120 Hz target, 250 Hz IK, 1000 Hz MuJoCo multi-rate validation."""

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


SIMULATION_DURATION_S = 2.0
TARGET_SWAY_HZ = 0.5
LEFT_SWAY_AMPLITUDE_M = 0.03
RIGHT_SWAY_AMPLITUDE_M = 0.025


class FakeDualArmTargetProducer:
    """Deterministic SE(3) sinusoidal source used instead of an XR producer."""

    def __init__(self, left_neutral: pin.SE3, right_neutral: pin.SE3) -> None:
        self._left_neutral = pin.SE3(
            left_neutral.rotation.copy(), left_neutral.translation.copy()
        )
        self._right_neutral = pin.SE3(
            right_neutral.rotation.copy(), right_neutral.translation.copy()
        )

    def sample(self, time_s: float) -> tuple[pin.SE3, pin.SE3]:
        """Return paired torso-frame operational targets with fixed rotations."""
        phase = 2.0 * np.pi * TARGET_SWAY_HZ * time_s
        left_translation = self._left_neutral.translation + np.array(
            [LEFT_SWAY_AMPLITUDE_M * np.sin(phase), 0.0, 0.0]
        )
        right_translation = self._right_neutral.translation + np.array(
            [-RIGHT_SWAY_AMPLITUDE_M * np.sin(phase), 0.0, 0.0]
        )
        return (
            pin.SE3(self._left_neutral.rotation.copy(), left_translation),
            pin.SE3(self._right_neutral.rotation.copy(), right_translation),
        )


def latency_statistics(latencies_s: list[float]) -> dict[str, float]:
    """Return requested solve-latency summary values for non-empty samples."""
    if not latencies_s:
        raise RuntimeError("No IK latency samples were recorded")
    values = np.asarray(latencies_s, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "max": float(np.max(values)),
        "p95": float(np.percentile(values, 95.0)),
    }


def main() -> None:
    simulation = WheelloongM2MuJoCo().load()
    if not np.isclose(simulation.model.opt.timestep, SIMULATION_PERIOD_S):
        raise RuntimeError(
            "Loaded MuJoCo timestep does not match the shared SIMULATION_HZ configuration"
        )
    simulation.reset()
    controller = MujocoArmPositionController(simulation)
    kinematics = WheelloongM2Kinematics()
    solver = WheelloongM2DualArmIK()
    scheduler = MultiRateScheduler()
    target_buffer = DualArmTargetBuffer()

    neutral_poses = kinematics.forward_kinematics(np.zeros(14))
    producer = FakeDualArmTargetProducer(
        neutral_poses["left_ee_pose"],
        neutral_poses["right_ee_pose"],
    )

    q_target = simulation.arm_qpos()
    target_update_count = 0
    ik_solve_count = 0
    physics_step_count = 0
    solve_latencies_s: list[float] = []
    simulation_step_total = round(SIMULATION_DURATION_S * SIMULATION_HZ)

    for _ in range(simulation_step_total):
        tick = scheduler.next_tick()
        if tick.target_due:
            left_target, right_target = producer.sample(tick.time_s)
            target_buffer.update(tick.time_s, left_target, right_target)
            target_update_count += 1

        if tick.ik_due:
            latest_target = target_buffer.get_latest()
            if latest_target is None:
                raise RuntimeError("IK tick occurred before the fake target producer updated")
            ik_result = solver.solve(
                latest_target.left_target_pose,
                latest_target.right_target_pose,
                q_init=q_target,
                q_prev=q_target,
            )
            if not ik_result["success"]:
                raise RuntimeError(f"IPOPT did not report success at t={tick.time_s:.6f} s")
            q_target = np.asarray(ik_result["q_arm"], dtype=float)
            solve_latencies_s.append(float(ik_result["solve_time"]))
            ik_solve_count += 1

        controller.set_arm_position_target(q_target)
        simulation.step()
        physics_step_count += 1

    if target_update_count != round(SIMULATION_DURATION_S * TARGET_HZ):
        raise RuntimeError("Target update count does not match the configured simulation duration")
    if ik_solve_count != round(SIMULATION_DURATION_S * IK_HZ):
        raise RuntimeError("IK solve count does not match the configured simulation duration")
    if physics_step_count != simulation_step_total:
        raise RuntimeError("Physics step count does not match the configured simulation duration")

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
    latency = latency_statistics(solve_latencies_s)

    print("Multi-rate configuration")
    print(f"  target / IK / simulation: {TARGET_HZ} / {IK_HZ} / {SIMULATION_HZ} Hz")
    print("Runtime counts and actual simulation-time frequencies")
    print(
        "  target updates: %d (%.6f Hz)"
        % (target_update_count, target_update_count / SIMULATION_DURATION_S)
    )
    print(
        "  IK solves: %d (%.6f Hz)"
        % (ik_solve_count, ik_solve_count / SIMULATION_DURATION_S)
    )
    print(
        "  physics steps: %d (%.6f Hz)"
        % (physics_step_count, physics_step_count / SIMULATION_DURATION_S)
    )
    print("Scheduler counters:", scheduler.target_tick_count, scheduler.ik_tick_count, scheduler.physics_tick_count)
    print("MuJoCo final simulation time: %.12f s" % simulation.data.time)
    print("Latest target timestamp: %.12f s" % latest_target.timestamp)
    print("IK solve latency")
    print("  mean: %.6f ms" % (latency["mean"] * 1000.0))
    print("  p95: %.6f ms" % (latency["p95"] * 1000.0))
    print("  max: %.6f ms" % (latency["max"] * 1000.0))
    print("Final tracking")
    print("  joint tracking ||qpos-q_target||: %.12e rad" % np.linalg.norm(final_qpos - q_target))
    print("  Left EE position error: %.12e m" % np.linalg.norm(ee_errors["left"]["position_error"]))
    print("  Left EE rotation error: %.12e rad" % np.linalg.norm(ee_errors["left"]["rotation_error"]))
    print("  Right EE position error: %.12e m" % np.linalg.norm(ee_errors["right"]["position_error"]))
    print("  Right EE rotation error: %.12e rad" % np.linalg.norm(ee_errors["right"]["rotation_error"]))


if __name__ == "__main__":
    main()
