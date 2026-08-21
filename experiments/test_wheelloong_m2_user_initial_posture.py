#!/usr/bin/env python3
"""EXP-015 validate the user MoveJ posture in relative-XR MuJoCo simulation."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheelloong_m2.kinematics import (
    USER_REQUESTED_TELEOP_INITIAL_POSTURE,
    WheelloongM2Kinematics,
)
from wheelloong_m2.simulation.runtime import IK_HZ, SIMULATION_HZ, TARGET_HZ
from wheelloong_m2.simulation.xr_mujoco_runtime import run_xr_mujoco_simulation
from wheelloong_m2.xr import FakeXRSource, XRControllerPose


SIMULATION_DURATION_S = 1.0
INITIAL_SETTLE_DURATION_S = 3.0


class RecordingSource:
    def __init__(self, source: FakeXRSource) -> None:
        self.source = source
        self.first_pair: tuple[XRControllerPose, XRControllerPose] | None = None
        self.last_pair: tuple[XRControllerPose, XRControllerPose] | None = None

    def sample(self, time_s: float) -> tuple[XRControllerPose, XRControllerPose]:
        pair = self.source.sample(time_s)
        if self.first_pair is None:
            self.first_pair = pair
        self.last_pair = pair
        return pair


def main() -> None:
    q_initial = USER_REQUESTED_TELEOP_INITIAL_POSTURE.q_arm()
    initial_poses = WheelloongM2Kinematics().forward_kinematics(q_initial)
    source = RecordingSource(
        FakeXRSource(
            XRControllerPose(0.0, np.array([-0.24, 1.12, -0.38]), np.eye(3)),
            XRControllerPose(0.0, np.array([0.26, 1.09, -0.41]), np.eye(3)),
            left_sway_amplitude_m=0.015,
            right_sway_amplitude_m=0.012,
            sway_frequency_hz=0.125,
        )
    )
    result = run_xr_mujoco_simulation(
        source.sample,
        SIMULATION_DURATION_S,
        initial_q_arm=q_initial,
        initial_settle_duration_s=INITIAL_SETTLE_DURATION_S,
    )
    if source.first_pair is None or source.last_pair is None:
        raise RuntimeError("fake XR source was not sampled")

    np.testing.assert_allclose(result.initial_q_requested, q_initial, atol=0.0)
    if result.initialization_physics_steps != round(
        INITIAL_SETTLE_DURATION_S * SIMULATION_HZ
    ):
        raise RuntimeError("initialization step count does not match settle duration")
    if result.initial_joint_tracking_error_rad > 0.02:
        raise RuntimeError("initial posture actuator tracking error exceeded 0.02 rad")

    expected_left_delta = np.array(
        [0.0, -(source.last_pair[0].position[0] - source.first_pair[0].position[0]), 0.0]
    )
    expected_right_delta = np.array(
        [0.0, -(source.last_pair[1].position[0] - source.first_pair[1].position[0]), 0.0]
    )
    actual_left_delta = (
        result.left_target_pose.translation - initial_poses["left_ee_pose"].translation
    )
    actual_right_delta = (
        result.right_target_pose.translation - initial_poses["right_ee_pose"].translation
    )
    np.testing.assert_allclose(actual_left_delta, expected_left_delta, atol=1e-12)
    np.testing.assert_allclose(actual_right_delta, expected_right_delta, atol=1e-12)

    if result.target_updates != round(SIMULATION_DURATION_S * TARGET_HZ):
        raise RuntimeError("target count does not match TARGET_HZ")
    if result.ik_solves != round(SIMULATION_DURATION_S * IK_HZ):
        raise RuntimeError("IK count does not match IK_HZ")
    if result.physics_steps != round(SIMULATION_DURATION_S * SIMULATION_HZ):
        raise RuntimeError("physics count does not match SIMULATION_HZ")
    if max(result.left_position_error_m, result.right_position_error_m) > 0.02:
        raise RuntimeError("final simulated EE position error exceeded 20 mm")
    if max(result.left_rotation_error_rad, result.right_rotation_error_rad) > 0.04:
        raise RuntimeError("final simulated EE rotation error exceeded 0.04 rad")

    print("User MoveJ posture -> initialized relative XR -> IK -> MuJoCo")
    print("  q initial requested:", np.array2string(q_initial, precision=7))
    print(
        "  initialization steps / tracking error: %d / %.12e rad"
        % (
            result.initialization_physics_steps,
            result.initial_joint_tracking_error_rad,
        )
    )
    print(
        "  target updates / IK solves / physics steps: %d / %d / %d"
        % (
            result.target_updates,
            result.ik_solves,
            result.physics_steps,
        )
    )
    print("  left target delta: ", np.array2string(actual_left_delta, precision=12))
    print("  right target delta:", np.array2string(actual_right_delta, precision=12))
    print("  final joint tracking error: %.12e rad" % result.joint_tracking_error_rad)
    print(
        "  left EE position / rotation error: %.12e m / %.12e rad"
        % (result.left_position_error_m, result.left_rotation_error_rad)
    )
    print(
        "  right EE position / rotation error: %.12e m / %.12e rad"
        % (result.right_position_error_m, result.right_rotation_error_rad)
    )


if __name__ == "__main__":
    main()
