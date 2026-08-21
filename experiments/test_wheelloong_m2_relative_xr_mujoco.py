#!/usr/bin/env python3
"""EXP-014 deterministic raw-XR relative adapter and MuJoCo integration."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheelloong_m2.kinematics import WheelloongM2Kinematics
from wheelloong_m2.simulation.xr_mujoco_runtime import run_xr_mujoco_simulation
from wheelloong_m2.simulation.runtime import IK_HZ, SIMULATION_HZ, TARGET_HZ
from wheelloong_m2.xr import FakeXRSource, XRControllerPose


SIMULATION_DURATION_S = 2.0


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
    # Deliberately arbitrary absolute PICO tracking-origin values. They are not
    # numerically copied from a robot target and must cancel during initialization.
    source = RecordingSource(
        FakeXRSource(
            XRControllerPose(
                timestamp=0.0,
                position=np.array([-0.24, 1.12, -0.38]),
                rotation=np.eye(3),
            ),
            XRControllerPose(
                timestamp=0.0,
                position=np.array([0.26, 1.09, -0.41]),
                rotation=np.eye(3),
            ),
            left_sway_amplitude_m=0.03,
            right_sway_amplitude_m=0.025,
            sway_frequency_hz=0.125,
        )
    )
    neutral = WheelloongM2Kinematics().forward_kinematics(np.zeros(14))
    result = run_xr_mujoco_simulation(source.sample, SIMULATION_DURATION_S)
    if source.first_pair is None or source.last_pair is None:
        raise RuntimeError("fake XR source was not sampled")

    expected_left_delta = np.array(
        [0.0, -(source.last_pair[0].position[0] - source.first_pair[0].position[0]), 0.0]
    )
    expected_right_delta = np.array(
        [0.0, -(source.last_pair[1].position[0] - source.first_pair[1].position[0]), 0.0]
    )
    actual_left_delta = result.left_target_pose.translation - neutral["left_ee_pose"].translation
    actual_right_delta = result.right_target_pose.translation - neutral["right_ee_pose"].translation
    np.testing.assert_allclose(actual_left_delta, expected_left_delta, atol=1e-12)
    np.testing.assert_allclose(actual_right_delta, expected_right_delta, atol=1e-12)

    expected_target_updates = round(SIMULATION_DURATION_S * TARGET_HZ)
    expected_ik_solves = round(SIMULATION_DURATION_S * IK_HZ)
    expected_physics_steps = round(SIMULATION_DURATION_S * SIMULATION_HZ)
    if result.target_updates != expected_target_updates:
        raise RuntimeError("target update count does not match TARGET_HZ")
    if result.ik_solves != expected_ik_solves:
        raise RuntimeError("IK solve count does not match IK_HZ")
    if result.physics_steps != expected_physics_steps:
        raise RuntimeError("physics step count does not match SIMULATION_HZ")
    if max(result.left_position_error_m, result.right_position_error_m) > 0.02:
        raise RuntimeError("final simulated EE position error exceeded 20 mm")
    if max(result.left_rotation_error_rad, result.right_rotation_error_rad) > 0.03:
        raise RuntimeError("final simulated EE rotation error exceeded 0.03 rad")

    print("Initialized relative XR -> IK -> MuJoCo integration")
    print(
        "  target updates: %d (%.6f Hz)"
        % (result.target_updates, result.target_updates / result.duration_s)
    )
    print("  IK solves: %d (%.6f Hz)" % (result.ik_solves, result.ik_solves / result.duration_s))
    print(
        "  physics steps: %d (%.6f Hz)"
        % (result.physics_steps, result.physics_steps / result.duration_s)
    )
    print("  wall time: %.6f s" % result.wall_time_s)
    print("  left target delta: ", np.array2string(actual_left_delta, precision=12))
    print("  right target delta:", np.array2string(actual_right_delta, precision=12))
    print("  joint tracking error: %.12e rad" % result.joint_tracking_error_rad)
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
