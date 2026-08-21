#!/usr/bin/env python3
"""Run PICO/fake initialized-relative dual-arm teleoperation in MuJoCo only."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheelloong_m2.kinematics import USER_REQUESTED_TELEOP_INITIAL_POSTURE
from wheelloong_m2.simulation.xr_mujoco_runtime import run_xr_mujoco_simulation
from wheelloong_m2.xr import (
    FakeXRSource,
    InitializedRelativeXRAdapter,
    RelativeXRMapping,
    XRControllerPose,
    XRoboToolkitSource,
)


class PassiveViewerCallback:
    """Lazily attach MuJoCo's passive viewer to the runtime-owned model/data."""

    def __init__(self) -> None:
        self._viewer = None

    def __call__(self, simulation) -> None:
        if self._viewer is None:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(
                simulation.model,
                simulation.data,
            )
        if not self._viewer.is_running():
            raise KeyboardInterrupt("MuJoCo viewer was closed")
        self._viewer.sync()

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None


def fake_source() -> FakeXRSource:
    """Return arbitrary absolute XR anchors plus a slow symmetric test motion."""
    return FakeXRSource(
        XRControllerPose(0.0, np.array([-0.24, 1.12, -0.38]), np.eye(3)),
        XRControllerPose(0.0, np.array([0.26, 1.09, -0.41]), np.eye(3)),
        left_sway_amplitude_m=0.03,
        right_sway_amplitude_m=0.025,
        sway_frequency_hz=0.125,
    )


def wait_for_first_robotoolkit_pair(
    source: XRoboToolkitSource,
    timeout_s: float,
) -> tuple[XRControllerPose, XRControllerPose]:
    """Wait for a positive SDK timestamp without manufacturing a fake sample."""
    deadline = time.monotonic() + timeout_s
    last_error: RuntimeError | None = None
    while time.monotonic() < deadline:
        try:
            return source.sample()
        except RuntimeError as exc:
            last_error = exc
            time.sleep(0.05)
    raise TimeoutError(
        f"XRoboToolkit produced no valid controller sample within {timeout_s:.1f} s"
    ) from last_error


def print_report(result) -> None:
    print("Simulation-only XR teleoperation report")
    print("  simulated duration: %.6f s" % result.duration_s)
    print("  wall time: %.6f s" % result.wall_time_s)
    print(
        "  initialization physics steps / tracking error: %d / %.12e rad"
        % (
            result.initialization_physics_steps,
            result.initial_joint_tracking_error_rad,
        )
    )
    print(
        "  target updates / IK solves / physics steps: %d / %d / %d"
        % (result.target_updates, result.ik_solves, result.physics_steps)
    )
    print("  joint tracking error: %.12e rad" % result.joint_tracking_error_rad)
    print(
        "  left EE position / rotation error: %.12e m / %.12e rad"
        % (result.left_position_error_m, result.left_rotation_error_rad)
    )
    print(
        "  right EE position / rotation error: %.12e m / %.12e rad"
        % (result.right_position_error_m, result.right_rotation_error_rad)
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run initialized PICO/OpenXR relative motion through Wheelloong M2 "
            "IK and MuJoCo; never controls a real robot."
        )
    )
    parser.add_argument("--source", choices=("fake", "robotoolkit"), default="fake")
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="simulation duration in seconds",
    )
    parser.add_argument(
        "--translation-scale",
        type=float,
        default=1.0,
        help="explicit XR translation gain applied after initialization",
    )
    parser.add_argument(
        "--initial-posture",
        choices=("user", "neutral"),
        default="user",
        help="start from the user-specified MoveJ posture or model neutral",
    )
    parser.add_argument(
        "--initial-settle-duration",
        type=float,
        default=3.0,
        help="MuJoCo actuator settling time before XR anchoring",
    )
    parser.add_argument("--startup-timeout", type=float, default=10.0)
    parser.add_argument(
        "--real-time",
        action="store_true",
        help="pace fake mode against wall time",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="open a passive MuJoCo viewer",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    adapter = InitializedRelativeXRAdapter(
        RelativeXRMapping(translation_scale=args.translation_scale)
    )
    viewer = PassiveViewerCallback() if args.visualize else None
    source: XRoboToolkitSource | None = None
    initial_q_arm = (
        USER_REQUESTED_TELEOP_INITIAL_POSTURE.q_arm()
        if args.initial_posture == "user"
        else None
    )

    try:
        if args.source == "fake":
            sampler = fake_source().sample
            real_time = args.real_time or args.visualize
        else:
            source = XRoboToolkitSource()
            source.connect()
            # Readiness probe only. It is deliberately discarded so the
            # post-settling XR anchor uses a fresh controller sample.
            wait_for_first_robotoolkit_pair(source, args.startup_timeout)

            def sampler(_: float) -> tuple[XRControllerPose, XRControllerPose]:
                return source.sample()

            # Live device sampling must be wall-clock paced; it remains MuJoCo-only.
            real_time = True

        result = run_xr_mujoco_simulation(
            sampler,
            args.duration,
            adapter=adapter,
            initial_q_arm=initial_q_arm,
            initial_settle_duration_s=args.initial_settle_duration,
            real_time=real_time,
            step_callback=viewer,
        )
        print_report(result)
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    except TimeoutError as exc:
        print(f"Simulation startup failed: {exc}", file=sys.stderr)
        raise SystemExit(2) from None
    finally:
        if viewer is not None:
            viewer.close()
        if source is not None:
            source.disconnect()


if __name__ == "__main__":
    main()
