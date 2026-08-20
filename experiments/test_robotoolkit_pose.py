#!/usr/bin/env python3
"""Log raw XRoboToolkit controller output and convert it to XRControllerPose."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from wheelloong_m2.xr import XRControllerPose, XRoboToolkitSource
from wheelloong_m2.xr.robotoolkit_source import (
    SDK_POSE_FORMAT,
    SDK_QUATERNION_ORDER,
    SDK_TIMESTAMP_UNIT,
    XR_CONTROLLER_TIMESTAMP_UNIT,
)


DEFAULT_DURATION_S = 10.0
POLL_INTERVAL_S = 0.001
STARTUP_TIMEOUT_S = 10.0
STARTUP_POLL_INTERVAL_S = 0.05


@dataclass(frozen=True)
class LoggedSample:
    """One source sample with raw SDK quaternion values retained for logging."""

    left_pose: XRControllerPose
    right_pose: XRControllerPose
    left_quaternion_xyzw: np.ndarray
    right_quaternion_xyzw: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read XRoboToolkit/PICO controller poses without robot mapping.",
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument(
        "--interactive-static-check",
        action=argparse.BooleanOptionalAction,
        default=sys.stdin.isatty(),
        help="Capture Pose A/B with Enter prompts; defaults on for an interactive terminal.",
    )
    return parser.parse_args()


def format_vector(values: np.ndarray) -> str:
    return np.array2string(np.asarray(values), precision=9, suppress_small=False)


def capture(source: XRoboToolkitSource) -> LoggedSample:
    """Read one pair and retain the exact SDK quaternion components for output."""
    left_pose, right_pose = source.sample()
    if source.last_left_quaternion_xyzw is None or source.last_right_quaternion_xyzw is None:
        raise RuntimeError("Source did not retain raw SDK quaternions")
    return LoggedSample(
        left_pose=left_pose,
        right_pose=right_pose,
        left_quaternion_xyzw=source.last_left_quaternion_xyzw.copy(),
        right_quaternion_xyzw=source.last_right_quaternion_xyzw.copy(),
    )


def wait_for_first_valid_sample(source: XRoboToolkitSource) -> LoggedSample:
    """Wait for the SDK timestamp stream to become nonzero before logging."""
    deadline = time.monotonic() + STARTUP_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            return capture(source)
        except RuntimeError as exc:
            if str(exc) != "XRoboToolkit SDK returned a non-positive timestamp":
                raise
        time.sleep(STARTUP_POLL_INTERVAL_S)
    raise TimeoutError(
        "XRoboToolkit timestamp did not become nonzero within "
        f"{STARTUP_TIMEOUT_S:.1f} seconds; check the PICO XRoboToolkit app and PC Service."
    )


def print_pose(label: str, pose: XRControllerPose, quaternion_xyzw: np.ndarray) -> None:
    print(f"  {label} position:   {format_vector(pose.position)}")
    print(f"  {label} quaternion: {format_vector(quaternion_xyzw)} ({SDK_QUATERNION_ORDER})")


def print_static_delta(label: str, pose_a: XRControllerPose, pose_b: XRControllerPose) -> None:
    position_delta = pose_b.position - pose_a.position
    rotation_delta = pin.log3(pose_b.rotation @ pose_a.rotation.T)
    print(f"  {label} delta position: {format_vector(position_delta)} m")
    print(f"  {label} delta rotation: {format_vector(rotation_delta)} rad")
    print(f"  {label} delta rotation magnitude: {np.linalg.norm(rotation_delta):.9f} rad")


def run_static_check(source: XRoboToolkitSource) -> None:
    """Let the operator capture two physical controller poses for later frame work."""
    input("\nPress Enter to save Pose A with the controllers held still... ")
    pose_a = capture(source)
    print("Pose A saved.")
    print_pose("left", pose_a.left_pose, pose_a.left_quaternion_xyzw)
    print_pose("right", pose_a.right_pose, pose_a.right_quaternion_xyzw)

    input("Change controller orientation, then press Enter to save Pose B... ")
    pose_b = capture(source)
    print("Pose B saved.")
    print_static_delta("left", pose_a.left_pose, pose_b.left_pose)
    print_static_delta("right", pose_a.right_pose, pose_b.right_pose)


def main() -> None:
    args = parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0.0:
        raise ValueError("--duration must be a finite positive number of seconds")

    source = XRoboToolkitSource()
    samples: list[LoggedSample] = []
    previous_timestamp: float | None = None
    try:
        source.connect()
        print("XRoboToolkit source connected")
        print(f"SDK distribution version: {source.sdk_version}")
        print(f"SDK pose format: {SDK_POSE_FORMAT}")
        print(f"SDK quaternion order: {SDK_QUATERNION_ORDER}")
        print(f"SDK timestamp source/unit: get_time_stamp_ns() / {SDK_TIMESTAMP_UNIT}")
        print(f"XRControllerPose timestamp unit: {XR_CONTROLLER_TIMESTAMP_UNIT}")
        print("Position values are copied without scale or coordinate conversion.")

        first_sample = wait_for_first_valid_sample(source)
        samples.append(first_sample)
        previous_timestamp = first_sample.left_pose.timestamp
        print(f"XRoboToolkit timestamp stream is nonzero; starting {args.duration:.1f}-second pose log.")

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            sample = capture(source)
            if previous_timestamp is None or sample.left_pose.timestamp != previous_timestamp:
                samples.append(sample)
                previous_timestamp = sample.left_pose.timestamp
            time.sleep(POLL_INTERVAL_S)

        if len(samples) < 2:
            raise RuntimeError("XRoboToolkit timestamp did not change during the pose log")
        timestamps = np.array([sample.left_pose.timestamp for sample in samples], dtype=float)
        dt_s = np.diff(timestamps)
        print("\n10-second pose log")
        print("  unique timestamp sample count:", len(samples))
        if dt_s.size:
            print("  dt mean: %.6f ms" % (np.mean(dt_s) * 1000.0))
            print("  dt std: %.6f ms" % (np.std(dt_s) * 1000.0))
            print("  dt max: %.6f ms" % (np.max(dt_s) * 1000.0))
            print("  observed timestamp rate: %.6f Hz" % (1.0 / np.mean(dt_s)))
        else:
            print("  dt statistics unavailable: fewer than two unique timestamps")
        print("  first sample:")
        print_pose("left", samples[0].left_pose, samples[0].left_quaternion_xyzw)
        print_pose("right", samples[0].right_pose, samples[0].right_quaternion_xyzw)
        print("  last sample:")
        print_pose("left", samples[-1].left_pose, samples[-1].left_quaternion_xyzw)
        print_pose("right", samples[-1].right_pose, samples[-1].right_quaternion_xyzw)

        if args.interactive_static_check:
            run_static_check(source)
        else:
            print("\nStatic Pose A/B check skipped (use --interactive-static-check in a terminal).")
    finally:
        source.disconnect()
        print("XRoboToolkit source disconnected")


if __name__ == "__main__":
    main()
