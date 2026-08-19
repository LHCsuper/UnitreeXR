#!/usr/bin/env python3
"""Probe XRoboToolkit stream availability without interpreting coordinates."""

from __future__ import annotations

import time
from typing import Any

import xrobotoolkit_sdk as xrt

POSE_LENGTH = 7
SAMPLE_INTERVAL_SECONDS = 0.2
STARTUP_TIMEOUT_SECONDS = 10.0
STARTUP_POLL_SECONDS = 0.05


def read_timestamp_ns() -> int:
    return int(xrt.get_time_stamp_ns())


def require_pose_length(name: str, pose: Any) -> None:
    try:
        length = len(pose)
    except TypeError as exc:
        raise RuntimeError(f"{name} pose is not array-like: {pose!r}") from exc

    if length != POSE_LENGTH:
        raise RuntimeError(f"{name} pose length is {length}, expected {POSE_LENGTH}: {pose!r}")


def format_pose(pose: Any) -> str:
    try:
        return repr(list(pose))
    except TypeError:
        return repr(pose)


def wait_for_timestamp_change(timeout_seconds: float) -> int:
    deadline = time.monotonic() + timeout_seconds
    previous = read_timestamp_ns()

    while time.monotonic() < deadline:
        current = read_timestamp_ns()
        if current != 0 and current != previous:
            return current
        previous = current
        time.sleep(STARTUP_POLL_SECONDS)

    raise TimeoutError("XR timestamp did not become non-zero and changing before timeout.")


def print_sample(previous_timestamp_ns: int) -> int:
    timestamp_ns = read_timestamp_ns()
    head = xrt.get_headset_pose()
    left = xrt.get_left_controller_pose()
    right = xrt.get_right_controller_pose()

    require_pose_length("head", head)
    require_pose_length("left", left)
    require_pose_length("right", right)

    dt_ms = (timestamp_ns - previous_timestamp_ns) / 1_000_000

    print(f"timestamp: {timestamp_ns}")
    print(f"dt_ms: {dt_ms:.3f}")
    print(f"head: {format_pose(head)}")
    print(f"left: {format_pose(left)}")
    print(f"right: {format_pose(right)}")
    print()

    return timestamp_ns


def main() -> None:
    initialized = False

    try:
        init_result = xrt.init()
        initialized = True
        print(f"xrt.init(): {init_result!r}")

        previous_timestamp_ns = wait_for_timestamp_change(STARTUP_TIMEOUT_SECONDS)
        print(f"timestamp started: {previous_timestamp_ns}")
        print()

        while True:
            previous_timestamp_ns = print_sample(previous_timestamp_ns)
            time.sleep(SAMPLE_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        if initialized:
            xrt.close()
            print("xrt.close(): done")


if __name__ == "__main__":
    main()