"""XRoboToolkit/PICO controller source with no robot-frame conversion."""

from __future__ import annotations

import importlib
from importlib import metadata
from types import ModuleType

import numpy as np

from .types import XRControllerPose


SDK_POSE_LENGTH = 7
SDK_POSE_FORMAT = "[x, y, z, qx, qy, qz, qw]"
SDK_QUATERNION_ORDER = "xyzw"
SDK_TIMESTAMP_UNIT = "nanoseconds"
XR_CONTROLLER_TIMESTAMP_UNIT = "seconds"


def quaternion_xyzw_to_rotation(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """Convert an SDK ``[qx, qy, qz, qw]`` quaternion to its rotation matrix.

    This changes only rotation representation. It does not remap coordinate
    axes, invert the transform, scale position, apply an offset, or reference
    any robot frame.
    """
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if quaternion.shape != (4,):
        raise ValueError(f"quaternion_xyzw must have shape (4,), got {quaternion.shape}")
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion_xyzw must contain only finite values")
    norm = np.linalg.norm(quaternion)
    if norm <= 1e-12:
        raise ValueError("quaternion_xyzw must have nonzero norm")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


class XRoboToolkitSource:
    """Acquire paired PICO controller poses through ``xrobotoolkit_sdk``.

    ``sample()`` returns the existing source-compatible tuple of left/right
    ``XRControllerPose`` values. Their rotations are converted from the SDK's
    raw ``xyzw`` quaternion representation, while positions are copied with
    no robot-side conversion. Raw SDK quaternions and timestamp are retained
    for experiment logging.
    """

    def __init__(self) -> None:
        self._sdk: ModuleType | None = None
        self._connected = False
        self.last_timestamp_ns: int | None = None
        self.last_left_quaternion_xyzw: np.ndarray | None = None
        self.last_right_quaternion_xyzw: np.ndarray | None = None
        self.sdk_version = self._distribution_version()

    @staticmethod
    def _distribution_version() -> str:
        try:
            return metadata.version("xrobotoolkit_sdk")
        except metadata.PackageNotFoundError:
            return "unavailable"

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        """Import the SDK lazily and initialize its local PC Service connection."""
        if self._connected:
            return
        try:
            self._sdk = importlib.import_module("xrobotoolkit_sdk")
        except ImportError as exc:
            raise RuntimeError("xrobotoolkit_sdk is required for XRoboToolkitSource") from exc
        self._sdk.init()
        self._connected = True

    @staticmethod
    def _sdk_pose_to_xr_controller_pose(timestamp_ns: int, sdk_pose: object) -> tuple[XRControllerPose, np.ndarray]:
        """Parse one raw SDK pose as position plus an ``xyzw`` rotation matrix."""
        raw = np.asarray(sdk_pose, dtype=float)
        if raw.shape != (SDK_POSE_LENGTH,):
            raise ValueError(
                f"SDK pose must have shape ({SDK_POSE_LENGTH},) in {SDK_POSE_FORMAT}; got {raw.shape}"
            )
        if not np.all(np.isfinite(raw)):
            raise ValueError("SDK pose must contain only finite values")
        quaternion_xyzw = raw[3:7].copy()
        return (
            XRControllerPose(
                timestamp=timestamp_ns * 1e-9,
                position=raw[:3],
                rotation=quaternion_xyzw_to_rotation(quaternion_xyzw),
            ),
            quaternion_xyzw,
        )

    def sample(self) -> tuple[XRControllerPose, XRControllerPose]:
        """Read one timestamped left/right PICO controller pose pair from the SDK."""
        if not self._connected or self._sdk is None:
            raise RuntimeError("XRoboToolkitSource is not connected; call connect() first")
        timestamp_ns = int(self._sdk.get_time_stamp_ns())
        if timestamp_ns <= 0:
            raise RuntimeError("XRoboToolkit SDK returned a non-positive timestamp")
        left_pose, left_quaternion = self._sdk_pose_to_xr_controller_pose(
            timestamp_ns,
            self._sdk.get_left_controller_pose(),
        )
        right_pose, right_quaternion = self._sdk_pose_to_xr_controller_pose(
            timestamp_ns,
            self._sdk.get_right_controller_pose(),
        )
        self.last_timestamp_ns = timestamp_ns
        self.last_left_quaternion_xyzw = left_quaternion
        self.last_right_quaternion_xyzw = right_quaternion
        return left_pose, right_pose

    def disconnect(self) -> None:
        """Close the SDK connection if this source opened it."""
        if self._connected and self._sdk is not None:
            self._sdk.close()
        self._connected = False
        self._sdk = None
