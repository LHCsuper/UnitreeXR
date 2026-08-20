"""Latest-value dual-arm target storage for offline teleoperation simulation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


def _copy_pose(pose: pin.SE3) -> pin.SE3:
    return pin.SE3(pose.rotation.copy(), pose.translation.copy())


@dataclass(frozen=True)
class DualArmTarget:
    """One timestamped pair of torso-relative logical operational EE targets."""

    timestamp: float
    left_target_pose: pin.SE3
    right_target_pose: pin.SE3


class DualArmTargetBuffer:
    """Store only the newest target pair; deliberately no replay queue exists."""

    def __init__(self) -> None:
        self._latest: DualArmTarget | None = None

    def update(
        self,
        timestamp: float,
        left_pose: pin.SE3,
        right_pose: pin.SE3,
    ) -> None:
        """Replace the stored target pair with one new timestamped value."""
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if not isinstance(left_pose, pin.SE3) or not isinstance(right_pose, pin.SE3):
            raise TypeError("left_pose and right_pose must be Pinocchio SE3 values")
        self._latest = DualArmTarget(
            timestamp=float(timestamp),
            left_target_pose=_copy_pose(left_pose),
            right_target_pose=_copy_pose(right_pose),
        )

    def get_latest(self) -> DualArmTarget | None:
        """Return a copy of the newest value, or ``None`` before the first update."""
        if self._latest is None:
            return None
        return DualArmTarget(
            timestamp=self._latest.timestamp,
            left_target_pose=_copy_pose(self._latest.left_target_pose),
            right_target_pose=_copy_pose(self._latest.right_target_pose),
        )
