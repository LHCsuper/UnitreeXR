"""XR-side pose contracts, intentionally independent of any device SDK."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class XRControllerPose:
    """One controller pose represented as ``^xr T_controller``.

    ``position`` has shape ``(3,)`` and ``rotation`` has shape ``(3, 3)``.
    This is an XR-side data contract only; it does not claim an XR-to-robot
    coordinate relationship.
    """

    timestamp: float
    position: np.ndarray
    rotation: np.ndarray

    def __post_init__(self) -> None:
        timestamp = float(self.timestamp)
        position = np.asarray(self.position, dtype=float)
        rotation = np.asarray(self.rotation, dtype=float)
        if not np.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if position.shape != (3,):
            raise ValueError(f"position must have shape (3,), got {position.shape}")
        if rotation.shape != (3, 3):
            raise ValueError(f"rotation must have shape (3, 3), got {rotation.shape}")
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError("position and rotation must contain only finite values")
        object.__setattr__(self, "timestamp", timestamp)
        object.__setattr__(self, "position", position.copy())
        object.__setattr__(self, "rotation", rotation.copy())
