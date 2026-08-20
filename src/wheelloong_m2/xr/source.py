"""Deterministic fake XR controller source for offline adapter testing."""

from __future__ import annotations

import numpy as np

from wheelloong_m2.simulation.runtime.config import TARGET_HZ

from .types import XRControllerPose


class FakeXRSource:
    """Produce paired ``^xr T_controller`` samples at the configured target rate.

    The source is intentionally synthetic. Callers provide reference XR poses
    so an experiment can construct reachable values under the temporary
    identity adapter convention; this is not coordinate calibration.
    """

    sample_rate_hz = TARGET_HZ

    def __init__(
        self,
        left_reference_pose: XRControllerPose,
        right_reference_pose: XRControllerPose,
        left_sway_amplitude_m: float = 0.03,
        right_sway_amplitude_m: float = 0.025,
        sway_frequency_hz: float = 0.5,
    ) -> None:
        if not isinstance(left_reference_pose, XRControllerPose) or not isinstance(
            right_reference_pose, XRControllerPose
        ):
            raise TypeError("reference poses must be XRControllerPose values")
        parameters = (
            left_sway_amplitude_m,
            right_sway_amplitude_m,
            sway_frequency_hz,
        )
        if not np.all(np.isfinite(parameters)) or sway_frequency_hz < 0.0:
            raise ValueError("fake source amplitudes and frequency must be finite; frequency >= 0")
        self._left_reference_pose = left_reference_pose
        self._right_reference_pose = right_reference_pose
        self._left_sway_amplitude_m = float(left_sway_amplitude_m)
        self._right_sway_amplitude_m = float(right_sway_amplitude_m)
        self._sway_frequency_hz = float(sway_frequency_hz)

    def sample(self, time_s: float) -> tuple[XRControllerPose, XRControllerPose]:
        """Return one deterministic left/right controller sample pair at ``time_s``."""
        if not np.isfinite(time_s):
            raise ValueError("time_s must be finite")
        timestamp = float(time_s)
        phase = 2.0 * np.pi * self._sway_frequency_hz * timestamp
        left_position = self._left_reference_pose.position + np.array(
            [self._left_sway_amplitude_m * np.sin(phase), 0.0, 0.0]
        )
        right_position = self._right_reference_pose.position + np.array(
            [-self._right_sway_amplitude_m * np.sin(phase), 0.0, 0.0]
        )
        return (
            XRControllerPose(
                timestamp=timestamp,
                position=left_position,
                rotation=self._left_reference_pose.rotation,
            ),
            XRControllerPose(
                timestamp=timestamp,
                position=right_position,
                rotation=self._right_reference_pose.rotation,
            ),
        )
