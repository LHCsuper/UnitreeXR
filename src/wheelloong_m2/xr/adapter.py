"""XR controller-pose adapter interface with a temporary identity implementation."""

from __future__ import annotations

import pinocchio as pin

from .types import XRControllerPose


class XRAdapter:
    """Convert paired XR controller poses into robot operational EE targets.

    This first implementation intentionally copies position and rotation. It
    exists only to verify the interface path with fake data. It is not a PICO
    convention, an XR-to-torso calibration, scale policy, offset policy, or
    a real-device coordinate conversion.
    """

    @staticmethod
    def _identity_target(pose: XRControllerPose) -> pin.SE3:
        return pin.SE3(pose.rotation.copy(), pose.position.copy())

    def convert(
        self,
        left_controller: XRControllerPose,
        right_controller: XRControllerPose,
    ) -> dict[str, pin.SE3]:
        """Return temporary identity-mapped ``^torso T_WL`` / ``^torso T_WR`` targets."""
        if not isinstance(left_controller, XRControllerPose) or not isinstance(
            right_controller, XRControllerPose
        ):
            raise TypeError("controller inputs must be XRControllerPose values")
        return {
            "left_target_pose": self._identity_target(left_controller),
            "right_target_pose": self._identity_target(right_controller),
        }
