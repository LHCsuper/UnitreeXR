"""Explicit XR-controller to robot-operational-frame adapters."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
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


# Source Evidence: Unitree TeleVuer commit 766de45e74373ae0ea66321d942ce538385655a5,
# ``src/televuer/tv_wrapper.py::T_ROBOT_OPENXR``.  It changes coordinate
# components from OpenXR (+X right, +Y up, +Z back) to the robot convention
# (+X front, +Y left, +Z up).  It is a proper rotation, not a reflection.
UNITREE_ROBOT_FROM_OPENXR_BASIS = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def _copy_se3(pose: pin.SE3) -> pin.SE3:
    return pin.SE3(pose.rotation.copy(), pose.translation.copy())


def _validate_rotation(name: str, rotation: np.ndarray) -> None:
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"{name} must be orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6):
        raise ValueError(f"{name} must have determinant +1")


@dataclass(frozen=True)
class RelativeXRMapping:
    """Named parameters for initialized spatial relative-motion mapping.

    ``robot_from_xr_basis`` changes vector components from the established
    PICO/OpenXR tracking basis into torso axes. ``translation_scale`` is an
    explicit dimensionless gain; no hidden per-axis scaling is performed.
    """

    robot_from_xr_basis: np.ndarray = field(
        default_factory=lambda: UNITREE_ROBOT_FROM_OPENXR_BASIS.copy()
    )
    translation_scale: float = 1.0

    def __post_init__(self) -> None:
        basis = np.asarray(self.robot_from_xr_basis, dtype=float)
        scale = float(self.translation_scale)
        if basis.shape != (3, 3) or not np.all(np.isfinite(basis)):
            raise ValueError("robot_from_xr_basis must be a finite 3x3 matrix")
        _validate_rotation("robot_from_xr_basis", basis)
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("translation_scale must be finite and greater than zero")
        object.__setattr__(self, "robot_from_xr_basis", basis.copy())
        object.__setattr__(self, "translation_scale", scale)


@dataclass(frozen=True)
class _ArmAnchor:
    controller: XRControllerPose
    robot_target: pin.SE3


class InitializedRelativeXRAdapter:
    """Map initialized PICO controller motion to torso-relative EE targets.

    For each arm, initialization records ``^D T_C(0)`` and the corresponding
    robot operational target ``^torso T_W(0)``.  Current motion is expressed
    spatially in the tracking basis:

    ``delta_p_D = p_D_C(t) - p_D_C(0)``
    ``delta_R_D = R_D_C(t) R_D_C(0)^T``

    The named basis rotation ``S`` then produces:

    ``p_target = p_W(0) + scale S delta_p_D``
    ``R_target = S delta_R_D S^T R_W(0)``

    Using spatial relative rotation cancels any fixed controller-local
    extrinsic present in both the initial and current samples.  This adapter
    therefore does not claim that XRoboToolkit/PICO controller axes equal the
    Unitree/WebXR controller-local convention.
    """

    def __init__(self, mapping: RelativeXRMapping | None = None) -> None:
        self.mapping = RelativeXRMapping() if mapping is None else mapping
        if not isinstance(self.mapping, RelativeXRMapping):
            raise TypeError("mapping must be a RelativeXRMapping or None")
        self._left_anchor: _ArmAnchor | None = None
        self._right_anchor: _ArmAnchor | None = None

    @property
    def initialized(self) -> bool:
        return self._left_anchor is not None and self._right_anchor is not None

    def reset(self) -> None:
        """Discard both controller/robot anchors; a new initialize is required."""
        self._left_anchor = None
        self._right_anchor = None

    @staticmethod
    def _validate_inputs(
        left_controller: XRControllerPose,
        right_controller: XRControllerPose,
        left_robot_target: pin.SE3 | None = None,
        right_robot_target: pin.SE3 | None = None,
    ) -> None:
        if not isinstance(left_controller, XRControllerPose) or not isinstance(
            right_controller, XRControllerPose
        ):
            raise TypeError("controller inputs must be XRControllerPose values")
        _validate_rotation("left_controller.rotation", left_controller.rotation)
        _validate_rotation("right_controller.rotation", right_controller.rotation)
        if left_robot_target is not None or right_robot_target is not None:
            if not isinstance(left_robot_target, pin.SE3) or not isinstance(
                right_robot_target, pin.SE3
            ):
                raise TypeError(
                    "left_robot_target and right_robot_target must be Pinocchio SE3 values"
                )
            _validate_rotation("left_robot_target.rotation", left_robot_target.rotation)
            _validate_rotation("right_robot_target.rotation", right_robot_target.rotation)

    def initialize(
        self,
        left_controller: XRControllerPose,
        right_controller: XRControllerPose,
        left_robot_target: pin.SE3,
        right_robot_target: pin.SE3,
    ) -> None:
        """Atomically capture the paired XR and robot operational anchors."""
        self._validate_inputs(
            left_controller,
            right_controller,
            left_robot_target,
            right_robot_target,
        )
        self._left_anchor = _ArmAnchor(left_controller, _copy_se3(left_robot_target))
        self._right_anchor = _ArmAnchor(right_controller, _copy_se3(right_robot_target))

    def _map_arm(self, current: XRControllerPose, anchor: _ArmAnchor) -> pin.SE3:
        basis = self.mapping.robot_from_xr_basis
        delta_position_xr = current.position - anchor.controller.position
        delta_rotation_xr = current.rotation @ anchor.controller.rotation.T
        target_position = (
            anchor.robot_target.translation
            + self.mapping.translation_scale * (basis @ delta_position_xr)
        )
        target_rotation = (
            basis @ delta_rotation_xr @ basis.T @ anchor.robot_target.rotation
        )
        return pin.SE3(target_rotation, target_position)

    def convert(
        self,
        left_controller: XRControllerPose,
        right_controller: XRControllerPose,
    ) -> dict[str, pin.SE3]:
        """Return initialized ``^torso T_WL`` and ``^torso T_WR`` targets."""
        self._validate_inputs(left_controller, right_controller)
        if not self.initialized or self._left_anchor is None or self._right_anchor is None:
            raise RuntimeError("adapter is not initialized; call initialize() first")
        return {
            "left_target_pose": self._map_arm(left_controller, self._left_anchor),
            "right_target_pose": self._map_arm(right_controller, self._right_anchor),
        }
