"""Fixed S0.5b logical operational EE-frame definitions for wheelloong_m2."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin


TORSO_FRAME_NAME = "torso_link"
LEFT_ARM_LINK_7_FRAME_NAME = "left_arm_link_7"
RIGHT_ARM_LINK_7_FRAME_NAME = "right_arm_link_7"


@dataclass(frozen=True)
class OperationalEEFrame:
    """A fixed logical operational EE transform under an existing URDF frame."""

    name: str
    parent_frame_name: str
    parent_T_operational: pin.SE3


def _se3(rotation: np.ndarray, translation: tuple[float, float, float]) -> pin.SE3:
    return pin.SE3(np.asarray(rotation, dtype=float), np.asarray(translation, dtype=float))


# S0.5b contract from EXP-004. These logical frames are not URDF links or
# calibrated fingertip TCPs. The right matrix retains the source URDF's
# rounded -1.5708 rad yaw residual rather than replacing it with exact Rz(pi).
LEFT_EE_FRAME = OperationalEEFrame(
    name="W_L",
    parent_frame_name=LEFT_ARM_LINK_7_FRAME_NAME,
    parent_T_operational=_se3(
        np.eye(3),
        (-0.0365, 0.21691, 0.0105),
    ),
)

RIGHT_EE_FRAME = OperationalEEFrame(
    name="W_R",
    parent_frame_name=RIGHT_ARM_LINK_7_FRAME_NAME,
    parent_T_operational=_se3(
        np.array(
            [
                [-0.9999999999932531, -0.0000036732051034, 0.0],
                [0.0000036732051034, -0.9999999999932531, 0.0],
                [0.0, 0.0, 1.0],
            ]
        ),
        (-0.0365, -0.21691, 0.0105),
    ),
)
