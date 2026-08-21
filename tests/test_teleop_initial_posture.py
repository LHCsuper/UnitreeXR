"""Checks for the user-specified Wheelloong M2 teleoperation posture."""

from __future__ import annotations

import numpy as np

from wheelloong_m2.kinematics import USER_REQUESTED_TELEOP_INITIAL_POSTURE
from wheelloong_m2.kinematics.robot_model import (
    ARM_JOINT_NAMES,
    arm_joint_limits,
    load_wheelloong_m2_model,
)


def test_user_posture_preserves_left_then_right_movej_order() -> None:
    posture = USER_REQUESTED_TELEOP_INITIAL_POSTURE
    np.testing.assert_allclose(
        posture.q_arm(),
        np.array(
            [
                -1.5707963,
                1.2217305,
                1.5707963,
                -1.5707963,
                1.5707963,
                0.0,
                0.0,
                1.5707963,
                1.2217305,
                -1.5707963,
                -1.5707963,
                -1.5707963,
                0.0,
                0.0,
            ]
        ),
        atol=0.0,
    )
    assert len(ARM_JOINT_NAMES) == 14
    assert posture.left_move_j_vel == 0.5
    assert posture.left_move_j_acc == 0.5
    assert posture.right_move_j_vel == 0.8
    assert posture.right_move_j_acc == 0.8


def test_user_posture_is_inside_all_named_urdf_joint_limits() -> None:
    q_arm = USER_REQUESTED_TELEOP_INITIAL_POSTURE.q_arm()
    limits = arm_joint_limits(load_wheelloong_m2_model().model)
    assert np.all(q_arm >= limits[:, 0])
    assert np.all(q_arm <= limits[:, 1])
