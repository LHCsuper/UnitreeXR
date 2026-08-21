"""Deterministic checks for initialized PICO/OpenXR relative-motion mapping."""

from __future__ import annotations

import numpy as np
import pinocchio as pin
import pytest

from wheelloong_m2.xr import (
    UNITREE_ROBOT_FROM_OPENXR_BASIS,
    InitializedRelativeXRAdapter,
    RelativeXRMapping,
    XRControllerPose,
)


def controller_pose(
    position: np.ndarray,
    rotation: np.ndarray | None = None,
    timestamp: float = 0.0,
) -> XRControllerPose:
    return XRControllerPose(
        timestamp=timestamp,
        position=np.asarray(position, dtype=float),
        rotation=np.eye(3) if rotation is None else np.asarray(rotation, dtype=float),
    )


def robot_pose(position: np.ndarray, rotation: np.ndarray | None = None) -> pin.SE3:
    return pin.SE3(
        np.eye(3) if rotation is None else np.asarray(rotation, dtype=float),
        np.asarray(position, dtype=float),
    )


def initialized_adapter(
    left: XRControllerPose,
    right: XRControllerPose,
    *,
    scale: float = 1.0,
) -> tuple[InitializedRelativeXRAdapter, pin.SE3, pin.SE3]:
    left_target = robot_pose(np.array([0.35, 0.25, 0.20]), pin.exp3(np.array([0.1, -0.2, 0.3])))
    right_target = robot_pose(np.array([0.35, -0.25, 0.20]), pin.exp3(np.array([-0.2, 0.1, -0.1])))
    adapter = InitializedRelativeXRAdapter(RelativeXRMapping(translation_scale=scale))
    adapter.initialize(left, right, left_target, right_target)
    return adapter, left_target, right_target


def test_unitree_openxr_basis_is_proper_rotation_with_documented_axis_map() -> None:
    basis = UNITREE_ROBOT_FROM_OPENXR_BASIS
    np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-12)
    assert np.linalg.det(basis) == pytest.approx(1.0)
    np.testing.assert_allclose(basis @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0])
    np.testing.assert_allclose(basis @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])
    np.testing.assert_allclose(basis @ np.array([0.0, 0.0, 1.0]), [-1.0, 0.0, 0.0])


def test_adapter_requires_explicit_initialization() -> None:
    pose = controller_pose(np.zeros(3))
    with pytest.raises(RuntimeError, match="not initialized"):
        InitializedRelativeXRAdapter().convert(pose, pose)


def test_initial_controller_pair_maps_exactly_to_robot_anchors() -> None:
    left = controller_pose(np.array([-0.25, 1.1, -0.4]), pin.exp3(np.array([0.2, 0.1, -0.1])))
    right = controller_pose(np.array([0.25, 1.1, -0.4]), pin.exp3(np.array([-0.1, 0.2, 0.1])))
    adapter, left_target, right_target = initialized_adapter(left, right)
    targets = adapter.convert(left, right)
    np.testing.assert_allclose(
        targets["left_target_pose"].homogeneous,
        left_target.homogeneous,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        targets["right_target_pose"].homogeneous,
        right_target.homogeneous,
        atol=1e-12,
    )


@pytest.mark.parametrize(
    ("xr_delta", "robot_delta"),
    [
        ([0.1, 0.0, 0.0], [0.0, -0.2, 0.0]),
        ([0.0, 0.1, 0.0], [0.0, 0.0, 0.2]),
        ([0.0, 0.0, 0.1], [-0.2, 0.0, 0.0]),
    ],
)
def test_translation_axis_mapping_and_explicit_scale(
    xr_delta: list[float],
    robot_delta: list[float],
) -> None:
    left = controller_pose(np.array([-0.25, 1.1, -0.4]))
    right = controller_pose(np.array([0.25, 1.1, -0.4]))
    adapter, left_target, _ = initialized_adapter(left, right, scale=2.0)
    current_left = controller_pose(left.position + np.asarray(xr_delta), timestamp=1.0)
    target = adapter.convert(current_left, right)["left_target_pose"]
    np.testing.assert_allclose(
        target.translation - left_target.translation,
        np.asarray(robot_delta),
        atol=1e-12,
    )


def test_spatial_rotation_cancels_fixed_controller_local_extrinsic() -> None:
    initial_spatial_rotation = pin.exp3(np.array([0.2, -0.15, 0.1]))
    motion_spatial_rotation = pin.exp3(np.array([-0.1, 0.25, 0.05]))
    local_extrinsic_a = pin.exp3(np.array([0.3, 0.1, -0.2]))
    local_extrinsic_b = pin.exp3(np.array([-0.2, 0.4, 0.1]))
    initial_position = np.array([-0.25, 1.1, -0.4])
    right = controller_pose(np.array([0.25, 1.1, -0.4]))

    initial_a = controller_pose(initial_position, initial_spatial_rotation @ local_extrinsic_a)
    initial_b = controller_pose(
        initial_position + np.array([4.0, -3.0, 2.0]),
        initial_spatial_rotation @ local_extrinsic_b,
    )
    adapter_a, _, _ = initialized_adapter(initial_a, right)
    adapter_b, _, _ = initialized_adapter(initial_b, right)

    current_a = controller_pose(
        initial_position + np.array([0.02, -0.03, 0.04]),
        motion_spatial_rotation @ initial_spatial_rotation @ local_extrinsic_a,
        timestamp=1.0,
    )
    current_b = controller_pose(
        initial_b.position + np.array([0.02, -0.03, 0.04]),
        motion_spatial_rotation @ initial_spatial_rotation @ local_extrinsic_b,
        timestamp=1.0,
    )
    target_a = adapter_a.convert(current_a, right)["left_target_pose"]
    target_b = adapter_b.convert(current_b, right)["left_target_pose"]
    np.testing.assert_allclose(target_a.homogeneous, target_b.homogeneous, atol=1e-12)

    basis = UNITREE_ROBOT_FROM_OPENXR_BASIS
    expected_delta_rotation = basis @ motion_spatial_rotation @ basis.T
    anchor_rotation = pin.exp3(np.array([0.1, -0.2, 0.3]))
    np.testing.assert_allclose(
        target_a.rotation,
        expected_delta_rotation @ anchor_rotation,
        atol=1e-12,
    )
