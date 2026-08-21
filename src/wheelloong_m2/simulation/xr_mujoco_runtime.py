"""Simulation-only XR -> relative adapter -> IK -> MuJoCo runtime."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter, sleep
from typing import Callable

import numpy as np
import pinocchio as pin

from wheelloong_m2.ik import WheelloongM2DualArmIK
from wheelloong_m2.ik.se3_error import compute_dual_arm_error
from wheelloong_m2.kinematics import ARM_JOINT_NAMES, WheelloongM2Kinematics
from wheelloong_m2.xr import InitializedRelativeXRAdapter, XRControllerPose

from .mujoco_arm_controller import MujocoArmPositionController
from .mujoco_model import WheelloongM2MuJoCo
from .runtime import DualArmTargetBuffer, MultiRateScheduler, SIMULATION_HZ


XRPairSampler = Callable[
    [float],
    tuple[XRControllerPose, XRControllerPose],
]
SimulationStepCallback = Callable[[WheelloongM2MuJoCo], None]


@dataclass(frozen=True)
class XRMuJoCoSimulationResult:
    """Plain numerical report from one simulation-only teleoperation run."""

    duration_s: float
    wall_time_s: float
    initialization_physics_steps: int
    initial_q_requested: np.ndarray
    initial_q_achieved: np.ndarray
    initial_joint_tracking_error_rad: float
    target_updates: int
    ik_solves: int
    physics_steps: int
    q_target: np.ndarray
    q_simulated: np.ndarray
    joint_tracking_error_rad: float
    left_position_error_m: float
    left_rotation_error_rad: float
    right_position_error_m: float
    right_rotation_error_rad: float
    left_target_pose: pin.SE3
    right_target_pose: pin.SE3


def run_xr_mujoco_simulation(
    sample_pair: XRPairSampler,
    duration_s: float,
    *,
    adapter: InitializedRelativeXRAdapter | None = None,
    initial_q_arm: np.ndarray | None = None,
    initial_settle_duration_s: float = 3.0,
    real_time: bool = False,
    step_callback: SimulationStepCallback | None = None,
) -> XRMuJoCoSimulationResult:
    """Run initialized relative XR targets against the checked-in MuJoCo model.

    ``sample_pair`` is invoked only on configured target ticks and receives
    simulation time. Its first pair anchors the controllers to the current
    requested initial robot operational frames after position-actuator
    settling. This function has no real robot transport or controller
    dependency.
    """
    duration = float(duration_s)
    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration_s must be finite and greater than zero")
    if not callable(sample_pair):
        raise TypeError("sample_pair must be callable")
    if step_callback is not None and not callable(step_callback):
        raise TypeError("step_callback must be callable or None")
    settle_duration = float(initial_settle_duration_s)
    if not np.isfinite(settle_duration) or settle_duration < 0.0:
        raise ValueError("initial_settle_duration_s must be finite and non-negative")

    relative_adapter = (
        InitializedRelativeXRAdapter() if adapter is None else adapter
    )
    if not isinstance(relative_adapter, InitializedRelativeXRAdapter):
        raise TypeError("adapter must be an InitializedRelativeXRAdapter or None")

    simulation = WheelloongM2MuJoCo().load()
    simulation.reset()
    controller = MujocoArmPositionController(simulation)
    kinematics = WheelloongM2Kinematics()
    solver = WheelloongM2DualArmIK(q_nom=initial_q_arm)
    scheduler = MultiRateScheduler()
    target_buffer = DualArmTargetBuffer()

    q_reset = simulation.arm_qpos()
    q_initial_requested = q_reset.copy()
    initialization_step_count = 0
    wall_start = perf_counter()
    if initial_q_arm is not None:
        q_initial_requested = np.asarray(initial_q_arm, dtype=float)
        if q_initial_requested.shape != (len(ARM_JOINT_NAMES),):
            raise ValueError(
                f"initial_q_arm must have shape ({len(ARM_JOINT_NAMES)},), "
                f"got {q_initial_requested.shape}"
            )
        if not np.all(np.isfinite(q_initial_requested)):
            raise ValueError("initial_q_arm must contain only finite values")
        if np.any(q_initial_requested < solver.q_limits[:, 0]) or np.any(
            q_initial_requested > solver.q_limits[:, 1]
        ):
            raise ValueError("initial_q_arm violates at least one URDF joint limit")

        controller.set_arm_position_target(q_initial_requested)
        initialization_step_count = round(settle_duration * SIMULATION_HZ)
        for step_index in range(initialization_step_count):
            simulation.step()
            if step_callback is not None:
                step_callback(simulation)
            if real_time:
                deadline = wall_start + (step_index + 1) / SIMULATION_HZ
                remaining = deadline - perf_counter()
                if remaining > 0.0:
                    sleep(remaining)

    q_initial_achieved = simulation.arm_qpos()
    q_target = q_initial_requested.copy()
    target_update_count = 0
    ik_solve_count = 0
    physics_step_count = 0
    simulation_step_total = round(duration * SIMULATION_HZ)
    if simulation_step_total <= 0:
        raise ValueError("duration_s is shorter than one simulation timestep")

    teleoperation_wall_start = perf_counter()
    for step_index in range(simulation_step_total):
        tick = scheduler.next_tick()
        if tick.target_due:
            left_controller, right_controller = sample_pair(tick.time_s)
            if not relative_adapter.initialized:
                initial_robot_poses = kinematics.forward_kinematics(q_target)
                relative_adapter.initialize(
                    left_controller,
                    right_controller,
                    initial_robot_poses["left_ee_pose"],
                    initial_robot_poses["right_ee_pose"],
                )
            target_poses = relative_adapter.convert(left_controller, right_controller)
            target_buffer.update(
                max(left_controller.timestamp, right_controller.timestamp),
                target_poses["left_target_pose"],
                target_poses["right_target_pose"],
            )
            target_update_count += 1

        if tick.ik_due:
            latest_target = target_buffer.get_latest()
            if latest_target is None:
                raise RuntimeError(
                    "IK tick occurred before the XR target buffer was initialized"
                )
            ik_result = solver.solve(
                latest_target.left_target_pose,
                latest_target.right_target_pose,
                q_init=q_target,
                q_prev=q_target,
            )
            if not ik_result["success"]:
                raise RuntimeError(
                    f"IPOPT did not report success at t={tick.time_s:.6f} s"
                )
            q_target = np.asarray(ik_result["q_arm"], dtype=float)
            ik_solve_count += 1

        controller.set_arm_position_target(q_target)
        simulation.step()
        physics_step_count += 1
        if step_callback is not None:
            step_callback(simulation)

        if real_time:
            deadline = teleoperation_wall_start + (step_index + 1) / SIMULATION_HZ
            remaining = deadline - perf_counter()
            if remaining > 0.0:
                sleep(remaining)

    wall_time = perf_counter() - wall_start
    latest_target = target_buffer.get_latest()
    if latest_target is None:
        raise RuntimeError("XR target buffer is empty after simulation")
    final_q = simulation.arm_qpos()
    final_poses = kinematics.forward_kinematics(final_q)
    errors = compute_dual_arm_error(
        final_poses["left_ee_pose"],
        final_poses["right_ee_pose"],
        latest_target.left_target_pose,
        latest_target.right_target_pose,
    )

    return XRMuJoCoSimulationResult(
        duration_s=physics_step_count / SIMULATION_HZ,
        wall_time_s=wall_time,
        initialization_physics_steps=initialization_step_count,
        initial_q_requested=q_initial_requested.copy(),
        initial_q_achieved=q_initial_achieved.copy(),
        initial_joint_tracking_error_rad=float(
            np.linalg.norm(q_initial_achieved - q_initial_requested)
        ),
        target_updates=target_update_count,
        ik_solves=ik_solve_count,
        physics_steps=physics_step_count,
        q_target=q_target.copy(),
        q_simulated=final_q.copy(),
        joint_tracking_error_rad=float(np.linalg.norm(final_q - q_target)),
        left_position_error_m=float(np.linalg.norm(errors["left"]["position_error"])),
        left_rotation_error_rad=float(np.linalg.norm(errors["left"]["rotation_error"])),
        right_position_error_m=float(np.linalg.norm(errors["right"]["position_error"])),
        right_rotation_error_rad=float(np.linalg.norm(errors["right"]["rotation_error"])),
        left_target_pose=pin.SE3(
            latest_target.left_target_pose.rotation.copy(),
            latest_target.left_target_pose.translation.copy(),
        ),
        right_target_pose=pin.SE3(
            latest_target.right_target_pose.rotation.copy(),
            latest_target.right_target_pose.translation.copy(),
        ),
    )
