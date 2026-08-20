"""Offline CasADi Opti/IPOPT baseline IK for wheelloong_m2 operational EEs."""

from __future__ import annotations

from time import perf_counter

import casadi as ca
import numpy as np
import pinocchio as pin

from wheelloong_m2.kinematics.casadi_fk import WheelloongM2CasadiKinematics
from wheelloong_m2.kinematics.robot_model import ARM_JOINT_NAMES, arm_joint_limits

from .casadi_se3_error import compute_pose_error
from .cost import IKWeights


class WheelloongM2DualArmIK:
    """Solve offline 14-DOF dual-arm pose IK with fixed S0.5b operational EEs.

    The optimization variable has the sole public ``ARM_JOINT_NAMES`` order.
    All targets are ``^torso T_W`` Pinocchio ``SE3`` values supplied at solve
    time. This class has no robot, XR, trajectory, or controller interface.
    """

    def __init__(self, weights: IKWeights | None = None) -> None:
        self.weights = IKWeights() if weights is None else weights
        if not isinstance(self.weights, IKWeights):
            raise TypeError("weights must be an IKWeights instance or None")

        self.symbolic_kinematics = WheelloongM2CasadiKinematics()
        self.q_nom = self._neutral_arm_configuration()
        self.q_limits = arm_joint_limits(self.symbolic_kinematics.numeric_model)

        self.opti = ca.Opti()
        self.q = self.opti.variable(len(ARM_JOINT_NAMES))
        self.left_target_position = self.opti.parameter(3)
        self.left_target_rotation = self.opti.parameter(3, 3)
        self.right_target_position = self.opti.parameter(3)
        self.right_target_rotation = self.opti.parameter(3, 3)
        self.q_prev_parameter = self.opti.parameter(len(ARM_JOINT_NAMES))

        self._build_problem()

    def _neutral_arm_configuration(self) -> np.ndarray:
        """Extract the named arm entries from Pinocchio's neutral full q."""
        full_neutral = pin.neutral(self.symbolic_kinematics.numeric_model)
        q_nom = np.empty(len(ARM_JOINT_NAMES), dtype=float)
        for address in self.symbolic_kinematics.arm_joint_addresses:
            q_nom[address.q_arm_index] = full_neutral[address.pinocchio_q_index]
        return q_nom

    def _build_problem(self) -> None:
        """Build one reusable parameterized Opti problem without duplicating FK."""
        (
            left_position,
            left_rotation,
            right_position,
            right_rotation,
        ) = self.symbolic_kinematics.FK_function(self.q)

        left_error = compute_pose_error(
            left_position,
            left_rotation,
            self.left_target_position,
            self.left_target_rotation,
        )
        right_error = compute_pose_error(
            right_position,
            right_rotation,
            self.right_target_position,
            self.right_target_rotation,
        )

        pose_cost = (
            self.weights.wp * ca.sumsqr(left_error["position_error"])
            + self.weights.wr * ca.sumsqr(left_error["rotation_error"])
            + self.weights.wp * ca.sumsqr(right_error["position_error"])
            + self.weights.wr * ca.sumsqr(right_error["rotation_error"])
        )
        q_nom = ca.DM(self.q_nom)
        regularization_cost = self.weights.wq * ca.sumsqr(self.q - q_nom)
        smooth_cost = self.weights.ws * ca.sumsqr(self.q - self.q_prev_parameter)
        self.total_cost = pose_cost + regularization_cost + smooth_cost

        self.opti.subject_to(self.opti.bounded(self.q_limits[:, 0], self.q, self.q_limits[:, 1]))
        self.opti.minimize(self.total_cost)
        self.opti.solver("ipopt", {"print_time": False}, {"print_level": 0})

    @staticmethod
    def _validate_q(name: str, q: np.ndarray) -> np.ndarray:
        values = np.asarray(q, dtype=float)
        if values.shape != (len(ARM_JOINT_NAMES),):
            raise ValueError(
                f"{name} must have shape ({len(ARM_JOINT_NAMES)},), got {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must contain only finite values")
        return values

    @staticmethod
    def _target_components(name: str, pose: pin.SE3) -> tuple[np.ndarray, np.ndarray]:
        """Validate and copy a torso-relative Pinocchio target pose."""
        if not isinstance(pose, pin.SE3):
            raise TypeError(f"{name} must be a Pinocchio SE3, got {type(pose).__name__}")
        position = np.asarray(pose.translation, dtype=float).copy()
        rotation = np.asarray(pose.rotation, dtype=float).copy()
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError(f"{name} must contain only finite position and rotation values")
        return position, rotation

    def _failure_result(self, q_fallback: np.ndarray, elapsed: float) -> dict[str, object]:
        """Return the documented plain-numpy result shape if IPOPT raises."""
        return {
            "q_arm": q_fallback.copy(),
            "success": False,
            "cost": float("nan"),
            "solve_time": elapsed,
            "iterations": 0,
        }

    def solve(
        self,
        left_target_pose: pin.SE3,
        right_target_pose: pin.SE3,
        q_init: np.ndarray | None = None,
        q_prev: np.ndarray | None = None,
    ) -> dict[str, object]:
        """Solve the constrained offline dual-arm pose problem.

        ``q_init`` seeds IPOPT. ``q_prev`` supplies the smoothness term and
        defaults to the neutral arm pose; when no explicit seed is provided,
        the same ``q_prev`` is used as the warm-start seed. The returned
        values are plain Python/Numpy data, never CasADi objects.
        """
        left_position, left_rotation = self._target_components("left_target_pose", left_target_pose)
        right_position, right_rotation = self._target_components("right_target_pose", right_target_pose)
        q_prev_values = self.q_nom if q_prev is None else self._validate_q("q_prev", q_prev)
        q_initial_values = q_prev_values if q_init is None else self._validate_q("q_init", q_init)

        self.opti.set_value(self.left_target_position, left_position)
        self.opti.set_value(self.left_target_rotation, left_rotation)
        self.opti.set_value(self.right_target_position, right_position)
        self.opti.set_value(self.right_target_rotation, right_rotation)
        self.opti.set_value(self.q_prev_parameter, q_prev_values)
        self.opti.set_initial(self.q, q_initial_values)

        start_time = perf_counter()
        try:
            solution = self.opti.solve()
        except RuntimeError:
            return self._failure_result(q_initial_values, perf_counter() - start_time)

        elapsed = perf_counter() - start_time
        stats = solution.stats()
        return {
            "q_arm": np.asarray(solution.value(self.q), dtype=float).reshape(len(ARM_JOINT_NAMES)),
            "success": bool(stats.get("success", True)),
            "cost": float(solution.value(self.total_cost)),
            "solve_time": elapsed,
            "iterations": int(stats.get("iter_count", 0)),
        }
