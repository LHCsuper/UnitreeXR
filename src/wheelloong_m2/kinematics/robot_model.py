"""Checked-in wheelloong_m2 URDF loading and explicit arm-joint addressing."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin


LEFT_ARM_JOINT_NAMES = tuple(f"left_arm_joint_{index}" for index in range(1, 8))
RIGHT_ARM_JOINT_NAMES = tuple(f"right_arm_joint_{index}" for index in range(1, 8))
ARM_JOINT_NAMES = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
ARM_Q_INDEX_BY_NAME = {name: index for index, name in enumerate(ARM_JOINT_NAMES)}


@dataclass(frozen=True)
class ArmJointAddress:
    """Named scalar arm joint addresses in both public and Pinocchio order."""

    name: str
    pinocchio_joint_id: int
    pinocchio_q_index: int
    pinocchio_v_index: int
    q_arm_index: int


@dataclass
class LoadedRobotModel:
    """One Pinocchio model/data pair built from the checked-in URDF."""

    model: pin.Model
    data: pin.Data
    urdf_path: Path


def repository_root() -> Path:
    """Find the repository root without relying on the caller's CWD."""
    expected_urdf = Path("src/description/wheelloong_m2/urdf/wheelloong_m2.urdf")
    for candidate in Path(__file__).resolve().parents:
        if (candidate / expected_urdf).is_file():
            return candidate
    raise FileNotFoundError("Could not locate UnitreeXR repository root from this module")


def wheelloong_m2_urdf_path() -> Path:
    """Return the absolute path to the checked-in wheelloong_m2 URDF."""
    return repository_root() / "src/description/wheelloong_m2/urdf/wheelloong_m2.urdf"


def load_wheelloong_m2_model() -> LoadedRobotModel:
    """Build a Pinocchio model/data pair from the repository URDF."""
    urdf_path = wheelloong_m2_urdf_path()
    model = pin.buildModelFromUrdf(str(urdf_path))
    return LoadedRobotModel(model=model, data=model.createData(), urdf_path=urdf_path)


def arm_joint_addresses(model: pin.Model) -> tuple[ArmJointAddress, ...]:
    """Resolve every public 14-DOF arm joint by name and validate scalar DOFs."""
    addresses: list[ArmJointAddress] = []
    for name in ARM_JOINT_NAMES:
        joint_id = int(model.getJointId(name))
        if joint_id >= model.njoints:
            raise KeyError(f"URDF/Pinocchio model is missing required arm joint {name}")

        joint = model.joints[joint_id]
        if joint.nq != 1 or joint.nv != 1:
            raise ValueError(
                f"Expected scalar configuration/velocity for {name}; "
                f"got nq={joint.nq}, nv={joint.nv}"
            )

        addresses.append(
            ArmJointAddress(
                name=name,
                pinocchio_joint_id=joint_id,
                pinocchio_q_index=int(model.idx_qs[joint_id]),
                pinocchio_v_index=int(model.idx_vs[joint_id]),
                q_arm_index=ARM_Q_INDEX_BY_NAME[name],
            )
        )
    return tuple(addresses)


def arm_joint_limits(model: pin.Model) -> np.ndarray:
    """Return finite lower/upper limits in the public ``q_arm`` order."""
    limits = np.empty((len(ARM_JOINT_NAMES), 2), dtype=float)
    for address in arm_joint_addresses(model):
        limits[address.q_arm_index] = (
            model.lowerPositionLimit[address.pinocchio_q_index],
            model.upperPositionLimit[address.pinocchio_q_index],
        )

    if not np.all(np.isfinite(limits)):
        raise ValueError("All 14 arm joints must have finite URDF position limits")
    if np.any(limits[:, 0] > limits[:, 1]):
        raise ValueError("At least one arm joint has an invalid lower/upper limit")
    return limits


def print_arm_joint_index_table(model: pin.Model) -> None:
    """Print the sole public arm ordering and the resolved Pinocchio addresses."""
    print("q_arm order: joint name | pin joint id | pin q index | pin v index | q_arm index")
    for address in arm_joint_addresses(model):
        print(
            f"  {address.name} | {address.pinocchio_joint_id} | "
            f"{address.pinocchio_q_index} | {address.pinocchio_v_index} | "
            f"{address.q_arm_index}"
        )
