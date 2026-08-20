"""Named loading and state access for the checked-in wheelloong_m2 MJCF."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from wheelloong_m2.kinematics.robot_model import ARM_JOINT_NAMES, repository_root


@dataclass(frozen=True)
class ArmJointControlAddress:
    """Resolved scalar qpos and position-actuator control addresses for one arm joint."""

    name: str
    joint_id: int
    qpos_index: int
    ctrl_index: int
    actuator_name: str
    q_arm_index: int


def wheelloong_m2_mjcf_path() -> Path:
    """Return the existing controlled MJCF without depending on shell CWD."""
    return (
        repository_root()
        / "src/description/wheelloong_m2/mujoco/wheelloong_m2_controlled.xml"
    )


class WheelloongM2MuJoCo:
    """Load and step the existing controlled wheelloong_m2 MuJoCo model."""

    def __init__(self, mjcf_path: Path | None = None) -> None:
        self.mjcf_path = wheelloong_m2_mjcf_path() if mjcf_path is None else Path(mjcf_path)
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.arm_joint_addresses: tuple[ArmJointControlAddress, ...] = ()

    def load(self) -> "WheelloongM2MuJoCo":
        """Load the checked-in MJCF, create data, and resolve all named arm controls."""
        if not self.mjcf_path.is_file():
            raise FileNotFoundError(f"MuJoCo model does not exist: {self.mjcf_path}")
        self.model = mujoco.MjModel.from_xml_path(str(self.mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.arm_joint_addresses = self._resolve_arm_joint_addresses()
        self.reset()
        self.print_arm_joint_mapping()
        return self

    def _require_loaded(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        if self.model is None or self.data is None:
            raise RuntimeError("MuJoCo model is not loaded; call load() first")
        return self.model, self.data

    @staticmethod
    def _is_position_actuator(model: mujoco.MjModel, actuator_id: int) -> bool:
        """Recognize MuJoCo's fixed-gain, affine-bias position-actuator form."""
        return bool(
            int(model.actuator_dyntype[actuator_id]) == int(mujoco.mjtDyn.mjDYN_NONE)
            and int(model.actuator_gaintype[actuator_id]) == int(mujoco.mjtGain.mjGAIN_FIXED)
            and int(model.actuator_biastype[actuator_id]) == int(mujoco.mjtBias.mjBIAS_AFFINE)
            and np.isclose(
                model.actuator_biasprm[actuator_id, 1],
                -model.actuator_gainprm[actuator_id, 0],
            )
        )

    def _resolve_arm_joint_addresses(self) -> tuple[ArmJointControlAddress, ...]:
        """Resolve joint-to-qpos-to-position-actuator mapping by loaded names."""
        model, _ = self._require_loaded()
        addresses: list[ArmJointControlAddress] = []
        for q_arm_index, name in enumerate(ARM_JOINT_NAMES):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise KeyError(f"MJCF model is missing required arm joint {name!r}")
            if int(model.jnt_type[joint_id]) not in (
                int(mujoco.mjtJoint.mjJNT_HINGE),
                int(mujoco.mjtJoint.mjJNT_SLIDE),
            ):
                raise ValueError(f"Expected scalar hinge/slide joint for {name!r}")

            actuator_ids = [
                actuator_id
                for actuator_id in range(model.nu)
                if int(model.actuator_trntype[actuator_id]) == int(mujoco.mjtTrn.mjTRN_JOINT)
                and int(model.actuator_trnid[actuator_id, 0]) == joint_id
                and self._is_position_actuator(model, actuator_id)
            ]
            if len(actuator_ids) != 1:
                raise ValueError(
                    f"Expected exactly one position actuator for {name!r}; "
                    f"found {len(actuator_ids)}"
                )
            actuator_id = actuator_ids[0]
            actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if actuator_name is None:
                raise KeyError(f"MJCF actuator {actuator_id} for {name!r} has no name")

            addresses.append(
                ArmJointControlAddress(
                    name=name,
                    joint_id=int(joint_id),
                    qpos_index=int(model.jnt_qposadr[joint_id]),
                    ctrl_index=int(actuator_id),
                    actuator_name=actuator_name,
                    q_arm_index=q_arm_index,
                )
            )
        return tuple(addresses)

    def print_arm_joint_mapping(self) -> None:
        """Print the required name-resolved left/right joint-qpos-ctrl mapping."""
        self._require_loaded()
        for side, addresses in (
            ("left", self.arm_joint_addresses[:7]),
            ("right", self.arm_joint_addresses[7:]),
        ):
            print(f"{side} arm joints: name | qpos adr | ctrl adr | actuator")
            for address in addresses:
                print(
                    f"  {address.name} | {address.qpos_index} | {address.ctrl_index} | "
                    f"{address.actuator_name}"
                )

    def reset(self) -> None:
        """Reset MuJoCo state to MJCF qpos0, then refresh derived quantities."""
        model, data = self._require_loaded()
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)

    def step(self) -> None:
        """Advance exactly one physics timestep; actuator commands remain in data.ctrl."""
        model, data = self._require_loaded()
        mujoco.mj_step(model, data)

    def arm_qpos(self) -> np.ndarray:
        """Return current named arm scalar positions in the public q_arm order."""
        _, data = self._require_loaded()
        if len(self.arm_joint_addresses) != len(ARM_JOINT_NAMES):
            raise RuntimeError("Named arm mapping is unavailable; call load() first")
        return np.array(
            [data.qpos[address.qpos_index] for address in self.arm_joint_addresses],
            dtype=float,
        )
