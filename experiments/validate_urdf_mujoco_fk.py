#!/usr/bin/env python3
"""EXP-003: compare wheelloong_m2 arm FK from URDF and MJCF without conversion."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pinocchio as pin


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPOSITORY_ROOT / "src/description/wheelloong_m2/urdf/wheelloong_m2.urdf"
MJCF_PATH = (
    REPOSITORY_ROOT
    / "src/description/wheelloong_m2/mujoco/wheelloong_m2_controlled.xml"
)

TORSO_FRAME = "torso_link"
LEFT_TARGET_FRAME = "left_arm_link_7"
RIGHT_TARGET_FRAME = "right_arm_link_7"
LEFT_ARM_JOINT_NAMES = tuple(f"left_arm_joint_{index}" for index in range(1, 8))
RIGHT_ARM_JOINT_NAMES = tuple(f"right_arm_joint_{index}" for index in range(1, 8))
ARM_JOINT_NAMES = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
TARGET_FRAME_NAMES = (TORSO_FRAME, LEFT_TARGET_FRAME, RIGHT_TARGET_FRAME)


@dataclass(frozen=True)
class JointSourceMetadata:
    """Joint data read directly from one source model file."""

    name: str
    joint_type: str
    axis: np.ndarray
    limits: tuple[float | None, float | None]
    parent: str
    child: str
    child_position: np.ndarray
    child_orientation: np.ndarray


@dataclass(frozen=True)
class JointAddress:
    """The independently resolved configuration address for one named joint."""

    name: str
    pinocchio_q_index: int
    pinocchio_nq: int
    mujoco_qpos_index: int


@dataclass(frozen=True)
class ArmPoseComparison:
    """The two directly computed relative poses and their measured difference."""

    pinocchio_torso_T_target: np.ndarray
    mujoco_torso_T_target: np.ndarray
    position_error_m: float
    rotation_error_rad: float


@dataclass(frozen=True)
class CaseResult:
    """FK comparison result for a complete, named arm configuration."""

    name: str
    left: ArmPoseComparison
    right: ArmPoseComparison


def format_vector(vector: np.ndarray | Iterable[float]) -> str:
    values = np.asarray(vector, dtype=float)
    return np.array2string(values, precision=10, suppress_small=False)


def format_transform(transform: np.ndarray) -> str:
    return np.array2string(transform, precision=10, suppress_small=False)


def parse_numbers(raw_value: str, default: str) -> np.ndarray:
    return np.fromstring(raw_value or default, sep=" ", dtype=float)


def parse_urdf(path: Path) -> tuple[set[str], dict[str, JointSourceMetadata]]:
    root = ET.parse(path).getroot()
    link_names = {node.attrib["name"] for node in root.findall("link")}
    joints: dict[str, JointSourceMetadata] = {}

    for joint_node in root.findall("joint"):
        name = joint_node.attrib["name"]
        axis_node = joint_node.find("axis")
        limit_node = joint_node.find("limit")
        origin_node = joint_node.find("origin")
        parent_node = joint_node.find("parent")
        child_node = joint_node.find("child")

        lower_limit = None
        upper_limit = None
        if limit_node is not None:
            if "lower" in limit_node.attrib:
                lower_limit = float(limit_node.attrib["lower"])
            if "upper" in limit_node.attrib:
                upper_limit = float(limit_node.attrib["upper"])

        joints[name] = JointSourceMetadata(
            name=name,
            joint_type=joint_node.attrib["type"],
            axis=parse_numbers(
                "" if axis_node is None else axis_node.attrib.get("xyz", ""),
                "0 0 0",
            ),
            limits=(lower_limit, upper_limit),
            parent="" if parent_node is None else parent_node.attrib["link"],
            child="" if child_node is None else child_node.attrib["link"],
            child_position=parse_numbers(
                "" if origin_node is None else origin_node.attrib.get("xyz", ""),
                "0 0 0",
            ),
            child_orientation=parse_numbers(
                "" if origin_node is None else origin_node.attrib.get("rpy", ""),
                "0 0 0",
            ),
        )

    return link_names, joints


def parse_mjcf(path: Path) -> tuple[set[str], dict[str, JointSourceMetadata]]:
    root = ET.parse(path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF has no worldbody element")

    body_names: set[str] = set()
    joints: dict[str, JointSourceMetadata] = {}

    def visit_body(body_node: ET.Element, parent_body_name: str) -> None:
        body_name = body_node.attrib.get("name")
        if body_name is None:
            raise ValueError("MJCF body without a name is unsupported by this experiment")
        body_names.add(body_name)

        child_position = parse_numbers(body_node.attrib.get("pos", ""), "0 0 0")
        child_orientation = parse_numbers(body_node.attrib.get("quat", ""), "1 0 0 0")
        for joint_node in body_node.findall("joint"):
            raw_range = joint_node.attrib.get("range")
            limits = (None, None)
            if raw_range is not None:
                parsed_range = parse_numbers(raw_range, "")
                if parsed_range.shape != (2,):
                    raise ValueError(
                        f"MJCF joint {joint_node.attrib['name']} has invalid range: {raw_range}"
                    )
                limits = (float(parsed_range[0]), float(parsed_range[1]))

            name = joint_node.attrib["name"]
            joints[name] = JointSourceMetadata(
                name=name,
                joint_type=joint_node.attrib.get("type", "hinge"),
                axis=parse_numbers(joint_node.attrib.get("axis", ""), "0 0 1"),
                limits=limits,
                parent=parent_body_name,
                child=body_name,
                child_position=child_position,
                child_orientation=child_orientation,
            )

        for child_body_node in body_node.findall("body"):
            visit_body(child_body_node, body_name)

    for body_node in worldbody.findall("body"):
        visit_body(body_node, "world")

    return body_names, joints


def pinocchio_joint_type(joint: pin.JointModel) -> str:
    shortname = getattr(joint, "shortname")
    return shortname() if callable(shortname) else str(shortname)


def pinocchio_joint_index(joint: pin.JointModel) -> int:
    index = getattr(joint, "idx_q")
    return int(index() if callable(index) else index)


def pinocchio_velocity_index(joint: pin.JointModel) -> int:
    index = getattr(joint, "idx_v")
    return int(index() if callable(index) else index)


def mujoco_joint_type_name(joint_type: int) -> str:
    joint_type_names = {
        int(mujoco.mjtJoint.mjJNT_FREE): "free",
        int(mujoco.mjtJoint.mjJNT_BALL): "ball",
        int(mujoco.mjtJoint.mjJNT_SLIDE): "slide",
        int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
    }
    return joint_type_names[int(joint_type)]


def print_loaded_model_metadata(pin_model: pin.Model, mj_model: mujoco.MjModel) -> None:
    print("=== Loaded model metadata ===")
    print(f"Pinocchio: nq={pin_model.nq}, nv={pin_model.nv}, joints={pin_model.njoints - 1}")
    print("Pinocchio joints: name | type | q index | nq | velocity index | nv")
    for joint_id in range(1, pin_model.njoints):
        joint = pin_model.joints[joint_id]
        print(
            f"  {pin_model.names[joint_id]} | {pinocchio_joint_type(joint)} | "
            f"{pinocchio_joint_index(joint)} | {joint.nq} | "
            f"{pinocchio_velocity_index(joint)} | {joint.nv}"
        )

    print(f"MuJoCo: nq={mj_model.nq}, nv={mj_model.nv}, joints={mj_model.njnt}")
    print("MuJoCo joints: name | type | qpos index | dof index | child body")
    for joint_id in range(mj_model.njnt):
        joint_name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        body_id = int(mj_model.jnt_bodyid[joint_id])
        body_name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        print(
            f"  {joint_name} | {mujoco_joint_type_name(mj_model.jnt_type[joint_id])} | "
            f"{int(mj_model.jnt_qposadr[joint_id])} | "
            f"{int(mj_model.jnt_dofadr[joint_id])} | {body_name}"
        )


def require_named_joint_addresses(
    pin_model: pin.Model,
    mj_model: mujoco.MjModel,
    joint_names: Iterable[str],
) -> dict[str, JointAddress]:
    addresses: dict[str, JointAddress] = {}
    missing_names: list[str] = []

    for name in joint_names:
        pin_joint_id = pin_model.getJointId(name)
        mj_joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if pin_joint_id >= pin_model.njoints or mj_joint_id < 0:
            missing_names.append(name)
            continue

        pin_joint = pin_model.joints[pin_joint_id]
        if pin_joint.nq != 1:
            raise ValueError(
                f"Expected scalar Pinocchio configuration for {name}; got nq={pin_joint.nq}"
            )
        if mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0:
            missing_names.append(name)
            continue

        addresses[name] = JointAddress(
            name=name,
            pinocchio_q_index=pinocchio_joint_index(pin_joint),
            pinocchio_nq=pin_joint.nq,
            mujoco_qpos_index=int(mj_model.jnt_qposadr[mj_joint_id]),
        )

    if missing_names:
        raise ValueError(f"Required arm joints missing from a loaded model: {missing_names}")

    return addresses


def require_target_frames(
    pin_model: pin.Model,
    mj_model: mujoco.MjModel,
) -> tuple[dict[str, int], dict[str, int]]:
    pin_frame_ids: dict[str, int] = {}
    mujoco_body_ids: dict[str, int] = {}
    missing_names: list[str] = []

    for name in TARGET_FRAME_NAMES:
        pin_frame_id = pin_model.getFrameId(name, pin.FrameType.BODY)
        mujoco_body_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, name)
        if pin_frame_id >= len(pin_model.frames) or pin_model.frames[pin_frame_id].name != name:
            missing_names.append(f"Pinocchio frame {name}")
        else:
            pin_frame_ids[name] = pin_frame_id
        if mujoco_body_id < 0:
            missing_names.append(f"MuJoCo body {name}")
        else:
            mujoco_body_ids[name] = mujoco_body_id

    if missing_names:
        raise ValueError(f"Required target frames/bodies missing: {missing_names}")

    return pin_frame_ids, mujoco_body_ids


def source_limits_match(
    urdf_limits: tuple[float | None, float | None],
    mjcf_limits: tuple[float | None, float | None],
) -> bool:
    if urdf_limits == (None, None) and mjcf_limits == (None, None):
        return True
    if None in urdf_limits or None in mjcf_limits:
        return False
    return bool(np.allclose(urdf_limits, mjcf_limits, atol=0.0, rtol=0.0))


def print_source_model_comparison(
    urdf_joints: dict[str, JointSourceMetadata],
    mjcf_joints: dict[str, JointSourceMetadata],
    urdf_link_names: set[str],
    mjcf_body_names: set[str],
) -> None:
    print("\n=== Source-model name and structure comparison ===")
    urdf_only_names = sorted(set(urdf_joints) - set(mjcf_joints))
    mjcf_only_names = sorted(set(mjcf_joints) - set(urdf_joints))
    print(f"URDF-only joint names: {urdf_only_names or 'none'}")
    print(f"MJCF-only joint names: {mjcf_only_names or 'none'}")

    print("Required target link/body presence:")
    for name in TARGET_FRAME_NAMES:
        print(
            f"  {name}: URDF link={name in urdf_link_names}, "
            f"MJCF body={name in mjcf_body_names}"
        )

    print("Arm joint mapping: name | Pinocchio q index | MuJoCo qpos index")
    print("  See the explicit addresses printed below after load validation.")
    print(
        "Arm joint source metadata: name | URDF type | MJCF type | axis | limits | "
        "topology | local child pose"
    )
    for name in ARM_JOINT_NAMES:
        urdf_joint = urdf_joints.get(name)
        mjcf_joint = mjcf_joints.get(name)
        if urdf_joint is None or mjcf_joint is None:
            print(f"  {name}: MISSING from one source file")
            continue
        print(
            f"  {name} | {urdf_joint.joint_type} | {mjcf_joint.joint_type} | "
            f"URDF {format_vector(urdf_joint.axis)} / "
            f"MJCF {format_vector(mjcf_joint.axis)} | "
            f"URDF {urdf_joint.limits} / MJCF {mjcf_joint.limits} | "
            f"URDF {urdf_joint.parent}->{urdf_joint.child} / "
            f"MJCF {mjcf_joint.parent}->{mjcf_joint.child} | "
            f"URDF xyz={format_vector(urdf_joint.child_position)}, "
            f"rpy={format_vector(urdf_joint.child_orientation)} / "
            f"MJCF pos={format_vector(mjcf_joint.child_position)}, "
            f"quat={format_vector(mjcf_joint.child_orientation)}"
        )

    discrepancies: list[str] = []
    for name in sorted(set(urdf_joints) & set(mjcf_joints)):
        urdf_joint = urdf_joints[name]
        mjcf_joint = mjcf_joints[name]
        if not np.array_equal(urdf_joint.axis, mjcf_joint.axis):
            discrepancies.append(
                f"{name}: axis URDF={format_vector(urdf_joint.axis)} "
                f"MJCF={format_vector(mjcf_joint.axis)}"
            )
        if not source_limits_match(urdf_joint.limits, mjcf_joint.limits):
            discrepancies.append(
                f"{name}: limits URDF={urdf_joint.limits} MJCF={mjcf_joint.limits}"
            )
        if (urdf_joint.parent, urdf_joint.child) != (mjcf_joint.parent, mjcf_joint.child):
            discrepancies.append(
                f"{name}: topology URDF={urdf_joint.parent}->{urdf_joint.child} "
                f"MJCF={mjcf_joint.parent}->{mjcf_joint.child}"
            )

    print("All named-joint axis/limit/topology discrepancies:")
    if discrepancies:
        for discrepancy in discrepancies:
            print(f"  {discrepancy}")
    else:
        print("  none")


def validated_arm_limits(
    urdf_joints: dict[str, JointSourceMetadata],
    mjcf_joints: dict[str, JointSourceMetadata],
) -> dict[str, tuple[float, float]]:
    limits: dict[str, tuple[float, float]] = {}
    for name in ARM_JOINT_NAMES:
        urdf_joint = urdf_joints[name]
        mjcf_joint = mjcf_joints[name]
        if not source_limits_match(urdf_joint.limits, mjcf_joint.limits):
            raise ValueError(
                f"Cannot create shared legal test value for {name}: "
                f"URDF={urdf_joint.limits}, MJCF={mjcf_joint.limits}"
            )
        lower, upper = urdf_joint.limits
        if lower is None or upper is None or lower > upper:
            raise ValueError(f"Joint {name} has no valid bounded range: {urdf_joint.limits}")
        limits[name] = (lower, upper)
    return limits


def neutral_value_from_limits(limits: tuple[float, float]) -> float:
    lower, upper = limits
    return float(np.clip(0.0, lower, upper))


def value_at_limit_fraction(limits: tuple[float, float], fraction: float) -> float:
    if not 0.0 <= fraction <= 1.0:
        raise ValueError(f"Limit fraction must be in [0, 1], got {fraction}")
    lower, upper = limits
    value = lower + fraction * (upper - lower)
    if not lower <= value <= upper:
        raise AssertionError(f"Generated out-of-range value {value} for limits {limits}")
    return value


def arm_neutral_configuration(
    limits_by_joint: dict[str, tuple[float, float]],
) -> dict[str, float]:
    return {
        name: neutral_value_from_limits(limits_by_joint[name])
        for name in ARM_JOINT_NAMES
    }


def with_limit_fractions(
    base_configuration: dict[str, float],
    limits_by_joint: dict[str, tuple[float, float]],
    fractions_by_joint: dict[str, float],
) -> dict[str, float]:
    configuration = dict(base_configuration)
    for name, fraction in fractions_by_joint.items():
        configuration[name] = value_at_limit_fraction(limits_by_joint[name], fraction)
    return configuration


def build_test_configurations(
    limits_by_joint: dict[str, tuple[float, float]],
) -> list[tuple[str, dict[str, float]]]:
    all_neutral = arm_neutral_configuration(limits_by_joint)
    random_generator = np.random.default_rng(seed=20260820)
    random_fractions = {
        name: float(random_generator.uniform(0.15, 0.85)) for name in ARM_JOINT_NAMES
    }

    return [
        ("Case 0 — both arms at limit-derived neutral", all_neutral),
        (
            "Case 1 — selected left-arm joints changed",
            with_limit_fractions(
                all_neutral,
                limits_by_joint,
                {
                    "left_arm_joint_1": 0.25,
                    "left_arm_joint_3": 0.70,
                    "left_arm_joint_4": 0.30,
                    "left_arm_joint_6": 0.75,
                },
            ),
        ),
        (
            "Case 2 — selected right-arm joints changed",
            with_limit_fractions(
                all_neutral,
                limits_by_joint,
                {
                    "right_arm_joint_1": 0.70,
                    "right_arm_joint_3": 0.30,
                    "right_arm_joint_5": 0.65,
                    "right_arm_joint_7": 0.25,
                },
            ),
        ),
        (
            "Case 3 — both arms changed",
            with_limit_fractions(
                all_neutral,
                limits_by_joint,
                {
                    "left_arm_joint_1": 0.65,
                    "left_arm_joint_2": 0.30,
                    "left_arm_joint_4": 0.70,
                    "left_arm_joint_5": 0.40,
                    "left_arm_joint_7": 0.75,
                    "right_arm_joint_1": 0.35,
                    "right_arm_joint_2": 0.70,
                    "right_arm_joint_4": 0.25,
                    "right_arm_joint_6": 0.65,
                    "right_arm_joint_7": 0.30,
                },
            ),
        ),
        (
            "Case 4 — fixed-seed safe random both arms",
            with_limit_fractions(all_neutral, limits_by_joint, random_fractions),
        ),
    ]


def apply_named_arm_configuration(
    pin_model: pin.Model,
    mj_model: mujoco.MjModel,
    pin_data: pin.Data,
    mj_data: mujoco.MjData,
    addresses_by_joint: dict[str, JointAddress],
    configuration_by_joint: dict[str, float],
) -> None:
    pin_q = pin.neutral(pin_model)
    mj_qpos = mj_model.qpos0.copy()

    for name, value in configuration_by_joint.items():
        address = addresses_by_joint[name]
        pin_q[address.pinocchio_q_index] = value
        mj_qpos[address.mujoco_qpos_index] = value

    mj_data.qpos[:] = mj_qpos
    pin.forwardKinematics(pin_model, pin_data, pin_q)
    pin.updateFramePlacements(pin_model, pin_data)
    mujoco.mj_forward(mj_model, mj_data)


def inverse_transform(from_T_to: np.ndarray) -> np.ndarray:
    from_R_to = from_T_to[:3, :3]
    from_p_to = from_T_to[:3, 3]
    to_T_from = np.eye(4)
    to_T_from[:3, :3] = from_R_to.T
    to_T_from[:3, 3] = -from_R_to.T @ from_p_to
    return to_T_from


def pinocchio_world_transform(pin_data: pin.Data, frame_id: int) -> np.ndarray:
    return pin_data.oMf[frame_id].homogeneous.copy()


def mujoco_world_transform(mj_data: mujoco.MjData, body_id: int) -> np.ndarray:
    world_T_body = np.eye(4)
    world_T_body[:3, :3] = mj_data.xmat[body_id].reshape(3, 3)
    world_T_body[:3, 3] = mj_data.xpos[body_id]
    return world_T_body


def compare_relative_pose(
    pinocchio_world_T_torso: np.ndarray,
    pinocchio_world_T_target: np.ndarray,
    mujoco_world_T_torso: np.ndarray,
    mujoco_world_T_target: np.ndarray,
) -> ArmPoseComparison:
    pinocchio_torso_T_target = (
        inverse_transform(pinocchio_world_T_torso) @ pinocchio_world_T_target
    )
    mujoco_torso_T_target = inverse_transform(mujoco_world_T_torso) @ mujoco_world_T_target

    position_error_m = float(
        np.linalg.norm(
            pinocchio_torso_T_target[:3, 3] - mujoco_torso_T_target[:3, 3]
        )
    )
    rotation_error_matrix = (
        pinocchio_torso_T_target[:3, :3] @ mujoco_torso_T_target[:3, :3].T
    )
    rotation_error_rad = float(np.linalg.norm(pin.log3(rotation_error_matrix)))

    return ArmPoseComparison(
        pinocchio_torso_T_target=pinocchio_torso_T_target,
        mujoco_torso_T_target=mujoco_torso_T_target,
        position_error_m=position_error_m,
        rotation_error_rad=rotation_error_rad,
    )


def print_named_addresses(addresses_by_joint: dict[str, JointAddress]) -> None:
    print("\n=== Explicit arm joint mapping ===")
    print("name | Pinocchio q index | MuJoCo qpos index")
    for name in ARM_JOINT_NAMES:
        address = addresses_by_joint[name]
        print(
            f"  {name} | {address.pinocchio_q_index} | {address.mujoco_qpos_index}"
        )


def print_case_configuration(
    configuration_by_joint: dict[str, float],
) -> None:
    print("Applied configuration [rad], by explicit joint name:")
    for arm_name, arm_joint_names in (
        ("Left", LEFT_ARM_JOINT_NAMES),
        ("Right", RIGHT_ARM_JOINT_NAMES),
    ):
        values = [configuration_by_joint[name] for name in arm_joint_names]
        print(f"  {arm_name}: {format_vector(values)}")


def print_arm_result(
    arm_name: str,
    target_frame: str,
    result: ArmPoseComparison,
) -> None:
    rotation_error_deg = np.degrees(result.rotation_error_rad)
    print(f"{arm_name}:")
    print(f"  Pinocchio ^torso T_{target_frame}:")
    print(format_transform(result.pinocchio_torso_T_target))
    print(f"  MuJoCo    ^torso T_{target_frame}:")
    print(format_transform(result.mujoco_torso_T_target))
    print(f"  position error [m]: {result.position_error_m:.12e}")
    print(f"  rotation error [rad]: {result.rotation_error_rad:.12e}")
    print(f"  rotation error [deg]: {rotation_error_deg:.12e}")


def run_case(
    case_name: str,
    configuration_by_joint: dict[str, float],
    pin_model: pin.Model,
    mj_model: mujoco.MjModel,
    pin_data: pin.Data,
    mj_data: mujoco.MjData,
    addresses_by_joint: dict[str, JointAddress],
    pin_frame_ids: dict[str, int],
    mujoco_body_ids: dict[str, int],
) -> CaseResult:
    apply_named_arm_configuration(
        pin_model,
        mj_model,
        pin_data,
        mj_data,
        addresses_by_joint,
        configuration_by_joint,
    )

    pinocchio_world_T_torso = pinocchio_world_transform(
        pin_data, pin_frame_ids[TORSO_FRAME]
    )
    mujoco_world_T_torso = mujoco_world_transform(mj_data, mujoco_body_ids[TORSO_FRAME])

    left = compare_relative_pose(
        pinocchio_world_T_torso,
        pinocchio_world_transform(pin_data, pin_frame_ids[LEFT_TARGET_FRAME]),
        mujoco_world_T_torso,
        mujoco_world_transform(mj_data, mujoco_body_ids[LEFT_TARGET_FRAME]),
    )
    right = compare_relative_pose(
        pinocchio_world_T_torso,
        pinocchio_world_transform(pin_data, pin_frame_ids[RIGHT_TARGET_FRAME]),
        mujoco_world_T_torso,
        mujoco_world_transform(mj_data, mujoco_body_ids[RIGHT_TARGET_FRAME]),
    )

    print(f"\n=== {case_name} ===")
    print_case_configuration(configuration_by_joint)
    print_arm_result("Left", LEFT_TARGET_FRAME, left)
    print_arm_result("Right", RIGHT_TARGET_FRAME, right)
    return CaseResult(name=case_name, left=left, right=right)


def print_summary(results: list[CaseResult]) -> None:
    max_left_position = max(result.left.position_error_m for result in results)
    max_left_rotation = max(result.left.rotation_error_rad for result in results)
    max_right_position = max(result.right.position_error_m for result in results)
    max_right_rotation = max(result.right.rotation_error_rad for result in results)

    print("\n=== Summary: measured error, no acceptance threshold applied ===")
    print(f"max left position error [m]: {max_left_position:.12e}")
    print(f"max left rotation error [rad]: {max_left_rotation:.12e}")
    print(f"max left rotation error [deg]: {np.degrees(max_left_rotation):.12e}")
    print(f"max right position error [m]: {max_right_position:.12e}")
    print(f"max right rotation error [rad]: {max_right_rotation:.12e}")
    print(f"max right rotation error [deg]: {np.degrees(max_right_rotation):.12e}")


def main() -> None:
    print("EXP-003 — wheelloong_m2 URDF/MJCF FK consistency")
    print(f"Repository root: {REPOSITORY_ROOT}")
    print(f"URDF: {URDF_PATH}")
    print(f"MJCF: {MJCF_PATH}")
    print(f"Pinocchio version: {pin.__version__}")
    print(f"MuJoCo version: {mujoco.__version__}")
    print(f"NumPy version: {np.__version__}")

    urdf_link_names, urdf_joints = parse_urdf(URDF_PATH)
    mjcf_body_names, mjcf_joints = parse_mjcf(MJCF_PATH)
    pin_model = pin.buildModelFromUrdf(str(URDF_PATH))
    mj_model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    pin_data = pin_model.createData()
    mj_data = mujoco.MjData(mj_model)

    print_loaded_model_metadata(pin_model, mj_model)
    print_source_model_comparison(
        urdf_joints,
        mjcf_joints,
        urdf_link_names,
        mjcf_body_names,
    )
    addresses_by_joint = require_named_joint_addresses(
        pin_model,
        mj_model,
        ARM_JOINT_NAMES,
    )
    pin_frame_ids, mujoco_body_ids = require_target_frames(pin_model, mj_model)
    print_named_addresses(addresses_by_joint)
    print("Required target frame/body addresses:")
    for name in TARGET_FRAME_NAMES:
        print(
            f"  {name}: Pinocchio frame={pin_frame_ids[name]}, "
            f"MuJoCo body={mujoco_body_ids[name]}"
        )

    limits_by_joint = validated_arm_limits(urdf_joints, mjcf_joints)
    results = [
        run_case(
            case_name,
            configuration_by_joint,
            pin_model,
            mj_model,
            pin_data,
            mj_data,
            addresses_by_joint,
            pin_frame_ids,
            mujoco_body_ids,
        )
        for case_name, configuration_by_joint in build_test_configurations(limits_by_joint)
    ]
    print_summary(results)


if __name__ == "__main__":
    main()