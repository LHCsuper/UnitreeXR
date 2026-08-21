#!/usr/bin/env python3
"""Mirror `/arm_driver/get_info` into a MuJoCo viewer.

This viewer does not run the physics controller itself. It only loads the
controlled M2 model and mirrors the latest arm joint feedback published by
`arm_driver` so you can watch the robot move while `fake_xr_node` drives the
system.
"""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import mujoco
import mujoco.viewer
import numpy as np
import yaml

try:
    import rclpy
    from rclpy.node import Node
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit(
        "rclpy is required. Source your ROS 2 workspace before running this script."
    ) from exc

from arm_common.msg import ArmInfo
from remote_common.msg import RemoteInfo


def default_model_path() -> str:
    env_path = os.environ.get("WHEELLOONG_M2_MUJOCO_MODEL")
    if env_path:
        return env_path

    candidates: list[Path] = []
    try:
        candidates.append(
            Path(get_package_share_directory("wheelloong_m2")) /
            "mujoco" /
            "wheelloong_m2_controlled.xml"
        )
    except PackageNotFoundError:
        pass

    candidates.append(
        Path(__file__).resolve().parents[1] /
        "mujoco" /
        "wheelloong_m2_controlled.xml"
    )

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(candidates[-1])


def find_wheelloong_root(start: Path) -> Path:
    for path in [start, *start.parents]:
        if path.name == "wheelloong":
            return path
    raise RuntimeError(f"cannot locate wheelloong root from {start}")


def default_motion_config_path() -> Path:
    wheelloong_root = find_wheelloong_root(Path(__file__).resolve())
    return wheelloong_root.parent / WHEELLOONG_CONFIG_DIR_NAME / MOTION_CONFIG_RELATIVE_PATH


DEFAULT_TOPIC = "/arm_driver/get_info"
DEFAULT_REMOTE_TOPIC = "/teleop_xr_node/state"
LEFT_EE_BODY = "left_arm_link_7"
RIGHT_EE_BODY = "right_arm_link_7"
FRAME_AXIS_LENGTH_M = 0.12
FRAME_AXIS_WIDTH_M = 0.012
AXIS_X_RGBA = np.array([1.0, 0.0, 0.0, 1.0], dtype=np.float32)
AXIS_Y_RGBA = np.array([0.0, 1.0, 0.0, 1.0], dtype=np.float32)
AXIS_Z_RGBA = np.array([0.0, 0.1, 1.0, 1.0], dtype=np.float32)
PICO_AXIS_ALPHA = 0.65
WHEELLOONG_CONFIG_DIR_NAME = "wheelloong_config"
MOTION_CONFIG_RELATIVE_PATH = Path("configs/motion_config/motion_config.yaml")
TOOL_CONFIG_ARM = "mujoco"
TOOL_CONFIG_NAME = "zhixing"
BASE_CONFIG_NAME = "body"
MOTION_CONFIG_TRANSLATION_TO_M = 0.001
LEFT_ARM_JOINTS = [
    "left_arm_joint_1",
    "left_arm_joint_2",
    "left_arm_joint_3",
    "left_arm_joint_4",
    "left_arm_joint_5",
    "left_arm_joint_6",
    "left_arm_joint_7",
]
RIGHT_ARM_JOINTS = [
    "right_arm_joint_1",
    "right_arm_joint_2",
    "right_arm_joint_3",
    "right_arm_joint_4",
    "right_arm_joint_5",
    "right_arm_joint_6",
    "right_arm_joint_7",
]
# User Instruction, 2026-08-21: initial posture supplied as left/right MoveJ
# arrays. Array index 0..6 is applied directly to named arm joint 1..7 below.
HOME_JOINTS_RAD = {
    "left": (
        -1.5707963,
        1.2217305,
        1.5707963,
        -1.5707963,
        1.5707963,
        0.0,
        0.0,
    ),
    "right": (
        1.5707963,
        1.2217305,
        -1.5707963,
        -1.5707963,
        -1.5707963,
        0.0,
        0.0,
    ),
}


@dataclass(frozen=True)
class FixedTransform:
    name: str
    offset_m: np.ndarray
    quat_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class FixedTransforms:
    left: FixedTransform
    right: FixedTransform


@dataclass(frozen=True)
class MotionFrameConfig:
    config_path: Path
    arm_name: str
    tool_name: str
    base_name: str
    tools: FixedTransforms
    bases: FixedTransforms


@dataclass(frozen=True)
class JointBinding:
    name: str
    qpos_index: int


class ArmInfoMirror(Node):
    def __init__(self, topic_name: str, remote_topic_name: str) -> None:
        super().__init__("mujoco_arm_info_mirror")
        self._latest_msg: ArmInfo | None = None
        self._latest_remote_msg: RemoteInfo | None = None
        self._msg_count = 0
        self._remote_msg_count = 0
        self._subscriber = self.create_subscription(
            ArmInfo,
            topic_name,
            self._on_arm_info,
            10,
        )
        self._remote_subscriber = self.create_subscription(
            RemoteInfo,
            remote_topic_name,
            self._on_remote_info,
            10,
        )
        self.get_logger().info(f"subscribed to {topic_name}")
        self.get_logger().info(f"subscribed to {remote_topic_name}")

    def _on_arm_info(self, msg: ArmInfo) -> None:
        self._latest_msg = msg
        self._msg_count += 1

    def _on_remote_info(self, msg: RemoteInfo) -> None:
        self._latest_remote_msg = msg
        self._remote_msg_count += 1

    def latest(self) -> ArmInfo | None:
        return self._latest_msg

    def latest_remote(self) -> RemoteInfo | None:
        return self._latest_remote_msg

    @property
    def msg_count(self) -> int:
        return self._msg_count

    @property
    def remote_msg_count(self) -> int:
        return self._remote_msg_count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mirror arm_driver feedback into a MuJoCo viewer",
    )
    parser.add_argument(
        "--model-path",
        default=default_model_path(),
        help="Path to wheelloong_m2 controlled MuJoCo XML",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help="ArmInfo topic to mirror",
    )
    parser.add_argument(
        "--remote-topic",
        default=DEFAULT_REMOTE_TOPIC,
        help="RemoteInfo topic that carries PICO controller/headset poses",
    )
    parser.add_argument(
        "--coords-rate-hz",
        type=float,
        default=0.5,
        help="Coordinate print rate. Default 0.5 Hz means once every 2 seconds. Set <= 0 to disable coordinate output",
    )
    parser.add_argument(
        "--show-markers",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show MuJoCo/PICO coordinate markers in the viewer",
    )
    parser.add_argument(
        "--wait-first-feedback",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Wait for the first ArmInfo message before opening the viewer",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=60.0,
        help="Viewer refresh rate",
    )
    return parser.parse_args()


def require_path(path: Path, label: str) -> None:
    if not path.exists():
        raise FileNotFoundError(f"missing {label}: {path}")


def parse_fixed_transform(name: str, raw_pose: object) -> FixedTransform:
    if not isinstance(raw_pose, list) or len(raw_pose) != 7:
        raise ValueError(f"invalid pose for {name}: expected [x,y,z,qw,qx,qy,qz]")

    values = [float(value) for value in raw_pose]
    offset_m = np.array(values[:3], dtype=np.float64) * MOTION_CONFIG_TRANSLATION_TO_M
    quat_wxyz = tuple(values[3:7])
    return FixedTransform(
        name=name,
        offset_m=offset_m,
        quat_wxyz=(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]),
    )


def parse_fixed_transform_pair(name: str, raw_poses: object) -> FixedTransforms:
    if not isinstance(raw_poses, list) or len(raw_poses) != 2:
        raise ValueError(f"invalid {name}: expected left/right poses")

    return FixedTransforms(
        left=parse_fixed_transform(f"{name}.left", raw_poses[0]),
        right=parse_fixed_transform(f"{name}.right", raw_poses[1]),
    )


def load_motion_frame_config(config_path: Path) -> MotionFrameConfig:
    require_path(config_path, "motion config")
    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    try:
        arm_config = config[TOOL_CONFIG_ARM]
        tool_poses = arm_config["toolsLocalPos"][TOOL_CONFIG_NAME]
        base_poses = arm_config["armBasePos"][BASE_CONFIG_NAME]
    except (TypeError, KeyError) as exc:
        raise KeyError(
            f"missing {TOOL_CONFIG_ARM}.toolsLocalPos.{TOOL_CONFIG_NAME} or "
            f"{TOOL_CONFIG_ARM}.armBasePos.{BASE_CONFIG_NAME} in {config_path}"
        ) from exc

    return MotionFrameConfig(
        config_path=config_path,
        arm_name=TOOL_CONFIG_ARM,
        tool_name=TOOL_CONFIG_NAME,
        base_name=BASE_CONFIG_NAME,
        tools=parse_fixed_transform_pair(
            f"{TOOL_CONFIG_ARM}.toolsLocalPos.{TOOL_CONFIG_NAME}",
            tool_poses,
        ),
        bases=parse_fixed_transform_pair(
            f"{TOOL_CONFIG_ARM}.armBasePos.{BASE_CONFIG_NAME}",
            base_poses,
        ),
    )


def build_joint_bindings(model: mujoco.MjModel) -> list[JointBinding]:
    bindings: list[JointBinding] = []
    for joint_name in [*LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS]:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise RuntimeError(f"joint not found in model: {joint_name}")
        qpos_index = int(model.jnt_qposadr[joint_id])
        bindings.append(JointBinding(name=joint_name, qpos_index=qpos_index))
    return bindings


def require_body_id(model: mujoco.MjModel, body_name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise RuntimeError(f"body not found in model: {body_name}")
    return int(body_id)


def xyz_text(values: Iterable[float]) -> str:
    x, y, z = values
    return f"x={x:+.4f} y={y:+.4f} z={z:+.4f}"


def remote_xyz_m(values: Iterable[float]) -> tuple[float, float, float]:
    x, y, z, *_ = values
    return float(x) / 1000.0, float(y) / 1000.0, float(z) / 1000.0


def pose_to_transform(position: Iterable[float], rotation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(position, dtype=np.float64).reshape(3)
    return transform


def print_transform(label: str, transform: np.ndarray, tag: str = "end_T") -> None:
    print(f"[{tag}] {label}")
    for row in np.asarray(transform, dtype=np.float64).reshape(4, 4):
        values = "  ".join(f"{value:+.6f}" for value in row)
        print(f"  [{values}]")


def fixed_transform_to_pose(transform: FixedTransform) -> tuple[np.ndarray, np.ndarray]:
    return transform.offset_m, quat_wxyz_to_mat(transform.quat_wxyz)


def fixed_transform_to_matrix(transform: FixedTransform) -> np.ndarray:
    position, rotation = fixed_transform_to_pose(transform)
    return pose_to_transform(position, rotation)


def print_startup_base_transforms(frames: MotionFrameConfig) -> None:
    print("\n===== fixed base transforms =====")
    print(
        "[base_config] "
        f"config={frames.config_path} "
        f"arm={frames.arm_name} "
        f"base={frames.base_name}"
    )
    print_transform(
        "left T_mujoco_worldbase_bodybase",
        fixed_transform_to_matrix(frames.bases.left),
        "base_T",
    )
    print_transform(
        "right T_mujoco_worldbase_bodybase",
        fixed_transform_to_matrix(frames.bases.right),
        "base_T",
    )


def print_coordinates(
    data: mujoco.MjData,
    left_body_id: int,
    right_body_id: int,
    frames: MotionFrameConfig,
    arm_msg: ArmInfo | None,
    remote_msg: RemoteInfo | None,
) -> None:
    left_ee, left_ee_rot = link7_to_tool_pose(
        data.xpos[left_body_id],
        np.asarray(data.xmat[left_body_id], dtype=np.float64).reshape(3, 3),
        frames.tools.left,
    )
    right_ee, right_ee_rot = link7_to_tool_pose(
        data.xpos[right_body_id],
        np.asarray(data.xmat[right_body_id], dtype=np.float64).reshape(3, 3),
        frames.tools.right,
    )

    print("\n===== mujoco arm state =====")
    if arm_msg is not None:
        print(
            "[arm_info] "
            f"heartbeat={arm_msg.heartbeat} "
            f"power_on={arm_msg.power_on} "
            f"enable={arm_msg.enable_state} "
            f"servo={arm_msg.servo_mode}"
        )
    print(
        "[end_xyz] "
        f"left({xyz_text(left_ee)}) "
        f"right({xyz_text(right_ee)})"
    )
    print_transform("left", pose_to_transform(left_ee, left_ee_rot))
    print_transform("right", pose_to_transform(right_ee, right_ee_rot))
    if remote_msg is None:
        print("[pico] waiting for /teleop_xr_node/state")
        return

    print(
        "[pico] "
        f"left_dpose({xyz_text(remote_xyz_m(remote_msg.left_controller_dpose))}) "
        f"right_dpose({xyz_text(remote_xyz_m(remote_msg.right_controller_dpose))}) "
        f"headset({xyz_text(remote_xyz_m(remote_msg.headset_pose))}) "
        f"grips=({remote_msg.grips[0]:.2f}, {remote_msg.grips[1]:.2f})"
    )


def quat_wxyz_to_mat(quat: Iterable[float]) -> np.ndarray:
    w, x, y, z = [float(v) for v in quat]
    norm = np.linalg.norm([w, x, y, z])
    if not np.isfinite(norm) or norm < 1.0e-9:
        return np.eye(3, dtype=np.float64)

    w, x, y, z = [v / norm for v in (w, x, y, z)]
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def link7_to_tool_pose(
    link7_pos: Iterable[float],
    link7_rot: np.ndarray,
    tool: FixedTransform,
) -> tuple[np.ndarray, np.ndarray]:
    link7_pos = np.asarray(link7_pos, dtype=np.float64)
    link7_rot = np.asarray(link7_rot, dtype=np.float64).reshape(3, 3)
    tool_rot = link7_rot @ quat_wxyz_to_mat(tool.quat_wxyz)
    tool_pos = link7_pos + link7_rot @ tool.offset_m
    return tool_pos, tool_rot


def rgba_with_alpha(rgba: np.ndarray, alpha: float) -> np.ndarray:
    result = np.array(rgba, copy=True)
    result[3] = alpha
    return result


def add_axis_arrow(
    scene: mujoco.MjvScene,
    start: np.ndarray,
    end: np.ndarray,
    rgba: np.ndarray,
    width_m: float = FRAME_AXIS_WIDTH_M,
) -> None:
    if scene.ngeom >= scene.maxgeom:
        return

    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        width_m,
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64),
    )
    geom.rgba[:] = rgba
    scene.ngeom += 1


def add_frame_marker(
    scene: mujoco.MjvScene,
    origin: Iterable[float],
    rotation: np.ndarray,
    alpha: float = 1.0,
) -> None:
    origin = np.asarray(origin, dtype=np.float64)
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    colors = [
        rgba_with_alpha(AXIS_X_RGBA, alpha),
        rgba_with_alpha(AXIS_Y_RGBA, alpha),
        rgba_with_alpha(AXIS_Z_RGBA, alpha),
    ]
    for axis_index, rgba in enumerate(colors):
        end = origin + rotation[:, axis_index] * FRAME_AXIS_LENGTH_M
        add_axis_arrow(scene, origin, end, rgba)


def update_visual_markers(
    scene: mujoco.MjvScene,
    data: mujoco.MjData,
    left_body_id: int,
    right_body_id: int,
    frames: MotionFrameConfig,
    remote_msg: RemoteInfo | None,
) -> None:
    scene.ngeom = 0

    add_frame_marker(scene, np.zeros(3, dtype=np.float64), np.eye(3, dtype=np.float64))
    left_base_pos, left_base_rot = fixed_transform_to_pose(frames.bases.left)
    right_base_pos, right_base_rot = fixed_transform_to_pose(frames.bases.right)
    add_frame_marker(scene, left_base_pos, left_base_rot)
    add_frame_marker(scene, right_base_pos, right_base_rot)

    left_link7 = np.asarray(data.xpos[left_body_id], dtype=np.float64)
    right_link7 = np.asarray(data.xpos[right_body_id], dtype=np.float64)
    left_link7_rot = np.asarray(data.xmat[left_body_id], dtype=np.float64).reshape(3, 3)
    right_link7_rot = np.asarray(data.xmat[right_body_id], dtype=np.float64).reshape(3, 3)
    left_ee, left_ee_rot = link7_to_tool_pose(left_link7, left_link7_rot, frames.tools.left)
    right_ee, right_ee_rot = link7_to_tool_pose(right_link7, right_link7_rot, frames.tools.right)
    add_frame_marker(scene, left_ee, left_ee_rot)
    add_frame_marker(scene, right_ee, right_ee_rot)

    if remote_msg is None:
        return

    left_delta_rot = quat_wxyz_to_mat(remote_msg.left_controller_dpose[3:7])
    right_delta_rot = quat_wxyz_to_mat(remote_msg.right_controller_dpose[3:7])
    left_pico_target = left_ee + np.asarray(remote_xyz_m(remote_msg.left_controller_dpose))
    right_pico_target = right_ee + np.asarray(remote_xyz_m(remote_msg.right_controller_dpose))
    add_frame_marker(scene, left_pico_target, left_delta_rot @ left_ee_rot, PICO_AXIS_ALPHA)
    add_frame_marker(scene, right_pico_target, right_delta_rot @ right_ee_rot, PICO_AXIS_ALPHA)


def wait_for_first_feedback(node: ArmInfoMirror, topic_name: str) -> ArmInfo | None:
    last_warn_time = 0.0
    print(f"waiting for first feedback on {topic_name} before opening viewer ...")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        msg = node.latest()
        if msg is not None:
            print(f"received first feedback on {topic_name}; opening viewer")
            return msg

        now = time.time()
        if now - last_warn_time >= 2.0:
            print(f"waiting for {topic_name} ...")
            last_warn_time = now

    return None


def apply_home_posture(data: mujoco.MjData, bindings: Iterable[JointBinding]) -> None:
    qpos_values = [
        *HOME_JOINTS_RAD["left"],
        *HOME_JOINTS_RAD["right"],
    ]
    for binding, value in zip(bindings, qpos_values, strict=True):
        data.qpos[binding.qpos_index] = float(value)


def apply_arm_info(
    data: mujoco.MjData,
    bindings: Iterable[JointBinding],
    msg: ArmInfo,
) -> None:
    qpos_values = [
        *msg.left_joint,
        *msg.right_joint,
    ]
    for binding, value in zip(bindings, qpos_values, strict=True):
        data.qpos[binding.qpos_index] = float(value)


def main() -> int:
    args = parse_args()
    model_path = Path(args.model_path).expanduser().resolve()
    motion_config_path = default_motion_config_path().expanduser().resolve()
    require_path(model_path, "MuJoCo model")
    frames = load_motion_frame_config(motion_config_path)

    rclpy.init()
    node = ArmInfoMirror(args.topic, args.remote_topic)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    bindings = build_joint_bindings(model)
    left_ee_body_id = require_body_id(model, LEFT_EE_BODY)
    right_ee_body_id = require_body_id(model, RIGHT_EE_BODY)
    apply_home_posture(data, bindings)
    mujoco.mj_forward(model, data)

    refresh_dt = 1.0 / max(args.rate_hz, 1.0)
    coords_dt = 1.0 / args.coords_rate_hz if args.coords_rate_hz > 0.0 else None
    last_warn_time = 0.0
    last_coords_time = 0.0

    print(f"model_path: {model_path}")
    print(f"topic: {args.topic}")
    print(f"remote_topic: {args.remote_topic}")
    print(f"coordinate bodies: left={LEFT_EE_BODY} right={RIGHT_EE_BODY}")
    print(
        "coordinate display: "
        f"link_7 * {frames.arm_name}.toolsLocalPos.{frames.tool_name} "
        f"config={frames.config_path} "
        f"left_tool_offset_m={frames.tools.left.offset_m.tolist()} "
        f"left_tool_quat_wxyz={frames.tools.left.quat_wxyz} "
        f"right_tool_offset_m={frames.tools.right.offset_m.tolist()} "
        f"right_tool_quat_wxyz={frames.tools.right.quat_wxyz} "
        f"base={frames.arm_name}.armBasePos.{frames.base_name} "
        f"left_base_offset_m={frames.bases.left.offset_m.tolist()} "
        f"left_base_quat_wxyz={frames.bases.left.quat_wxyz} "
        f"right_base_offset_m={frames.bases.right.offset_m.tolist()} "
        f"right_base_quat_wxyz={frames.bases.right.quat_wxyz}"
    )
    print_startup_base_transforms(frames)

    try:
        if args.wait_first_feedback:
            first_msg = wait_for_first_feedback(node, args.topic)
            if first_msg is None:
                return 1
            apply_arm_info(data, bindings, first_msg)
            mujoco.mj_forward(model, data)
        else:
            print("opening viewer with local home posture; waiting for arm_driver feedback ...")

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                msg = node.latest()
                if msg is None:
                    now = time.time()
                    if now - last_warn_time >= 2.0:
                        print("waiting for /arm_driver/get_info ...")
                        last_warn_time = now
                else:
                    apply_arm_info(data, bindings, msg)
                    mujoco.mj_forward(model, data)
                if coords_dt is not None:
                    now = time.time()
                    if now - last_coords_time >= coords_dt:
                        print_coordinates(
                            data,
                            left_ee_body_id,
                            right_ee_body_id,
                            frames,
                            node.latest(),
                            node.latest_remote(),
                        )
                        last_coords_time = now
                if args.show_markers:
                    update_visual_markers(
                        viewer.user_scn,
                        data,
                        left_ee_body_id,
                        right_ee_body_id,
                        frames,
                        node.latest_remote(),
                    )
                viewer.sync()
                time.sleep(refresh_dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
