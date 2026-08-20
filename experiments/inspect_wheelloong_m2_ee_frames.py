#!/usr/bin/env python3
"""EXP-004: inspect wheelloong_m2 end-effector (gripper) frame candidates.

Scope: model inspection only. This script does NOT implement IK, does NOT
connect XR, does NOT control a robot, and does NOT modify the URDF/MJCF.

It reads the checked-in URDF, loads the model with Pinocchio, and reports:
- the torso-relative poses of ``left_arm_link_7`` / ``right_arm_link_7``;
- the fixed transform from each ``*_arm_link_7`` frame to every directly
  attached gripper joint origin;
- a geometric center candidate derived only from those joint origins.

Notation: ``^a T_b`` is the homogeneous transform taking coordinates from
frame ``b`` to frame ``a``; ``^a R_b`` is its rotation part; ``^a p_b`` is the
translation part.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPOSITORY_ROOT / "src/description/wheelloong_m2/urdf/wheelloong_m2.urdf"
PACKAGE_DIR = REPOSITORY_ROOT / "src" / "description"

ARMS = {
    "left": {
        "link7": "left_arm_link_7",
        "gripper_prefix": "left_gripper",
    },
    "right": {
        "link7": "right_arm_link_7",
        "gripper_prefix": "right_gripper",
    },
}


@dataclass(frozen=True)
class JointSource:
    """One joint read directly from the URDF source."""

    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    mimic_joint: str | None
    mimic_multiplier: float | None


def parse_numbers(raw_value: str | None, default: str) -> np.ndarray:
    return np.fromstring(raw_value or default, sep=" ", dtype=float)


def parse_urdf(path: Path) -> tuple[set[str], dict[str, JointSource]]:
    root = ET.parse(path).getroot()
    link_names = {node.attrib["name"] for node in root.findall("link")}
    joints: dict[str, JointSource] = {}

    for joint_node in root.findall("joint"):
        name = joint_node.attrib["name"]
        origin_node = joint_node.find("origin")
        axis_node = joint_node.find("axis")
        parent_node = joint_node.find("parent")
        child_node = joint_node.find("child")
        mimic_node = joint_node.find("mimic")

        mimic_joint = None
        mimic_multiplier = None
        if mimic_node is not None:
            mimic_joint = mimic_node.attrib.get("joint")
            mimic_multiplier = float(mimic_node.attrib.get("multiplier", "1"))

        joints[name] = JointSource(
            name=name,
            joint_type=joint_node.attrib["type"],
            parent="" if parent_node is None else parent_node.attrib["link"],
            child="" if child_node is None else child_node.attrib["link"],
            origin_xyz=parse_numbers(
                None if origin_node is None else origin_node.attrib.get("xyz"),
                "0 0 0",
            ),
            origin_rpy=parse_numbers(
                None if origin_node is None else origin_node.attrib.get("rpy"),
                "0 0 0",
            ),
            axis=parse_numbers(
                None if axis_node is None else axis_node.attrib.get("xyz"),
                "0 0 1",
            ),
            mimic_joint=mimic_joint,
            mimic_multiplier=mimic_multiplier,
        )

    return link_names, joints


def format_vector(vector: np.ndarray) -> str:
    return np.array2string(np.asarray(vector, dtype=float), precision=6, suppress_small=True)


def format_transform(transform: np.ndarray) -> str:
    return np.array2string(transform, precision=6, suppress_small=True)


def rpy_to_rotation_matrix(rpy: np.ndarray) -> np.ndarray:
    """URDF fixed-axis convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = [float(value) for value in rpy]
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def origin_to_transform(joint: JointSource) -> np.ndarray:
    """^parent T_joint built from the URDF joint origin."""
    transform = np.eye(4)
    transform[:3, :3] = rpy_to_rotation_matrix(joint.origin_rpy)
    transform[:3, 3] = joint.origin_xyz
    return transform


def print_structure_tree(
    joints: dict[str, JointSource],
    link7: str,
    prefix: str,
) -> None:
    direct_children = {
        name: joint
        for name, joint in joints.items()
        if joint.parent == link7
    }
    secondary_children = {
        name: joint
        for name, joint in joints.items()
        if joint.parent not in {link7} and joint.parent.startswith(prefix)
    }

    def mimic_text(joint: JointSource) -> str:
        if joint.mimic_joint is None:
            return ""
        return (
            f" mimic({joint.mimic_joint}, mult={joint.mimic_multiplier:g})"
        )

    print(link7)
    direct_names = sorted(direct_children, key=lambda n: int(n.rsplit("_", 1)[1]))
    for index, name in enumerate(direct_names):
        joint = direct_children[name]
        is_last = index == len(direct_names) - 1
        branch = "└──" if is_last else "├──"
        print(
            f"{branch} {name} -> {joint.child} "
            f"origin={format_vector(joint.origin_xyz)} "
            f"rpy={format_vector(joint.origin_rpy)} "
            f"axis={format_vector(joint.axis)} "
            f"type={joint.joint_type}{mimic_text(joint)}"
        )
        child_joint = secondary_children.get(
            _joint_with_parent(secondary_children, joint.child)
        )
        if child_joint is not None:
            print(
                f"    └── {child_joint.name} -> {child_joint.child} "
                f"origin={format_vector(child_joint.origin_xyz)} "
                f"rpy={format_vector(child_joint.origin_rpy)} "
                f"axis={format_vector(child_joint.axis)} "
                f"type={child_joint.joint_type}{mimic_text(child_joint)}"
            )


def _joint_with_parent(
    joints: dict[str, JointSource],
    parent_link: str,
) -> str | None:
    for name, joint in joints.items():
        if joint.parent == parent_link:
            return name
    return None


def inverse_transform(from_T_to: np.ndarray) -> np.ndarray:
    from_R_to = from_T_to[:3, :3]
    from_p_to = from_T_to[:3, 3]
    to_T_from = np.eye(4)
    to_T_from[:3, :3] = from_R_to.T
    to_T_from[:3, 3] = -from_R_to.T @ from_p_to
    return to_T_from


def placement_to_transform(placement: pin.SE3) -> np.ndarray:
    return placement.homogeneous.copy()


def direct_gripper_joints(
    joints: dict[str, JointSource],
    link7: str,
) -> list[JointSource]:
    ordered = sorted(
        (joint for joint in joints.values() if joint.parent == link7),
        key=lambda joint: int(joint.name.rsplit("_", 1)[1]),
    )
    return ordered


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="open a MeshCat viewer and draw the inspected frames",
    )
    args = parser.parse_args()

    print("EXP-004 — wheelloong_m2 end-effector frame inspection")
    print(f"URDF: {URDF_PATH}")
    print(f"Pinocchio version: {pin.__version__}")
    print(f"NumPy version: {np.__version__}")
    print(f"Python version: {sys.version.split()[0]}")
    print()

    link_names, joints = parse_urdf(URDF_PATH)
    model = pin.buildModelFromUrdf(str(URDF_PATH))
    data = model.createData()

    q_neutral = pin.neutral(model)
    pin.forwardKinematics(model, data, q_neutral)
    pin.updateFramePlacements(model, data)

    # ------------------------------------------------------------------
    # Structure trees
    # ------------------------------------------------------------------
    print("=== Robot-side gripper structure trees (source evidence) ===")
    print()
    print_structure_tree(joints, ARMS["left"]["link7"], ARMS["left"]["gripper_prefix"])
    print()
    print_structure_tree(joints, ARMS["right"]["link7"], ARMS["right"]["gripper_prefix"])
    print()

    # ------------------------------------------------------------------
    # torso-relative arm_link_7 poses
    # ------------------------------------------------------------------
    print("=== ^torso T_arm_link_7 at neutral configuration ===")
    torso_frame_id = model.getFrameId("torso_link", pin.FrameType.BODY)
    world_T_torso = placement_to_transform(data.oMf[torso_frame_id])

    results: dict[str, dict] = {}
    for side, names in ARMS.items():
        link7 = names["link7"]
        frame_id = model.getFrameId(link7, pin.FrameType.BODY)
        world_T_link7 = placement_to_transform(data.oMf[frame_id])
        torso_T_link7 = inverse_transform(world_T_torso) @ world_T_link7
        print(f"  ^torso T_{link7}:")
        print(f"    {format_transform(torso_T_link7)}")
        results[side] = {
            "link7": link7,
            "torso_T_link7": torso_T_link7,
            "world_T_link7": world_T_link7,
        }
    print()

    # ------------------------------------------------------------------
    # Fixed transforms: ^arm_link_7 T_gripper_joint_origin
    # ------------------------------------------------------------------
    print("=== Fixed transforms ^arm_link_7 T_gripper_joint_origin ===")
    direct_origins: dict[str, list[np.ndarray]] = {}
    for side, names in ARMS.items():
        link7 = names["link7"]
        frame_id = model.getFrameId(link7, pin.FrameType.BODY)
        world_T_link7 = placement_to_transform(data.oMf[frame_id])
        link7_T_world = inverse_transform(world_T_link7)

        print(f"  {side.upper()} arm (parent frame: {link7})")
        direct_joints = direct_gripper_joints(joints, link7)
        origins: list[np.ndarray] = []
        for joint in direct_joints:
            joint_id = model.getJointId(joint.name)
            world_T_joint_origin = placement_to_transform(data.oMi[joint_id])
            link7_T_joint = link7_T_world @ world_T_joint_origin
            origin_position = link7_T_joint[:3, 3]
            origins.append(origin_position)

            source_transform = origin_to_transform(joint)
            source_position = source_transform[:3, 3]
            print(
                f"    {joint.name}: ^arm7 p = {format_vector(origin_position)} "
                f"(URDF xyz = {format_vector(source_position)})"
            )
            print(f"      ^arm7 R = {format_vector(link7_T_joint[:3, :3].flatten())}")
            print(
                f"      axis (child frame) = {format_vector(joint.axis)} | "
                f"type={joint.joint_type}"
            )
        direct_origins[side] = origins
        print()

    # ------------------------------------------------------------------
    # Geometric center candidates
    # ------------------------------------------------------------------
    print("=== Geometric gripper-center candidates (NOT confirmed TCP) ===")
    for side, names in ARMS.items():
        link7 = names["link7"]
        origins = direct_origins[side]
        stacked = np.vstack(origins)
        center = stacked.mean(axis=0)

        print(f"  {side.upper()} gripper")
        for index, origin in enumerate(origins, start=1):
            print(f"    ^arm7 p_direct_joint_{index} = {format_vector(origin)}")
        print(f"    ^arm7 p_candidate (mean of direct joint origins) = {format_vector(center)}")
        print(f"    ||^arm7 p_candidate|| = {np.linalg.norm(center):.6f} m")

        # Pairwise midpoints to expose the symmetry structure.
        if len(origins) == 4:
            mid_rows = [
                (origins[0] + origins[1]) / 2.0,
                (origins[2] + origins[3]) / 2.0,
            ]
            mid_jaws = [
                (origins[0] + origins[2]) / 2.0,
                (origins[1] + origins[3]) / 2.0,
            ]
            print(
                f"    pairwise row midpoints: {format_vector(mid_rows[0])}, "
                f"{format_vector(mid_rows[1])}"
            )
            print(
                f"    pairwise jaw midpoints: {format_vector(mid_jaws[0])}, "
                f"{format_vector(mid_jaws[1])}"
            )

        print(f"    candidate center world transform = ^world T_{side}_candidate_pos")
        world_T_candidate = results[side]["world_T_link7"].copy()
        world_T_candidate[:3, 3] = (
            results[side]["world_T_link7"][:3, :3] @ center
            + results[side]["world_T_link7"][:3, 3]
        )
        print(f"      {format_transform(world_T_candidate)}")
        print()
        results[side]["candidate_center_arm7"] = center
        results[side]["candidate_center_world"] = world_T_candidate[:3, 3]
        results[side]["direct_origins"] = origins

    # ------------------------------------------------------------------
    # Orientation analysis (axes of candidate frame)
    # ------------------------------------------------------------------
    print("=== Candidate EE orientation: geometric inference ===")
    for side in ("left", "right"):
        origins = direct_origins[side]
        # All direct gripper joints share axis +z in their child frame.
        joint_axes = [
            joints[j.name].axis
            for j in direct_gripper_joints(joints, ARMS[side]["link7"])
        ]
        print(f"  {side.upper()} gripper:")
        print(f"    direct joint axes: {[format_vector(a) for a in joint_axes]}")
        # Finger extension direction: child links extend in +y (link_2/5 offset +y).
        # Jaw closing direction: mounting points spread along x.
        span_x = float(np.ptp([origin[0] for origin in origins]))
        span_y = float(np.ptp([origin[1] for origin in origins]))
        span_z = float(np.ptp([origin[2] for origin in origins]))
        print(
            f"    mounting-point spread [x, y, z]: "
            f"[{span_x:.6f}, {span_y:.6f}, {span_z:.6f}] m"
        )
        print(
            "    geometric inference: closing ~ x, extension ~ +y, "
            "normal ~ z (joint axis)."
        )
    print(
        "    Note: axis signs for closing (x) and the exact EE frame are NOT "
        "mechanically fixed; orientation remains a candidate, not a Confirmed Fact."
    )
    print()

    if args.visualize:
        visualize(results, model, data, q_neutral)


def visualize(
    results: dict,
    model: pin.Model,
    data: pin.Data,
    q_neutral: np.ndarray,
) -> None:
    try:
        import meshcat
        import meshcat.geometry as g
        import meshcat.transformations as tf
    except Exception as exc:  # pragma: no cover - depends on environment
        print(f"MeshCat unavailable, skipping visualization: {exc}")
        return

    vis = meshcat.Visualizer()
    vis.delete()
    print(f"MeshCat URL: {vis.url()}")

    AXIS_RADIUS = 0.004

    red = g.MeshLambertMaterial(color=0xFF0000)
    green = g.MeshLambertMaterial(color=0x00FF00)
    blue = g.MeshLambertMaterial(color=0x0000FF)
    yellow = g.MeshLambertMaterial(color=0xFFAA00)
    magenta = g.MeshLambertMaterial(color=0xFF00FF)

    def make_frame(name: str, axis_length: float) -> None:
        vis[f"frames/{name}/x"].set_object(g.Cylinder(axis_length, AXIS_RADIUS), red)
        vis[f"frames/{name}/y"].set_object(g.Cylinder(axis_length, AXIS_RADIUS), green)
        vis[f"frames/{name}/z"].set_object(g.Cylinder(axis_length, AXIS_RADIUS), blue)

    def set_frame(name: str, transform: np.ndarray, axis_length: float) -> None:
        r_y_to_x = tf.rotation_matrix(-np.pi / 2.0, [0, 0, 1])
        r_y_to_z = tf.rotation_matrix(np.pi / 2.0, [1, 0, 0])
        t_center_y = tf.translation_matrix([0, axis_length / 2.0, 0])
        vis[f"frames/{name}/x"].set_transform(transform @ r_y_to_x @ t_center_y)
        vis[f"frames/{name}/y"].set_transform(transform @ t_center_y)
        vis[f"frames/{name}/z"].set_transform(transform @ r_y_to_z @ t_center_y)

    def set_point(name: str, position: np.ndarray, material: g.Material, radius: float) -> None:
        vis[name].set_object(g.Sphere(radius), material)
        vis[name].set_transform(tf.translation_matrix(position))

    # Robot neutral pose meshes (best effort; frames are the required output).
    try:
        from pinocchio.visualize import MeshcatVisualizer

        geom_model = pin.buildGeomFromUrdf(
            model,
            str(URDF_PATH),
            pin.GeometryType.VISUAL,
            package_dirs=[str(PACKAGE_DIR)],
        )
        mesh_viz = MeshcatVisualizer(model, geom_model, geom_model)
        mesh_viz.initViewer(viewer=vis, loadModel=False)
        mesh_viz.loadViewerModel()
        mesh_viz.display(q_neutral)
    except Exception as exc:
        print(f"Robot mesh visualization skipped ({exc}); showing frames only.")

    # Frames.
    torso_frame_id = model.getFrameId("torso_link", pin.FrameType.BODY)
    world_T_torso = placement_to_transform(data.oMf[torso_frame_id])

    make_frame("torso_link", 0.20)
    set_frame("torso_link", world_T_torso, 0.20)

    for side in ("left", "right"):
        link7 = results[side]["link7"]
        make_frame(link7, 0.12)
        set_frame(link7, results[side]["world_T_link7"], 0.12)

        for index, origin in enumerate(results[side]["direct_origins"], start=1):
            world_pos = (
                results[side]["world_T_link7"][:3, :3] @ origin
                + results[side]["world_T_link7"][:3, 3]
            )
            set_point(
                f"gripper_origins/{side}_{index}",
                world_pos,
                yellow,
                0.006,
            )

        set_point(
            f"candidate_center/{side}",
            results[side]["candidate_center_world"],
            magenta,
            0.010,
        )

    print("Visualization ready. Candidate center is drawn as a position-only marker.")
    print("Orientation of the candidate frame is intentionally NOT drawn: it is Unknown.")


if __name__ == "__main__":
    main()