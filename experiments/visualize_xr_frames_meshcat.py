import math
import time
import webbrowser

import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

import xrobotoolkit_sdk as xrt


# ============================================================
# Configuration
# ============================================================

UPDATE_HZ = 60.0

WORLD_AXIS_LENGTH = 0.30
TRACKING_AXIS_LENGTH = 0.22
DEVICE_AXIS_LENGTH = 0.10

AXIS_RADIUS = 0.003


# ============================================================
# Fixed transform:
#
#       MeshCat World / future robot base_link {W}
#                         |
#                         |  ^W T_D
#                         v
#                 Tracking Origin {D}
#
# 修改这里即可调整 Tracking Origin 相对于 World 的固定位置。
#
# 当前先只设置 translation，不设置 rotation。
# 单位：m
# ============================================================

TRACKING_ORIGIN_POSITION_IN_WORLD = np.array([
    0.50,   # world X
    0.00,   # world Y
    0.50,   # world Z
], dtype=float)


# 当前 Tracking Origin 和 MeshCat World 的轴方向保持一致
R_WORLD_TRACKING = np.eye(3)


T_WORLD_TRACKING = np.eye(4)

T_WORLD_TRACKING[:3, :3] = R_WORLD_TRACKING
T_WORLD_TRACKING[:3, 3] = TRACKING_ORIGIN_POSITION_IN_WORLD


# ============================================================
# Quaternion
#
# XRoboToolkit pose:
#
# [x, y, z, qx, qy, qz, qw]
#
# 当前已经验证：
#
# pose = ^D T_object
#
# quaternion = ^D R_object
# ============================================================


def quat_xyzw_to_rotation_matrix(q):
    """
    q = [qx, qy, qz, qw]

    Return:
        ^D R_object
    """

    qx, qy, qz, qw = [float(v) for v in q]

    norm = math.sqrt(
        qx * qx
        + qy * qy
        + qz * qz
        + qw * qw
    )

    if norm < 1e-8:
        return None

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    R = np.array([
        [
            1.0 - 2.0 * (qy*qy + qz*qz),
            2.0 * (qx*qy - qz*qw),
            2.0 * (qx*qz + qy*qw),
        ],
        [
            2.0 * (qx*qy + qz*qw),
            1.0 - 2.0 * (qx*qx + qz*qz),
            2.0 * (qy*qz - qx*qw),
        ],
        [
            2.0 * (qx*qz - qy*qw),
            2.0 * (qy*qz + qx*qw),
            1.0 - 2.0 * (qx*qx + qy*qy),
        ],
    ], dtype=float)

    return R


def pose_to_transform(pose):
    """
    XRoboToolkit:

        pose = [x, y, z, qx, qy, qz, qw]

    Convert to:

        ^D T_object

    Returns None if pose is invalid.
    """

    pose = np.asarray(pose, dtype=float)

    if pose.shape != (7,):
        return None

    R = quat_xyzw_to_rotation_matrix(
        pose[3:7]
    )

    if R is None:
        return None

    T = np.eye(4)

    T[:3, :3] = R
    T[:3, 3] = pose[:3]

    return T


# ============================================================
# MeshCat coordinate frame
#
# X = Red
# Y = Green
# Z = Blue
# ============================================================


def create_coordinate_frame(
    vis,
    name,
    axis_length,
):
    """
    Create X/Y/Z cylinders for one coordinate frame.
    """

    red = g.MeshLambertMaterial(
        color=0xFF0000
    )

    green = g.MeshLambertMaterial(
        color=0x00FF00
    )

    blue = g.MeshLambertMaterial(
        color=0x0000FF
    )

    vis[f"frames/{name}/x"].set_object(
        g.Cylinder(
            axis_length,
            AXIS_RADIUS
        ),
        red
    )

    vis[f"frames/{name}/y"].set_object(
        g.Cylinder(
            axis_length,
            AXIS_RADIUS
        ),
        green
    )

    vis[f"frames/{name}/z"].set_object(
        g.Cylinder(
            axis_length,
            AXIS_RADIUS
        ),
        blue
    )


def set_coordinate_frame_transform(
    vis,
    name,
    T_frame,
    axis_length,
):
    """
    Set coordinate-frame pose.

    MeshCat Cylinder is along local Y by default.

    Therefore:

    cylinder Y -> desired X/Y/Z
    """

    # --------------------------------------------------------
    # +X
    # --------------------------------------------------------

    R_y_to_x = tf.rotation_matrix(
        -np.pi / 2.0,
        [0, 0, 1]
    )

    T_center_y = tf.translation_matrix(
        [0, axis_length / 2.0, 0]
    )

    T_x = (
        T_frame
        @ R_y_to_x
        @ T_center_y
    )

    # --------------------------------------------------------
    # +Y
    # --------------------------------------------------------

    T_y = (
        T_frame
        @ T_center_y
    )

    # --------------------------------------------------------
    # +Z
    # --------------------------------------------------------

    R_y_to_z = tf.rotation_matrix(
        np.pi / 2.0,
        [1, 0, 0]
    )

    T_z = (
        T_frame
        @ R_y_to_z
        @ T_center_y
    )

    vis[f"frames/{name}/x"].set_transform(
        T_x
    )

    vis[f"frames/{name}/y"].set_transform(
        T_y
    )

    vis[f"frames/{name}/z"].set_transform(
        T_z
    )


# ============================================================
# Print helper
# ============================================================


def print_pose(name, pose):
    pose = np.asarray(
        pose,
        dtype=float
    )

    p = pose[:3]
    q = pose[3:7]

    print(
        f"{name:6s} "
        f"p=["
        f"{p[0]:+.3f}, "
        f"{p[1]:+.3f}, "
        f"{p[2]:+.3f}] "
        f"q=["
        f"{q[0]:+.3f}, "
        f"{q[1]:+.3f}, "
        f"{q[2]:+.3f}, "
        f"{q[3]:+.3f}]"
    )


# ============================================================
# Main
# ============================================================


def main():

    # --------------------------------------------------------
    # Start MeshCat
    # --------------------------------------------------------

    vis = meshcat.Visualizer()

    vis.delete()

    url = vis.url()

    print()
    print("=" * 80)
    print("MeshCat URL:")
    print(url)
    print("=" * 80)
    print()

    try:
        webbrowser.open(url)
    except Exception:
        pass

    # --------------------------------------------------------
    # Create frames
    #
    # World
    #   |
    #   +-- TrackingOrigin
    #          |
    #          +-- Head
    #          +-- Left
    #          +-- Right
    #
    # Scene graph here is visually flat,
    # but transforms obey this mathematical relationship.
    # --------------------------------------------------------

    create_coordinate_frame(
        vis,
        "World",
        WORLD_AXIS_LENGTH
    )

    create_coordinate_frame(
        vis,
        "TrackingOrigin",
        TRACKING_AXIS_LENGTH
    )

    create_coordinate_frame(
        vis,
        "Head",
        DEVICE_AXIS_LENGTH
    )

    create_coordinate_frame(
        vis,
        "Left",
        DEVICE_AXIS_LENGTH
    )

    create_coordinate_frame(
        vis,
        "Right",
        DEVICE_AXIS_LENGTH
    )

    # --------------------------------------------------------
    # MeshCat World / future base_link
    #
    # ^W T_W = I
    # --------------------------------------------------------

    T_WORLD_WORLD = np.eye(4)

    set_coordinate_frame_transform(
        vis,
        "World",
        T_WORLD_WORLD,
        WORLD_AXIS_LENGTH
    )

    # --------------------------------------------------------
    # Tracking Origin
    #
    # ^W T_D = fixed transform
    # --------------------------------------------------------

    set_coordinate_frame_transform(
        vis,
        "TrackingOrigin",
        T_WORLD_TRACKING,
        TRACKING_AXIS_LENGTH
    )

    print("Frame hierarchy:")
    print()
    print("World / future base_link {W}")
    print("    |")
    print("    | fixed ^W T_D")
    print("    v")
    print("TrackingOrigin {D}")
    print("    |")
    print("    +-- Head")
    print("    +-- Left")
    print("    +-- Right")
    print()

    print("^W T_D =")
    print(T_WORLD_TRACKING)
    print()

    print(
        "Tracking origin position in world =",
        TRACKING_ORIGIN_POSITION_IN_WORLD
    )

    print()

    # --------------------------------------------------------
    # XRoboToolkit
    # --------------------------------------------------------

    print("Initializing XRoboToolkit...")

    xrt.init()

    time.sleep(2.0)

    print("XRoboToolkit initialized.")
    print()

    print("Coordinate colors:")
    print("  X = RED")
    print("  Y = GREEN")
    print("  Z = BLUE")
    print()

    print("Ctrl+C to stop.")
    print()

    period = 1.0 / UPDATE_HZ

    last_print_time = 0.0

    try:

        while True:

            loop_start = time.monotonic()

            # =================================================
            # Read raw XR poses
            #
            # ^D T_H
            # ^D T_L
            # ^D T_R
            # =================================================

            head_pose = list(
                xrt.get_headset_pose()
            )

            left_pose = list(
                xrt.get_left_controller_pose()
            )

            right_pose = list(
                xrt.get_right_controller_pose()
            )

            # =================================================
            # Raw XR pose -> SE(3)
            # =================================================

            T_TRACKING_HEAD = pose_to_transform(
                head_pose
            )

            T_TRACKING_LEFT = pose_to_transform(
                left_pose
            )

            T_TRACKING_RIGHT = pose_to_transform(
                right_pose
            )

            # =================================================
            # Transform to MeshCat world
            #
            # ^W T_object
            #
            # =
            #
            # ^W T_D
            #
            # *
            #
            # ^D T_object
            # =================================================

            if T_TRACKING_HEAD is not None:

                T_WORLD_HEAD = (
                    T_WORLD_TRACKING
                    @ T_TRACKING_HEAD
                )

                set_coordinate_frame_transform(
                    vis,
                    "Head",
                    T_WORLD_HEAD,
                    DEVICE_AXIS_LENGTH
                )

            if T_TRACKING_LEFT is not None:

                T_WORLD_LEFT = (
                    T_WORLD_TRACKING
                    @ T_TRACKING_LEFT
                )

                set_coordinate_frame_transform(
                    vis,
                    "Left",
                    T_WORLD_LEFT,
                    DEVICE_AXIS_LENGTH
                )

            if T_TRACKING_RIGHT is not None:

                T_WORLD_RIGHT = (
                    T_WORLD_TRACKING
                    @ T_TRACKING_RIGHT
                )

                set_coordinate_frame_transform(
                    vis,
                    "Right",
                    T_WORLD_RIGHT,
                    DEVICE_AXIS_LENGTH
                )

            # =================================================
            # Print raw XR data once per second
            # =================================================

            now = time.monotonic()

            if (
                now - last_print_time
                >= 1.0
            ):

                print("-" * 80)

                print_pose(
                    "HEAD",
                    head_pose
                )

                print_pose(
                    "LEFT",
                    left_pose
                )

                print_pose(
                    "RIGHT",
                    right_pose
                )

                last_print_time = now

            # =================================================
            # Maintain loop rate
            # =================================================

            elapsed = (
                time.monotonic()
                - loop_start
            )

            remaining = (
                period
                - elapsed
            )

            if remaining > 0:
                time.sleep(
                    remaining
                )

    except KeyboardInterrupt:

        print()
        print("Stopped by user.")

    finally:

        xrt.close()

        print(
            "XRoboToolkit closed."
        )


if __name__ == "__main__":
    main()