from ast import literal_eval
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "wheelloong_m2"
DEFAULT_LEFT_ARM_INITIAL = "[1.570796327, -1.221730476, -1.570796327, -1.570796327, 0.0, 0.0, 0.0]"
DEFAULT_RIGHT_ARM_INITIAL = "[-1.570796327, -1.221730476, 1.570796327, -1.570796327, 0.0, 0.0, 0.0]"
ARM_JOINT_NAMES = (
    ("left_arm_joint_1", "left_arm_joint_2", "left_arm_joint_3", "left_arm_joint_4", "left_arm_joint_5", "left_arm_joint_6", "left_arm_joint_7"),
    ("right_arm_joint_1", "right_arm_joint_2", "right_arm_joint_3", "right_arm_joint_4", "right_arm_joint_5", "right_arm_joint_6", "right_arm_joint_7"),
)


def _read_required_text(path: Path, label: str):
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path.read_text(encoding="utf-8")


def _robot_description_parameter(robot_description):
    return {"robot_description": ParameterValue(robot_description, value_type=str)}


def _parse_initial_joint_values(raw_value, side_name):
    try:
        values = literal_eval(raw_value)
    except Exception as exc:
        raise ValueError(f"{side_name} initial arm pose must be a Python list literal: {raw_value!r}") from exc

    if not isinstance(values, (list, tuple)) or len(values) != 7:
        raise ValueError(f"{side_name} initial arm pose must contain exactly 7 values: {raw_value!r}")

    try:
        return [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{side_name} initial arm pose must contain numeric values: {raw_value!r}") from exc


def _build_joint_publisher_parameters(context, *, left_arm_initial, right_arm_initial):
    left_raw = left_arm_initial.perform(context)
    right_raw = right_arm_initial.perform(context)
    left_values = _parse_initial_joint_values(left_raw, "left")
    right_values = _parse_initial_joint_values(right_raw, "right")

    zeros = {}
    for joint_name, value in zip(ARM_JOINT_NAMES[0], left_values):
        zeros[f"zeros.{joint_name}"] = value
    for joint_name, value in zip(ARM_JOINT_NAMES[1], right_values):
        zeros[f"zeros.{joint_name}"] = value
    return [zeros]


def _create_joint_publisher_nodes(
    context,
    *,
    urdf_path,
    left_arm_initial,
    right_arm_initial,
    use_gui,
):
    joint_publisher_parameters = _build_joint_publisher_parameters(
        context,
        left_arm_initial=left_arm_initial,
        right_arm_initial=right_arm_initial,
    )
    joint_publisher_arguments = [str(urdf_path)]

    return [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="wheelloong_m2_joint_state_publisher_gui",
            condition=IfCondition(use_gui),
            output="screen",
            arguments=joint_publisher_arguments,
            parameters=joint_publisher_parameters,
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="wheelloong_m2_joint_state_publisher",
            condition=UnlessCondition(use_gui),
            output="screen",
            arguments=joint_publisher_arguments,
            parameters=joint_publisher_parameters,
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    urdf_path = package_share / "urdf" / "wheelloong_m2.urdf"
    rviz_config_path = package_share / "rviz" / "view_urdf_joint_gui.rviz"

    robot_description = _read_required_text(urdf_path, "URDF")

    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    left_arm_initial = LaunchConfiguration("left_arm_initial")
    right_arm_initial = LaunchConfiguration("right_arm_initial")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Use joint_state_publisher_gui sliders. Set false for non-GUI joint_state_publisher.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with the wheelloong_m2 URDF view config.",
        ),
        DeclareLaunchArgument(
            "left_arm_initial",
            default_value=DEFAULT_LEFT_ARM_INITIAL,
            description="Initial left arm joint values in radians, as a Python list literal with 7 values.",
        ),
        DeclareLaunchArgument(
            "right_arm_initial",
            default_value=DEFAULT_RIGHT_ARM_INITIAL,
            description="Initial right arm joint values in radians, as a Python list literal with 7 values.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="wheelloong_m2_robot_state_publisher",
            output="screen",
            parameters=[_robot_description_parameter(robot_description)],
        ),
        OpaqueFunction(
            function=_create_joint_publisher_nodes,
            kwargs={
                "urdf_path": urdf_path,
                "left_arm_initial": left_arm_initial,
                "right_arm_initial": right_arm_initial,
                "use_gui": use_gui,
            },
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="wheelloong_m2_rviz2",
            arguments=["-d", str(rviz_config_path)],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
