#!/usr/bin/env python3
"""
MoveIt + RViz for HyperFusion UR3e.

Fork of ur_moveit_config/ur_moveit.launch.py. IncludeLaunchDescription +
GroupAction(SetRemap) breaks launch arguments on ROS 2 Jazzy, so we vendor the
launch file. Joint states use /joint_states from hyperfusion_joint_states_stamper.
"""
import os
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument(
                "ur_type",
                description="Typo/series of used UR robot.",
                choices=[
                    "ur3",
                    "ur5",
                    "ur10",
                    "ur3e",
                    "ur5e",
                    "ur7e",
                    "ur10e",
                    "ur12e",
                    "ur16e",
                    "ur8long",
                    "ur15",
                    "ur18",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                description="Path where the warehouse database should be stored",
            ),
            DeclareLaunchArgument(
                "launch_servo", default_value="false", description="Launch Servo?"
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Using or not time from simulation",
            ),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
                description="MoveGroup publishes robot description semantic",
            ),
        ]
    )


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    _pkg_root = Path(os.environ.get("HYPERFUSION_UR3E_REPO", Path(__file__).resolve().parent.parent))
    _default_description = str(_pkg_root / "urdf" / "hyperfusion_ur3e.urdf.xacro")
    _ceiling_height_m = os.environ.get("HYPERFUSION_CEILING_MOUNT_HEIGHT_M", "0.65")

    hyperfusion_robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                _default_description,
                " ",
                "ur_type:=",
                ur_type,
                " ",
                "name:=",
                ur_type,
                " ",
                "use_mock_hardware:=true",
                " ",
                "mock_sensor_commands:=true",
                " ",
                "headless_mode:=true",
                " ",
                "ceiling_mount:=true",
                " ",
                "ceiling_mount_height_m:=",
                _ceiling_height_m,
                " ",
            ]
        ),
        value_type=str,
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params["robot_description"] = hyperfusion_robot_description

    def _truthy_env(name: str, default: str = "true") -> bool:
        return os.environ.get(name, default).strip().lower() in ("1", "true", "yes", "on")

    use_mock_hardware = _truthy_env("HYPERFUSION_USE_MOCK_HARDWARE", "true")
    controllers_file = (
        _pkg_root / "config" / "moveit_controllers_hyperfusion.yaml"
        if use_mock_hardware
        else _pkg_root / "config" / "moveit_controllers_hyperfusion_hardware.yaml"
    )
    try:
        with open(controllers_file, encoding="utf-8") as controllers_stream:
            moveit_controller_params = yaml.safe_load(controllers_stream)
    except OSError as exc:
        raise RuntimeError(f"Missing MoveIt controllers config: {controllers_file}") from exc

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }
    planning_scene_monitor_config = {
        "planning_scene_monitor_options": {
            "joint_state_topic": "/joint_states_stamped",
        },
    }

    ld = LaunchDescription()
    ld.add_entity(declare_arguments())

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    ld.add_action(wait_robot_description)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_params,
            moveit_controller_params,
            warehouse_ros_config,
            planning_scene_monitor_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_params,
            servo_params,
        ],
        output="screen",
    )

    rviz_config_file = str(_pkg_root / "config" / "hyperfusion_moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"robot_description": hyperfusion_robot_description},
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            planning_scene_monitor_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node, servo_node],
            )
        ),
    )

    return ld
