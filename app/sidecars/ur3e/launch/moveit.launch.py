#!/usr/bin/env python3
"""Launch MoveIt 2 + RViz for UR3e (expects UR driver already running)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    ros_distro = LaunchConfiguration("ros_distro")
    ur_type = LaunchConfiguration("ur_type")

    script = PathJoinSubstitution(
        [FindPackageShare("hyperfusion_ur3e"), "scripts", "launch_moveit.sh"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ros_distro", default_value="jazzy"),
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            ExecuteProcess(
                cmd=["bash", script, ros_distro, ur_type],
                output="screen",
            ),
        ]
    )
