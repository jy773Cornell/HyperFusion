#!/usr/bin/env python3
"""Launch the HyperFusion UR3e HTTP sidecar (WSL)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    robot_ip = LaunchConfiguration("robot_ip")
    ros_distro = LaunchConfiguration("ros_distro")
    ur_type = LaunchConfiguration("ur_type")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8766"),
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.10"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            DeclareLaunchArgument("prestart_driver", default_value="false"),
            DeclareLaunchArgument("ros_distro", default_value="jazzy"),
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            ExecuteProcess(
                cmd=[
                    "python3",
                    "-m",
                    "hyperfusion_ur3e.sidecar.server",
                    "--host",
                    host,
                    "--port",
                    port,
                    "--robot-ip",
                    robot_ip,
                    "--ros-distro",
                    ros_distro,
                    "--ur-type",
                    ur_type,
                ],
                output="screen",
            ),
        ]
    )
