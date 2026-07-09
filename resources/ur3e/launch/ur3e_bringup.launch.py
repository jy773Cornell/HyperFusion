#!/usr/bin/env python3
"""
UR3e driver bring-up (legacy placeholder).

Prefer: ros2 launch ur_robot_driver ur_control.launch.py
Or connect via the HyperFusion sidecar HTTP API.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="192.168.0.10"),
            DeclareLaunchArgument("ur_type", default_value="ur3e"),
            Node(
                package="ur_robot_driver",
                executable="ur_ros2_control_node",
                name="ur_ros2_control_node",
                output="screen",
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "ur_type": ur_type,
                        "launch_rviz": False,
                    }
                ],
            ),
        ]
    )
