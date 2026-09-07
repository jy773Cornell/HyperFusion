# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# HyperFusion fork of ur_rsp.launch.py — passes initial_positions_file to xacro for mock home pose.
#
# Author: Felix Exner

import math
import os
from pathlib import Path

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    physical_params_file = LaunchConfiguration("physical_params_file")
    visual_params_file = LaunchConfiguration("visual_params_file")
    joint_limit_params_file = LaunchConfiguration("joint_limit_params_file")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    ceiling_mount = LaunchConfiguration("ceiling_mount")
    ceiling_mount_height_m = LaunchConfiguration("ceiling_mount_height_m")
    mount_roll_rad = LaunchConfiguration("mount_roll_rad")
    mount_pitch_rad = LaunchConfiguration("mount_pitch_rad")
    mount_yaw_rad = LaunchConfiguration("mount_yaw_rad")
    mount_x_m = LaunchConfiguration("mount_x_m")
    mount_y_m = LaunchConfiguration("mount_y_m")

    _pkg_root = Path(os.environ.get("HYPERFUSION_UR3E_REPO", Path(__file__).resolve().parent.parent))
    _default_initial_positions = str(_pkg_root / "config" / "initial_positions.yaml")
    _default_description = str(_pkg_root / "urdf" / "hyperfusion_ur3e.urdf.xacro")
    # ur_control.launch.py only forwards ur_type + robot_ip to this file — read mock mode from env.
    _ceiling_height_m = os.environ.get("HYPERFUSION_CEILING_MOUNT_HEIGHT_M", "0.65")
    _mount_roll_rad = str(math.radians(float(os.environ.get("HYPERFUSION_MOUNT_ROLL_DEG", "180"))))
    _mount_pitch_rad = str(math.radians(float(os.environ.get("HYPERFUSION_MOUNT_PITCH_DEG", "0"))))
    _mount_yaw_rad = str(math.radians(float(os.environ.get("HYPERFUSION_MOUNT_YAW_DEG", "0"))))
    _mount_x_m = os.environ.get("HYPERFUSION_MOUNT_OFFSET_X_M", "0")
    _mount_y_m = os.environ.get("HYPERFUSION_MOUNT_OFFSET_Y_M", "0")
    _tool_payload_enabled = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_ENABLED", "true").lower()
    _tool_payload_shape = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_SHAPE", "mesh").lower()
    _tool_payload_radius_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_RADIUS_M", "0.077")
    _tool_payload_box_x_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_BOX_X_M", "0.08")
    _tool_payload_box_y_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_BOX_Y_M", "0.06")
    _tool_payload_box_z_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_BOX_Z_M", "0.06")
    _tool_payload_x_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_X_M", "0")
    _tool_payload_y_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_Y_M", "0")
    _tool_payload_z_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_Z_M", "0")
    _tool_payload_collision_gap_m = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_COLLISION_GAP_M", "0.0")
    _tool_payload_mesh_dir = str(_pkg_root / "urdf" / "meshes").replace("\\", "/") + "/"
    _tool_payload_mesh_file = os.environ.get(
        "HYPERFUSION_TOOL_PAYLOAD_MESH_FILE", "ur_tool_payload.stl"
    )
    _tool_tcp_x_m = os.environ.get("HYPERFUSION_TOOL_TCP_X_M", "0.000715")
    _tool_tcp_y_m = os.environ.get("HYPERFUSION_TOOL_TCP_Y_M", "-0.054197")
    _tool_tcp_z_m = os.environ.get("HYPERFUSION_TOOL_TCP_Z_M", "0.073755")
    _tool_tcp_roll_rad = str(
        math.radians(float(os.environ.get("HYPERFUSION_TOOL_TCP_ROLL_DEG", "-1.9138")))
    )
    _tool_tcp_pitch_rad = str(
        math.radians(float(os.environ.get("HYPERFUSION_TOOL_TCP_PITCH_DEG", "0.7450")))
    )
    _tool_tcp_yaw_rad = str(
        math.radians(float(os.environ.get("HYPERFUSION_TOOL_TCP_YAW_DEG", "0.2868")))
    )
    _use_mock_hardware = os.environ.get("HYPERFUSION_USE_MOCK_HARDWARE", "false").lower()
    _mock_sensor_commands = os.environ.get(
        "HYPERFUSION_MOCK_SENSOR_COMMANDS",
        "true" if _use_mock_hardware in ("true", "1", "yes") else "false",
    ).lower()

    script_filename = PathJoinSubstitution(
        [
            FindPackageShare("ur_client_library"),
            "resources",
            "external_control.urscript",
        ]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    _generated_urdf = os.environ.get("HYPERFUSION_GENERATED_URDF", "").strip()
    if _generated_urdf and Path(_generated_urdf).is_file():
        robot_description = {
            "robot_description": Path(_generated_urdf).read_text(encoding="utf-8")
        }
    else:
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                description_file,
                " ",
                "robot_ip:=",
                robot_ip,
                " ",
                "joint_limit_params:=",
                joint_limit_params_file,
                " ",
                "kinematics_params:=",
                kinematics_params_file,
                " ",
                "physical_params:=",
                physical_params_file,
                " ",
                "visual_params:=",
                visual_params_file,
                " ",
                "safety_limits:=",
                safety_limits,
                " ",
                "safety_pos_margin:=",
                safety_pos_margin,
                " ",
                "safety_k_position:=",
                safety_k_position,
                " ",
                "name:=",
                ur_type,
                " ",
                "ur_type:=",
                ur_type,
                " ",
                "initial_positions_file:=",
                initial_positions_file,
                " ",
                "script_filename:=",
                script_filename,
                " ",
                "input_recipe_filename:=",
                input_recipe_filename,
                " ",
                "output_recipe_filename:=",
                output_recipe_filename,
                " ",
                "tf_prefix:=",
                tf_prefix,
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
                " ",
                "mock_sensor_commands:=",
                mock_sensor_commands,
                " ",
                "headless_mode:=",
                headless_mode,
                " ",
                "use_tool_communication:=",
                use_tool_communication,
                " ",
                "tool_parity:=",
                tool_parity,
                " ",
                "tool_baud_rate:=",
                tool_baud_rate,
                " ",
                "tool_stop_bits:=",
                tool_stop_bits,
                " ",
                "tool_rx_idle_chars:=",
                tool_rx_idle_chars,
                " ",
                "tool_tx_idle_chars:=",
                tool_tx_idle_chars,
                " ",
                "tool_device_name:=",
                tool_device_name,
                " ",
                "tool_tcp_port:=",
                tool_tcp_port,
                " ",
                "tool_voltage:=",
                tool_voltage,
                " ",
                "reverse_ip:=",
                reverse_ip,
                " ",
                "script_command_port:=",
                script_command_port,
                " ",
                "reverse_port:=",
                reverse_port,
                " ",
                "script_sender_port:=",
                script_sender_port,
                " ",
                "trajectory_port:=",
                trajectory_port,
                " ",
                "ceiling_mount:=",
                ceiling_mount,
                " ",
                "ceiling_mount_height_m:=",
                ceiling_mount_height_m,
                " ",
                "mount_roll_rad:=",
                mount_roll_rad,
                " ",
                "mount_pitch_rad:=",
                mount_pitch_rad,
                " ",
                "mount_yaw_rad:=",
                mount_yaw_rad,
                " ",
                "mount_x_m:=",
                mount_x_m,
                " ",
                "mount_y_m:=",
                mount_y_m,
                " ",
                "tool_payload_enabled:=",
                "true" if _tool_payload_enabled in ("1", "true", "yes", "on") else "false",
                " ",
                "tool_payload_shape:=",
                _tool_payload_shape,
                " ",
                "tool_payload_radius_m:=",
                _tool_payload_radius_m,
                " ",
                "tool_payload_box_x_m:=",
                _tool_payload_box_x_m,
                " ",
                "tool_payload_box_y_m:=",
                _tool_payload_box_y_m,
                " ",
                "tool_payload_box_z_m:=",
                _tool_payload_box_z_m,
                " ",
                "tool_payload_x_m:=",
                _tool_payload_x_m,
                " ",
                "tool_payload_y_m:=",
                _tool_payload_y_m,
                " ",
                "tool_payload_z_m:=",
                _tool_payload_z_m,
                " ",
                "tool_payload_collision_gap_m:=",
                _tool_payload_collision_gap_m,
                " ",
                "tool_payload_mesh_dir:=",
                _tool_payload_mesh_dir,
                " ",
                "tool_payload_mesh_file:=",
                _tool_payload_mesh_file,
                " ",
                "tool_tcp_x_m:=",
                _tool_tcp_x_m,
                " ",
                "tool_tcp_y_m:=",
                _tool_tcp_y_m,
                " ",
                "tool_tcp_z_m:=",
                _tool_tcp_z_m,
                " ",
                "tool_tcp_roll_rad:=",
                _tool_tcp_roll_rad,
                " ",
                "tool_tcp_pitch_rad:=",
                _tool_tcp_pitch_rad,
                " ",
                "tool_tcp_yaw_rad:=",
                _tool_tcp_yaw_rad,
                " ",
            ]
        )
        robot_description = {
            "robot_description": ParameterValue(robot_description_content, value_type=str)
        }

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ceiling_mount",
            default_value=os.environ.get("HYPERFUSION_CEILING_MOUNT", "true"),
            description="Ceiling-mount robot in URDF (upside-down at workspace top).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ceiling_mount_height_m",
            default_value=os.environ.get("HYPERFUSION_CEILING_MOUNT_HEIGHT_M", "0.65"),
            description="Ceiling mount plane height in meters (Z=0 tray floor).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mount_roll_rad",
            default_value=_mount_roll_rad,
            description="Mount roll (rad) for world->base_link.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mount_pitch_rad",
            default_value=_mount_pitch_rad,
            description="Mount pitch (rad) for world->base_link.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mount_yaw_rad",
            default_value=_mount_yaw_rad,
            description="Mount yaw (rad) for world->base_link (left/right rotation).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mount_x_m",
            default_value=_mount_x_m,
            description="Mount X offset (m) from workspace origin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mount_y_m",
            default_value=_mount_y_m,
            description="Mount Y offset (m) from workspace origin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=_default_initial_positions,
            description="YAML with mock-hardware initial joint positions (radians).",
        )
    )
    # UR specific arguments
    declared_arguments.append(
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
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_limit_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    ur_type,
                    "joint_limits.yaml",
                ]
            ),
            description="Config file containing the joint limits (e.g. velocities, positions) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    ur_type,
                    "default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "physical_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    ur_type,
                    "physical_parameters.yaml",
                ]
            ),
            description="Config file containing the physical parameters (e.g. masses, inertia) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "visual_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "config",
                    ur_type,
                    "visual_parameters.yaml",
                ]
            ),
            description="Config file containing the visual parameters (e.g. meshes) of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=_default_description,
            description="HyperFusion URDF/XACRO (ceiling-mounted UR3e).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value=_use_mock_hardware,
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value=_mock_sensor_commands,
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication. Only effective, if "
            "use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_rx_idle_chars",
            default_value="1.5",
            description="RX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX idle chars configuration for serial communication. Only effective, "
            "if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has be be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",  # 0 being a conservative value that won't destroy anything
            description="Tool voltage that will be setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="0.0.0.0",
            description="IP that will be used for the robot controller to communicate back to the driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port that will be opened to forward URScript commands to the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port that will be opened to send cyclic instructions from the driver to the robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="The driver will offer an interface to query the external_control URScript on this port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "trajectory_port",
            default_value="50003",
            description="Port that will be opened for trajectory control.",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )
