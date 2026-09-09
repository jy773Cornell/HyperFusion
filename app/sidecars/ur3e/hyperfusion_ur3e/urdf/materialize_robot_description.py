"""Materialize HyperFusion UR3e URDF (with tool payload) for robot_state_publisher."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from hyperfusion_ur3e.urdf.mount_config import MountConfig
from hyperfusion_ur3e.urdf.tool_payload_config import (
    TOOL_PAYLOAD_LINK,
    ToolPayloadConfig,
    ToolTcpConfig,
    robot_description_has_tool_payload,
)

RUNTIME_URDF_NAME = "runtime_robot_description.urdf"


def resolve_runtime_robot_description_path(pkg_root: Path) -> Path:
    """Return the HyperFusion materialized URDF path (env override or default)."""
    env_path = os.environ.get("HYPERFUSION_GENERATED_URDF", "").strip()
    if env_path:
        candidate = Path(env_path)
        if candidate.is_file():
            return candidate.resolve()
    from hyperfusion_ur3e.runtime_paths import runtime_urdf_path

    runtime = runtime_urdf_path()
    if runtime.is_file():
        return runtime
    return (pkg_root / "config" / RUNTIME_URDF_NAME).resolve()


def load_runtime_robot_description_text(pkg_root: Path) -> str | None:
    """Read materialized URDF when present."""
    path = resolve_runtime_robot_description_path(pkg_root)
    if not path.is_file():
        return None
    return path.read_text(encoding="utf-8")


def _abs_mesh_uris_to_file_urls(urdf_text: str) -> str:
    """RViz/assimp often fail bare /mnt/... mesh paths; file:// is required."""
    import re

    def repl(match: re.Match[str]) -> str:
        path = match.group(1)
        if path.startswith(("package://", "file://", "model://")):
            return match.group(0)
        if path.startswith("/"):
            return f'filename="file://{path}"'
        return match.group(0)

    return re.sub(r'filename="([^"]+)"', repl, urdf_text)


def _ros_pkg_prefix(ros_distro: str, package: str) -> str:
    cmd = (
        f"source /opt/ros/{ros_distro}/setup.bash && "
        f"ros2 pkg prefix {package}"
    )
    proc = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=30.0,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"Could not locate ROS package '{package}': {proc.stderr.strip()}")
    return proc.stdout.strip()


def materialize_runtime_robot_description(
    pkg_root: Path,
    *,
    ros_distro: str = "jazzy",
    ur_type: str = "ur3e",
    robot_ip: str = "0.0.0.0",
    reverse_ip: str = "0.0.0.0",
    use_mock_hardware: bool = False,
    headless_mode: bool = True,
    initial_positions_file: Path | None = None,
    cfg: ToolPayloadConfig | None = None,
) -> Path:
    """Run xacro once and write the URDF file the driver will publish."""
    if cfg is None:
        cfg = ToolPayloadConfig.from_env()
    tcp = ToolTcpConfig.from_env()

    pkg_root = pkg_root.resolve()
    from hyperfusion_ur3e.runtime_paths import initial_positions_path, runtime_urdf_path

    out_path = runtime_urdf_path()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    ur_description = _ros_pkg_prefix(ros_distro, "ur_description")
    ur_robot_driver = _ros_pkg_prefix(ros_distro, "ur_robot_driver")
    ur_client_library = _ros_pkg_prefix(ros_distro, "ur_client_library")

    xacro_file = pkg_root / "urdf" / "hyperfusion_ur3e.urdf.xacro"
    if initial_positions_file is None:
        initial_positions_file = initial_positions_path()
        if not initial_positions_file.is_file():
            initial_positions_file = pkg_root / "config" / "initial_positions.yaml"
    mesh_dir = (pkg_root / "urdf" / "meshes").as_posix().rstrip("/") + "/"
    mount = MountConfig.from_env()
    payload_enabled = "true" if cfg.enabled else "false"
    mock_flag = "true" if use_mock_hardware else "false"
    mock_sensor = mock_flag if use_mock_hardware else "false"
    headless_flag = "true" if headless_mode else "false"
    from hyperfusion_ur3e.ros_isolation import ur_driver_ports

    ports = ur_driver_ports()

    cmd = (
        f"source /opt/ros/{ros_distro}/setup.bash && "
        f"xacro '{xacro_file.as_posix()}' "
        f"robot_ip:={robot_ip} "
        f"reverse_ip:={reverse_ip} "
        f"joint_limit_params:={ur_description}/share/ur_description/config/{ur_type}/joint_limits.yaml "
        f"kinematics_params:={ur_description}/share/ur_description/config/{ur_type}/default_kinematics.yaml "
        f"physical_params:={ur_description}/share/ur_description/config/{ur_type}/physical_parameters.yaml "
        f"visual_params:={ur_description}/share/ur_description/config/{ur_type}/visual_parameters.yaml "
        f"safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20 "
        f"name:={ur_type} ur_type:={ur_type} "
        f"initial_positions_file:={initial_positions_file.as_posix()} "
        f"script_filename:={ur_client_library}/share/ur_client_library/resources/external_control.urscript "
        f"input_recipe_filename:={ur_robot_driver}/share/ur_robot_driver/resources/rtde_input_recipe.txt "
        f"output_recipe_filename:={ur_robot_driver}/share/ur_robot_driver/resources/rtde_output_recipe.txt "
        f"tf_prefix:= use_mock_hardware:={mock_flag} mock_sensor_commands:={mock_sensor} "
        f"headless_mode:={headless_flag} use_tool_communication:=false "
        f"tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 "
        f"tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR tool_tcp_port:=54321 tool_voltage:=0 "
        f"script_command_port:={ports['script_command_port']} "
        f"reverse_port:={ports['reverse_port']} "
        f"script_sender_port:={ports['script_sender_port']} "
        f"trajectory_port:={ports['trajectory_port']} "
        f"ceiling_mount:=true "
        f"ceiling_mount_height_m:={mount.height_m:.6f} "
        f"mount_roll_rad:={mount.roll_rad:.6f} "
        f"mount_pitch_rad:={mount.pitch_rad:.6f} "
        f"mount_yaw_rad:={mount.yaw_rad:.6f} "
        f"mount_x_m:={mount.offset_x_m:.6f} "
        f"mount_y_m:={mount.offset_y_m:.6f} "
        f"tool_payload_enabled:={payload_enabled} "
        f"tool_payload_shape:={cfg.shape} "
        f"tool_payload_radius_m:={cfg.radius_m:.6f} "
        f"tool_payload_mesh_file:={cfg.mesh_file} "
        f"tool_payload_collision_gap_m:={cfg.collision_gap_m:.6f} "
        f"tool_payload_mesh_dir:='{mesh_dir}' "
        f"tool_tcp_x_m:={tcp.x_m:.6f} "
        f"tool_tcp_y_m:={tcp.y_m:.6f} "
        f"tool_tcp_z_m:={tcp.z_m:.6f} "
        f"tool_tcp_roll_rad:={tcp.roll_rad:.8f} "
        f"tool_tcp_pitch_rad:={tcp.pitch_rad:.8f} "
        f"tool_tcp_yaw_rad:={tcp.yaw_rad:.8f}"
    )
    proc = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=60.0,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "Failed to materialize HyperFusion URDF with tool payload:\n"
            f"{proc.stderr[-1200:]}"
        )

    urdf_text = _abs_mesh_uris_to_file_urls(proc.stdout)
    out_path.write_text(urdf_text, encoding="utf-8")
    if cfg.enabled and not robot_description_has_tool_payload(
        urdf_text, expect_shape=cfg.shape, mesh_file=cfg.mesh_file
    ):
        raise RuntimeError(
            f"Materialized URDF at {out_path} is missing '{TOOL_PAYLOAD_LINK}'."
        )

    os.environ["HYPERFUSION_GENERATED_URDF"] = out_path.as_posix()
    tcp.apply_to_environ()
    sys.stderr.write(
        "UR3e driver: materialized robot_description "
        f"({cfg.shape}, {cfg.radius_m * 1000.0:.0f} mm, "
        f"tcp=({tcp.x_m * 1000.0:.3f},{tcp.y_m * 1000.0:.3f},{tcp.z_m * 1000.0:.3f}) mm) "
        f"-> {out_path.as_posix()}\n"
    )
    return out_path
