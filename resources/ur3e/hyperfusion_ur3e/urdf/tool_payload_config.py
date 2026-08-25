"""Tool-flange payload (URDF collision mesh) env for ROS launch subprocesses."""
from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

TOOL_PAYLOAD_LINK = "hyperfusion_tool_payload"
TOOL_TCP_LINK = "hyperfusion_tcp"
TOOL_PAYLOAD_HEMISPHERE_MESH = "tool_payload_hemisphere.stl"
DEFAULT_MESH_FILE = "ur_tool_payload.stl"
# Pinch sphere is the C403A0 flange↔forearm guard, not mesh size.
# ur_tool_payload.stl max vertex distance from flange origin is ~133 mm.
DEFAULT_MESH_PINCH_RADIUS_M = 0.077
# BFS optical origin in tool0 (Tsai hand-eye, metres + URDF rpy deg).
DEFAULT_TOOL_TCP_X_M = 0.000715
DEFAULT_TOOL_TCP_Y_M = -0.054197
DEFAULT_TOOL_TCP_Z_M = 0.073755
DEFAULT_TOOL_TCP_ROLL_DEG = -1.9138
DEFAULT_TOOL_TCP_PITCH_DEG = 0.7450
DEFAULT_TOOL_TCP_YAW_DEG = 0.2868


def _env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return default
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return default
    return float(raw)


@dataclass(frozen=True)
class ToolTcpConfig:
    """Optical TCP from tool0 (metres + URDF rpy deg), used for MoveIt scan IK tip."""

    x_m: float = DEFAULT_TOOL_TCP_X_M
    y_m: float = DEFAULT_TOOL_TCP_Y_M
    z_m: float = DEFAULT_TOOL_TCP_Z_M
    roll_deg: float = DEFAULT_TOOL_TCP_ROLL_DEG
    pitch_deg: float = DEFAULT_TOOL_TCP_PITCH_DEG
    yaw_deg: float = DEFAULT_TOOL_TCP_YAW_DEG

    @classmethod
    def from_env(cls) -> ToolTcpConfig:
        return cls(
            x_m=_env_float("HYPERFUSION_TOOL_TCP_X_M", DEFAULT_TOOL_TCP_X_M),
            y_m=_env_float("HYPERFUSION_TOOL_TCP_Y_M", DEFAULT_TOOL_TCP_Y_M),
            z_m=_env_float("HYPERFUSION_TOOL_TCP_Z_M", DEFAULT_TOOL_TCP_Z_M),
            roll_deg=_env_float("HYPERFUSION_TOOL_TCP_ROLL_DEG", DEFAULT_TOOL_TCP_ROLL_DEG),
            pitch_deg=_env_float("HYPERFUSION_TOOL_TCP_PITCH_DEG", DEFAULT_TOOL_TCP_PITCH_DEG),
            yaw_deg=_env_float("HYPERFUSION_TOOL_TCP_YAW_DEG", DEFAULT_TOOL_TCP_YAW_DEG),
        )

    @classmethod
    def from_mm(
        cls,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float = DEFAULT_TOOL_TCP_ROLL_DEG,
        pitch_deg: float = DEFAULT_TOOL_TCP_PITCH_DEG,
        yaw_deg: float = DEFAULT_TOOL_TCP_YAW_DEG,
    ) -> ToolTcpConfig:
        return cls(
            x_m=float(x_mm) / 1000.0,
            y_m=float(y_mm) / 1000.0,
            z_m=float(z_mm) / 1000.0,
            roll_deg=float(roll_deg),
            pitch_deg=float(pitch_deg),
            yaw_deg=float(yaw_deg),
        )

    def apply_to_environ(self) -> None:
        os.environ["HYPERFUSION_TOOL_TCP_X_M"] = f"{self.x_m:.6f}"
        os.environ["HYPERFUSION_TOOL_TCP_Y_M"] = f"{self.y_m:.6f}"
        os.environ["HYPERFUSION_TOOL_TCP_Z_M"] = f"{self.z_m:.6f}"
        os.environ["HYPERFUSION_TOOL_TCP_ROLL_DEG"] = f"{self.roll_deg:.6f}"
        os.environ["HYPERFUSION_TOOL_TCP_PITCH_DEG"] = f"{self.pitch_deg:.6f}"
        os.environ["HYPERFUSION_TOOL_TCP_YAW_DEG"] = f"{self.yaw_deg:.6f}"

    def bash_exports(self) -> str:
        return (
            f"export HYPERFUSION_TOOL_TCP_X_M='{self.x_m:.6f}' && "
            f"export HYPERFUSION_TOOL_TCP_Y_M='{self.y_m:.6f}' && "
            f"export HYPERFUSION_TOOL_TCP_Z_M='{self.z_m:.6f}' && "
            f"export HYPERFUSION_TOOL_TCP_ROLL_DEG='{self.roll_deg:.6f}' && "
            f"export HYPERFUSION_TOOL_TCP_PITCH_DEG='{self.pitch_deg:.6f}' && "
            f"export HYPERFUSION_TOOL_TCP_YAW_DEG='{self.yaw_deg:.6f}' && "
        )

    @property
    def roll_rad(self) -> float:
        return math.radians(self.roll_deg)

    @property
    def pitch_rad(self) -> float:
        return math.radians(self.pitch_deg)

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)


@dataclass(frozen=True)
class ToolPayloadConfig:
    enabled: bool = True
    shape: str = "mesh"
    mesh_file: str = DEFAULT_MESH_FILE
    # Bounding sphere at tool0 for UR C403A0 pinch approximation (not MoveIt mesh size).
    radius_m: float = DEFAULT_MESH_PINCH_RADIUS_M
    collision_gap_m: float = 0.0

    @classmethod
    def from_env(cls) -> ToolPayloadConfig:
        shape = (
            os.environ.get("HYPERFUSION_TOOL_PAYLOAD_SHAPE", "mesh").strip().lower() or "mesh"
        )
        default_radius = (
            DEFAULT_MESH_PINCH_RADIUS_M if shape == "mesh" else 0.10
        )
        return cls(
            enabled=_env_bool("HYPERFUSION_TOOL_PAYLOAD_ENABLED", True),
            shape=shape,
            mesh_file=(
                os.environ.get("HYPERFUSION_TOOL_PAYLOAD_MESH_FILE", DEFAULT_MESH_FILE).strip()
                or DEFAULT_MESH_FILE
            ),
            radius_m=_env_float("HYPERFUSION_TOOL_PAYLOAD_RADIUS_M", default_radius),
            collision_gap_m=_env_float("HYPERFUSION_TOOL_PAYLOAD_COLLISION_GAP_M", 0.0),
        )

    @classmethod
    def from_radius_mm(
        cls,
        radius_mm: float,
        *,
        enabled: bool = True,
        shape: str = "mesh",
        mesh_file: str = DEFAULT_MESH_FILE,
    ) -> ToolPayloadConfig:
        pinch_m = float(radius_mm) / 1000.0
        if shape == "mesh" and pinch_m <= 0.0:
            pinch_m = DEFAULT_MESH_PINCH_RADIUS_M
        return cls(
            enabled=enabled,
            shape=shape,
            mesh_file=mesh_file,
            radius_m=pinch_m,
        )

    def apply_to_environ(self) -> None:
        os.environ["HYPERFUSION_TOOL_PAYLOAD_ENABLED"] = "true" if self.enabled else "false"
        os.environ["HYPERFUSION_TOOL_PAYLOAD_SHAPE"] = self.shape
        os.environ["HYPERFUSION_TOOL_PAYLOAD_MESH_FILE"] = self.mesh_file
        os.environ["HYPERFUSION_TOOL_PAYLOAD_RADIUS_M"] = f"{self.radius_m:.6f}"
        os.environ["HYPERFUSION_TOOL_PAYLOAD_COLLISION_GAP_M"] = f"{self.collision_gap_m:.6f}"

    def bash_exports(self) -> str:
        enabled = "true" if self.enabled else "false"
        return (
            f"export HYPERFUSION_TOOL_PAYLOAD_ENABLED='{enabled}' && "
            f"export HYPERFUSION_TOOL_PAYLOAD_SHAPE='{self.shape}' && "
            f"export HYPERFUSION_TOOL_PAYLOAD_MESH_FILE='{self.mesh_file}' && "
            f"export HYPERFUSION_TOOL_PAYLOAD_RADIUS_M='{self.radius_m:.6f}' && "
            f"export HYPERFUSION_TOOL_PAYLOAD_COLLISION_GAP_M='{self.collision_gap_m:.6f}' && "
        )


def robot_description_has_tool_payload(
    urdf_text: str,
    *,
    expect_shape: str = "mesh",
    mesh_file: str = DEFAULT_MESH_FILE,
) -> bool:
    if TOOL_PAYLOAD_LINK not in urdf_text:
        return False
    if expect_shape == "hemisphere" and TOOL_PAYLOAD_HEMISPHERE_MESH not in urdf_text:
        return False
    if expect_shape == "mesh" and mesh_file not in urdf_text:
        return False
    return True


def fetch_robot_description_param(
    node_name: str = "/robot_state_publisher",
    *,
    ros_distro: str = "jazzy",
    timeout_s: float = 5.0,
) -> str:
    cmd = (
        f"export ROS_LOCALHOST_ONLY=1 && "
        f"export ROS2CLI_DISABLE_DAEMON=1 && "
        f"source /opt/ros/{ros_distro}/setup.bash && "
        f"timeout {max(2, int(timeout_s))} "
        f"ros2 param get {node_name} robot_description 2>/dev/null"
    )
    try:
        proc = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=timeout_s + 2.0,
        )
    except subprocess.TimeoutExpired:
        return ""
    if proc.returncode != 0:
        return ""
    return proc.stdout


def verify_xacro_tool_payload(
    pkg_root: str,
    ros_distro: str = "jazzy",
    cfg: ToolPayloadConfig | None = None,
) -> bool:
    """Run xacro locally and confirm the tool payload link is present."""
    if cfg is None:
        cfg = ToolPayloadConfig.from_env()
    if not cfg.enabled:
        return True

    mesh_dir = f"{pkg_root.rstrip('/')}/urdf/meshes/"
    xacro_file = f"{pkg_root.rstrip('/')}/urdf/hyperfusion_ur3e.urdf.xacro"
    cmd = (
        f"export ROS_LOCALHOST_ONLY=1 && "
        f"source /opt/ros/{ros_distro}/setup.bash && "
        f"xacro '{xacro_file}' "
        f"ur_type:=ur3e name:=ur3e use_mock_hardware:=false ceiling_mount:=true "
        f"tool_payload_enabled:=true tool_payload_shape:={cfg.shape} "
        f"tool_payload_radius_m:={cfg.radius_m:.6f} "
        f"tool_payload_mesh_file:={cfg.mesh_file} "
        f"tool_payload_mesh_dir:='{mesh_dir}'"
    )
    try:
        proc = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=30.0,
        )
    except subprocess.TimeoutExpired:
        return False
    if proc.returncode != 0:
        sys.stderr.write(
            "UR3e: xacro tool payload preflight failed:\n"
            f"{proc.stderr[-800:]}\n"
        )
        return False
    return robot_description_has_tool_payload(
        proc.stdout, expect_shape=cfg.shape, mesh_file=cfg.mesh_file
    )


def fetch_robot_description_topic(
    *,
    ros_distro: str = "jazzy",
    timeout_s: float = 10.0,
) -> str:
    tmp_path = Path("/tmp/hyperfusion_robot_description_probe.xml")
    cmd = (
        f"export ROS_LOCALHOST_ONLY=1 && "
        f"export ROS2CLI_DISABLE_DAEMON=1 && "
        f"source /opt/ros/{ros_distro}/setup.bash && "
        f"timeout {max(3, int(timeout_s))} ros2 topic echo /robot_description --once "
        f"> '{tmp_path.as_posix()}' 2>/dev/null"
    )
    try:
        proc = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=timeout_s + 3.0,
        )
    except subprocess.TimeoutExpired:
        return ""
    if proc.returncode != 0 or not tmp_path.is_file():
        return ""
    return tmp_path.read_text(encoding="utf-8", errors="replace")


def probe_robot_state_publisher_payload(
    ros_distro: str = "jazzy",
    cfg: ToolPayloadConfig | None = None,
    *,
    retries: int = 1,
    retry_delay_s: float = 2.0,
) -> bool:
    """Return True when the live driver URDF includes the tool payload link."""
    if cfg is None:
        cfg = ToolPayloadConfig.from_env()
    if not cfg.enabled:
        return True

    generated = os.environ.get("HYPERFUSION_GENERATED_URDF", "").strip()
    if generated:
        generated_path = Path(generated)
        if generated_path.is_file():
            generated_text = generated_path.read_text(encoding="utf-8", errors="replace")
            if robot_description_has_tool_payload(
                generated_text, expect_shape=cfg.shape, mesh_file=cfg.mesh_file
            ):
                return True

    attempts = max(1, int(retries))
    for attempt in range(attempts):
        urdf = fetch_robot_description_topic(ros_distro=ros_distro)
        if not urdf:
            urdf = fetch_robot_description_param("/robot_state_publisher", ros_distro=ros_distro)
        if urdf and robot_description_has_tool_payload(
            urdf, expect_shape=cfg.shape, mesh_file=cfg.mesh_file
        ):
            return True
        if attempt + 1 < attempts:
            time.sleep(max(0.5, float(retry_delay_s)))

    return False


def log_payload_probe_failure(context: str, cfg: ToolPayloadConfig | None = None) -> None:
    if cfg is None:
        cfg = ToolPayloadConfig.from_env()
    detail = cfg.mesh_file if cfg.shape == "mesh" else f"{cfg.radius_m * 1000.0:.0f} mm"
    sys.stderr.write(
        f"UR3e: WARNING — {context}: robot_description is missing "
        f"'{TOOL_PAYLOAD_LINK}' ({cfg.shape}, {detail}). "
        "Kill stale ROS (kill_stale_ur_ros.sh) and reconnect so hyperfusion_ur_rsp.launch.py reloads.\n"
    )
