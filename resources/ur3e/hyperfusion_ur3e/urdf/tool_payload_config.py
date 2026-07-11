"""Tool-flange camera dome (URDF collision) env for ROS launch subprocesses."""
from __future__ import annotations

import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

TOOL_PAYLOAD_LINK = "hyperfusion_tool_payload"
TOOL_PAYLOAD_MESH = "tool_payload_hemisphere.stl"


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
class ToolPayloadConfig:
    enabled: bool = True
    shape: str = "hemisphere"
    radius_m: float = 0.10
    collision_gap_m: float = 0.002

    @classmethod
    def from_env(cls) -> ToolPayloadConfig:
        return cls(
            enabled=_env_bool("HYPERFUSION_TOOL_PAYLOAD_ENABLED", True),
            shape=os.environ.get("HYPERFUSION_TOOL_PAYLOAD_SHAPE", "hemisphere").strip().lower()
            or "hemisphere",
            radius_m=_env_float("HYPERFUSION_TOOL_PAYLOAD_RADIUS_M", 0.10),
            collision_gap_m=_env_float("HYPERFUSION_TOOL_PAYLOAD_COLLISION_GAP_M", 0.002),
        )

    @classmethod
    def from_radius_mm(cls, radius_mm: float, *, enabled: bool = True, shape: str = "hemisphere") -> ToolPayloadConfig:
        return cls(enabled=enabled, shape=shape, radius_m=float(radius_mm) / 1000.0)

    def apply_to_environ(self) -> None:
        os.environ["HYPERFUSION_TOOL_PAYLOAD_ENABLED"] = "true" if self.enabled else "false"
        os.environ["HYPERFUSION_TOOL_PAYLOAD_SHAPE"] = self.shape
        os.environ["HYPERFUSION_TOOL_PAYLOAD_RADIUS_M"] = f"{self.radius_m:.6f}"
        os.environ["HYPERFUSION_TOOL_PAYLOAD_COLLISION_GAP_M"] = f"{self.collision_gap_m:.6f}"

    def bash_exports(self) -> str:
        enabled = "true" if self.enabled else "false"
        return (
            f"export HYPERFUSION_TOOL_PAYLOAD_ENABLED='{enabled}' && "
            f"export HYPERFUSION_TOOL_PAYLOAD_SHAPE='{self.shape}' && "
            f"export HYPERFUSION_TOOL_PAYLOAD_RADIUS_M='{self.radius_m:.6f}' && "
            f"export HYPERFUSION_TOOL_PAYLOAD_COLLISION_GAP_M='{self.collision_gap_m:.6f}' && "
        )


def robot_description_has_tool_payload(
    urdf_text: str,
    *,
    expect_shape: str = "hemisphere",
) -> bool:
    if TOOL_PAYLOAD_LINK not in urdf_text:
        return False
    if expect_shape == "hemisphere" and TOOL_PAYLOAD_MESH not in urdf_text:
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
    return robot_description_has_tool_payload(proc.stdout, expect_shape=cfg.shape)


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
            if robot_description_has_tool_payload(generated_text, expect_shape=cfg.shape):
                return True

    attempts = max(1, int(retries))
    for attempt in range(attempts):
        urdf = fetch_robot_description_topic(ros_distro=ros_distro)
        if not urdf:
            urdf = fetch_robot_description_param("/robot_state_publisher", ros_distro=ros_distro)
        if urdf and robot_description_has_tool_payload(urdf, expect_shape=cfg.shape):
            return True
        if attempt + 1 < attempts:
            time.sleep(max(0.5, float(retry_delay_s)))

    return False


def log_payload_probe_failure(context: str, cfg: ToolPayloadConfig | None = None) -> None:
    if cfg is None:
        cfg = ToolPayloadConfig.from_env()
    sys.stderr.write(
        f"UR3e: WARNING — {context}: robot_description is missing "
        f"'{TOOL_PAYLOAD_LINK}' ({cfg.shape}, {cfg.radius_m * 1000.0:.0f} mm). "
        "Kill stale ROS (kill_stale_ur_ros.sh) and reconnect so hyperfusion_ur_rsp.launch.py reloads.\n"
    )
