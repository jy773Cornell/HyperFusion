"""MoveIt IK + collision checking for UR3e hemisphere scan routes (WSL sidecar)."""
from __future__ import annotations

import copy
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

from hyperfusion_ur3e import PKG_ROOT
from hyperfusion_ur3e.joint_angles import (
    CANONICAL_JOINT_NAMES,
    MAX_IK_BRANCH_STEPS,
    TWO_PI,
    UR3E_JOINT_LIMITS_RAD,
    coalesce_joints_for_moveit,
    joint_delta_rad,
    joint_distance_rad,
    moveit_joint_limit_violations,
    normalize_joint_solution_to_reference,
    pick_joint_branch,
    wrap_to_pi,
)

GROUP_NAME = "ur_manipulator"
EE_LINK = "tool0"
PLANNING_FRAME = "world"
RVIZ_UPDATE_CUSTOM_GOAL_TOPIC = "/rviz/moveit/update_custom_goal_state"
WORKSPACE_OBJECT_ID = "hyperfusion_workspace_boundary"  # legacy id (unused)
BOUNDARY_SLAB_THICKNESS_M = 0.02
BOUNDARY_OBJECT_IDS = (
    "hyperfusion_boundary_floor",
    "hyperfusion_boundary_ceiling",
    "hyperfusion_boundary_wall_x_pos",
    "hyperfusion_boundary_wall_x_neg",
    "hyperfusion_boundary_wall_y_pos",
    "hyperfusion_boundary_wall_y_neg",
)

TOOL_PAYLOAD_LINK = "hyperfusion_tool_payload"

# Matches HyperFusion mock / default home pose (degrees → radians).
DEFAULT_HOME_JOINTS_RAD = [
    math.radians(0.0),
    math.radians(-150.0),
    math.radians(120.0),
    math.radians(0.0),
    math.radians(90.0),
    math.radians(0.0),
]

HOME_JOINT_TOLERANCE_RAD = 0.05
TRAJECTORY_START_TOLERANCE_RAD = 0.05
JOINT_SETTLE_TIMEOUT_S = 3.0

# For each base IK seed, also try +45° … +315° on each joint (one joint at a time).
IK_SEED_JOINT_OFFSETS_RAD = tuple(math.radians(float(deg)) for deg in range(45, 360, 45))

# UR3e joint limits from ur_description/config/ur3e/joint_limits.yaml (radians).
# Canonical definitions live in hyperfusion_ur3e.joint_angles (imported above).
ELBOW_JOINT_INDEX = 2

# MoveIt execute: OMPL first (collision-aware paths); Pilz PTP as backup.
EXECUTE_PLANNER_ATTEMPTS = (
    ("ompl", "RRTConnect"),
    ("ompl", "RRTstar"),
    ("pilz_industrial_motion_planner", "PTP"),
)

# Reject IK solutions whose tool0 +Z faces away from the dome center (180° flip).
TOOL_Z_ALIGNMENT_MIN_DOT = 0.95

# Trajectory scoring: sample pinch guard along planned paths before execute.
TRAJECTORY_PINCH_MIN_GAP_M = 0.028
TRAJECTORY_MAX_PINCH_SAMPLES = 48

# UR3e external-control peak joint velocity (matches teach pendant safety).
UR3E_HARDWARE_MAX_JOINT_VELOCITY_DEG_S = 190.0
UR3E_HARDWARE_MAX_JOINT_VELOCITY_RAD_S = math.radians(UR3E_HARDWARE_MAX_JOINT_VELOCITY_DEG_S)
UR3E_DEFAULT_MAX_JOINT_VELOCITY_DEG_S = 60.0
UR3E_JOINT_VELOCITY_TIME_MARGIN = 1.05
TRAJECTORY_MAX_VELOCITY_SCALEUP = 20.0
# UR External Control servos at 500 Hz; never send consecutive setpoints closer than this.
MIN_TRAJECTORY_SEGMENT_S = 0.02
WRIST_3_JOINT_INDEX = 5


def configured_max_joint_velocity_deg_s() -> float:
    """Peak joint speed cap for planned trajectories (env / hyperfusion.cfg)."""
    raw = os.environ.get("HYPERFUSION_MAX_JOINT_VELOCITY_DEG_S", "").strip()
    if raw:
        try:
            value = float(raw)
            if value > 0.0:
                return min(value, UR3E_HARDWARE_MAX_JOINT_VELOCITY_DEG_S)
        except ValueError:
            pass
    return UR3E_DEFAULT_MAX_JOINT_VELOCITY_DEG_S


def configured_max_joint_velocity_rad_s() -> float:
    return math.radians(configured_max_joint_velocity_deg_s())


@dataclass
class WorkspaceBox:
    enabled: bool = False
    length_m: float = 0.6
    width_m: float = 0.6
    height_m: float = 0.65


def workspace_from_dict(cfg: Optional[Dict[str, Any]]) -> WorkspaceBox:
    if not isinstance(cfg, dict):
        return WorkspaceBox()
    return WorkspaceBox(
        enabled=bool(cfg.get("enabled", False)),
        length_m=float(cfg.get("length_m", 0.6)),
        width_m=float(cfg.get("width_m", 0.6)),
        height_m=float(cfg.get("height_m", 0.65)),
    )


def home_joints_rad_from_body(body: Optional[Dict[str, Any]]) -> List[float]:
    """Read scan home pose from HyperFusion cfg payload (degrees → radians)."""
    if not isinstance(body, dict):
        return list(DEFAULT_HOME_JOINTS_RAD)
    joints_deg = body.get("home_joints_deg")
    if not isinstance(joints_deg, list) or len(joints_deg) != 6:
        return list(DEFAULT_HOME_JOINTS_RAD)
    return [math.radians(float(value)) for value in joints_deg]


@dataclass
class ScanPoseTarget:
    index: int
    x_m: float
    y_m: float
    z_m: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0
    tool_z_x: float = 0.0
    tool_z_y: float = 0.0
    tool_z_z: float = -1.0


@dataclass
class ScanPoseResult:
    index: int
    reachable: bool
    joint_positions: List[float] = field(default_factory=list)
    error: str = ""


def _normalize(x: float, y: float, z: float) -> tuple[float, float, float]:
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1.0e-9:
        return 0.0, 0.0, -1.0
    return x / length, y / length, z / length


def tool_z_to_rotation_vector(zx: float, zy: float, zz: float) -> tuple[float, float, float]:
    """UR rotation vector (axis * angle) with tool +Z aligned to (zx, zy, zz)."""
    zx, zy, zz = _normalize(zx, zy, zz)
    ref_x, ref_y, ref_z = (1.0, 0.0, 0.0) if abs(zx) < 0.9 else (0.0, 1.0, 0.0)
    yx = zy * ref_z - zz * ref_y
    yy = zz * ref_x - zx * ref_z
    yz = zx * ref_y - zy * ref_x
    yx, yy, yz = _normalize(yx, yy, yz)
    xx = yy * zz - yz * zy
    xy = yz * zx - yx * zz
    xz = yx * zy - yy * zx

    # Rotation matrix columns are x, y, z tool axes.
    trace = xx + yy + zz
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5)))
    if angle <= 1.0e-9:
        return 0.0, 0.0, 0.0

    ax = (yz - zy) / (2.0 * math.sin(angle))
    ay = (zx - xz) / (2.0 * math.sin(angle))
    az = (xy - yx) / (2.0 * math.sin(angle))
    return ax * angle, ay * angle, az * angle


def pose_target_to_ur_pose(target: ScanPoseTarget) -> tuple[float, float, float, float, float, float]:
    """World-frame TCP pose; orientation from tool +Z (Python path, matches MoveIt quaternion IK)."""
    rx, ry, rz = tool_z_to_rotation_vector(target.tool_z_x, target.tool_z_y, target.tool_z_z)
    return target.x_m, target.y_m, target.z_m, rx, ry, rz


def rotvec_to_quaternion(rx: float, ry: float, rz: float) -> tuple[float, float, float, float]:
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle <= 1.0e-9:
        return 0.0, 0.0, 0.0, 1.0
    half = angle * 0.5
    s = math.sin(half) / angle
    return rx * s, ry * s, rz * s, math.cos(half)


def quat_tool_z_axis(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """World-frame tool0 +Z axis from a unit quaternion."""
    zx = 2.0 * (qx * qz + qw * qy)
    zy = 2.0 * (qy * qz - qw * qx)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)
    return _normalize(zx, zy, zz)


class MoveItProcessManager:
    """Starts headless move_group when planning is requested."""

    def __init__(self, *, ros_distro: str, ur_type: str) -> None:
        self.ros_distro = ros_distro
        self.ur_type = ur_type
        self._process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    @staticmethod
    def _move_group_running() -> bool:
        proc = subprocess.run(
            ["pgrep", "-f", "move_group"],
            capture_output=True,
            text=True,
        )
        return proc.returncode == 0

    @staticmethod
    def current_move_group_pids() -> tuple[int, ...]:
        """Sorted PIDs of running move_group processes (empty when none)."""
        proc = subprocess.run(
            ["pgrep", "-f", "move_group"],
            capture_output=True,
            text=True,
        )
        if proc.returncode != 0:
            return ()
        pids = []
        for line in (proc.stdout or "").split():
            try:
                pids.append(int(line.strip()))
            except ValueError:
                continue
        return tuple(sorted(pids))

    @staticmethod
    def _use_mock_hardware() -> bool:
        return os.environ.get("HYPERFUSION_USE_MOCK_HARDWARE", "false").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

    def _ros_param_bool(self, node: str, param: str) -> Optional[bool]:
        cmd = (
            f"export ROS_LOCALHOST_ONLY=1 && "
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            f"timeout 5 ros2 param get {node} {param}"
        )
        try:
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            return None
        if proc.returncode != 0:
            return None
        text = proc.stdout.strip().lower()
        if "true" in text:
            return True
        if "false" in text:
            return False
        return None

    def _move_group_controller_config_ok(self) -> bool:
        if not self._move_group_running():
            return False
        if self._use_mock_hardware():
            # Mock scan uses joint_trajectory_controller; skip slow ros2 param CLI on WSL.
            return True
        jtc_default = self._ros_param_bool(
            "/move_group",
            "moveit_simple_controller_manager.joint_trajectory_controller.default",
        )
        scaled_default = self._ros_param_bool(
            "/move_group",
            "moveit_simple_controller_manager.scaled_joint_trajectory_controller.default",
        )
        if jtc_default is None or scaled_default is None:
            return False
        if self._use_mock_hardware():
            return jtc_default and not scaled_default
        return scaled_default and not jtc_default

    @staticmethod
    def _stop_move_group() -> None:
        subprocess.run(["pkill", "-f", "moveit_ros_move_group/[m]ove_group"], check=False)
        time.sleep(0.5)

    def _start_headless_stderr_forwarder(self) -> None:
        proc = self._process
        if proc is None or proc.stderr is None:
            return

        def _forward() -> None:
            try:
                for line in proc.stderr:
                    text = line.decode("utf-8", errors="replace").rstrip()
                    if text:
                        sys.stderr.write(f"UR3e MoveIt headless: {text}\n")
            except Exception:
                pass

        threading.Thread(target=_forward, daemon=True, name="ur3e-moveit-headless-log").start()

    def _move_group_ready_for_planning(self) -> bool:
        if not self._compute_ik_service_ready(timeout_s=2.0):
            return False
        if not self._move_group_running():
            return False
        if self._use_mock_hardware():
            return self._move_group_controller_config_ok()
        # Real robot: /compute_ik + move_group up is enough; driver already owns execution.
        return True

    def ensure_running(self, timeout_s: float = 120.0) -> None:
        if not self._use_mock_hardware():
            timeout_s = max(timeout_s, 180.0)
        with self._lock:
            if self._move_group_running() and self._move_group_ready_for_planning():
                return
            if self._move_group_running():
                sys.stderr.write(
                    "UR3e MoveIt: restarting move_group with HyperFusion controller mapping.\n"
                )
                self._stop_move_group()
                self._process = None

            if self._process is not None and self._process.poll() is None:
                deadline = time.time() + timeout_s
                while time.time() < deadline:
                    if self._move_group_running() and self._move_group_ready_for_planning():
                        return
                    time.sleep(0.5)
                raise RuntimeError("MoveIt move_group did not become ready in time.")

            script = PKG_ROOT / "scripts" / "launch_moveit_headless.sh"
            if not script.is_file():
                raise RuntimeError(f"Missing MoveIt launch script: {script}")

            pkg_root = PKG_ROOT.as_posix()
            mock_flag = "true" if self._use_mock_hardware() else "false"
            server_port = os.environ.get("HYPERFUSION_UR3E_SERVER_PORT", "8766")
            sys.stderr.write("UR3e MoveIt: starting headless move_group for scan planning…\n")
            from hyperfusion_ur3e.urdf.mount_config import MountConfig
            from hyperfusion_ur3e.urdf.tool_payload_config import ToolPayloadConfig

            mount_exports = MountConfig.from_env().bash_exports()
            payload_exports = ToolPayloadConfig.from_env().bash_exports()
            runtime_urdf_export = ""
            try:
                from hyperfusion_ur3e.urdf.materialize_robot_description import (
                    materialize_runtime_robot_description,
                )

                runtime_urdf = materialize_runtime_robot_description(
                    PKG_ROOT,
                    ros_distro=self.ros_distro,
                    ur_type=self.ur_type,
                    use_mock_hardware=self._use_mock_hardware(),
                    headless_mode=self._use_mock_hardware(),
                    cfg=ToolPayloadConfig.from_env(),
                )
                runtime_urdf_export = (
                    f"export HYPERFUSION_GENERATED_URDF='{runtime_urdf.as_posix()}' && "
                )
            except Exception as exc:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — could not materialize robot_description "
                    f"before move_group start: {exc}\n"
                )
            cmd = (
                f"export ROS_LOCALHOST_ONLY=1 && "
                f"export HYPERFUSION_UR3E_REPO='{pkg_root}' && "
                f"export HYPERFUSION_UR3E_SERVER_PORT='{server_port}' && "
                f"export HYPERFUSION_USE_MOCK_HARDWARE='{mock_flag}' && "
                f"{mount_exports}"
                f"{payload_exports}"
                f"{runtime_urdf_export}"
                f"sed 's/\\r$//' '{script.as_posix()}' | bash -s {self.ros_distro} {self.ur_type}"
            )
            self._process = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            self._start_headless_stderr_forwarder()

            deadline = time.time() + timeout_s
            last_log_s = 0.0
            while time.time() < deadline:
                if self._process.poll() is not None:
                    stderr_tail = ""
                    if self._process.stderr is not None:
                        stderr_tail = self._process.stderr.read().decode("utf-8", errors="replace")[-1500:]
                    raise RuntimeError(
                        f"MoveIt headless launch exited (code={self._process.returncode}).\n{stderr_tail}"
                    )
                if self._move_group_ready_for_planning():
                    self._verify_move_group_tool_payload()
                    sys.stderr.write("UR3e MoveIt: move_group ready for scan planning.\n")
                    return
                now = time.time()
                if now - last_log_s >= 10.0:
                    elapsed = int(now - (deadline - timeout_s))
                    sys.stderr.write(
                        f"UR3e MoveIt: waiting for move_group /compute_ik ({elapsed}s)…\n"
                    )
                    last_log_s = now
                time.sleep(1.0)

            raise RuntimeError("Timed out waiting for MoveIt /compute_ik service.")

    @staticmethod
    def _compute_ik_service_ready(timeout_s: float) -> bool:
        """Probe /compute_ik via rclpy (ros2 service list can hang on WSL)."""
        try:
            import rclpy
            from moveit_msgs.srv import GetPositionIK
            from rclpy.node import Node
        except Exception:
            return False

        owns_init = False
        if not rclpy.ok():
            rclpy.init()
            owns_init = True
        node = Node("hyperfusion_moveit_ready_probe")
        client = node.create_client(GetPositionIK, "/compute_ik")
        try:
            return client.wait_for_service(timeout_sec=max(0.5, timeout_s))
        finally:
            node.destroy_node()
            # Keep rclpy initialized for MoveItScanPlanner._ensure_ros().
            if owns_init and not rclpy.ok():
                rclpy.shutdown()

    @staticmethod
    def _verify_move_group_tool_payload() -> None:
        """Warn when MoveIt's robot model is missing the tool-flange payload link."""
        from hyperfusion_ur3e.urdf.tool_payload_config import (
            ToolPayloadConfig,
            robot_description_has_tool_payload,
        )
        from hyperfusion_ur3e.urdf.materialize_robot_description import (
            load_runtime_robot_description_text,
        )

        cfg = ToolPayloadConfig.from_env()
        if not cfg.enabled:
            return

        urdf = load_runtime_robot_description_text(PKG_ROOT)
        if urdf and robot_description_has_tool_payload(urdf, expect_shape=cfg.shape):
            sys.stderr.write(
                "UR3e MoveIt: tool payload collision enabled "
                f"({cfg.shape}, radius={cfg.radius_m * 1000.0:.0f} mm, materialized URDF).\n"
            )
            return

        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.parameter_client import SyncParametersClient
        except Exception:
            return

        owns_init = False
        if not rclpy.ok():
            rclpy.init()
            owns_init = True

        node = Node("hyperfusion_moveit_payload_probe")
        try:
            client = SyncParametersClient(node, "/move_group")
            if not client.wait_for_services(timeout_sec=5.0):
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — cannot verify tool payload link "
                    "(move_group parameter service unavailable).\n"
                )
                return

            values = client.get_parameters(["robot_description"])
            if not values or values[0].type_ == 0:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — move_group robot_description unavailable; "
                    "tool payload dome may not be in collision checks.\n"
                )
                return

            urdf = values[0].string_value
            if TOOL_PAYLOAD_LINK not in urdf:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — robot_description has no "
                    f"'{TOOL_PAYLOAD_LINK}' link; the camera dome is NOT in MoveIt "
                    "collision checks. Restart sidecar after changing tool_payload_radius_mm.\n"
                )
                return

            if "tool_payload_hemisphere.stl" not in urdf and cfg.shape == "hemisphere":
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — tool payload hemisphere mesh missing from "
                    "robot_description; dome collision may be disabled.\n"
                )
                return

            sys.stderr.write(
                "UR3e MoveIt: tool payload collision enabled "
                f"({cfg.shape}, radius={cfg.radius_m * 1000.0:.0f} mm).\n"
            )
        except Exception as exc:
            sys.stderr.write(
                "UR3e MoveIt: WARNING — tool payload verification failed: "
                f"{exc}\n"
            )
        finally:
            node.destroy_node()
            if owns_init and rclpy.ok():
                rclpy.shutdown()

    @staticmethod
    def _service_ready(service_name: str, timeout_s: float, ros_distro: str) -> bool:
        if service_name == "/compute_ik":
            return MoveItProcessManager._compute_ik_service_ready(timeout_s)
        cmd = (
            f"export ROS_LOCALHOST_ONLY=1 && "
            f"source /opt/ros/{ros_distro}/setup.bash && "
            f"timeout {max(1, int(timeout_s))} ros2 service list"
        )
        try:
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=max(2.0, timeout_s + 2.0),
            )
        except subprocess.TimeoutExpired:
            return False
        return service_name in (proc.stdout or "")


class MoveItScanPlanner:
    """IK + collision validity for hemisphere scan poses."""

    def __init__(self, *, ros_distro: str = "jazzy", ur_type: str = "ur3e") -> None:
        self.ros_distro = ros_distro
        self.ur_type = ur_type
        self._process_manager = MoveItProcessManager(ros_distro=ros_distro, ur_type=ur_type)
        self._lock = threading.Lock()
        self._node = None
        self._ik_client = None
        self._validity_client = None
        self._fk_client = None
        self._pinch_guard_warned = False
        self._move_client = None
        self._execute_trajectory_client = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_stop = threading.Event()
        self._workspace_applied: Optional[tuple[Any, ...]] = None
        self._last_workspace: Optional[WorkspaceBox] = None
        self._default_workspace: Optional[WorkspaceBox] = None
        self._boundary_keepalive_thread: Optional[threading.Thread] = None
        self._boundary_keepalive_stop = threading.Event()
        self._last_plan_start_seed: List[float] = []
        self._home_joints_rad: List[float] = list(DEFAULT_HOME_JOINTS_RAD)
        self._planning_scene_client = None
        self._move_goal_lock = threading.Lock()
        self._active_move_goal_handle: Any = None
        self._latest_joint_positions: List[float] = []
        self._hardware_joint_positions_cache: List[float] = []
        self._joint_state_sub = None
        self._rviz_goal_pub = None
    def apply_home_joints_from_body(self, body: Optional[Dict[str, Any]]) -> None:
        self._home_joints_rad = home_joints_rad_from_body(body)

    def home_joints_rad(self) -> List[float]:
        return list(self._home_joints_rad)

    def _ensure_ros(self, *, require_ik: bool = True) -> None:
        if self._node is not None:
            if require_ik:
                self._ensure_ik_clients()
            return

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("hyperfusion_ur3e_scan_planner")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True, name="ur3e-moveit-spin")
        self._spin_thread.start()

        if require_ik:
            self._ensure_ik_clients()

    def _ensure_ik_clients(self) -> None:
        if self._node is None:
            raise RuntimeError("ROS node not initialized.")

        if self._ik_client is not None and self._validity_client is not None:
            return

        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionIK, GetStateValidity

        if self._ik_client is None:
            self._ik_client = self._node.create_client(GetPositionIK, "/compute_ik")
        if self._validity_client is None:
            self._validity_client = self._node.create_client(
                GetStateValidity, "/check_state_validity"
            )
        if self._rviz_goal_pub is None:
            self._rviz_goal_pub = self._node.create_publisher(
                RobotState, RVIZ_UPDATE_CUSTOM_GOAL_TOPIC, 10
            )

        if not self._ik_client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("MoveIt /compute_ik service not available.")
        if not self._validity_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("MoveIt /check_state_validity service not available.")

        if self._joint_state_sub is None:
            from sensor_msgs.msg import JointState

            def _on_joint_state(msg: JointState) -> None:
                if not msg.name:
                    return
                name_to_pos = {
                    name: float(pos)
                    for name, pos in zip(msg.name, msg.position)
                    if name in CANONICAL_JOINT_NAMES
                }
                if len(name_to_pos) == 6:
                    self._latest_joint_positions = [
                        name_to_pos[name] for name in CANONICAL_JOINT_NAMES
                    ]

            def _on_hardware_joint_state(msg: JointState) -> None:
                if not msg.name:
                    return
                name_to_pos = {
                    name: float(pos)
                    for name, pos in zip(msg.name, msg.position)
                    if name in CANONICAL_JOINT_NAMES
                }
                if len(name_to_pos) == 6:
                    self._hardware_joint_positions_cache = [
                        name_to_pos[name] for name in CANONICAL_JOINT_NAMES
                    ]

            self._joint_state_sub = self._node.create_subscription(
                JointState,
                "/joint_states_stamped",
                _on_joint_state,
                10,
            )
            self._hardware_joint_state_sub = self._node.create_subscription(
                JointState,
                "/joint_state_broadcaster/joint_states",
                _on_hardware_joint_state,
                10,
            )

    def _ensure_fk_client(self) -> None:
        if self._node is None:
            raise RuntimeError("ROS node not initialized.")
        if self._fk_client is not None:
            return

        from moveit_msgs.srv import GetPositionFK

        for service_name in ("/compute_fk", "/move_group/compute_fk"):
            client = self._node.create_client(GetPositionFK, service_name)
            if client.wait_for_service(timeout_sec=2.0):
                self._fk_client = client
                return
        self._fk_client = None

    @staticmethod
    def _use_mock_hardware() -> bool:
        return os.environ.get("HYPERFUSION_USE_MOCK_HARDWARE", "false").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

    def _fk_link_positions(self, joints: Sequence[float]) -> Optional[dict[str, tuple[float, float, float]]]:
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        from hyperfusion_ur3e.moveit.ur_pinch_guard import PINCH_GUARD_LINKS, vec3_from_pose

        if self._use_mock_hardware():
            return None

        try:
            self._ensure_fk_client()
        except Exception:
            return None
        if self._fk_client is None:
            return None

        from moveit_msgs.srv import GetPositionFK

        request = GetPositionFK.Request()
        request.header.frame_id = PLANNING_FRAME
        request.fk_link_names = list(PINCH_GUARD_LINKS)
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        request.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._fk_client.call_async(request)
        response = self._wait_future(future, 5.0)
        if response is None or response.error_code.val != 1:
            return None

        positions: dict[str, tuple[float, float, float]] = {}
        for link_name, pose_stamped in zip(PINCH_GUARD_LINKS, response.pose_stamped):
            positions[link_name] = vec3_from_pose(pose_stamped.pose)
        return positions

    def _ur_pinch_guard_ok(self, joints: Sequence[float]) -> tuple[bool, str]:
        from hyperfusion_ur3e.moveit.ur_pinch_guard import ur_pinch_violation

        if self._use_mock_hardware():
            return True, ""

        link_positions = self._fk_link_positions(joints)
        if link_positions is None:
            if not self._pinch_guard_warned:
                self._pinch_guard_warned = True
                sys.stderr.write(
                    "UR3e MoveIt: pinch guard skipped — /compute_fk unavailable.\n"
                )
            return True, ""

        tool_z = self._fk_tool0_z_axis(joints)
        reason = ur_pinch_violation(link_positions, tool_z=tool_z)
        if reason:
            return False, reason
        return True, ""

    def _fk_tool0_z_axis(
        self, joints: Sequence[float]
    ) -> Optional[tuple[float, float, float]]:
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        if self._use_mock_hardware():
            return None

        try:
            self._ensure_fk_client()
        except Exception:
            return None
        if self._fk_client is None:
            return None

        from moveit_msgs.srv import GetPositionFK

        request = GetPositionFK.Request()
        request.header.frame_id = PLANNING_FRAME
        request.fk_link_names = [EE_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        request.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._fk_client.call_async(request)
        response = self._wait_future(future, 5.0)
        if response is None or response.error_code.val != 1 or not response.pose_stamped:
            return None

        orientation = response.pose_stamped[0].pose.orientation
        return quat_tool_z_axis(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

    def _tool_z_alignment(
        self,
        joints: Sequence[float],
        desired_tool_z: Sequence[float],
    ) -> Optional[float]:
        actual = self._fk_tool0_z_axis(joints)
        if actual is None:
            return None

        desired = _normalize(
            float(desired_tool_z[0]),
            float(desired_tool_z[1]),
            float(desired_tool_z[2]),
        )
        return actual[0] * desired[0] + actual[1] * desired[1] + actual[2] * desired[2]

    def set_default_workspace(self, workspace: WorkspaceBox) -> None:
        self._default_workspace = workspace
        self._last_workspace = workspace

    def start_boundary_keepalive(self) -> None:
        with self._lock:
            if (
                self._boundary_keepalive_thread is not None
                and self._boundary_keepalive_thread.is_alive()
            ):
                return
            self._boundary_keepalive_stop.clear()
            self._boundary_keepalive_thread = threading.Thread(
                target=self._boundary_keepalive_loop,
                daemon=True,
                name="ur3e-boundary-keepalive",
            )
            self._boundary_keepalive_thread.start()

    def _boundary_keepalive_loop(self) -> None:
        while True:
            if self._boundary_keepalive_stop.wait(3.0):
                break

            workspace = self._default_workspace
            if workspace is None or not self._process_manager.current_move_group_pids():
                continue

            with self._move_goal_lock:
                motion_active = self._active_move_goal_handle is not None
            if motion_active:
                continue

            workspace_key = self._workspace_key(workspace)
            if workspace_key == self._workspace_applied:
                continue

            try:
                self.ensure_workspace_boundary_visible(workspace)
            except Exception as exc:
                sys.stderr.write(f"UR3e MoveIt: boundary keepalive: {exc}\n")

    def ensure_workspace_boundary_visible(self, workspace: WorkspaceBox) -> bool:
        """Publish workspace collision walls into the active MoveIt planning scene."""
        if not workspace.enabled:
            return False
        if not self._process_manager.current_move_group_pids():
            return False

        workspace_key = self._workspace_key(workspace)
        if workspace_key == self._workspace_applied:
            return True

        with self._lock:
            self._ensure_ros(require_ik=False)
            self._last_workspace = workspace
        return self._apply_workspace_collision(workspace)

    def _current_joint_positions(self) -> Optional[List[float]]:
        if len(self._latest_joint_positions) != 6:
            return None
        return list(self._latest_joint_positions)

    def _hardware_joint_positions(self) -> Optional[List[float]]:
        if len(self._hardware_joint_positions_cache) != 6:
            return None
        return list(self._hardware_joint_positions_cache)

    def _ensure_hardware_joint_positions(self, timeout_s: float = 2.0) -> Optional[List[float]]:
        """Wait for raw /joint_state_broadcaster feedback (RTDE branch for execute)."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            hardware = self._hardware_joint_positions()
            if hardware is not None:
                return hardware
            if self._executor is not None:
                try:
                    self._executor.spin_once(timeout_sec=0.05)
                except Exception:
                    break
            else:
                time.sleep(0.05)
        hardware = self._hardware_joint_positions()
        if hardware is None:
            sys.stderr.write(
                "UR3e MoveIt execute: raw hardware joint_states unavailable — "
                "trajectory may use MoveIt branch (controller reject risk).\n"
            )
        return hardware

    def _wait_for_planner_joint_feedback(self, timeout_s: float = 2.0) -> None:
        """Spin until MoveIt / hardware joint caches are populated."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._moveit_start_joint_positions() is not None:
                return
            if self._executor is not None:
                try:
                    self._executor.spin_once(timeout_sec=0.05)
                except Exception:
                    break
            else:
                time.sleep(0.05)

    def _resolve_execute_branch(
        self,
        goal_joints: Sequence[float],
        *,
        direct_only: bool,
        tcp_target: Optional[Dict[str, Any]],
    ) -> tuple[List[float], Optional[List[float]]]:
        """Coalesce goal to live MoveIt branch after ROS joint feedback is available."""
        resolved_goal = [float(v) for v in goal_joints]
        current_joints = self._current_joint_positions()
        if current_joints is None:
            hardware = self._hardware_joint_positions()
            if hardware is not None:
                current_joints = coalesce_joints_for_moveit(hardware, hardware)

        moveit_start = self._moveit_start_joint_positions()
        branch_ref = (
            list(moveit_start)
            if moveit_start is not None
            else (list(current_joints) if current_joints is not None else None)
        )

        if branch_ref is not None:
            resolved_goal = self._coalesce_goal_joints(branch_ref, resolved_goal)
            if not direct_only and len(self._last_plan_start_seed) == 6:
                resolved_goal = self._coalesce_goal_joints(
                    self._last_plan_start_seed, resolved_goal
                )
            if isinstance(tcp_target, dict):
                refreshed = self._goal_joints_from_tcp(tcp_target, branch_ref)
                if refreshed is not None:
                    resolved_goal = refreshed

        return resolved_goal, branch_ref

    def _moveit_start_joint_positions(self) -> Optional[List[float]]:
        """Current joints on the MoveIt /joint_states branch (from stamper)."""
        current = self._current_joint_positions()
        if current is not None:
            return list(current)
        hardware = self._hardware_joint_positions()
        if hardware is None:
            return None
        return coalesce_joints_for_moveit(hardware, hardware)

    def _coalesce_goal_joints(
        self,
        reference: Sequence[float],
        goal: Sequence[float],
    ) -> List[float]:
        """Align UI / plan goal angles to the same 2π branch as *reference*."""
        return coalesce_joints_for_moveit(reference, goal)

    def _wait_for_joint_positions_near(
        self,
        target: Sequence[float],
        *,
        tolerance_rad: float = HOME_JOINT_TOLERANCE_RAD,
        timeout_s: float = JOINT_SETTLE_TIMEOUT_S,
        stop_event: Optional[threading.Event] = None,
        reference: Optional[Sequence[float]] = None,
    ) -> Optional[List[float]]:
        """Poll /joint_states_stamped until the robot is near *target* (or timeout)."""
        ref = reference if reference is not None else target
        target_norm = self._normalize_joint_solution_to_reference(ref, target)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if stop_event is not None and stop_event.is_set():
                self.cancel_active_move()
                raise RuntimeError("Motion stopped.")
            current = self._current_joint_positions()
            if current is not None:
                current = self._normalize_joint_solution_to_reference(ref, current)
                if self._joint_distance_rad(current, target_norm) <= tolerance_rad:
                    return current
            time.sleep(0.05)
        current = self._current_joint_positions()
        if current is not None:
            return self._normalize_joint_solution_to_reference(ref, current)
        return None

    def _waypoint_after_commanded_move(
        self,
        commanded: Sequence[float],
        *,
        stop_event: Optional[threading.Event] = None,
        reference: Optional[Sequence[float]] = None,
    ) -> List[float]:
        """Use commanded joints as the motion waypoint; prefer settled joint feedback when close."""
        ref = reference if reference is not None else commanded
        commanded_norm = self._normalize_joint_solution_to_reference(ref, commanded)
        settled = self._wait_for_joint_positions_near(
            commanded_norm,
            stop_event=stop_event,
            reference=ref,
        )
        if settled is None:
            sys.stderr.write(
                "UR3e MoveIt execute: joint feedback unavailable after move — "
                "using commanded waypoint.\n"
            )
            return commanded_norm
        if self._joint_distance_rad(settled, commanded_norm) > HOME_JOINT_TOLERANCE_RAD:
            sys.stderr.write(
                "UR3e MoveIt execute: joint feedback not yet at commanded waypoint — "
                "using commanded joints for approach reference.\n"
            )
            return commanded_norm
        return settled

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return wrap_to_pi(angle)

    @classmethod
    def _pick_joint_branch(cls, index: int, value: float, reference: float) -> float:
        return pick_joint_branch(index, value, reference)

    @classmethod
    def _unwrap_joint_to_reference(
        cls,
        joint_index: int,
        raw: float,
        reference: float,
    ) -> float:
        """Chain trajectory samples; wrist_3 is continuous — never wrap to (-π, π]."""
        if joint_index == WRIST_3_JOINT_INDEX:
            return float(reference) + cls._joint_delta_rad(reference, raw)
        return cls._pick_joint_branch(joint_index, raw, reference)

    @classmethod
    def _normalize_joint_solution_to_reference(
        cls,
        reference: Sequence[float],
        joints: Sequence[float],
    ) -> List[float]:
        return normalize_joint_solution_to_reference(reference, joints)

    @staticmethod
    def _joint_delta_rad(a: float, b: float) -> float:
        return joint_delta_rad(a, b)

    @classmethod
    def _joint_distance_rad(cls, reference: Sequence[float], candidate: Sequence[float]) -> float:
        return joint_distance_rad(reference, candidate)

    @classmethod
    def _joints_within_ur3e_limits(cls, joints: Sequence[float]) -> bool:
        if len(joints) != 6:
            return False
        for index, value in enumerate(joints):
            limits = UR3E_JOINT_LIMITS_RAD[index]
            if limits is None:
                continue
            lo, hi = limits
            if float(value) < lo - 1e-6 or float(value) > hi + 1e-6:
                return False
        return True

    @classmethod
    def _ik_joint_angles_are_sane(cls, joints: Sequence[float]) -> bool:
        """Reject IK branches outside UR3e/MoveIt joint limits."""
        return cls._joints_within_ur3e_limits(joints)

    def _spin_loop(self) -> None:
        while not self._spin_stop.is_set():
            if self._executor is None:
                break
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                if self._spin_stop.is_set():
                    break
                raise

    def _wait_future(
        self,
        future: Any,
        timeout_s: float,
        stop_event: Optional[threading.Event] = None,
    ) -> Any:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if stop_event is not None and stop_event.is_set():
                self.cancel_active_move()
                raise RuntimeError("Motion stopped.")
            if future.done():
                return future.result()
            time.sleep(0.02)
        raise TimeoutError(f"MoveIt call timed out after {timeout_s:.0f}s.")

    def cancel_active_move(self) -> None:
        with self._move_goal_lock:
            handle = self._active_move_goal_handle
        if handle is None:
            return
        try:
            cancel_future = handle.cancel_goal_async()
            deadline = time.time() + 2.0
            while time.time() < deadline and not cancel_future.done():
                time.sleep(0.02)
        except Exception:
            pass
        with self._move_goal_lock:
            self._active_move_goal_handle = None

    def _workspace_key(self, workspace: WorkspaceBox) -> tuple[Any, ...]:
        # Include the live move_group PID set: a restarted or GUI-launched move_group
        # starts with an empty planning scene, so the boundary must be re-published.
        return (
            self._process_manager.current_move_group_pids(),
            workspace.enabled,
            round(workspace.length_m, 6),
            round(workspace.width_m, 6),
            round(workspace.height_m, 6),
        )

    def _ensure_planning_scene_client(self) -> None:
        if self._planning_scene_client is not None:
            return

        from moveit_msgs.srv import ApplyPlanningScene

        self._planning_scene_client = self._node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        if not self._planning_scene_client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("MoveIt /apply_planning_scene service not available.")

    def _apply_planning_scene_object(self, collision_object: Any) -> None:
        self._apply_planning_scene_objects([collision_object])

    def _apply_planning_scene_objects(self, collision_objects: Sequence[Any]) -> None:
        if not collision_objects:
            return

        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene

        self._ensure_planning_scene_client()

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend(collision_objects)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._planning_scene_client.call_async(request)
        self._wait_future(future, 30.0)

    def _remove_planning_scene_object(self, object_id: str) -> None:
        from moveit_msgs.msg import CollisionObject

        remove_object = CollisionObject()
        remove_object.id = object_id
        remove_object.operation = CollisionObject.REMOVE
        self._apply_planning_scene_object(remove_object)

    def _make_box_collision_object(
        self,
        object_id: str,
        *,
        size_x: float,
        size_y: float,
        size_z: float,
        center_x: float,
        center_y: float,
        center_z: float,
    ) -> Any:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = PLANNING_FRAME

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]

        pose = Pose()
        pose.position.x = float(center_x)
        pose.position.y = float(center_y)
        pose.position.z = float(center_z)
        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _make_workspace_boundary_objects(self, workspace: WorkspaceBox) -> List[Any]:
        """Thin collision slabs on all six faces — any robot link contact fails planning."""
        thickness = BOUNDARY_SLAB_THICKNESS_M
        half_l = workspace.length_m * 0.5
        half_w = workspace.width_m * 0.5
        height_m = workspace.height_m
        wall_height = max(thickness, height_m)

        return [
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[0],
                size_x=workspace.length_m,
                size_y=workspace.width_m,
                size_z=thickness,
                center_x=0.0,
                center_y=0.0,
                center_z=-thickness * 0.5,
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[1],
                size_x=workspace.length_m,
                size_y=workspace.width_m,
                size_z=thickness,
                center_x=0.0,
                center_y=0.0,
                center_z=height_m + (thickness * 0.5),
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[2],
                size_x=thickness,
                size_y=workspace.width_m,
                size_z=wall_height,
                center_x=half_l + (thickness * 0.5),
                center_y=0.0,
                center_z=wall_height * 0.5,
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[3],
                size_x=thickness,
                size_y=workspace.width_m,
                size_z=wall_height,
                center_x=-half_l - (thickness * 0.5),
                center_y=0.0,
                center_z=wall_height * 0.5,
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[4],
                size_x=workspace.length_m,
                size_y=thickness,
                size_z=wall_height,
                center_x=0.0,
                center_y=half_w + (thickness * 0.5),
                center_z=wall_height * 0.5,
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[5],
                size_x=workspace.length_m,
                size_y=thickness,
                size_z=wall_height,
                center_x=0.0,
                center_y=-half_w - (thickness * 0.5),
                center_z=wall_height * 0.5,
            ),
        ]

    def _clear_workspace_boundary_objects(self) -> None:
        from moveit_msgs.msg import CollisionObject

        remove_objects = []
        for object_id in BOUNDARY_OBJECT_IDS:
            remove_object = CollisionObject()
            remove_object.id = object_id
            remove_object.operation = CollisionObject.REMOVE
            remove_objects.append(remove_object)
        try:
            self._apply_planning_scene_objects(remove_objects)
        except Exception as exc:
            sys.stderr.write(f"UR3e MoveIt: failed to clear boundary objects: {exc}\n")

    def _validity_failure_reason(self, response: Any) -> str:
        contacts_state = getattr(response, "contacts", None)
        contact_infos = getattr(contacts_state, "contacts", None) if contacts_state else None
        if contact_infos:
            for contact in contact_infos:
                bodies = [
                    str(getattr(contact, attr, "") or "")
                    for attr in ("contact_body_1", "contact_body_2")
                ]
                for body in bodies:
                    if "hyperfusion_boundary_" in body:
                        return "robot arm outside workspace boundary"
                if any(TOOL_PAYLOAD_LINK in body for body in bodies):
                    other = bodies[1] if TOOL_PAYLOAD_LINK in bodies[0] else bodies[0]
                    if "forearm" in other:
                        return "tool payload near forearm (pinch / fold risk)"
                    if "hyperfusion_boundary_" in other:
                        return "tool payload outside workspace boundary"
                    return f"tool payload collision ({bodies[0]} vs {bodies[1]})"
            return f"collision ({len(contact_infos)} contact(s))"
        return "collision or joint limit violation"

    def _apply_workspace_collision(self, workspace: WorkspaceBox) -> bool:
        """Install full workspace box collision (floor, walls, ceiling) for whole-arm checks."""
        workspace_key = self._workspace_key(workspace)
        if self._workspace_applied == workspace_key:
            return True

        if not workspace.enabled:
            if self._workspace_applied is not None:
                self._clear_workspace_boundary_objects()
            self._workspace_applied = workspace_key
            sys.stderr.write("UR3e MoveIt: workspace boundary disabled.\n")
            return True

        collision_objects = self._make_workspace_boundary_objects(workspace)
        self._apply_planning_scene_objects(collision_objects)

        self._workspace_applied = workspace_key
        sys.stderr.write(
            "UR3e MoveIt: workspace boundary enabled "
            f"(whole-arm collision box {workspace.length_m:.3f} x "
            f"{workspace.width_m:.3f} x {workspace.height_m:.3f} m).\n"
        )
        return True

    @staticmethod
    def _moveit_error_name(code: int) -> str:
        from moveit_msgs.msg import MoveItErrorCodes

        names = {
            MoveItErrorCodes.SUCCESS: "success",
            MoveItErrorCodes.FAILURE: "failure",
            MoveItErrorCodes.PLANNING_FAILED: "planning failed",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "no collision-free path (invalid motion plan)",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: (
                "motion plan invalidated by environment change"
            ),
            MoveItErrorCodes.CONTROL_FAILED: "controller rejected trajectory",
            MoveItErrorCodes.START_STATE_INVALID: "start state invalid (joint limits?)",
            MoveItErrorCodes.GOAL_STATE_INVALID: "goal state invalid (joint limits?)",
            MoveItErrorCodes.NO_IK_SOLUTION: "no IK solution",
            MoveItErrorCodes.TIMED_OUT: "timed out",
            MoveItErrorCodes.PREEMPTED: "preempted",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "start state in collision",
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "start state violates constraints",
            MoveItErrorCodes.GOAL_IN_COLLISION: "goal in collision",
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "goal violates constraints",
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "goal constraints violated",
            MoveItErrorCodes.INVALID_GROUP_NAME: "invalid group name",
            MoveItErrorCodes.INVALID_LINK_NAME: "invalid link name",
            MoveItErrorCodes.INVALID_OBJECT_NAME: "invalid object name",
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "frame transform failure",
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "collision checking unavailable",
            MoveItErrorCodes.ROBOT_STATE_STALE: "robot state stale",
            MoveItErrorCodes.SENSOR_INFO_STALE: "sensor info stale",
        }
        return names.get(code, f"MoveIt error {code}")

    def _make_robot_state(self, joint_positions: Sequence[float]) -> Any:
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        if len(joint_positions) != 6:
            raise ValueError("joint seed must have 6 values")

        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        state.joint_state.position = [float(v) for v in joint_positions]
        return state

    def _publish_rviz_goal_state(self, joint_positions: Sequence[float]) -> None:
        """Update MoveIt RViz Query Goal State (orange ghost) during plan/execute."""
        if self._rviz_goal_pub is None:
            return
        try:
            self._rviz_goal_pub.publish(self._make_robot_state(joint_positions))
        except Exception as exc:
            sys.stderr.write(f"UR3e MoveIt: could not publish RViz goal state: {exc}\n")

    def update_manual_target_preview(
        self,
        joint_positions: Sequence[float],
        workspace: Optional[WorkspaceBox] = None,
    ) -> Dict[str, Any]:
        """Push the UI joint target into MoveIt/RViz and refresh the collision boundary."""
        if len(joint_positions) != 6:
            raise ValueError("Manual target must have 6 joint values.")

        goal_joints = [float(v) for v in joint_positions]
        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            ws = workspace if workspace is not None else self._last_workspace
            if ws is None:
                ws = WorkspaceBox(enabled=True)
            self._apply_workspace_collision(ws)
            self._publish_rviz_goal_state(goal_joints)

        return {"ok": True}

    def _strip_trajectory_derivatives(self, trajectory: Any) -> None:
        """UR hardware controller expects position-only trajectories with time_from_start."""
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None:
            return
        for point in joint_traj.points:
            point.velocities = []
            point.accelerations = []

    def _unwrap_trajectory_waypoints(
        self,
        joint_traj: Any,
        indices: Sequence[int],
        previous: Sequence[float],
        *,
        start_point_index: int = 0,
    ) -> float:
        """Rewrite trajectory samples onto the branch chain starting at *previous*."""
        max_step_rad = 0.0
        prev = [float(v) for v in previous]
        for point_index, point in enumerate(joint_traj.points):
            if point_index < start_point_index:
                continue
            if len(point.positions) <= max(indices):
                continue
            unwrapped: List[float] = []
            for joint_index, traj_index in enumerate(indices):
                raw = float(point.positions[traj_index])
                value = self._unwrap_joint_to_reference(
                    joint_index, raw, prev[joint_index]
                )
                unwrapped.append(value)
                max_step_rad = max(
                    max_step_rad,
                    abs(self._joint_delta_rad(prev[joint_index], value)),
                )
                point.positions[traj_index] = value
            prev = unwrapped
        return max_step_rad

    def _unwrap_trajectory_continuous(self, trajectory: Any) -> bool:
        """Remap trajectory joints to the hardware branch and keep waypoints continuous."""
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None or not joint_traj.joint_names or not joint_traj.points:
            return False

        name_to_index = {
            name: index for index, name in enumerate(joint_traj.joint_names)
        }
        indices: List[int] = []
        for joint_name in CANONICAL_JOINT_NAMES:
            if joint_name not in name_to_index:
                return False
            indices.append(name_to_index[joint_name])

        hardware = self._ensure_hardware_joint_positions(timeout_s=1.5)
        if hardware is not None:
            previous = [float(v) for v in hardware]
        elif self._moveit_start_joint_positions() is not None:
            previous = self._moveit_start_joint_positions()  # type: ignore[assignment]
        else:
            first = joint_traj.points[0]
            if len(first.positions) <= max(indices):
                return False
            previous = [
                float(first.positions[index]) for index in indices
            ]

        max_step_rad = self._unwrap_trajectory_waypoints(
            joint_traj, indices, previous, start_point_index=0
        )

        if hardware is not None and joint_traj.points:
            first = joint_traj.points[0]
            if len(first.positions) > max(indices):
                first_positions = [
                    float(first.positions[index]) for index in indices
                ]
                start_gap = self._joint_distance_rad(hardware, first_positions)
                for joint_index, traj_index in enumerate(indices):
                    first.positions[traj_index] = float(hardware[joint_index])
                if len(joint_traj.points) > 1:
                    tail_step = self._unwrap_trajectory_waypoints(
                        joint_traj,
                        indices,
                        hardware,
                        start_point_index=1,
                    )
                    max_step_rad = max(max_step_rad, tail_step)
                if start_gap > TRAJECTORY_START_TOLERANCE_RAD:
                    sys.stderr.write(
                        "UR3e MoveIt execute: trajectory start snapped to hardware joints "
                        f"(plan start was {math.degrees(start_gap):.1f}°·joint away).\n"
                    )

        if max_step_rad > math.radians(90.0):
            sys.stderr.write(
                "UR3e MoveIt execute: trajectory unwrap — "
                f"max joint step {math.degrees(max_step_rad):.1f}° between samples.\n"
            )
        return True

    @staticmethod
    def _trajectory_joint_indices(trajectory: Any) -> Optional[List[int]]:
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None or not joint_traj.joint_names:
            return None
        name_to_index = {
            name: index for index, name in enumerate(joint_traj.joint_names)
        }
        indices: List[int] = []
        for joint_name in CANONICAL_JOINT_NAMES:
            if joint_name not in name_to_index:
                return None
            indices.append(name_to_index[joint_name])
        return indices

    @staticmethod
    def _point_time_from_start_sec(point: Any) -> float:
        stamp = point.time_from_start
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _set_point_time_from_start_sec(point: Any, seconds: float) -> None:
        seconds = max(0.0, float(seconds))
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

    def _trajectory_max_joint_velocity_rad_s(self, trajectory: Any) -> float:
        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if indices is None or joint_traj is None or len(joint_traj.points) < 2:
            return 0.0

        max_velocity = 0.0
        previous_positions: Optional[List[float]] = None
        previous_time = 0.0
        for point in joint_traj.points:
            if len(point.positions) <= max(indices):
                continue
            positions = [float(point.positions[index]) for index in indices]
            time_s = self._point_time_from_start_sec(point)
            if previous_positions is not None:
                dt = time_s - previous_time
                if dt > 1.0e-6:
                    for prev, curr in zip(previous_positions, positions):
                        velocity = abs(self._joint_delta_rad(prev, curr)) / dt
                        if velocity > max_velocity:
                            max_velocity = velocity
            previous_positions = positions
            previous_time = time_s
        return max_velocity

    def _stretch_trajectory_segment_times(self, trajectory: Any) -> None:
        """Ensure each segment respects joint speed and UR External Control sample period."""
        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if indices is None or joint_traj is None or len(joint_traj.points) < 2:
            return

        limit = configured_max_joint_velocity_rad_s()
        margin = UR3E_JOINT_VELOCITY_TIME_MARGIN
        prev_positions: Optional[List[float]] = None
        prev_time = 0.0
        for point_index, point in enumerate(joint_traj.points):
            if len(point.positions) <= max(indices):
                continue
            positions = [float(point.positions[index]) for index in indices]
            if point_index == 0:
                self._set_point_time_from_start_sec(point, 0.0)
                prev_positions = positions
                prev_time = 0.0
                continue

            dt_required = MIN_TRAJECTORY_SEGMENT_S
            if prev_positions is not None:
                for prev, curr in zip(prev_positions, positions):
                    delta = abs(self._joint_delta_rad(prev, curr))
                    if delta > 1e-9:
                        dt_required = max(dt_required, (delta / limit) * margin)

            new_time = max(self._point_time_from_start_sec(point), prev_time + dt_required)
            self._set_point_time_from_start_sec(point, new_time)
            prev_time = new_time
            prev_positions = positions

    def _ensure_trajectory_hardware_start(self, trajectory: Any) -> None:
        """Insert a t=0 hold at live RTDE joints so UR never sees a 2 ms first step."""
        from trajectory_msgs.msg import JointTrajectoryPoint

        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        hardware = self._ensure_hardware_joint_positions(timeout_s=0.5)
        if (
            indices is None
            or joint_traj is None
            or not joint_traj.points
            or hardware is None
        ):
            return

        if len(joint_traj.points) == 1:
            goal = joint_traj.points[0]
            start = JointTrajectoryPoint()
            start.positions = list(goal.positions)
            for joint_index, traj_index in enumerate(indices):
                start.positions[traj_index] = float(hardware[joint_index])
            self._set_point_time_from_start_sec(start, 0.0)
            joint_traj.points = [start, goal]
            self._unwrap_trajectory_waypoints(
                joint_traj, indices, hardware, start_point_index=1
            )
            return

        first = joint_traj.points[0]
        if len(first.positions) <= max(indices):
            return

        first_time = self._point_time_from_start_sec(first)
        first_positions = [float(first.positions[index]) for index in indices]
        start_gap = self._joint_distance_rad(hardware, first_positions)
        if first_time <= 1e-6 and start_gap <= TRAJECTORY_START_TOLERANCE_RAD:
            return

        hold = JointTrajectoryPoint()
        hold.positions = list(first.positions)
        for joint_index, traj_index in enumerate(indices):
            hold.positions[traj_index] = float(hardware[joint_index])
        self._set_point_time_from_start_sec(hold, 0.0)
        joint_traj.points.insert(0, hold)
        self._unwrap_trajectory_waypoints(
            joint_traj, indices, hardware, start_point_index=1
        )

    def _prepare_trajectory_for_robot(self, trajectory: Any) -> bool:
        """Unwrap ±π joint branches and enforce configurable peak joint velocity."""
        if not self._unwrap_trajectory_continuous(trajectory):
            return False

        self._ensure_trajectory_hardware_start(trajectory)
        self._strip_trajectory_derivatives(trajectory)
        self._stretch_trajectory_segment_times(trajectory)

        max_velocity = self._trajectory_max_joint_velocity_rad_s(trajectory)
        limit = configured_max_joint_velocity_rad_s()
        limit_deg = configured_max_joint_velocity_deg_s()
        if max_velocity <= limit * UR3E_JOINT_VELOCITY_TIME_MARGIN:
            if max_velocity > 0.0:
                sys.stderr.write(
                    "UR3e MoveIt execute: trajectory peak joint velocity "
                    f"{math.degrees(max_velocity):.1f} deg/s "
                    f"(limit {limit_deg:.0f} deg/s).\n"
                )
            return True

        scale = (max_velocity / limit) * UR3E_JOINT_VELOCITY_TIME_MARGIN
        if scale > TRAJECTORY_MAX_VELOCITY_SCALEUP:
            sys.stderr.write(
                "UR3e MoveIt execute: trajectory peak joint velocity "
                f"{math.degrees(max_velocity):.0f} deg/s exceeds "
                f"{limit_deg:.0f} deg/s even after unwrap — rejecting.\n"
            )
            return False

        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None:
            return False
        for point in joint_traj.points:
            self._set_point_time_from_start_sec(
                point,
                self._point_time_from_start_sec(point) * scale,
            )
        sys.stderr.write(
            "UR3e MoveIt execute: slowed trajectory "
            f"{scale:.2f}x to respect {limit_deg:.0f} deg/s joint limit.\n"
        )
        return True

    @classmethod
    def _joint_permutation_variants(cls, base: Sequence[float]) -> List[List[float]]:
        """Base seed plus +45°…+315° offset on each joint individually."""
        if len(base) != 6:
            return []
        base_list = [float(v) for v in base]
        variants: List[List[float]] = [list(base_list)]
        for joint_index in range(6):
            for offset in IK_SEED_JOINT_OFFSETS_RAD:
                perturbed = list(base_list)
                perturbed[joint_index] += offset
                variants.append(perturbed)
        return variants

    def _generate_ik_seeds(
        self,
        home: Sequence[float],
        *,
        current_pin: Optional[Sequence[float]] = None,
    ) -> List[List[float]]:
        """Build IK seeds: home and current pin, each with 45° joint permutations."""
        seen: set[tuple[float, ...]] = set()
        seeds: List[List[float]] = []

        def add(seed: Optional[Sequence[float]]) -> None:
            if seed is None or len(seed) != 6:
                return
            key = tuple(round(float(v), 4) for v in seed)
            if key in seen:
                return
            seen.add(key)
            seeds.append([float(v) for v in seed])

        base_seeds: List[Sequence[float]] = []
        if len(home) == 6:
            base_seeds.append(home)
        if current_pin is not None and len(current_pin) == 6:
            base_seeds.append(current_pin)

        for base in base_seeds:
            for variant in self._joint_permutation_variants(base):
                add(variant)

        return seeds

    def _solve_ik(
        self,
        pose: Sequence[float],
        seed_joints: Sequence[float],
        *,
        frame_id: str = PLANNING_FRAME,
    ) -> tuple[Optional[List[float]], str]:
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import MoveItErrorCodes

        if len(seed_joints) != 6:
            return None, "no joint seed (robot state unavailable)"

        x_m, y_m, z_m, rx, ry, rz = pose
        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP_NAME
        request.ik_request.ik_link_name = EE_LINK
        request.ik_request.avoid_collisions = True
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = frame_id
        request.ik_request.pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = float(x_m)
        request.ik_request.pose_stamped.pose.position.y = float(y_m)
        request.ik_request.pose_stamped.pose.position.z = float(z_m)
        qx, qy, qz, qw = rotvec_to_quaternion(rx, ry, rz)
        request.ik_request.pose_stamped.pose.orientation.x = qx
        request.ik_request.pose_stamped.pose.orientation.y = qy
        request.ik_request.pose_stamped.pose.orientation.z = qz
        request.ik_request.pose_stamped.pose.orientation.w = qw
        request.ik_request.robot_state = self._make_robot_state(seed_joints)

        future = self._ik_client.call_async(request)
        response = self._wait_future(future, 10.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, self._moveit_error_name(response.error_code.val)

        name_to_pos = {
            name: pos
            for name, pos in zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        }
        joints = [float(name_to_pos.get(name, 0.0)) for name in CANONICAL_JOINT_NAMES]
        joints = self._normalize_joint_solution_to_reference(seed_joints, joints)
        return joints, ""

    def _solve_ik_multi_seed(
        self,
        pose: Sequence[float],
        reference_joints: Sequence[float],
        ik_seeds: Sequence[Sequence[float]],
        *,
        desired_tool_z: Optional[Sequence[float]] = None,
    ) -> tuple[Optional[List[float]], str, bool]:
        """Try IK seeds in order; return the first collision-free solution."""
        last_error = "no IK solution"
        seen_solutions: set[tuple[float, ...]] = set()
        check_tool_z = (
            desired_tool_z is not None
            and len(desired_tool_z) == 3
            and not self._use_mock_hardware()
        )

        for index, seed_joints in enumerate(ik_seeds):
            seed_for_ik = self._normalize_joint_solution_to_reference(
                reference_joints, seed_joints
            )
            if not self._ik_joint_angles_are_sane(seed_for_ik):
                continue
            seed_valid, seed_reason = self._state_is_valid(seed_for_ik)
            if not seed_valid:
                if seed_reason:
                    last_error = seed_reason
                continue

            joints, ik_error = self._solve_ik(pose, seed_for_ik)
            if joints is None:
                if ik_error:
                    last_error = ik_error
                continue

            joints = self._normalize_joint_solution_to_reference(reference_joints, joints)
            if not self._ik_joint_angles_are_sane(joints):
                last_error = "invalid IK joint angles"
                continue

            solution_key = tuple(round(v, 4) for v in joints)
            if solution_key in seen_solutions:
                continue
            seen_solutions.add(solution_key)

            valid, reason = self._state_is_valid(joints)
            if not valid:
                last_error = reason or "invalid state"
                continue

            if check_tool_z:
                alignment = self._tool_z_alignment(joints, desired_tool_z)
                if alignment is not None and alignment < TOOL_Z_ALIGNMENT_MIN_DOT:
                    if alignment < 0.0:
                        last_error = "IK tool Z faces away from dome center"
                    else:
                        last_error = "IK tool Z misaligned with dome center"
                    continue

            recovered = index > 0
            return joints, last_error, recovered

        return None, last_error, False

    def _state_is_valid(
        self,
        joints: Sequence[float],
        *,
        check_pinch: bool = True,
    ) -> tuple[bool, str]:
        from moveit_msgs.srv import GetStateValidity
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        request = GetStateValidity.Request()
        request.group_name = GROUP_NAME
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        request.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._validity_client.call_async(request)
        response = self._wait_future(future, 10.0)
        if not response.valid:
            reason = self._validity_failure_reason(response)
            return False, reason

        if check_pinch:
            pinch_ok, pinch_reason = self._ur_pinch_guard_ok(joints)
            if not pinch_ok:
                return False, pinch_reason
        return True, ""

    def plan_poses(
        self,
        targets: Sequence[ScanPoseTarget],
        workspace: WorkspaceBox,
        *,
        initial_seed: Optional[Sequence[float]] = None,
    ) -> List[ScanPoseResult]:
        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            self._last_workspace = workspace
            self._apply_workspace_collision(workspace)

            if initial_seed is None or len(initial_seed) != 6:
                raise RuntimeError(
                    "Cannot plan scan — current joint state unavailable (connect the robot first)."
                )

            results: List[ScanPoseResult] = []
            # Home-centric plan: every pin is IK-seeded and normalized from fixed home,
            # matching execute retreat-to-home (not chained from the previous pin).
            plan_start_seed = [float(v) for v in self.home_joints_rad()]
            self._last_plan_start_seed = list(plan_start_seed)
            initial_robot = [float(v) for v in initial_seed]
            last_reachable_pin: Optional[List[float]] = None
            multi_seed_recoveries = 0
            total = len(targets)
            sys.stderr.write(
                "UR3e MoveIt: planning "
                f"{total} scan pose(s) (IK seeds: home + current, 45° permutations)…\n"
            )

            for pose_index, target in enumerate(targets):
                if pose_index > 0 and pose_index % 10 == 0:
                    sys.stderr.write(
                        f"UR3e MoveIt: planned {pose_index}/{total} pose(s)…\n"
                    )
                pose = pose_target_to_ur_pose(target)
                result = ScanPoseResult(index=target.index, reachable=False)
                try:
                    current_pin_seed = (
                        last_reachable_pin
                        if last_reachable_pin is not None
                        else initial_robot
                    )
                    ik_seeds = self._generate_ik_seeds(
                        plan_start_seed,
                        current_pin=current_pin_seed,
                    )
                    joints, ik_error, recovered = self._solve_ik_multi_seed(
                        pose,
                        plan_start_seed,
                        ik_seeds,
                        desired_tool_z=(
                            target.tool_z_x,
                            target.tool_z_y,
                            target.tool_z_z,
                        ),
                    )
                    if joints is None:
                        result.error = ik_error or "IK failed"
                    else:
                        joints = self._normalize_joint_solution_to_reference(
                            plan_start_seed, joints
                        )
                        if not self._ik_joint_angles_are_sane(joints):
                            result.error = "invalid IK joint angles"
                        else:
                            valid, reason = self._state_is_valid(joints)
                            if not valid:
                                result.error = reason or "invalid state"
                            else:
                                if recovered:
                                    multi_seed_recoveries += 1
                                result.reachable = True
                                result.joint_positions = joints
                                last_reachable_pin = list(joints)
                except Exception as exc:
                    result.error = str(exc)
                results.append(result)

            if multi_seed_recoveries > 0:
                sys.stderr.write(
                    "UR3e MoveIt: multi-seed IK recovered "
                    f"{multi_seed_recoveries} additional reachable pose(s).\n"
                )
            reachable = sum(1 for item in results if item.reachable)
            sys.stderr.write(
                f"UR3e MoveIt: plan complete — {reachable}/{total} reachable.\n"
            )

            return results

    def _ensure_move_client(self) -> Any:
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient

        if self._move_client is None:
            self._move_client = ActionClient(self._node, MoveGroup, "/move_action")
        if not self._move_client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("MoveIt /move_action server not available.")
        return self._move_client

    def _ensure_execute_trajectory_client(self) -> Optional[Any]:
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient

        if self._execute_trajectory_client is not None:
            return self._execute_trajectory_client

        for action_name in ("/execute_trajectory", "/move_group/execute_trajectory"):
            client = ActionClient(self._node, ExecuteTrajectory, action_name)
            if client.wait_for_server(timeout_sec=2.0):
                self._execute_trajectory_client = client
                return client
        return None

    def _build_move_group_joint_goal(
        self,
        goal_joints: Sequence[float],
        *,
        pipeline_id: str,
        planner_id: str,
        motion_scale: float,
        plan_only: bool,
        start_joints: Optional[Sequence[float]] = None,
    ) -> Any:
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint, PlanningOptions

        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.pipeline_id = pipeline_id
        goal.request.planner_id = planner_id
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = motion_scale
        goal.request.max_acceleration_scaling_factor = motion_scale

        if start_joints is not None and len(start_joints) == 6:
            goal.request.start_state = self._make_robot_state(start_joints)

        constraints = Constraints()
        for joint_name, value in zip(CANONICAL_JOINT_NAMES, goal_joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(value)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only
        return goal

    @staticmethod
    def _trajectory_joint_waypoints(trajectory: Any) -> List[List[float]]:
        if trajectory is None:
            return []
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None or not joint_traj.joint_names or not joint_traj.points:
            return []

        name_to_index = {
            name: index for index, name in enumerate(joint_traj.joint_names)
        }
        indices: List[int] = []
        for joint_name in CANONICAL_JOINT_NAMES:
            if joint_name not in name_to_index:
                return []
            indices.append(name_to_index[joint_name])

        waypoints: List[List[float]] = []
        for point in joint_traj.points:
            if len(point.positions) <= max(indices):
                continue
            waypoints.append([float(point.positions[index]) for index in indices])
        return waypoints

    def _trajectory_joint_travel_rad(self, waypoints: Sequence[Sequence[float]]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for start, end in zip(waypoints[:-1], waypoints[1:]):
            total += self._joint_distance_rad(start, end)
        return total

    def _trajectory_pinch_ok(
        self,
        waypoints: Sequence[Sequence[float]],
    ) -> tuple[bool, float, str]:
        from hyperfusion_ur3e.moveit.ur_pinch_guard import (
            effective_pinch_surface_gap_m,
            tool_payload_radius_m,
        )

        if self._use_mock_hardware() or not waypoints:
            return True, float("inf"), ""

        count = len(waypoints)
        if count <= TRAJECTORY_MAX_PINCH_SAMPLES:
            sample_indices = list(range(count))
        else:
            step = max(1, count // TRAJECTORY_MAX_PINCH_SAMPLES)
            sample_indices = list(range(0, count, step))
            if sample_indices[-1] != count - 1:
                sample_indices.append(count - 1)

        payload_radius_m = tool_payload_radius_m()
        min_gap_m = float("inf")
        for index in sample_indices:
            joints = waypoints[index]
            link_positions = self._fk_link_positions(joints)
            if link_positions is None:
                return True, float("inf"), ""
            tool_z = self._fk_tool0_z_axis(joints)
            gap_m = effective_pinch_surface_gap_m(
                link_positions,
                tool_z=tool_z,
                payload_radius_m=payload_radius_m,
            )
            min_gap_m = min(min_gap_m, gap_m)
            if gap_m < TRAJECTORY_PINCH_MIN_GAP_M:
                return (
                    False,
                    gap_m,
                    f"trajectory pinch gap {gap_m * 1000.0:.1f} mm at sample {index}",
                )
        return True, min_gap_m, ""

    def _log_moveit_joint_issues(self, joints: Sequence[float], *, label: str) -> None:
        issues = moveit_joint_limit_violations(joints)
        if issues:
            sys.stderr.write(
                f"UR3e MoveIt: {label} outside URDF joint limits — "
                + "; ".join(issues)
                + "\n"
            )

    def _format_state_invalid_reason(
        self,
        joints: Sequence[float],
        validity_reason: str,
    ) -> str:
        limit_issues = moveit_joint_limit_violations(joints)
        if limit_issues:
            return "; ".join(limit_issues)
        if validity_reason:
            return validity_reason
        return "collision or joint limit violation"

    def _plan_move_group_joint_goal(
        self,
        goal_joints: Sequence[float],
        move_client: Any,
        *,
        pipeline_id: str,
        planner_id: str,
        motion_scale: float,
        start_joints: Optional[Sequence[float]] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> tuple[int, Optional[Any]]:
        goal = self._build_move_group_joint_goal(
            goal_joints,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            motion_scale=motion_scale,
            plan_only=True,
            start_joints=start_joints,
        )
        send_future = move_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, 20.0, stop_event)
        if goal_handle is None or not goal_handle.accepted:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("stopped")
            raise RuntimeError("MoveIt rejected motion goal.")

        result_future = goal_handle.get_result_async()
        result = self._wait_future(result_future, 120.0, stop_event)
        error_code = int(result.result.error_code.val)
        planned = getattr(result.result, "planned_trajectory", None)
        return error_code, planned

    def _execute_planned_trajectory(
        self,
        trajectory: Any,
        *,
        already_prepared: bool = False,
        stop_event: Optional[threading.Event] = None,
    ) -> int:
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import MoveItErrorCodes

        execute_client = self._ensure_execute_trajectory_client()
        if execute_client is None:
            return MoveItErrorCodes.FAILURE

        if not already_prepared and not self._prepare_trajectory_for_robot(trajectory):
            sys.stderr.write(
                "UR3e MoveIt execute: trajectory failed unwrap/velocity check.\n"
            )
            return MoveItErrorCodes.FAILURE

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = execute_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, 20.0, stop_event)
        if goal_handle is None or not goal_handle.accepted:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("stopped")
            return MoveItErrorCodes.FAILURE

        with self._move_goal_lock:
            self._active_move_goal_handle = goal_handle

        try:
            result_future = goal_handle.get_result_async()
            result = self._wait_future(result_future, 120.0, stop_event)
            return int(result.result.error_code.val)
        finally:
            with self._move_goal_lock:
                self._active_move_goal_handle = None

    def _goal_joints_from_tcp(
        self,
        tcp_target: Dict[str, Any],
        current_joints: Sequence[float],
    ) -> Optional[List[float]]:
        """Re-run IK from the live robot pose so execute matches current joint branches."""
        target = ScanPoseTarget(
            index=0,
            x_m=float(tcp_target.get("x_m", tcp_target.get("x", 0.0))),
            y_m=float(tcp_target.get("y_m", tcp_target.get("y", 0.0))),
            z_m=float(tcp_target.get("z_m", tcp_target.get("z", 0.0))),
            rx=float(tcp_target.get("rx", 0.0)),
            ry=float(tcp_target.get("ry", 0.0)),
            rz=float(tcp_target.get("rz", 0.0)),
            tool_z_x=float(tcp_target.get("tool_z_x", 0.0)),
            tool_z_y=float(tcp_target.get("tool_z_y", 0.0)),
            tool_z_z=float(tcp_target.get("tool_z_z", -1.0)),
        )
        pose = pose_target_to_ur_pose(target)
        reference = list(current_joints)
        plan_start = (
            self._last_plan_start_seed
            if len(self._last_plan_start_seed) == 6
            else self.home_joints_rad()
        )
        ik_seeds = self._generate_ik_seeds(
            plan_start,
            current_pin=reference,
        )
        joints, ik_error, _recovered = self._solve_ik_multi_seed(
            pose,
            reference,
            ik_seeds,
            desired_tool_z=(
                target.tool_z_x,
                target.tool_z_y,
                target.tool_z_z,
            ),
        )
        if joints is None:
            sys.stderr.write(
                f"UR3e MoveIt execute: live IK refresh failed ({ik_error or 'unknown'}).\n"
            )
            return None

        joints = self._normalize_joint_solution_to_reference(plan_start, joints)
        joints = self._normalize_joint_solution_to_reference(reference, joints)
        if not self._ik_joint_angles_are_sane(joints):
            sys.stderr.write("UR3e MoveIt execute: live IK refresh produced invalid joint angles.\n")
            return None

        valid, reason = self._state_is_valid(joints)
        if not valid:
            sys.stderr.write(
                f"UR3e MoveIt execute: live IK refresh invalid ({reason or 'collision'}).\n"
            )
            return None

        sys.stderr.write("UR3e MoveIt execute: refreshed IK goal from current robot pose.\n")
        return joints

    def _motion_scale_for(
        self,
        a: Optional[Sequence[float]],
        b: Optional[Sequence[float]],
    ) -> float:
        travel = 0.0
        if a is not None and b is not None:
            travel = self._joint_distance_rad(a, b)
        if travel > 5.0:
            return 0.05
        if travel > 3.0:
            return 0.08
        return 0.15

    def _plan_and_execute_move(
        self,
        goal_joints: Sequence[float],
        move_client: Any,
        *,
        motion_scale: float,
        stop_event: Optional[threading.Event] = None,
    ) -> tuple[bool, int, str]:
        """Plan with each MoveIt pipeline, pick the easiest valid trajectory, then execute."""
        from moveit_msgs.msg import MoveItErrorCodes

        # MoveIt plans against /joint_states (stamper branch). RTDE hardware branch is
        # applied only when unwrapping the trajectory for scaled_joint_trajectory_controller.
        hardware_start = self._ensure_hardware_joint_positions(timeout_s=1.5)
        moveit_start = self._moveit_start_joint_positions()
        plan_goal = list(goal_joints)
        if moveit_start is not None:
            plan_goal = coalesce_joints_for_moveit(moveit_start, plan_goal)
        if hardware_start is not None:
            sys.stderr.write(
                "UR3e MoveIt execute: plan in MoveIt joint branch; "
                "execute unwrap anchors to RTDE hardware.\n"
            )

        self._publish_rviz_goal_state(plan_goal)
        if moveit_start is not None:
            self._log_moveit_joint_issues(moveit_start, label="current")
        self._log_moveit_joint_issues(plan_goal, label="goal")

        candidates: List[tuple[float, float, str, str, Any]] = []
        last_code = MoveItErrorCodes.FAILURE
        last_name = "failure"

        for pipeline_id, planner_id in EXECUTE_PLANNER_ATTEMPTS:
            try:
                error_code, planned = self._plan_move_group_joint_goal(
                    plan_goal,
                    move_client,
                    pipeline_id=pipeline_id,
                    planner_id=planner_id,
                    motion_scale=motion_scale,
                    start_joints=moveit_start,
                    stop_event=stop_event,
                )
            except RuntimeError:
                raise

            if error_code != MoveItErrorCodes.SUCCESS or planned is None:
                last_code = error_code
                last_name = self._moveit_error_name(error_code)
                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} plan failed "
                    f"({last_name}, code={error_code}).\n"
                )
                continue

            prepared = copy.deepcopy(planned)
            if not self._prepare_trajectory_for_robot(prepared):
                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} rejected (unwrap/velocity limit).\n"
                )
                continue

            waypoints = self._trajectory_joint_waypoints(prepared)
            if len(waypoints) < 2:
                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} returned empty trajectory.\n"
                )
                continue

            pinch_ok, min_gap_m, pinch_reason = self._trajectory_pinch_ok(waypoints)
            if not pinch_ok:
                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} rejected ({pinch_reason}).\n"
                )
                continue

            travel_rad = self._trajectory_joint_travel_rad(waypoints)
            candidates.append(
                (travel_rad, -min_gap_m, pipeline_id, planner_id, prepared)
            )
            sys.stderr.write(
                "UR3e MoveIt execute: "
                f"{pipeline_id}/{planner_id} candidate "
                f"travel={math.degrees(travel_rad):.1f}°·joint "
                f"min pinch gap={min_gap_m * 1000.0:.1f} mm.\n"
            )

        if candidates:
            candidates.sort(key=lambda item: (item[0], item[1]))
            travel_rad, _neg_gap, pipeline_id, planner_id, prepared = candidates[0]
            sys.stderr.write(
                "UR3e MoveIt execute: selected "
                f"{pipeline_id}/{planner_id} "
                f"(easiest: {math.degrees(travel_rad):.1f}°·joint travel).\n"
            )
            execute_code = self._execute_planned_trajectory(
                prepared, already_prepared=True, stop_event=stop_event
            )
            if execute_code == MoveItErrorCodes.SUCCESS:
                return True, MoveItErrorCodes.SUCCESS, "success"
            last_code = execute_code
            last_name = self._moveit_error_name(execute_code)
            sys.stderr.write(
                "UR3e MoveIt execute: execute_trajectory failed "
                f"({last_name}, code={execute_code}) — trying alternate planner.\n"
            )

        for pipeline_id, planner_id in EXECUTE_PLANNER_ATTEMPTS:
            error_code, planned = self._plan_move_group_joint_goal(
                plan_goal,
                move_client,
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                motion_scale=motion_scale,
                start_joints=moveit_start,
                stop_event=stop_event,
            )
            if error_code != MoveItErrorCodes.SUCCESS or planned is None:
                last_code = error_code
                last_name = self._moveit_error_name(error_code)
                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} replan failed "
                    f"({last_name}, code={error_code}).\n"
                )
                continue

            prepared = copy.deepcopy(planned)
            if not self._prepare_trajectory_for_robot(prepared):
                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} rejected (unwrap/velocity limit).\n"
                )
                continue

            execute_code = self._execute_planned_trajectory(
                prepared, already_prepared=True, stop_event=stop_event
            )
            if execute_code == MoveItErrorCodes.SUCCESS:
                return True, MoveItErrorCodes.SUCCESS, "success"
            last_code = execute_code
            last_name = self._moveit_error_name(execute_code)
            sys.stderr.write(
                "UR3e MoveIt execute: "
                f"{pipeline_id}/{planner_id} execute failed "
                f"({last_name}, code={execute_code}).\n"
            )
        return False, last_code, last_name

    def _resolve_goal_joints(
        self,
        goal_joints: Sequence[float],
        at_joints: Sequence[float],
        tcp_target: Optional[Dict[str, Any]] = None,
        *,
        refresh_ik: bool = True,
    ) -> tuple[Optional[List[float]], str]:
        """Joint-space goal at *at_joints*, optionally refreshed with live IK for scan pins."""
        resolved = self._normalize_joint_solution_to_reference(at_joints, goal_joints)
        if refresh_ik and isinstance(tcp_target, dict):
            refreshed = self._goal_joints_from_tcp(tcp_target, at_joints)
            if refreshed is not None:
                resolved = refreshed

        if not self._ik_joint_angles_are_sane(resolved):
            return None, "goal outside UR3e limits"

        goal_valid, goal_reason = self._state_is_valid(resolved)
        if not goal_valid:
            return None, goal_reason or "collision or limits"

        return resolved, ""

    def _resolve_pin_approach_goal(
        self,
        goal_joints: Sequence[float],
        at_joints: Sequence[float],
        tcp_target: Optional[Dict[str, Any]] = None,
    ) -> tuple[Optional[List[float]], str]:
        """Resolve scan-pin goal joints; prefer stored plan joints over live IK refresh."""
        planned, planned_error = self._resolve_goal_joints(
            goal_joints, at_joints, tcp_target=None, refresh_ik=False
        )
        if planned is not None:
            return planned, ""

        if isinstance(tcp_target, dict):
            refreshed, refresh_error = self._resolve_goal_joints(
                goal_joints, at_joints, tcp_target, refresh_ik=True
            )
            if refreshed is not None:
                return refreshed, ""
            return None, refresh_error or planned_error or "goal unreachable"

        return None, planned_error or "goal unreachable"

    def _execute_via_home(
        self,
        goal_joints: Sequence[float],
        current_joints: Optional[Sequence[float]],
        move_client: Any,
        *,
        tcp_target: Optional[Dict[str, Any]] = None,
        motion_scale: Optional[float] = None,
        stop_event: Optional[threading.Event] = None,
        log_prefix: str = "no direct collision-free path",
    ) -> Dict[str, Any]:
        """Retreat to fixed home, then approach the pin from the home branch."""
        sys.stderr.write(
            f"UR3e MoveIt execute: {log_prefix} — "
            "retreating to home pose, then approaching pin.\n"
        )
        home_ref = current_joints if current_joints is not None else self.home_joints_rad()
        home_joints = self._normalize_joint_solution_to_reference(
            home_ref, self.home_joints_rad()
        )
        home_valid, home_reason = self._state_is_valid(home_joints)
        if not home_valid:
            return {
                "ok": False,
                "skipped": True,
                "error": f"no direct path; home pose invalid ({home_reason or 'collision'})",
            }

        live_before_retreat = self._current_joint_positions()
        if live_before_retreat is not None:
            live_before_retreat = self._normalize_joint_solution_to_reference(
                live_before_retreat, live_before_retreat
            )
        retreat_from = live_before_retreat if live_before_retreat is not None else current_joints

        retreat_scale = motion_scale
        if retreat_scale is None:
            retreat_scale = self._motion_scale_for(retreat_from, home_joints)
        ok_retreat, _rcode, rname = self._plan_and_execute_move(
            home_joints, move_client, motion_scale=retreat_scale, stop_event=stop_event
        )
        if not ok_retreat:
            return {
                "ok": False,
                "skipped": True,
                "error": f"no direct path and could not retreat to home ({rname})",
            }

        current2 = self._waypoint_after_commanded_move(
            home_joints,
            stop_event=stop_event,
            reference=home_ref,
        )

        goal2, goal_error = self._resolve_pin_approach_goal(
            goal_joints, current2, tcp_target
        )
        if goal2 is None:
            return {
                "ok": False,
                "skipped": True,
                "error": goal_error or "pin unreachable from home",
            }

        approach_scale = motion_scale
        if approach_scale is None:
            approach_scale = self._motion_scale_for(current2, goal2)
        ok2, _c2, n2 = self._plan_and_execute_move(
            goal2, move_client, motion_scale=approach_scale, stop_event=stop_event
        )
        if not ok2 and isinstance(tcp_target, dict):
            refreshed, _refresh_error = self._resolve_goal_joints(
                goal_joints, current2, tcp_target, refresh_ik=True
            )
            if (
                refreshed is not None
                and self._joint_distance_rad(refreshed, goal2) > HOME_JOINT_TOLERANCE_RAD
            ):
                sys.stderr.write(
                    "UR3e MoveIt execute: planned home approach failed "
                    f"({n2}) — retrying with live IK refresh.\n"
                )
                approach_scale = self._motion_scale_for(current2, refreshed)
                ok2, _c2, n2 = self._plan_and_execute_move(
                    refreshed,
                    move_client,
                    motion_scale=approach_scale,
                    stop_event=stop_event,
                )
        if ok2:
            sys.stderr.write("UR3e MoveIt execute: reached pin via home pose.\n")
            return {"ok": True, "executed": 1, "via_home": True}

        return {
            "ok": False,
            "skipped": True,
            "error": f"no collision-free path even via home ({n2})",
        }

    def execute_single_waypoint(
        self,
        joints: Sequence[float],
        workspace: Optional[WorkspaceBox] = None,
        *,
        tcp_target: Optional[Dict[str, Any]] = None,
        stop_event: Optional[threading.Event] = None,
        require_home_first: bool = False,
        direct_only: bool = False,
    ) -> Dict[str, Any]:
        """Plan and execute one collision-aware joint-space motion via MoveIt.

        Order: direct current→pin; then retreat home→pin. Skips the pin if both fail.

        When *direct_only* is True, only attempt a path from the current robot state;
        never retreat via home. Failures return MoveIt error names (collision, IK,
        planning failed, etc.).

        Manual joint **Move** sets *direct_only*: same MoveIt plan+execute as scan pins,
        but without home retreat on failure.
        """
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "error": "stopped"}

        if len(joints) != 6:
            raise ValueError("Waypoint must have 6 joint values.")

        goal_joints = [float(v) for v in joints]

        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            ws = workspace if workspace is not None else self._last_workspace
            if ws is None:
                ws = WorkspaceBox(enabled=True)
            self._apply_workspace_collision(ws)
            move_client = self._ensure_move_client()

        self._wait_for_planner_joint_feedback(timeout_s=2.0)
        goal_joints, branch_ref = self._resolve_execute_branch(
            goal_joints,
            direct_only=direct_only,
            tcp_target=tcp_target,
        )
        if branch_ref is not None and not self._ik_joint_angles_are_sane(goal_joints):
            return {
                "ok": False,
                "skipped": not direct_only,
                "error": "goal joints outside UR3e limits",
            }
        if branch_ref is None:
            sys.stderr.write(
                "UR3e MoveIt execute: joint feedback unavailable — "
                "using goal without live branch coalesce.\n"
            )

        try:
            if branch_ref is not None:
                start_check = branch_ref
                start_valid, start_reason = self._state_is_valid(start_check)
                if not start_valid:
                    detail = self._format_state_invalid_reason(start_check, start_reason)
                    return {
                        "ok": False,
                        "skipped": not direct_only,
                        "error": f"start state invalid: {detail}",
                    }
            goal_valid, goal_reason = self._state_is_valid(goal_joints)
            if not goal_valid:
                detail = self._format_state_invalid_reason(goal_joints, goal_reason)
                return {
                    "ok": False,
                    "skipped": not direct_only,
                    "error": f"goal state invalid: {detail}",
                }

            # 1) Direct path from current pose (skipped when home-first is required).
            if not require_home_first:
                motion_scale = self._motion_scale_for(branch_ref, goal_joints)
                ok, error_code, error_name = self._plan_and_execute_move(
                    goal_joints, move_client, motion_scale=motion_scale, stop_event=stop_event
                )
                if ok:
                    return {"ok": True, "executed": 1}
                if direct_only:
                    return {
                        "ok": False,
                        "error": error_name,
                        "moveit_error_code": int(error_code),
                    }

            if direct_only:
                return {
                    "ok": False,
                    "error": "home-first motion is not allowed for direct-only moves",
                }

            # 2) Retreat to fixed home pose, then approach the pin.
            home_prefix = (
                "ring boundary — home-first required"
                if require_home_first
                else "no direct collision-free path"
            )
            return self._execute_via_home(
                goal_joints,
                branch_ref,
                move_client,
                tcp_target=tcp_target,
                stop_event=stop_event,
                log_prefix=home_prefix,
            )
        except RuntimeError as exc:
            if stop_event is not None and stop_event.is_set():
                return {"ok": False, "stopped": True, "error": str(exc)}
            raise

    def execute_move_home(
        self,
        workspace: Optional[WorkspaceBox] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> Dict[str, Any]:
        """Collision-aware MoveIt motion to the configured scan home pose."""
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "error": "stopped"}

        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            ws = workspace if workspace is not None else self._last_workspace
            if ws is None:
                ws = WorkspaceBox(enabled=True)
            self._apply_workspace_collision(ws)
            move_client = self._ensure_move_client()

        current_joints = self._current_joint_positions()
        if current_joints is not None:
            current_joints = self._normalize_joint_solution_to_reference(
                current_joints, current_joints
            )
        home_ref = current_joints if current_joints is not None else self.home_joints_rad()
        home_joints = self._normalize_joint_solution_to_reference(
            home_ref, self.home_joints_rad()
        )

        if current_joints is not None:
            if self._joint_distance_rad(current_joints, home_joints) <= HOME_JOINT_TOLERANCE_RAD:
                current_valid, current_reason = self._state_is_valid(current_joints)
                if current_valid:
                    sys.stderr.write("UR3e MoveIt execute: already at scan home pose.\n")
                    return {"ok": True, "already_at_home": True}
                return {
                    "ok": False,
                    "skipped": True,
                    "error": (
                        "at configured home joints but pose is invalid "
                        f"({current_reason or 'collision or limits'})"
                    ),
                }

        home_valid, home_reason = self._state_is_valid(home_joints)
        if not home_valid:
            return {
                "ok": False,
                "skipped": True,
                "error": f"home pose invalid ({home_reason or 'collision or limits'})",
            }

        motion_scale = self._motion_scale_for(current_joints, home_joints)
        ok, _code, error_name = self._plan_and_execute_move(
            home_joints, move_client, motion_scale=motion_scale, stop_event=stop_event
        )
        if ok:
            sys.stderr.write("UR3e MoveIt execute: reached scan home pose.\n")
            return {"ok": True}

        return {
            "ok": False,
            "skipped": True,
            "error": f"could not move to home ({error_name})",
        }

    def execute_waypoints(
        self,
        waypoints: Sequence[Sequence[float]],
        *,
        workspace: Optional[WorkspaceBox] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> Dict[str, Any]:
        """Plan collision-aware joint motions and execute each waypoint in order."""
        ws = workspace if workspace is not None else self._last_workspace
        executed = 0
        for index, joints in enumerate(waypoints):
            if stop_event is not None and stop_event.is_set():
                return {"ok": True, "stopped": True, "executed": executed}

            try:
                result = self.execute_single_waypoint(
                    joints,
                    workspace=ws,
                    stop_event=stop_event,
                )
            except (RuntimeError, ValueError, TimeoutError) as exc:
                if stop_event is not None and stop_event.is_set():
                    return {"ok": True, "stopped": True, "executed": executed}
                raise RuntimeError(f"MoveIt failed at waypoint {index}: {exc}") from exc

            if not result.get("ok"):
                if result.get("stopped"):
                    return {"ok": True, "stopped": True, "executed": executed}
                raise RuntimeError(result.get("error") or "MoveIt execution failed.")
            executed += 1

        return {"ok": True, "executed": executed}


_planner: Optional[MoveItScanPlanner] = None
_planner_lock = threading.Lock()


def get_scan_planner(*, ros_distro: str = "jazzy", ur_type: str = "ur3e") -> MoveItScanPlanner:
    global _planner
    with _planner_lock:
        if _planner is None:
            _planner = MoveItScanPlanner(ros_distro=ros_distro, ur_type=ur_type)
        return _planner
