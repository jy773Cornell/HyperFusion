"""MoveIt IK + collision checking for UR3e hemisphere scan routes (WSL sidecar)."""
from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

from hyperfusion_ur3e import PKG_ROOT

GROUP_NAME = "ur_manipulator"
EE_LINK = "tool0"
PLANNING_FRAME = "world"
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

CANONICAL_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

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

# Extra IK seeds = plan-start (home) + small joint nudges (±45°).
# Large ±π perturbations often land in self-collision inside the workspace box.
_QUARTER_PI = math.pi * 0.25
IK_SEED_PERTURBATIONS_RAD = (
    (1, _QUARTER_PI),    # shoulder_lift +45°
    (1, -_QUARTER_PI),
    (2, _QUARTER_PI),    # elbow +45°
    (2, -_QUARTER_PI),
    (3, _QUARTER_PI),    # wrist_1 +45°
    (3, -_QUARTER_PI),
    (4, _QUARTER_PI),    # wrist_2 +45°
    (4, -_QUARTER_PI),
)

# UR3e joint limits from ur_description/config/ur3e/joint_limits.yaml (radians).
TWO_PI = 2.0 * math.pi
UR3E_JOINT_LIMITS_RAD: tuple[Optional[tuple[float, float]], ...] = (
    (-TWO_PI, TWO_PI),   # shoulder_pan ±360°
    (-TWO_PI, TWO_PI),   # shoulder_lift ±360°
    (-math.pi, math.pi),  # elbow ±180°
    (-TWO_PI, TWO_PI),   # wrist_1 ±360°
    (-TWO_PI, TWO_PI),   # wrist_2 ±360°
    None,                # wrist_3: continuous (wrap to ±180° for consistency)
)
ELBOW_JOINT_INDEX = 2
MAX_IK_BRANCH_STEPS = 4

# MoveIt execute: OMPL first (collision-aware paths); Pilz PTP as backup.
EXECUTE_PLANNER_ATTEMPTS = (
    ("ompl", "RRTConnect"),
    ("ompl", "RRTstar"),
    ("pilz_industrial_motion_planner", "PTP"),
)


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
    def _use_mock_hardware() -> bool:
        return os.environ.get("HYPERFUSION_USE_MOCK_HARDWARE", "true").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

    def _ros_param_bool(self, node: str, param: str) -> Optional[bool]:
        cmd = (
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            f"ros2 param get {node} {param}"
        )
        proc = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True)
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
        subprocess.run(["pkill", "-f", "moveit_ros_move_group/move_group"], check=False)
        time.sleep(0.5)

    def ensure_running(self, timeout_s: float = 120.0) -> None:
        with self._lock:
            if self._move_group_running() and self._move_group_controller_config_ok():
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
                    if self._move_group_running() and self._move_group_controller_config_ok():
                        return
                    time.sleep(0.5)
                raise RuntimeError("MoveIt move_group did not become ready in time.")

            script = PKG_ROOT / "scripts" / "launch_moveit_headless.sh"
            if not script.is_file():
                raise RuntimeError(f"Missing MoveIt launch script: {script}")

            pkg_root = PKG_ROOT.as_posix()
            mock_flag = "true" if self._use_mock_hardware() else "false"
            cmd = (
                f"export HYPERFUSION_UR3E_REPO='{pkg_root}' && "
                f"export HYPERFUSION_USE_MOCK_HARDWARE='{mock_flag}' && "
                f"export HYPERFUSION_CEILING_MOUNT=true && "
                f"export HYPERFUSION_CEILING_MOUNT_HEIGHT_M=0.65 && "
                f"sed 's/\\r$//' '{script.as_posix()}' | bash -s {self.ros_distro} {self.ur_type}"
            )
            self._process = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )

            deadline = time.time() + timeout_s
            while time.time() < deadline:
                if self._process.poll() is not None:
                    stderr_tail = ""
                    if self._process.stderr is not None:
                        stderr_tail = self._process.stderr.read().decode("utf-8", errors="replace")[-1500:]
                    raise RuntimeError(
                        f"MoveIt headless launch exited (code={self._process.returncode}).\n{stderr_tail}"
                    )
                if self._service_ready("/compute_ik", timeout_s=2.0, ros_distro=self.ros_distro) and (
                    self._move_group_controller_config_ok()
                ):
                    sys.stderr.write("UR3e MoveIt: move_group ready for scan planning.\n")
                    return
                time.sleep(1.0)

            raise RuntimeError("Timed out waiting for MoveIt /compute_ik service.")

    @staticmethod
    def _service_ready(service_name: str, timeout_s: float, ros_distro: str) -> bool:
        cmd = (
            f"source /opt/ros/{ros_distro}/setup.bash && "
            f"timeout {max(1, int(timeout_s))} ros2 service list | grep -Fx '{service_name}'"
        )
        proc = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True)
        return proc.returncode == 0


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
        self._move_client = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_stop = threading.Event()
        self._workspace_applied: Optional[tuple[Any, ...]] = None
        self._last_workspace: Optional[WorkspaceBox] = None
        self._last_plan_start_seed: List[float] = []
        self._home_joints_rad: List[float] = list(DEFAULT_HOME_JOINTS_RAD)
        self._planning_scene_client = None
        self._move_goal_lock = threading.Lock()
        self._active_move_goal_handle: Any = None
        self._latest_joint_positions: List[float] = []
        self._joint_state_sub = None

    def apply_home_joints_from_body(self, body: Optional[Dict[str, Any]]) -> None:
        self._home_joints_rad = home_joints_rad_from_body(body)

    def home_joints_rad(self) -> List[float]:
        return list(self._home_joints_rad)

    def _ensure_ros(self) -> None:
        if self._node is not None:
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

        from moveit_msgs.srv import GetPositionIK, GetStateValidity

        self._ik_client = self._node.create_client(GetPositionIK, "/compute_ik")
        self._validity_client = self._node.create_client(GetStateValidity, "/check_state_validity")

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

            self._joint_state_sub = self._node.create_subscription(
                JointState,
                "/joint_states_stamped",
                _on_joint_state,
                10,
            )

    def _current_joint_positions(self) -> Optional[List[float]]:
        if len(self._latest_joint_positions) != 6:
            return None
        return list(self._latest_joint_positions)

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        value = float(angle)
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    @classmethod
    def _pick_joint_branch(cls, index: int, value: float, reference: float) -> float:
        """Pick an equivalent joint angle within limits, nearest reference; tie-break compact branch."""
        limits = UR3E_JOINT_LIMITS_RAD[index]
        candidates: List[float] = []
        raw = float(value)
        for step in range(-MAX_IK_BRANCH_STEPS, MAX_IK_BRANCH_STEPS + 1):
            candidate = raw + (step * TWO_PI)
            if limits is not None:
                if candidate < limits[0] - 1e-9 or candidate > limits[1] + 1e-9:
                    continue
            else:
                candidate = cls._wrap_to_pi(candidate)
            if not any(abs(cls._joint_delta_rad(candidate, existing)) < 1e-6 for existing in candidates):
                candidates.append(candidate)

        if not candidates:
            if limits is None:
                return cls._wrap_to_pi(raw)
            return max(limits[0], min(limits[1], cls._wrap_to_pi(raw)))

        def sort_key(candidate: float) -> tuple[float, float, float]:
            return (
                abs(cls._joint_delta_rad(reference, candidate)),
                abs(cls._wrap_to_pi(candidate)),
                abs(candidate),
            )

        candidates.sort(key=sort_key)
        best = candidates[0]
        if abs(cls._joint_delta_rad(reference, best)) < 1e-4:
            if limits is None or (limits[0] - 1e-6 <= float(reference) <= limits[1] + 1e-6):
                return float(reference)
        return best

    @classmethod
    def _normalize_joint_solution_to_reference(
        cls,
        reference: Sequence[float],
        joints: Sequence[float],
    ) -> List[float]:
        """Pick nearest valid 2*pi branch within UR3e joint limits."""
        if len(reference) != 6 or len(joints) != 6:
            return [float(v) for v in joints]

        return [
            cls._pick_joint_branch(index, float(joint), float(ref))
            for index, (ref, joint) in enumerate(zip(reference, joints))
        ]

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
        return (
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
        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene

        self._ensure_planning_scene_client()

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._planning_scene_client.call_async(request)
        self._wait_future(future, 10.0)

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
        for object_id in BOUNDARY_OBJECT_IDS:
            try:
                self._remove_planning_scene_object(object_id)
            except Exception as exc:
                sys.stderr.write(
                    f"UR3e MoveIt: failed to clear boundary object {object_id}: {exc}\n"
                )

    def _validity_failure_reason(self, response: Any) -> str:
        contacts_state = getattr(response, "contacts", None)
        contact_infos = getattr(contacts_state, "contacts", None) if contacts_state else None
        if contact_infos:
            for contact in contact_infos:
                for attr in ("contact_body_1", "contact_body_2"):
                    body = str(getattr(contact, attr, "") or "")
                    if "hyperfusion_boundary_" in body:
                        return "robot arm outside workspace boundary"
            return f"collision ({len(contact_infos)} contact(s))"
        return "collision or joint limit violation"

    def _apply_workspace_collision(self, workspace: WorkspaceBox) -> None:
        """Install full workspace box collision (floor, walls, ceiling) for whole-arm checks."""
        workspace_key = self._workspace_key(workspace)
        if self._workspace_applied == workspace_key:
            return

        if not workspace.enabled:
            if self._workspace_applied is not None:
                self._clear_workspace_boundary_objects()
            self._workspace_applied = workspace_key
            sys.stderr.write("UR3e MoveIt: workspace boundary disabled.\n")
            return

        for collision_object in self._make_workspace_boundary_objects(workspace):
            self._apply_planning_scene_object(collision_object)

        self._workspace_applied = workspace_key
        sys.stderr.write(
            "UR3e MoveIt: workspace boundary enabled "
            f"(whole-arm collision box {workspace.length_m:.3f} x "
            f"{workspace.width_m:.3f} x {workspace.height_m:.3f} m).\n"
        )

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

    @staticmethod
    def _joint_delta_rad(a: float, b: float) -> float:
        delta = float(b) - float(a)
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        return delta

    @classmethod
    def _joint_distance_rad(cls, reference: Sequence[float], candidate: Sequence[float]) -> float:
        if len(reference) != 6 or len(candidate) != 6:
            return float("inf")
        total_sq = 0.0
        for ref, cand in zip(reference, candidate):
            delta = cls._joint_delta_rad(ref, cand)
            total_sq += delta * delta
        return math.sqrt(total_sq)

    @staticmethod
    def _generate_ik_seeds(plan_start_seed: Sequence[float]) -> List[List[float]]:
        """Build distinct joint seeds for multi-solution IK search from the plan start pose."""
        seen: set[tuple[float, ...]] = set()
        seeds: List[List[float]] = []

        def add(seed: Sequence[float]) -> None:
            if len(seed) != 6:
                return
            key = tuple(round(float(v), 4) for v in seed)
            if key in seen:
                return
            seen.add(key)
            seeds.append([float(v) for v in seed])

        add(plan_start_seed)

        base = [float(v) for v in plan_start_seed]
        for joint_index, delta in IK_SEED_PERTURBATIONS_RAD:
            perturbed = list(base)
            perturbed[joint_index] += delta
            add(perturbed)

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
    ) -> tuple[Optional[List[float]], str, bool]:
        """Try multiple IK seeds; pick the collision-free solution closest to plan start."""
        best_joints: Optional[List[float]] = None
        best_distance = float("inf")
        last_error = "no IK solution"
        seen_solutions: set[tuple[float, ...]] = set()
        primary_seed_valid = False

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

            if index == 0:
                primary_seed_valid = True

            distance = self._joint_distance_rad(reference_joints, joints)
            if distance < best_distance:
                best_distance = distance
                best_joints = joints

        recovered = best_joints is not None and not primary_seed_valid
        return best_joints, last_error, recovered

    def _state_is_valid(self, joints: Sequence[float]) -> tuple[bool, str]:
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
        if response.valid:
            return True, ""
        return False, self._validity_failure_reason(response)

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
            multi_seed_recoveries = 0

            for target in targets:
                pose = pose_target_to_ur_pose(target)
                result = ScanPoseResult(index=target.index, reachable=False)
                try:
                    ik_seeds = self._generate_ik_seeds(plan_start_seed)
                    joints, ik_error, recovered = self._solve_ik_multi_seed(
                        pose, plan_start_seed, ik_seeds
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
                except Exception as exc:
                    result.error = str(exc)
                results.append(result)

            if multi_seed_recoveries > 0:
                sys.stderr.write(
                    "UR3e MoveIt: multi-seed IK recovered "
                    f"{multi_seed_recoveries} additional reachable pose(s).\n"
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

    def _execute_move_group_joint_goal(
        self,
        goal_joints: Sequence[float],
        move_client: Any,
        *,
        pipeline_id: str,
        planner_id: str,
        motion_scale: float,
        stop_event: Optional[threading.Event] = None,
    ) -> int:
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
        goal.planning_options.plan_only = False

        send_future = move_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, 20.0, stop_event)
        if goal_handle is None or not goal_handle.accepted:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("stopped")
            raise RuntimeError("MoveIt rejected motion goal.")

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
        ik_seeds = self._generate_ik_seeds(plan_start)
        joints, ik_error, _recovered = self._solve_ik_multi_seed(
            pose, reference, ik_seeds
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
        """Run the planner attempts for one goal. Returns (ok, error_code, error_name)."""
        from moveit_msgs.msg import MoveItErrorCodes

        last_code = MoveItErrorCodes.FAILURE
        last_name = "failure"
        for pipeline_id, planner_id in EXECUTE_PLANNER_ATTEMPTS:
            error_code = self._execute_move_group_joint_goal(
                goal_joints,
                move_client,
                pipeline_id=pipeline_id,
                planner_id=planner_id,
                motion_scale=motion_scale,
                stop_event=stop_event,
            )
            if error_code == MoveItErrorCodes.SUCCESS:
                return True, MoveItErrorCodes.SUCCESS, "success"
            last_code = error_code
            last_name = self._moveit_error_name(error_code)
            sys.stderr.write(
                "UR3e MoveIt execute: "
                f"{pipeline_id}/{planner_id} failed "
                f"({last_name}, code={error_code}).\n"
            )
        return False, last_code, last_name

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

        retreat_scale = motion_scale
        if retreat_scale is None:
            retreat_scale = self._motion_scale_for(current_joints, home_joints)
        ok_retreat, _rcode, rname = self._plan_and_execute_move(
            home_joints, move_client, motion_scale=retreat_scale, stop_event=stop_event
        )
        if not ok_retreat:
            return {
                "ok": False,
                "skipped": True,
                "error": f"no direct path and could not retreat to home ({rname})",
            }

        current2 = self._current_joint_positions()
        if current2 is not None:
            current2 = self._normalize_joint_solution_to_reference(current2, current2)
        else:
            current2 = home_joints

        goal2: Optional[List[float]] = None
        if isinstance(tcp_target, dict):
            goal2 = self._goal_joints_from_tcp(tcp_target, current2)
        if goal2 is None:
            goal2 = self._normalize_joint_solution_to_reference(current2, goal_joints)

        if not self._ik_joint_angles_are_sane(goal2):
            return {
                "ok": False,
                "skipped": True,
                "error": "pin unreachable from home (goal outside UR3e limits)",
            }
        goal2_valid, goal2_reason = self._state_is_valid(goal2)
        if not goal2_valid:
            return {
                "ok": False,
                "skipped": True,
                "error": f"pin unreachable from home ({goal2_reason or 'collision'})",
            }

        approach_scale = motion_scale
        if approach_scale is None:
            approach_scale = self._motion_scale_for(current2, goal2)
        ok2, _c2, n2 = self._plan_and_execute_move(
            goal2, move_client, motion_scale=approach_scale, stop_event=stop_event
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
    ) -> Dict[str, Any]:
        """Plan and execute one collision-aware joint-space motion via MoveIt.

        If there is no collision-free path directly from the current pose, retreat
        to the fixed home pose and approach the pin from there. If the pin is still
        unreachable, report it as skipped so the scan can continue.
        """
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "error": "stopped"}

        if len(joints) != 6:
            raise ValueError("Waypoint must have 6 joint values.")

        goal_joints = [float(v) for v in joints]
        current_joints = self._current_joint_positions()
        if current_joints is not None:
            current_joints = self._normalize_joint_solution_to_reference(
                current_joints, current_joints
            )
            goal_joints = self._normalize_joint_solution_to_reference(current_joints, goal_joints)
            if len(self._last_plan_start_seed) == 6:
                goal_joints = self._normalize_joint_solution_to_reference(
                    self._last_plan_start_seed, goal_joints
                )
            goal_joints = self._normalize_joint_solution_to_reference(current_joints, goal_joints)
            if not self._ik_joint_angles_are_sane(goal_joints):
                return {
                    "ok": False,
                    "skipped": True,
                    "error": "goal joints outside UR3e limits",
                }
            if isinstance(tcp_target, dict):
                refreshed = self._goal_joints_from_tcp(tcp_target, current_joints)
                if refreshed is not None:
                    goal_joints = refreshed

        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            ws = workspace if workspace is not None else self._last_workspace
            if ws is None:
                ws = WorkspaceBox(enabled=True)
            self._apply_workspace_collision(ws)
            move_client = self._ensure_move_client()

        try:
            if current_joints is not None:
                start_valid, start_reason = self._state_is_valid(current_joints)
                if not start_valid:
                    return {
                        "ok": False,
                        "skipped": True,
                        "error": f"start state invalid: {start_reason or 'collision or limits'}",
                    }
            goal_valid, goal_reason = self._state_is_valid(goal_joints)
            if not goal_valid:
                return {
                    "ok": False,
                    "skipped": True,
                    "error": f"goal state invalid: {goal_reason or 'collision or limits'}",
                }

            # 1) Direct path from current pose (skipped when home-first is required).
            if not require_home_first:
                motion_scale = self._motion_scale_for(current_joints, goal_joints)
                ok, _code, _name = self._plan_and_execute_move(
                    goal_joints, move_client, motion_scale=motion_scale, stop_event=stop_event
                )
                if ok:
                    return {"ok": True, "executed": 1}

            # 2) Retreat to fixed home pose, then approach the pin.
            home_prefix = (
                "ring boundary — home-first required"
                if require_home_first
                else "no direct collision-free path"
            )
            return self._execute_via_home(
                goal_joints,
                current_joints,
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
                sys.stderr.write("UR3e MoveIt execute: already at scan home pose.\n")
                return {"ok": True, "already_at_home": True}

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
