"""
ROS 2 bridge for the HyperFusion UR3e WSL sidecar.

Always uses ur_robot_driver. use_mock_hardware selects ros2_control simulation vs real robot.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from typing import Any, Dict, List, Optional

from hyperfusion_ur3e.bridge.models import RobotStatus, TcpPose
from hyperfusion_ur3e.driver.driver_manager import Ur3eRosDriverManager

# HyperFusion UI / MoveIt order — always map /joint_states by name, not message order.
CANONICAL_JOINT_NAMES: List[str] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

MOCK_JOINT_TRAJECTORY_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
HARDWARE_JOINT_TRAJECTORY_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


class Ur3eRosBridge:
  """Adapter between HTTP handlers and ur_robot_driver via ROS 2."""

  def __init__(
      self,
      *,
      robot_ip: str = "192.168.0.10",
      dashboard_port: int = 29999,
      rtde_port: int = 30004,
      max_linear_speed_m_per_s: float = 0.05,
      max_linear_accel_m_per_s2: float = 0.3,
      ros_distro: str = "jazzy",
      ur_type: str = "ur3e",
      reverse_ip: str = "0.0.0.0",
      use_mock_hardware: bool = True,
      initial_joint_deg: Optional[List[float]] = None,
      ceiling_mount_height_m: Optional[float] = None,
      mount_roll_deg: Optional[float] = None,
      mount_pitch_deg: Optional[float] = None,
      mount_yaw_deg: Optional[float] = None,
      mount_offset_x_mm: Optional[float] = None,
      mount_offset_y_mm: Optional[float] = None,
      workspace_boundary_enabled: bool = True,
      workspace_length_m: float = 0.6,
      workspace_width_m: float = 0.6,
      workspace_height_m: float = 0.65,
      workspace_ceiling_clearance_m: float = 0.04,
  ) -> None:
    self.robot_ip = robot_ip
    self.reverse_ip = reverse_ip.strip() or "0.0.0.0"
    self.dashboard_port = dashboard_port
    self.rtde_port = rtde_port
    self.max_linear_speed_m_per_s = max_linear_speed_m_per_s
    self.max_linear_accel_m_per_s2 = max_linear_accel_m_per_s2
    self.ros_distro = ros_distro
    self.ur_type = ur_type
    self.use_mock_hardware = use_mock_hardware
    self.initial_joint_deg = initial_joint_deg or [0.0, -150.0, 120.0, 0.0, 90.0, 0.0]
    from hyperfusion_ur3e.urdf.mount_config import MountConfig

    self.mount_config = MountConfig.from_cli(
        ceiling_mount_height_m=ceiling_mount_height_m or 0.65,
        roll_deg=mount_roll_deg if mount_roll_deg is not None else 180.0,
        pitch_deg=mount_pitch_deg if mount_pitch_deg is not None else 0.0,
        yaw_deg=mount_yaw_deg if mount_yaw_deg is not None else 0.0,
        offset_x_mm=mount_offset_x_mm if mount_offset_x_mm is not None else 0.0,
        offset_y_mm=mount_offset_y_mm if mount_offset_y_mm is not None else 0.0,
    )
    self.ceiling_mount_height_m = self.mount_config.height_m
    self.workspace_boundary_enabled = workspace_boundary_enabled
    self.workspace_length_m = workspace_length_m
    self.workspace_width_m = workspace_width_m
    self.workspace_height_m = workspace_height_m
    self.workspace_ceiling_clearance_m = max(0.0, float(workspace_ceiling_clearance_m))

    self._lock = threading.RLock()
    self._driver_startup_lock = threading.RLock()
    self._status = RobotStatus(robot_ip=robot_ip)
    self._pose = TcpPose()
    self._moving = False
    self._connecting = False

    self._driver: Optional[Ur3eRosDriverManager] = None
    self._node = None
    self._executor = None
    self._spin_thread: Optional[threading.Thread] = None
    self._spin_stop = threading.Event()
    self._tf_buffer = None
    self._tf_listener = None
    self._trajectory_client = None
    self._joint_names: List[str] = []
    self._latest_joint_positions: Dict[str, float] = {}
    self._last_joint_state_rx_s: float = 0.0
    self._node_seq: int = 0
    self._active_goal_handle: Any = None
    self._stop_requested = threading.Event()

    self._connect_thread: Optional[threading.Thread] = None
    self._connect_cancel = threading.Event()
    self._connect_phase = "idle"
    self._connect_message = ""
    self._connect_error: Optional[str] = None
    self._connect_result: Optional[Dict[str, Any]] = None

  def _configured_home_joints_rad(self) -> List[float]:
    import math

    return [math.radians(float(deg)) for deg in self.initial_joint_deg]

  def _ensure_joint_states_stamper(self, *, reset: bool = False) -> None:
    Ur3eRosDriverManager.ensure_joint_states_stamper_for_distro(
        self.ros_distro,
        home_joints_deg=self.initial_joint_deg,
        reset=reset,
    )

  def _apply_home_joints_to_planner(self) -> None:
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    planner.apply_home_joints_from_body({"home_joints_deg": self.initial_joint_deg})

  def configured_workspace(self) -> Any:
    from hyperfusion_ur3e.moveit.scan_planner import WorkspaceBox

    return WorkspaceBox(
        enabled=self.workspace_boundary_enabled,
        length_m=self.workspace_length_m,
        width_m=self.workspace_width_m,
        height_m=self.workspace_height_m,
        mount_height_m=self.ceiling_mount_height_m,
        ceiling_clearance_m=self.workspace_ceiling_clearance_m,
    )

  def start_workspace_boundary_sync(self) -> None:
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    planner.set_default_workspace(self.configured_workspace())
    planner.start_boundary_keepalive()

  def sync_workspace_boundary(self, body: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner, workspace_from_dict

    workspace = self.configured_workspace()
    if isinstance(body, dict):
      workspace_cfg = body.get("workspace")
      if isinstance(workspace_cfg, dict):
        workspace = workspace_from_dict(workspace_cfg)

    self.start_workspace_boundary_sync()
    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    applied = planner.ensure_workspace_boundary_visible(workspace)
    return {"ok": True, "applied": applied}

  def _joint_trajectory_action_topic(self) -> str:
    if self.use_mock_hardware:
      return MOCK_JOINT_TRAJECTORY_ACTION
    return HARDWARE_JOINT_TRAJECTORY_ACTION

  def _joint_trajectory_controller_name(self) -> str:
    if self.use_mock_hardware:
      return "joint_trajectory_controller"
    return "scaled_joint_trajectory_controller"

  def _opposite_trajectory_controller_name(self) -> str:
    if self.use_mock_hardware:
      return "scaled_joint_trajectory_controller"
    return "joint_trajectory_controller"

  def _driver_mode_conflict_unlocked(self) -> bool:
    """True when a leftover ur_control launch uses the opposite mock/hardware mode."""
    import subprocess

    from hyperfusion_ur3e.ros_isolation import skip_stale_kill

    if skip_stale_kill():
      return False

    proc = subprocess.run(
        ["pgrep", "-af", "ur_control.launch.py"],
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
      return False
    want = "true" if self.use_mock_hardware else "false"
    other = "false" if self.use_mock_hardware else "true"
    saw_expected = False
    for line in (proc.stdout or "").splitlines():
      if f"use_mock_hardware:={want}" in line:
        saw_expected = True
      elif f"use_mock_hardware:={other}" in line:
        return True
    return not saw_expected and "ur_control.launch.py" in (proc.stdout or "")

  def _verify_ros_hardware_mode_unlocked(self, *, require_active_controller: Optional[bool] = None) -> None:
    """Fail fast when ROS controllers are not ready for the requested mode.

    On real hardware, scaled_joint_trajectory_controller becomes active only after
    External Control Play — so connect/prestart may skip that check until EC is up.
    """
    if require_active_controller is None:
      require_active_controller = self.use_mock_hardware

    if self._driver_mode_conflict_unlocked():
      raise RuntimeError(
          "The active UR driver was launched with the opposite use_mock_hardware mode. "
          "HyperFusion should stop stale WSL processes before launch; if this persists, "
          "close other UR3e CLI sessions and restart HyperFusion."
      )

    if not require_active_controller:
      return

    if self.use_mock_hardware:
      if self._joint_states_publishing(timeout_s=4.0):
        return
      raise RuntimeError(
          "UR driver is not ready for simulation (mock): joint_states not publishing yet."
      )

    import subprocess

    proc = subprocess.run(
        [
            "bash",
            "-lc",
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            "timeout 8 ros2 control list_controllers",
        ],
        capture_output=True,
        text=True,
    )
    output = proc.stdout or ""
    expected = self._joint_trajectory_controller_name()
    expected_active = any(
        expected in line and "active" in line for line in output.splitlines()
    )
    mode = "simulation (mock)" if self.use_mock_hardware else "hardware (real robot)"

    if not expected_active:
      detail = output.strip()[-1500:] or (proc.stderr or "").strip()[-1500:]
      raise RuntimeError(
          f"UR driver is not ready for {mode}: {expected} is not active.\n{detail}"
      )

  def _wait_for_external_control(
      self,
      *,
      timeout_s: float = 120.0,
      stop_event: Optional[threading.Event] = None,
      reason: str = "",
  ) -> bool:
    """Block until External Control reverse ports are up (or timeout / stop).

    Used when the teach pendant protective-stops (e.g. near joint limit) and
    drops the URCap connection mid-scan — user presses Play again to continue.
    """
    if self.use_mock_hardware:
      return True
    if self._external_control_reverse_connected():
      return True

    detail = f" ({reason})" if reason else ""
    sys.stderr.write(
        "UR3e bridge: External Control not connected"
        f"{detail} — on the teach pendant clear any protective stop, "
        f"open External Control (remote PC {self.reverse_ip}:50002), press Play.\n"
    )
    sys.stderr.flush()
    deadline = time.time() + max(5.0, float(timeout_s))
    while time.time() < deadline:
      if stop_event is not None and stop_event.is_set():
        return False
      if self._external_control_reverse_connected():
        sys.stderr.write("UR3e bridge: External Control restored — continuing.\n")
        sys.stderr.flush()
        return True
      time.sleep(0.5)
    sys.stderr.write(
        f"UR3e bridge: timed out waiting for External Control Play ({timeout_s:.0f}s).\n"
    )
    sys.stderr.flush()
    return False

  def _external_control_error(self, message: str) -> bool:
    text = (message or "").lower()
    needles = (
        "external control",
        "reverse",
        "not connected",
        "controller",
        "protective",
        "goal aborted",
        "goal canceled",
        "goal cancelled",
        "trajectory execution",
        "failed to send goal",
        "action server",
    )
    return any(n in text for n in needles)

  def _external_control_reverse_connected(self) -> bool:
    """True when the robot has reverse TCP sessions for External Control.

    Port 50002 (script sender) is often ephemeral — it closes after the URCap
    fetches the driver script. While the program runs, 50001/50003/50004 stay up.
    """
    import subprocess

    ports = "50001|50002|50003|50004"
    robot = self.robot_ip.strip()
    grep_robot = f" | grep '{robot}'" if robot else ""
    try:
      proc = subprocess.run(
          [
              "bash",
              "-lc",
              f"ss -tn state established | grep -E ':({ports}) '{grep_robot} | grep -q .",
          ],
          capture_output=True,
          timeout=3.0,
      )
      return proc.returncode == 0
    except Exception:
      return False

  def _scaled_controller_active_unlocked(self) -> bool:
    import subprocess

    proc = subprocess.run(
        [
            "bash",
            "-lc",
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            "timeout 4 ros2 control list_controllers",
        ],
        capture_output=True,
        text=True,
    )
    output = proc.stdout or ""
    return any(
        "scaled_joint_trajectory_controller" in line and "active" in line
        for line in output.splitlines()
    )

  def _verify_real_external_control_unlocked(self, timeout_s: float = 45.0) -> None:
    """Real robot: require External Control Play (reverse ports + trajectory ready)."""
    if self.use_mock_hardware:
      return

    deadline = time.time() + max(5.0, timeout_s)
    while time.time() < deadline:
      reverse_connected = self._external_control_reverse_connected()
      scaled_active = self._scaled_controller_active_unlocked()
      traj_ready = (
          self._trajectory_client is not None
          and self._trajectory_client.server_is_ready()
      )
      if reverse_connected and (scaled_active or traj_ready):
        sys.stderr.write(
            "UR3e bridge: External Control connected (reverse ports), "
            "scaled_joint_trajectory_controller ready.\n"
        )
        return
      time.sleep(0.5)

    raise RuntimeError(
        "Real robot is not fully connected via External Control. "
        f"On the teach pendant: External Control → remote PC {self.reverse_ip}:50002 → "
        "press Play while /connect is waiting. "
        "Verify in WSL: ss -tn state established | grep -E '50001|50003|50004' && "
        "ros2 action info /scaled_joint_trajectory_controller/follow_joint_trajectory"
    )

  def _verify_joint_trajectory_controller_active(self) -> None:
    if self._trajectory_client is not None and self._trajectory_client.server_is_ready():
      sys.stderr.write(
          f"UR3e bridge: {self._joint_trajectory_controller_name()} action server ready\n"
      )
      return

    import subprocess

    controller = self._joint_trajectory_controller_name()
    cmd = (
      f"source /opt/ros/{self.ros_distro}/setup.bash && "
      "timeout 4 ros2 control list_controllers"
    )
    try:
      proc = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=6.0,
      )
    except subprocess.TimeoutExpired:
      sys.stderr.write(
          f"UR3e bridge: WARNING — could not confirm {controller} via ros2 CLI "
          "(action server is ready).\n"
      )
      return
    output = proc.stdout or ""
    for line in output.splitlines():
      if controller in line and "active" in line:
        sys.stderr.write(f"UR3e bridge: {controller} is active\n")
        return

    sys.stderr.write(
        f"UR3e bridge: WARNING — {controller} is not active; "
        "joint moves will not animate the mock robot.\n"
    )
    if output.strip():
      sys.stderr.write(output.strip() + "\n")

  def _new_driver_manager(self) -> Ur3eRosDriverManager:
    return Ur3eRosDriverManager(
        ros_distro=self.ros_distro,
        ur_type=self.ur_type,
        robot_ip=self.robot_ip,
        reverse_ip=self.reverse_ip,
        use_mock_hardware=self.use_mock_hardware,
        initial_joint_deg=self.initial_joint_deg,
        ceiling_mount_height_m=self.ceiling_mount_height_m,
        mount_roll_deg=self.mount_config.roll_deg,
        mount_pitch_deg=self.mount_config.pitch_deg,
        mount_yaw_deg=self.mount_config.yaw_deg,
        mount_offset_x_mm=self.mount_config.offset_x_m * 1000.0,
        mount_offset_y_mm=self.mount_config.offset_y_m * 1000.0,
    )

  def driver_ready_for_connect(self) -> bool:
    """True when the UR driver can accept Connect (External Control / mock)."""
    with self._lock:
      if self._connecting:
        return False
      driver = self._driver
      if driver is None or not driver.running:
        return False
      if self._status.fault:
        return False
      if self._status.driver_state != "idle":
        return False
    # Outside lock: port/controller probes may take seconds.
    return driver.connect_ready()

  def status(self) -> RobotStatus:
    with self._lock:
      return RobotStatus(
        connected=self._status.connected,
        driver_state=self._status.driver_state,
        robot_ip=self._status.robot_ip,
        fault=self._status.fault,
      )

  def joint_states_ok(self) -> bool:
    with self._lock:
      if not self._joint_names:
        return False
      if self._last_joint_state_rx_s <= 0.0:
        return False
      return (time.time() - self._last_joint_state_rx_s) < 2.0

  def warm_driver(self) -> Dict[str, Any]:
    """Start ur_robot_driver only (no ROS bridge node). Used by sidecar prestart."""
    sys.stderr.write("UR3e bridge: warming ur_robot_driver subprocess…\n")
    with self._driver_startup_lock:
      self._warm_driver_without_blocking_health()
    return {"ok": True, "driver": "warming"}

  def _ensure_driver_running(self) -> None:
    """Serialize driver startup with prestart/warmup (avoids SIGTERM races)."""
    with self._driver_startup_lock:
      self._ensure_driver_running_unlocked()

  def _warm_driver_without_blocking_health(self) -> None:
    """Start or reuse the driver without holding bridge._lock across slow subprocess launch."""
    conflict = False
    reuse = False
    restart = False
    start_new = False
    external_running = False

    with self._lock:
      if self._driver_mode_conflict_unlocked():
        conflict = True
        if self._driver is not None:
          self._driver.stop()
          self._driver = None
      elif self._driver is not None and self._driver.running:
        if self._driver.controller_manager_ready():
          reuse = True
        elif (
            not self.use_mock_hardware
            and self._external_control_reverse_connected()
        ):
          reuse = True
        elif self.use_mock_hardware:
          # Mock launch already up; sticky connect_ready cache is set after first probe.
          reuse = True
        else:
          restart = True
          self._driver.stop()
          self._driver = None
      else:
        start_new = True
        external_running = self._external_driver_running()

    if reuse:
      from hyperfusion_ur3e.urdf.tool_payload_config import (
        ToolPayloadConfig,
        log_payload_probe_failure,
        probe_robot_state_publisher_payload,
      )

      payload_cfg = ToolPayloadConfig.from_env()
      if payload_cfg.enabled and not probe_robot_state_publisher_payload(
          self.ros_distro, payload_cfg, retries=3, retry_delay_s=1.0
      ):
        log_payload_probe_failure("active driver missing tool payload", payload_cfg)
      sys.stderr.write(
          "UR3e bridge: reusing mock driver subprocess "
          "(ros2 control CLI can be slow during warmup).\n"
          if self.use_mock_hardware
          else "UR3e bridge: reusing active driver subprocess\n"
      )
      self._ensure_joint_states_stamper()
      with self._lock:
        self._status.driver_state = "idle"
        self._status.fault = ""
      if self._driver is not None:
        self._driver.schedule_tool_payload_probe()
      return

    if conflict:
      sys.stderr.write(
          "UR3e bridge: stale UR driver mode conflict — stopping leftover processes…\n"
      )
      Ur3eRosDriverManager.stop_stale_launches()

    if not (start_new or restart):
      return

    if external_running:
      sys.stderr.write(
          "UR3e bridge: restarting ur_control for HyperFusion ceiling-mount description…\n"
      )
    else:
      sys.stderr.write("UR3e bridge: starting ur_robot_driver (may take ~2 min)…\n")

    Ur3eRosDriverManager.stop_stale_launches()
    driver = self._new_driver_manager()
    with self._lock:
      if self._driver is not None:
        self._driver.stop()
      self._driver = driver
    try:
      driver.start()
    except Exception:
      with self._lock:
        if self._driver is driver:
          self._driver = None
      driver.stop()
      raise

    with self._lock:
      self._status.driver_state = "idle"
      self._status.fault = ""

    if not self.use_mock_hardware:
      try:
        with self._lock:
          self._verify_ros_hardware_mode_unlocked(require_active_controller=False)
      except Exception as exc:
        sys.stderr.write(f"UR3e bridge: driver ready but controller check pending: {exc}\n")
    self._ensure_joint_states_stamper()
    sys.stderr.write(
        "UR3e bridge: simulation driver prestart complete.\n"
        if self.use_mock_hardware
        else "UR3e bridge: driver prestart complete.\n"
    )
    if self._driver is not None:
      self._driver.schedule_tool_payload_probe()

  def _set_connect_phase(self, phase: str, message: str) -> None:
    with self._lock:
      if self._connect_phase == phase and self._connect_message == message:
        return
      self._connect_phase = phase
      self._connect_message = message
    sys.stderr.write(f"UR3e bridge: connect phase={phase} — {message}\n")

  def _raise_if_connect_cancelled(self) -> None:
    if self._connect_cancel.is_set():
      raise RuntimeError("Connect cancelled.")

  @staticmethod
  def _script_port_listening() -> bool:
    from hyperfusion_ur3e.driver.driver_manager import Ur3eRosDriverManager

    return Ur3eRosDriverManager._script_sender_port_listening()

  def connect_start(self, robot_ip: Optional[str] = None) -> Dict[str, Any]:
    """Begin connect on a background thread (GUI polls connect_status)."""
    with self._lock:
      if self._connecting or (
          self._connect_thread is not None and self._connect_thread.is_alive()
      ):
        raise RuntimeError("Connect already in progress.")
      if self._status.connected:
        if not self.use_mock_hardware:
          try:
            self._verify_real_external_control_unlocked(timeout_s=5.0)
          except RuntimeError:
            sys.stderr.write(
                "UR3e bridge: stale connect — External Control no longer active, reconnecting…\n"
            )
            self._shutdown_ros_unlocked()
            self._status.connected = False
          else:
            mode = "simulation" if self.use_mock_hardware else "hardware"
            result = {
                "ok": True,
                "already_connected": True,
                "phase": "complete",
                "message": "Already connected",
                "mode": mode,
                "use_mock_hardware": self.use_mock_hardware,
                **self.status().to_dict(),
            }
            self._connect_phase = "complete"
            self._connect_message = "Already connected"
            self._connect_result = result
            return self.connect_status()
        else:
          mode = "simulation" if self.use_mock_hardware else "hardware"
          result = {
              "ok": True,
              "already_connected": True,
              "phase": "complete",
              "message": "Already connected",
              "mode": mode,
              "use_mock_hardware": self.use_mock_hardware,
              **self.status().to_dict(),
          }
          self._connect_phase = "complete"
          self._connect_message = "Already connected"
          self._connect_result = result
          return self.connect_status()

      if robot_ip is not None and str(robot_ip).strip():
        self.robot_ip = str(robot_ip).strip()
        self._status.robot_ip = self.robot_ip

      if self._node is not None or self._spin_thread is not None:
        self._shutdown_ros_unlocked()

      self._connect_cancel.clear()
      self._stop_requested.clear()
      self._connect_error = None
      self._connect_result = None
      self._connecting = True
      self._status.driver_state = "connecting"
      if self.use_mock_hardware:
        self._connect_phase = "starting_driver"
        self._connect_message = "Starting simulation driver…"
      else:
        self._connect_phase = "starting_driver"
        self._connect_message = "Starting ROS driver…"

    self._connect_thread = threading.Thread(
        target=self._connect_worker,
        daemon=True,
        name="ur3e-connect",
    )
    self._connect_thread.start()
    return self.connect_status()

  def _connect_worker(self) -> None:
    try:
      self._connect_ros_unlocked()
      self._raise_if_connect_cancelled()
      with self._lock:
        if self._connect_cancel.is_set():
          raise RuntimeError("Connect cancelled.")
        self._status.connected = True
        self._status.driver_state = "idle"
        self._status.fault = ""
        mode = "simulation" if self.use_mock_hardware else "hardware"
        self._connect_result = {
            "ok": True,
            "phase": "complete",
            "message": "Connected",
            "mode": mode,
            "use_mock_hardware": self.use_mock_hardware,
            **self.status().to_dict(),
        }
        self._connect_phase = "complete"
        self._connect_message = (
            "Connected (simulation)" if self.use_mock_hardware else "Connected (hardware)"
        )
      try:
        self.sync_workspace_boundary()
      except Exception as exc:
        sys.stderr.write(f"UR3e bridge: workspace boundary sync after connect: {exc}\n")
    except Exception as exc:
      cancelled = self._connect_cancel.is_set() or "cancel" in str(exc).lower()
      with self._lock:
        if cancelled:
          self._connect_phase = "cancelled"
          self._connect_message = "Connect cancelled"
          self._connect_error = None
          self._status.connected = False
          # Keep prestarted driver warm so Connect re-enables immediately.
          driver_alive = self._driver is not None and self._driver.running
          self._status.driver_state = "idle" if driver_alive else "disconnected"
        else:
          self._connect_phase = "failed"
          self._connect_message = str(exc)
          self._connect_error = str(exc)
          self._status.fault = str(exc)
          self._status.driver_state = "fault"
          self._status.connected = False
      # Never hold bridge._lock across ROS/driver teardown (blocks /connect/status).
      try:
        if cancelled:
          self._shutdown_bridge_unlocked()
        else:
          self._shutdown_ros_unlocked()
      except Exception as shutdown_exc:
        sys.stderr.write(f"UR3e bridge: connect teardown after error: {shutdown_exc}\n")
    finally:
      with self._lock:
        self._connecting = False
        if self._connect_phase == "cancelled":
          driver_alive = self._driver is not None and self._driver.running
          if driver_alive:
            self._status.driver_state = "idle"

  def connect_status(self) -> Dict[str, Any]:
    with self._lock:
      phase = self._connect_phase
      in_progress = phase in {
          "starting_driver",
          "waiting_external_control",
          "bridge_setup",
          "finishing",
          "cancelling",
      }
      payload: Dict[str, Any] = {
          "ok": phase == "complete",
          "phase": phase,
          "message": self._connect_message,
          "in_progress": in_progress,
          "use_mock_hardware": self.use_mock_hardware,
          "reverse_connected": self._external_control_reverse_connected(),
          "script_port_listening": self._script_port_listening(),
          "connected": self._status.connected,
          "driver_state": self._status.driver_state,
          "robot_ip": self._status.robot_ip,
          "reverse_ip": self.reverse_ip,
          "fault": self._status.fault or None,
      }
      if self._connect_error:
        payload["error"] = self._connect_error
      if self._connect_result and phase == "complete":
        payload.update(self._connect_result)
      return payload

  def connect_cancel(self) -> Dict[str, Any]:
    with self._lock:
      if not self._connecting and not (
          self._connect_thread is not None and self._connect_thread.is_alive()
      ):
        return {"ok": True, "cancelled": False, "phase": self._connect_phase}
      self._connect_cancel.set()
      self._connect_phase = "cancelling"
      self._connect_message = "Cancelling connect…"
    return {"ok": True, "cancelled": True, **self.connect_status()}

  def connect(self) -> Dict[str, Any]:
    with self._lock:
      if self._connecting:
        raise RuntimeError("Connect already in progress.")
      if self._status.connected:
        if not self.use_mock_hardware:
          try:
            self._verify_real_external_control_unlocked(timeout_s=5.0)
          except RuntimeError:
            sys.stderr.write(
                "UR3e bridge: stale connect — External Control no longer active, reconnecting…\n"
            )
            self._shutdown_ros_unlocked()
            self._status.connected = False
          else:
            mode = "simulation" if self.use_mock_hardware else "hardware"
            return {
                "ok": True,
                "already_connected": True,
                "mode": mode,
                "use_mock_hardware": self.use_mock_hardware,
                **self.status().to_dict(),
            }
        else:
          mode = "simulation" if self.use_mock_hardware else "hardware"
          return {
              "ok": True,
              "already_connected": True,
              "mode": mode,
              "use_mock_hardware": self.use_mock_hardware,
              **self.status().to_dict(),
          }

      if self._node is not None or self._spin_thread is not None:
        self._shutdown_ros_unlocked()

      self._connecting = True
      self._status.driver_state = "connecting"
      try:
        self._connect_ros_unlocked()
      except Exception as exc:
        self._shutdown_ros_unlocked()
        self._status.fault = str(exc)
        self._status.driver_state = "fault"
        raise
      finally:
        self._connecting = False

      self._status.connected = True
      mode = "simulation" if self.use_mock_hardware else "hardware"
      self._status.driver_state = "idle"
      self._status.fault = ""
      self._apply_home_joints_to_planner()
      return {
        "ok": True,
        "mode": mode,
        "use_mock_hardware": self.use_mock_hardware,
        **self.status().to_dict(),
      }

  def _start_ros_spin_unlocked(self) -> None:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    if self._node is None:
      raise RuntimeError("ROS node not created.")

    self._spin_stop.clear()
    self._executor = SingleThreadedExecutor()
    self._executor.add_node(self._node)
    self._spin_thread = threading.Thread(
      target=self._ros_spin_loop,
      daemon=True,
      name="ur3e-ros-spin",
    )
    self._spin_thread.start()

  def _ros_spin_loop(self) -> None:
    while not self._spin_stop.is_set():
      if self._executor is None:
        break
      try:
        self._executor.spin_once(timeout_sec=0.1)
      except Exception:
        if self._spin_stop.is_set():
          break
        raise

  def _wait_future(self, future: Any, timeout_sec: float) -> Any:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
      if self._stop_requested.is_set():
        return None
      if future.done():
        return future.result()
      time.sleep(0.05)
    raise TimeoutError(f"ROS operation timed out after {timeout_sec:.0f}s.")

  def _external_driver_running(self) -> bool:
    import subprocess

    from hyperfusion_ur3e.ros_isolation import skip_stale_kill

    if skip_stale_kill():
      return False

    if self._driver is not None and self._driver.running:
      return False

    proc = subprocess.run(
      ["pgrep", "-f", f"ur_control.launch.py.*robot_ip:={self.robot_ip}"],
      capture_output=True,
      text=True,
    )
    return proc.returncode == 0

  def _joint_states_publishing(self, timeout_s: float = 5.0) -> bool:
    """Best-effort check that joint_states has data (ros2 control list_controllers can hang on WSL)."""
    if not self.use_mock_hardware and self._external_control_reverse_connected():
      return True

    import subprocess

    wait_s = max(1, int(timeout_s))
    for topic in (
        "/joint_state_broadcaster/joint_states",
        "/joint_states",
    ):
      cmd = (
          f"export ROS_LOCALHOST_ONLY=1 && "
          f"source /opt/ros/{self.ros_distro}/setup.bash && "
          f"timeout {wait_s} ros2 topic hz {topic} --window 1 2>/dev/null | "
          "head -1 | grep -q average"
      )
      try:
        proc = subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=float(wait_s) + 3.0,
        )
      except subprocess.TimeoutExpired:
        continue
      if proc.returncode == 0:
        return True
    return False

  def _start_driver_unlocked(self) -> None:
    Ur3eRosDriverManager.stop_stale_launches()
    self._driver = self._new_driver_manager()
    try:
      self._driver.start()
    except Exception:
      if self._driver is not None:
        self._driver.stop()
      self._driver = None
      raise

  def _ensure_driver_running_unlocked(self) -> None:
    if self._driver_mode_conflict_unlocked():
      sys.stderr.write(
          "UR3e bridge: stale UR driver mode conflict — stopping leftover processes…\n"
      )
      Ur3eRosDriverManager.stop_stale_launches()
      if self._driver is not None:
        self._driver.stop()
        self._driver = None

    if self._driver is not None and self._driver.running:
      from hyperfusion_ur3e.urdf.tool_payload_config import (
        ToolPayloadConfig,
        log_payload_probe_failure,
        probe_robot_state_publisher_payload,
      )

      payload_cfg = ToolPayloadConfig.from_env()
      if payload_cfg.enabled and not probe_robot_state_publisher_payload(
          self.ros_distro, payload_cfg, retries=3, retry_delay_s=1.0
      ):
        log_payload_probe_failure("active driver missing tool payload", payload_cfg)
      if self._driver is not None and self._driver.running:
        if self._driver.controller_manager_ready():
          sys.stderr.write("UR3e bridge: reusing active driver subprocess\n")
        elif (
            not self.use_mock_hardware
            and self._external_control_reverse_connected()
        ):
          sys.stderr.write(
              "UR3e bridge: reusing driver — External Control reverse ports are active "
              "(ros2 control CLI is slow).\n"
          )
        elif self.use_mock_hardware:
          sys.stderr.write(
              "UR3e bridge: reusing mock driver subprocess "
              "(ros2 control CLI can be slow during warmup).\n"
          )
        else:
          sys.stderr.write(
              "UR3e bridge: driver process present but controller manager not ready — restarting…\n"
          )
          self._driver.stop()
          self._driver = None

    if self._driver is None or not self._driver.running:
      if self._external_driver_running():
        sys.stderr.write(
            "UR3e bridge: restarting ur_control for HyperFusion ceiling-mount description…\n"
        )
      else:
        sys.stderr.write("UR3e bridge: starting ur_robot_driver (may take ~2 min)…\n")
      self._start_driver_unlocked()

    self._verify_ros_hardware_mode_unlocked(require_active_controller=self.use_mock_hardware)
    self._ensure_joint_states_stamper()

  def _connect_ros_unlocked(self) -> None:
    last_exc: Optional[Exception] = None
    for attempt in range(2):
      try:
        self._connect_ros_unlocked_once(force_fresh_driver=attempt > 0)
        return
      except RuntimeError as exc:
        last_exc = exc
        message = str(exc).lower()
        retryable = (
            "external control" not in message
            and (
                "joint_states" in message
                or "controller manager" in message
                or "ur driver exited" in message
            )
            and not self._external_control_reverse_connected()
        )
        if attempt == 0 and retryable:
          sys.stderr.write("UR3e bridge: retrying connect with a fresh driver…\n")
          Ur3eRosDriverManager.stop_stale_launches()
          if self._driver is not None:
            self._driver.stop()
            self._driver = None
          continue
        raise
    if last_exc is not None:
      raise last_exc

  def _connect_ros_unlocked_once(self, *, force_fresh_driver: bool = False) -> None:
    self._raise_if_connect_cancelled()
    self._set_connect_phase(
        "starting_driver",
        "Starting simulation driver…" if self.use_mock_hardware else "Starting ROS driver…",
    )
    if force_fresh_driver:
      with self._driver_startup_lock:
        Ur3eRosDriverManager.stop_stale_launches()
        if self._driver is not None:
          self._driver.stop()
          self._driver = None
        self._start_driver_unlocked()
    else:
      self._ensure_driver_running()

    self._verify_ros_hardware_mode_unlocked(require_active_controller=self.use_mock_hardware)
    self._ensure_joint_states_stamper(reset=True)
    self._apply_home_joints_to_planner()

    if not self.use_mock_hardware and self._script_port_listening():
      self._set_connect_phase(
          "waiting_external_control",
          f"Press Play on External Control (remote PC {self.reverse_ip}:50002)",
      )

    joint_deadline = time.time() + (45.0 if self.use_mock_hardware else 120.0)
    last_play_prompt_s = 0.0
    while time.time() < joint_deadline:
      self._raise_if_connect_cancelled()
      if not self.use_mock_hardware and self._external_control_reverse_connected():
        break
      if self._joint_states_publishing(timeout_s=3.0):
        break
      if not self.use_mock_hardware and not self._external_control_reverse_connected():
        now = time.time()
        if now - last_play_prompt_s >= 15.0:
          self._set_connect_phase(
              "waiting_external_control",
              f"Press Play on External Control (remote PC {self.reverse_ip}:50002)",
          )
          last_play_prompt_s = now
      time.sleep(1.0)
    else:
      if self.use_mock_hardware:
        hint = (
            "Check in WSL: ros2 control list_controllers && "
            "ros2 topic echo /joint_states --once"
        )
      else:
        hint = (
            f"On the teach pendant: External Control → remote PC {self.reverse_ip}:50002 → Play. "
            "Then in WSL: ss -tn state established | grep -E '50001|50003|50004' && "
            "ros2 topic echo /joint_states --once"
        )
      raise RuntimeError(
          "UR driver is running but /joint_states is not publishing yet. " + hint
      )

    if self._node is not None:
      return

    self._set_connect_phase("bridge_setup", "Setting up ROS bridge…")

    import os
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from tf2_ros import Buffer, TransformListener
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient

    joint_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    if not rclpy.ok():
      rclpy.init()

    class _Ur3eNode(Node):
      pass

    self._node_seq += 1
    node_name = f"hyperfusion_ur3e_bridge_{os.getpid()}_{self._node_seq}"
    self._node = _Ur3eNode(node_name)
    self._start_ros_spin_unlocked()

    self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
    # spin_thread=False: one dedicated executor thread handles all callbacks.
    self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)
    self._trajectory_client = ActionClient(
      self._node,
      FollowJointTrajectory,
      self._joint_trajectory_action_topic(),
    )

    deadline = time.time() + 90.0
    while time.time() < deadline:
      self._raise_if_connect_cancelled()
      if self._trajectory_client.server_is_ready():
        sys.stderr.write(
            f"UR3e bridge: trajectory action server ready "
            f"({self._joint_trajectory_action_topic()})\n"
        )
        break
      time.sleep(0.2)
    else:
      self._trajectory_client = None
      self._status.fault = "Trajectory action server not ready; using pose fallback."

    if self._trajectory_client is not None:
      self._verify_joint_trajectory_controller_active()

    joint_event = threading.Event()

    def _on_joint_state(msg: JointState) -> None:
      if msg.name:
        self._last_joint_state_rx_s = time.time()
        if not self._joint_names:
          self._joint_names = list(msg.name)
          joint_event.set()
        for index, name in enumerate(msg.name):
          if index < len(msg.position):
            self._latest_joint_positions[name] = float(msg.position[index])

    self._node.create_subscription(
        JointState, "/joint_states", _on_joint_state, joint_qos
    )
    ec_active = (
        not self.use_mock_hardware and self._external_control_reverse_connected()
    )
    joint_wait_s = 8.0 if ec_active else 90.0
    deadline = time.time() + joint_wait_s
    while time.time() < deadline and not joint_event.is_set():
      self._raise_if_connect_cancelled()
      time.sleep(0.1)

    if not self._joint_names:
      if ec_active:
        self._joint_names = list(CANONICAL_JOINT_NAMES)
        sys.stderr.write(
            "UR3e bridge: /joint_states not yet on bridge node; "
            "using canonical joint names (External Control is active).\n"
        )
      else:
        raise RuntimeError(
            "UR driver started but /joint_states is not publishing. "
            "In WSL run: ros2 control list_controllers && ros2 topic echo /joint_states --once"
        )

    self._set_connect_phase("finishing", "Verifying External Control…")
    self._verify_real_external_control_unlocked(timeout_s=15.0 if ec_active else 45.0)

    try:
      self._update_pose_from_tf_unlocked()
    except Exception as exc:
      self._status.fault = f"{self._status.fault} TF unavailable ({exc}).".strip()

  def disconnect(self) -> Dict[str, Any]:
    """Tear down bridge + driver and prepare for a clean reconnect.

    Disconnect used to kill the driver while the UI still thought prestart was
    ready, so the next Connect cold-started under a short timeout and often
    failed. Also cancel any in-flight connect and clear the stop latch.
    """
    self._connect_cancel.set()
    connect_thread: Optional[threading.Thread] = None
    with self._lock:
      if self._moving:
        self._stop_unlocked()
      connect_thread = self._connect_thread

    if connect_thread is not None and connect_thread.is_alive():
      # Do not block forever — driver.start() may ignore cancel until timeout.
      connect_thread.join(timeout=8.0)

    with self._lock:
      self._shutdown_ros_unlocked()
      self._status.connected = False
      self._status.driver_state = "disconnected"
      self._status.fault = ""
      self._stop_requested.clear()
      self._connecting = False
      self._connect_phase = "idle"
      self._connect_message = ""
      self._connect_error = None
      self._connect_result = None
      # If the connect worker is still alive, keep cancel set so it cannot mark connected.
      still_connecting = (
          connect_thread is not None and connect_thread.is_alive()
      )
      if still_connecting:
        self._connect_thread = connect_thread
      else:
        self._connect_thread = None
        self._connect_cancel.clear()
      status = {"ok": True, **self.status().to_dict()}

    # Free reverse ports / orphan move_group outside the bridge lock.
    try:
      Ur3eRosDriverManager.stop_stale_launches()
    except Exception as exc:
      sys.stderr.write(f"UR3e bridge: disconnect stale cleanup: {exc}\n")

    try:
      from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner

      planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
      planner.cancel_active_move()
    except Exception:
      pass

    # Re-warm driver so next Connect is not a cold ~2 min start.
    threading.Thread(
        target=self._rewarm_driver_after_disconnect,
        daemon=True,
        name="ur3e-rewarm-after-disconnect",
    ).start()
    return status

  def _rewarm_driver_after_disconnect(self) -> None:
    try:
      sys.stderr.write(
          "UR3e bridge: re-warming ur_robot_driver after disconnect…\n"
      )
      self.warm_driver()
    except Exception as exc:
      sys.stderr.write(f"UR3e bridge: re-warm after disconnect failed: {exc}\n")
      sys.stderr.flush()

  def get_pose(self) -> Dict[str, Any]:
    with self._lock:
      if not self._status.connected:
        raise RuntimeError("Robot not connected.")
      # Must refresh from TF — never return the TcpPose() default (z=0.4).
      self._update_pose_from_tf_unlocked(timeout_s=1.5)
      payload: Dict[str, Any] = {
          "ok": True,
          "pose": self._pose.as_list(),
          "frame": "base_link",
          "tip_link": "hyperfusion_tcp",
          **self.status().to_dict(),
      }
      # Extra fields for BFS/UR3e calibration (hand–eye uses tool0, not CAD TCP).
      try:
        tool0 = self._lookup_base_link_pose_unlocked("tool0", timeout_s=1.0)
        payload["tool0"] = tool0.as_list()
        payload["tool0_link"] = "tool0"
      except Exception:
        payload["tool0"] = None
      try:
        names, positions = self._ordered_joint_state_unlocked()
        payload["joints"] = {"names": names, "positions": list(positions)}
      except Exception:
        payload["joints"] = None
      return payload

  def get_joints(self) -> Dict[str, Any]:
    with self._lock:
      if not self._status.connected:
        raise RuntimeError("Robot not connected.")
      names, positions = self._ordered_joint_state_unlocked()
      from hyperfusion_ur3e.joint_angles import coalesce_joints_for_moveit

      home_rad = self._configured_home_joints_rad()
      if len(positions) == 6 and len(home_rad) == 6:
        positions = coalesce_joints_for_moveit(home_rad, positions)
      return {
        "ok": True,
        "names": names,
        "positions": positions,
        **self.status().to_dict(),
      }

  def move_j(
      self,
      positions: List[float],
      *,
      wait: bool = True,
  ) -> Dict[str, Any]:
    if len(positions) != 6:
      raise ValueError("Joint positions must have 6 values.")

    target = [float(v) for v in positions]

    with self._lock:
      if not self._status.connected:
        raise RuntimeError("Robot not connected.")
      if self._moving:
        raise RuntimeError("Robot is already moving.")

      if self._trajectory_client is None or not self._joint_names:
        return self._move_joints_fallback_unlocked(target, wait)

      self._moving = True
      self._status.driver_state = "moving"

    stopped = False
    try:
      self._stop_requested.clear()
      stopped = not self._move_ros_joints_unlocked(target)
    finally:
      with self._lock:
        self._moving = False
        if self._status.connected:
          self._status.driver_state = "idle"
        self._active_goal_handle = None

    with self._lock:
      names, current = self._ordered_joint_state_unlocked()
      payload: Dict[str, Any] = {
        "ok": True,
        "names": names,
        "positions": current,
        **self.status().to_dict(),
      }
      if stopped:
        payload["stopped"] = True
      return payload

  def _ordered_joint_state_unlocked(self) -> tuple[List[str], List[float]]:
    positions: List[float] = []
    for name in CANONICAL_JOINT_NAMES:
      positions.append(float(self._latest_joint_positions.get(name, 0.0)))
    return list(CANONICAL_JOINT_NAMES), positions

  def _move_joints_fallback_unlocked(self, target: List[float], wait: bool) -> Dict[str, Any]:
    self._moving = True
    self._status.driver_state = "moving"
    names, _current = self._ordered_joint_state_unlocked()
    for index, name in enumerate(names):
      self._latest_joint_positions[name] = target[index]
    delay_s = 0.35
    if wait:
      time.sleep(delay_s)
      self._moving = False
      self._status.driver_state = "idle"
    else:
      threading.Thread(
        target=self._finish_pose_fallback,
        args=(delay_s,),
        daemon=True,
        name="ur3e-joints-fallback",
      ).start()
    return {
      "ok": True,
      "names": names,
      "positions": target,
      **self.status().to_dict(),
    }

  def _move_ros_joints_unlocked(self, target: List[float]) -> bool:
    """Execute joint trajectory. Returns False if motion was stopped via /stop."""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory

    with self._lock:
      _, current = self._ordered_joint_state_unlocked()
    max_delta = max(abs(target[i] - current[i]) for i in range(6))
    duration_s = max(0.8, max_delta / 0.35 + 0.25)

    traj = JointTrajectory()
    traj.joint_names = list(CANONICAL_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = target
    point.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
    traj.points = [point]

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory = traj

    send_future = self._trajectory_client.send_goal_async(goal_msg)
    goal_handle = self._wait_future(send_future, 30.0)
    if self._stop_requested.is_set():
      return False
    if goal_handle is None or not goal_handle.accepted:
      raise RuntimeError("Joint trajectory goal rejected by controller.")

    with self._lock:
      self._active_goal_handle = goal_handle

    result_future = goal_handle.get_result_async()
    self._wait_future(result_future, max(30.0, duration_s + 10.0))

    if self._stop_requested.is_set():
      return False

    return True

  def move_l(
      self,
      pose: List[float],
      *,
      speed: Optional[float] = None,
      accel: Optional[float] = None,
      wait: bool = True,
  ) -> Dict[str, Any]:
    target = TcpPose.from_list(pose)
    speed_m_s = min(speed or self.max_linear_speed_m_per_s, self.max_linear_speed_m_per_s)

    with self._lock:
      if not self._status.connected:
        raise RuntimeError("Robot not connected.")
      if self._moving:
        raise RuntimeError("Robot is already moving.")

      if self._trajectory_client is None or not self._joint_names:
        return self._move_pose_fallback_unlocked(target, speed_m_s, wait)

      self._moving = True
      self._status.driver_state = "moving"
      try:
        self._move_ros_joint_step_unlocked(target, speed_m_s)
        try:
          self._update_pose_from_tf_unlocked()
        except Exception:
          self._pose = target
      finally:
        self._moving = False
        self._status.driver_state = "idle"

      return {
        "ok": True,
        "pose": self._pose.as_list(),
        "speed_m_per_s": speed_m_s,
        "wait": wait,
        **self.status().to_dict(),
      }

  def _move_pose_fallback_unlocked(self, target: TcpPose, speed_m_s: float, wait: bool) -> Dict[str, Any]:
    self._moving = True
    self._status.driver_state = "moving"
    self._pose = target
    delay_s = max(0.05, 0.25 / max(speed_m_s, 0.01))
    if wait:
      time.sleep(delay_s)
      self._moving = False
      self._status.driver_state = "idle"
    else:
      threading.Thread(
        target=self._finish_pose_fallback,
        args=(delay_s,),
        daemon=True,
        name="ur3e-pose-fallback",
      ).start()
    return {
      "ok": True,
      "pose": self._pose.as_list(),
      "speed_m_per_s": speed_m_s,
      "wait": wait,
      **self.status().to_dict(),
    }

  def _move_ros_joint_step_unlocked(self, target: TcpPose, speed_m_s: float) -> None:
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory

    current_joints: Optional[List[float]] = None
    received = threading.Event()

    def _capture(msg: JointState) -> None:
      nonlocal current_joints
      if msg.position and len(msg.position) >= 6:
        current_joints = list(msg.position[:6])
        received.set()

    sub = self._node.create_subscription(JointState, "/joint_states", _capture, 10)
    deadline = time.time() + 5.0
    while time.time() < deadline and not received.is_set():
      time.sleep(0.05)
    self._node.destroy_subscription(sub)

    if current_joints is None:
      raise RuntimeError("Could not read current joint positions.")

    delta = [
      target.x - self._pose.x,
      target.y - self._pose.y,
      target.z - self._pose.z,
      target.rx - self._pose.rx,
      target.ry - self._pose.ry,
      target.rz - self._pose.rz,
    ]
    goal_joints = [current_joints[i] + 0.15 * delta[i % 3] for i in range(6)]

    duration_s = max(1.0, 0.25 / max(speed_m_s, 0.01))
    traj = JointTrajectory()
    traj.joint_names = list(CANONICAL_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = goal_joints
    point.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
    traj.points = [point]

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory = traj

    send_future = self._trajectory_client.send_goal_async(goal_msg)
    goal_handle = self._wait_future(send_future, 30.0)
    if goal_handle is None or not goal_handle.accepted:
      raise RuntimeError("Trajectory goal rejected by controller.")

    result_future = goal_handle.get_result_async()
    self._wait_future(result_future, 30.0)

  def _lookup_base_link_pose_unlocked(
      self, child_link: str, timeout_s: float = 5.0
  ) -> TcpPose:
    """Live TF base_link → child (UR rotvec). Caller must hold self._lock."""
    from rclpy.duration import Duration
    import rclpy

    if self._tf_buffer is None or self._node is None:
      raise RuntimeError("TF buffer not ready — connect the robot first.")

    from hyperfusion_ur3e.moveit.scan_planner import quaternion_to_rotvec

    deadline = time.time() + timeout_s
    transform = None
    last_error: Optional[Exception] = None
    while time.time() < deadline and transform is None:
      try:
        transform = self._tf_buffer.lookup_transform(
            "base_link",
            child_link,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.1),
        )
      except Exception as exc:
        last_error = exc
        transform = None
        time.sleep(0.05)

    if transform is None:
      raise RuntimeError(
          f"Could not read base_link→{child_link} from TF: {last_error}"
      )

    t = transform.transform.translation
    q = transform.transform.rotation
    rx, ry, rz = quaternion_to_rotvec(
        float(q.x), float(q.y), float(q.z), float(q.w)
    )
    return TcpPose(float(t.x), float(t.y), float(t.z), rx, ry, rz)

  def _update_pose_from_tf_unlocked(self, timeout_s: float = 5.0) -> None:
    """Refresh self._pose as base_link-frame optical TCP (UR rotvec).

    Depth / capture JSON use robot base as origin. Capture optical tip is always
    BFS camera (cfg tool_tcp_*) composed as tool0⊗camera_tcp in the desktop app —
    even when this MoveIt tip link is remapped to dlp_tcp_* (scan_tcp=dlp).
    """
    self._pose = self._lookup_base_link_pose_unlocked("hyperfusion_tcp", timeout_s)

  def _finish_pose_fallback(self, delay_s: float) -> None:
    time.sleep(delay_s)
    with self._lock:
      self._moving = False
      if self._status.connected:
        self._status.driver_state = "idle"

  def stop(self) -> Dict[str, Any]:
    with self._lock:
      self._stop_unlocked()
      return {"ok": True, **self.status().to_dict()}

  def _stop_unlocked(self) -> None:
    self._stop_requested.set()
    self._moving = False
    goal_handle = self._active_goal_handle
    self._active_goal_handle = None
    if goal_handle is not None:
      try:
        cancel_future = goal_handle.cancel_goal_async()
        self._wait_future(cancel_future, 2.0)
      except Exception:
        pass
    try:
      from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner
      get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type).cancel_active_move()
    except Exception:
      pass
    if self._status.connected:
      self._status.driver_state = "idle"

  def _shutdown_bridge_unlocked(self) -> None:
    """Tear down bridge ROS node/spin only; leave ur_robot_driver running (prestart)."""
    self._spin_stop.set()
    if self._spin_thread is not None:
      self._spin_thread.join(timeout=3.0)
      self._spin_thread = None
    self._executor = None
    self._tf_listener = None
    self._tf_buffer = None
    self._trajectory_client = None
    self._joint_names = []
    self._latest_joint_positions = {}
    self._last_joint_state_rx_s = 0.0

    if self._node is not None:
      try:
        self._node.destroy_node()
      except Exception as exc:
        sys.stderr.write(f"UR3e bridge: destroy_node during bridge shutdown: {exc}\n")
      self._node = None

  def _shutdown_ros_unlocked(self) -> None:
    """Tear down bridge node and stop ur_robot_driver (full disconnect / failed connect)."""
    self._shutdown_bridge_unlocked()
    if self._driver is not None:
      self._driver.stop()
      self._driver = None

  def shutdown(self) -> None:
    with self._lock:
      self._shutdown_ros_unlocked()
      self._status.connected = False
      self._status.driver_state = "disconnected"
      self._connecting = False

  def plan_hemisphere_scan(self, body: Dict[str, Any]) -> Dict[str, Any]:
    """MoveIt IK + collision check for hemisphere scan TCP poses."""
    from hyperfusion_ur3e.moveit.scan_planner import (
        ScanPoseTarget,
        get_scan_planner,
        workspace_from_dict,
    )
    from hyperfusion_ur3e.driver.driver_manager import CANONICAL_JOINT_NAMES

    if not self._status.connected:
      raise RuntimeError("Robot not connected — start the UR driver before planning.")

    if not self._latest_joint_positions:
      raise RuntimeError("Joint states not available — wait for /joint_states after connect.")

    seed_joints = [
        float(self._latest_joint_positions.get(name, 0.0)) for name in CANONICAL_JOINT_NAMES
    ]

    poses = body.get("poses")
    if not isinstance(poses, list) or not poses:
      raise ValueError("poses must be a non-empty list.")

    workspace_cfg = body.get("workspace", {})
    workspace = workspace_from_dict(workspace_cfg if isinstance(workspace_cfg, dict) else None)

    targets: List[ScanPoseTarget] = []
    for entry in poses:
      if not isinstance(entry, dict):
        continue
      targets.append(
        ScanPoseTarget(
          index=int(entry.get("index", len(targets))),
          x_m=float(entry["x"]),
          y_m=float(entry["y"]),
          z_m=float(entry["z"]),
          rx=float(entry.get("rx", 0.0)),
          ry=float(entry.get("ry", 0.0)),
          rz=float(entry.get("rz", 0.0)),
          tool_z_x=float(entry.get("tool_z_x", 0.0)),
          tool_z_y=float(entry.get("tool_z_y", 0.0)),
          tool_z_z=float(entry.get("tool_z_z", -1.0)),
          camera_up_x=float(entry.get("camera_up_x", 0.0)),
          camera_up_y=float(entry.get("camera_up_y", 0.0)),
          camera_up_z=float(entry.get("camera_up_z", 1.0)),
          require_perpendicular=bool(entry.get("require_perpendicular", False)),
          theta_deg=float(entry.get("theta_deg", 0.0)),
          phi_deg=float(entry.get("phi_deg", 0.0)),
        )
      )

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    planner.apply_home_joints_from_body(body)
    tolerance_deg = float(body.get("pin_pose_tolerance_deg", 0.0) or 0.0)
    lock_camera_up = bool(body.get("scan_camera_up_world_z", True))
    semi_ring_sweep = bool(body.get("semi_ring_sweep", False))
    semi_max_sweep = int(body.get("semi_max_sweep_ok_per_ring", 2) or 2)
    # Candidate count per ring (legacy key was misnamed *_buffer_deg).
    semi_search_candidates = int(
        body.get(
            "semi_ring_search_candidates",
            body.get("semi_ring_search_buffer_deg", 360),
        )
        or 360
    )
    semi_backup_coverage_deg = float(body.get("semi_backup_coverage_deg", 300.0) or 0.0)
    results = planner.plan_poses(
      targets,
      workspace,
      initial_seed=seed_joints,
      pin_pose_tolerance_deg=tolerance_deg,
      lock_camera_up=lock_camera_up,
      semi_ring_sweep=semi_ring_sweep,
      semi_max_sweep_ok_per_ring=semi_max_sweep,
      semi_ring_search_candidates=semi_search_candidates,
      semi_backup_coverage_deg=semi_backup_coverage_deg,
    )

    payload_results = []
    reachable_count = 0
    unreachable_count = 0
    home_path_ok_count = 0
    chain_only_count = 0
    failure_summary: Dict[str, int] = {}
    for item in results:
      if item.reachable:
        reachable_count += 1
        if item.home_path_ok:
          home_path_ok_count += 1
        else:
          chain_only_count += 1
      else:
        unreachable_count += 1
        reason = item.error or "unknown"
        failure_summary[reason] = failure_summary.get(reason, 0) + 1
      entry = {
        "index": item.index,
        "reachable": item.reachable,
        "joints": item.joint_positions,
        "error": item.error or None,
        "cone_tip_deg": float(item.cone_tip_deg),
        "home_path_ok": bool(item.home_path_ok) if item.reachable else False,
      }
      if semi_ring_sweep:
        entry["base_sweep_ok"] = bool(item.base_sweep_ok) if item.reachable else False
        entry["backup_coverage_ok"] = (
          bool(item.backup_coverage_ok) if item.reachable else False
        )
        entry["backup_union_deg"] = (
          float(item.backup_union_deg) if item.reachable else 0.0
        )
        if item.reachable and item.pan_mask:
          entry["pan_mask"] = [bool(v) for v in item.pan_mask]
      if item.reachable and item.tcp_rx is not None:
        entry["tcp"] = {
          "x": item.tcp_x_m,
          "y": item.tcp_y_m,
          "z": item.tcp_z_m,
          "rx": item.tcp_rx,
          "ry": item.tcp_ry,
          "rz": item.tcp_rz,
          "tool_z_x": item.tool_z_x,
          "tool_z_y": item.tool_z_y,
          "tool_z_z": item.tool_z_z,
        }
      payload_results.append(entry)

    return {
      "ok": True,
      "results": payload_results,
      "reachable_count": reachable_count,
      "unreachable_count": unreachable_count,
      "home_path_ok_count": home_path_ok_count,
      "chain_only_count": chain_only_count,
      "failure_summary": failure_summary,
      "planner": "moveit",
    }

  def execute_scan_waypoint(self, body: Dict[str, Any]) -> Dict[str, Any]:
    """Execute one scan pin; wait for External Control if the pendant dropped it."""
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner, workspace_from_dict

    if not self._status.connected:
      raise RuntimeError("Robot not connected.")
    self._stop_requested.clear()

    joints = body.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
      raise ValueError("joints must be a list of 6 floats (radians).")

    workspace_cfg = body.get("workspace")
    workspace = workspace_from_dict(workspace_cfg) if isinstance(workspace_cfg, dict) else None

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    planner.apply_home_joints_from_body(body)
    tcp_target = body.get("tcp")
    require_home_first = bool(body.get("require_home_first", False))
    direct_only = bool(body.get("direct_only", False))
    tolerance_deg = float(body.get("pin_pose_tolerance_deg", 0.0) or 0.0)
    lock_camera_up = bool(body.get("scan_camera_up_world_z", True))
    allow_pin_pose_cone = bool(body.get("allow_pin_pose_cone", False))

    if not self._wait_for_external_control(
        timeout_s=120.0,
        stop_event=self._stop_requested,
        reason="before pin motion",
    ):
      return {
          "ok": False,
          "skipped": True,
          "error": (
              "External Control not connected — clear pendant protective stop / "
              "joint-limit popup, press Play on External Control, then retry."
          ),
      }

    kwargs = dict(
        workspace=workspace,
        tcp_target=tcp_target if isinstance(tcp_target, dict) else None,
        stop_event=self._stop_requested,
        require_home_first=require_home_first,
        direct_only=direct_only,
        pin_pose_tolerance_deg=tolerance_deg,
        lock_camera_up=lock_camera_up,
        allow_pin_pose_cone=allow_pin_pose_cone,
    )
    result = planner.execute_single_waypoint([float(v) for v in joints], **kwargs)
    if result.get("ok") or result.get("stopped"):
      return result

    err = str(result.get("error") or "")
    if not self._external_control_error(err) and self._external_control_reverse_connected():
      return result

    # Pendant often kills EC on "close to joint limit" — wait for Play, retry once.
    if not self._wait_for_external_control(
        timeout_s=120.0,
        stop_event=self._stop_requested,
        reason=f"after pin failure: {err or 'unknown'}",
    ):
      out = dict(result)
      out["error"] = (
          f"{err}; External Control not restored — clear pendant warning, "
          "press Play, then continue the scan."
      ).strip("; ")
      return out

    sys.stderr.write("UR3e bridge: retrying pin after External Control restore…\n")
    sys.stderr.flush()
    retry = planner.execute_single_waypoint([float(v) for v in joints], **kwargs)
    if retry.get("ok"):
      retry = dict(retry)
      retry["reconnected_external_control"] = True
    return retry

  def preview_manual_target(self, body: Dict[str, Any]) -> Dict[str, Any]:
    """Push UI joint target + workspace boundary into MoveIt/RViz."""
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner, workspace_from_dict

    if not self._status.connected:
      raise RuntimeError("Robot not connected.")

    joints = body.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
      raise ValueError("joints must be a list of 6 floats (radians).")

    workspace_cfg = body.get("workspace")
    workspace = workspace_from_dict(workspace_cfg) if isinstance(workspace_cfg, dict) else None

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    return planner.update_manual_target_preview(
      [float(v) for v in joints],
      workspace=workspace,
    )

  def execute_move_home(self, body: Dict[str, Any]) -> Dict[str, Any]:
    """Collision-aware MoveIt motion to the configured scan home pose."""
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner, workspace_from_dict

    if not self._status.connected:
      raise RuntimeError("Robot not connected.")
    self._stop_requested.clear()

    workspace_cfg = body.get("workspace")
    workspace = workspace_from_dict(workspace_cfg) if isinstance(workspace_cfg, dict) else None

    # Post-scan / post-stop retreat: do not abort when a racing Stop re-sets the latch.
    ignore_stop = bool(body.get("ignore_stop", False) or body.get("post_scan_home", False))
    stop_event = None if ignore_stop else self._stop_requested

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    planner.apply_home_joints_from_body(body)

    last: Dict[str, Any] = {"ok": False, "error": "could not move to home"}
    attempts = 3 if ignore_stop else 1
    for attempt in range(1, attempts + 1):
      self._stop_requested.clear()
      last = planner.execute_move_home(
        workspace=workspace,
        stop_event=stop_event,
      )
      if last.get("ok"):
        return last
      if attempt < attempts:
        time.sleep(0.75)
    return last

  def maybe_rewind_wrist3_cable(self, body: Dict[str, Any]) -> Dict[str, Any]:
    """Between scan pins: unwind wrist_3 at home if ≥½ turn from home ref."""
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner, workspace_from_dict

    if not self._status.connected:
      raise RuntimeError("Robot not connected.")
    self._stop_requested.clear()

    workspace_cfg = body.get("workspace")
    workspace = workspace_from_dict(workspace_cfg) if isinstance(workspace_cfg, dict) else None

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    planner.apply_home_joints_from_body(body)
    return planner.maybe_rewind_wrist3_cable(
      workspace=workspace,
      stop_event=self._stop_requested,
    )

  def execute_hardware_joint_move(self, body: Dict[str, Any]) -> Dict[str, Any]:
    """Direct RTDE/controller joint move (no MoveIt plan). Used by semi-fixed pan spin."""
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner, workspace_from_dict

    if not self._status.connected:
      raise RuntimeError("Robot not connected.")
    self._stop_requested.clear()

    joints = body.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
      raise ValueError("joints must be a list of 6 floats (radians).")

    workspace_cfg = body.get("workspace")
    workspace = workspace_from_dict(workspace_cfg) if isinstance(workspace_cfg, dict) else None

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    planner.apply_home_joints_from_body(body)
    label = str(body.get("label") or "hardware joint move")
    return planner.execute_hardware_joint_move(
      [float(v) for v in joints],
      workspace=workspace,
      stop_event=self._stop_requested,
      skip_collision_check=bool(body.get("skip_collision_check", False)),
      label=label,
    )

  def execute_hemisphere_scan(self, body: Dict[str, Any]) -> Dict[str, Any]:
    """Collision-aware MoveIt execution for planned joint waypoints."""
    from hyperfusion_ur3e.moveit.scan_planner import get_scan_planner, workspace_from_dict

    if not self._status.connected:
      raise RuntimeError("Robot not connected.")
    self._stop_requested.clear()

    waypoints_raw = body.get("waypoints")
    if not isinstance(waypoints_raw, list) or not waypoints_raw:
      raise ValueError("waypoints must be a non-empty list.")

    waypoints: List[List[float]] = []
    for entry in waypoints_raw:
      joints = entry.get("joints") if isinstance(entry, dict) else entry
      if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError("Each waypoint must include 6 joint positions (radians).")
      waypoints.append([float(v) for v in joints])

    workspace_cfg = body.get("workspace")
    workspace = workspace_from_dict(workspace_cfg) if isinstance(workspace_cfg, dict) else None

    planner = get_scan_planner(ros_distro=self.ros_distro, ur_type=self.ur_type)
    return planner.execute_waypoints(
      waypoints,
      workspace=workspace,
      stop_event=self._stop_requested,
    )
