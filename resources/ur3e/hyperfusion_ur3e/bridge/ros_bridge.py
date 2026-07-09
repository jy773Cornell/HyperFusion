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
      use_mock_hardware: bool = True,
  ) -> None:
    self.robot_ip = robot_ip
    self.dashboard_port = dashboard_port
    self.rtde_port = rtde_port
    self.max_linear_speed_m_per_s = max_linear_speed_m_per_s
    self.max_linear_accel_m_per_s2 = max_linear_accel_m_per_s2
    self.ros_distro = ros_distro
    self.ur_type = ur_type
    self.use_mock_hardware = use_mock_hardware

    self._lock = threading.RLock()
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
    with self._lock:
      sys.stderr.write("UR3e bridge: warming ur_robot_driver subprocess…\n")
      self._ensure_driver_running_unlocked()
      return {"ok": True, "driver": "warming"}

  def connect(self) -> Dict[str, Any]:
    with self._lock:
      if self._connecting:
        raise RuntimeError("Connect already in progress.")
      if self._status.connected:
        return {"ok": True, "already_connected": True, **self.status().to_dict()}

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

    proc = subprocess.run(
      ["pgrep", "-f", f"ur_control.launch.py.*robot_ip:={self.robot_ip}"],
      capture_output=True,
      text=True,
    )
    return proc.returncode == 0

  def _joint_states_publishing(self, timeout_s: float = 5.0) -> bool:
    import subprocess

    cmd = (
      f"source /opt/ros/{self.ros_distro}/setup.bash && "
      "ros2 topic echo /joint_states sensor_msgs/msg/JointState "
      f"--once --timeout {max(1, int(timeout_s))}"
    )
    try:
      proc = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=timeout_s + 15.0,
      )
    except subprocess.TimeoutExpired:
      return False
    return proc.returncode == 0

  def _ensure_driver_running_unlocked(self) -> None:
    if self._driver is not None and self._driver.running:
      sys.stderr.write("UR3e bridge: reusing active driver subprocess\n")
    elif self._external_driver_running():
      if self._joint_states_publishing(timeout_s=8.0):
        sys.stderr.write("UR3e bridge: attaching to existing ur_control launch\n")
        self._driver = None
      else:
        sys.stderr.write(
            "UR3e bridge: stale ur_control launch (no /joint_states) — restarting driver…\n"
        )
        Ur3eRosDriverManager.stop_stale_launches()
        self._driver = Ur3eRosDriverManager(
          ros_distro=self.ros_distro,
          ur_type=self.ur_type,
          robot_ip=self.robot_ip,
          use_mock_hardware=self.use_mock_hardware,
        )
        self._driver.start()
    else:
      sys.stderr.write("UR3e bridge: starting ur_robot_driver (may take ~2 min)…\n")
      Ur3eRosDriverManager.stop_stale_launches()
      self._driver = Ur3eRosDriverManager(
        ros_distro=self.ros_distro,
        ur_type=self.ur_type,
        robot_ip=self.robot_ip,
        use_mock_hardware=self.use_mock_hardware,
      )
      self._driver.start()

    Ur3eRosDriverManager.ensure_joint_states_stamper_for_distro(self.ros_distro)

  def _connect_ros_unlocked(self) -> None:
    last_exc: Optional[Exception] = None
    for attempt in range(2):
      try:
        self._connect_ros_unlocked_once(force_fresh_driver=attempt > 0)
        return
      except RuntimeError as exc:
        last_exc = exc
        if attempt == 0 and "joint_states" in str(exc):
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
    if force_fresh_driver:
      Ur3eRosDriverManager.stop_stale_launches()
      if self._driver is not None:
        self._driver.stop()
        self._driver = None
      self._driver = Ur3eRosDriverManager(
        ros_distro=self.ros_distro,
        ur_type=self.ur_type,
        robot_ip=self.robot_ip,
        use_mock_hardware=self.use_mock_hardware,
      )
      self._driver.start()
    else:
      self._ensure_driver_running_unlocked()

    if self._node is not None:
      return

    import os
    import rclpy
    from rclpy.node import Node
    from rclpy.duration import Duration
    from tf2_ros import Buffer, TransformListener
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient

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
      "/scaled_joint_trajectory_controller/follow_joint_trajectory",
    )

    deadline = time.time() + 90.0
    while time.time() < deadline:
      if self._trajectory_client.server_is_ready():
        sys.stderr.write("UR3e bridge: trajectory action server ready\n")
        break
      time.sleep(0.2)
    else:
      self._trajectory_client = None
      self._status.fault = "Trajectory action server not ready; using pose fallback."

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

    self._node.create_subscription(JointState, "/joint_states", _on_joint_state, 10)
    deadline = time.time() + 90.0
    while time.time() < deadline and not joint_event.is_set():
      time.sleep(0.1)

    if not self._joint_names:
      raise RuntimeError(
          "UR driver started but /joint_states is not publishing. "
          "In WSL run: ros2 control list_controllers && ros2 topic echo /joint_states --once"
      )

    try:
      self._update_pose_from_tf_unlocked()
    except Exception as exc:
      self._status.fault = f"{self._status.fault} TF unavailable ({exc}).".strip()

  def disconnect(self) -> Dict[str, Any]:
    with self._lock:
      if self._moving:
        self._stop_unlocked()
      self._shutdown_ros_unlocked()
      self._status.connected = False
      self._status.driver_state = "disconnected"
      return {"ok": True, **self.status().to_dict()}

  def get_pose(self) -> Dict[str, Any]:
    with self._lock:
      if not self._status.connected:
        raise RuntimeError("Robot not connected.")
      try:
        self._update_pose_from_tf_unlocked(timeout_s=1.0)
      except Exception as exc:
        self._status.fault = f"Using fallback pose; TF unavailable ({exc})."
      return {"ok": True, "pose": self._pose.as_list(), **self.status().to_dict()}

  def get_joints(self) -> Dict[str, Any]:
    with self._lock:
      if not self._status.connected:
        raise RuntimeError("Robot not connected.")
      names, positions = self._ordered_joint_state_unlocked()
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
    goal_handle = self._wait_future(send_future, 10.0)
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

    with self._lock:
      for index, name in enumerate(CANONICAL_JOINT_NAMES):
        self._latest_joint_positions[name] = target[index]
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
    goal_handle = self._wait_future(send_future, 10.0)
    if goal_handle is None or not goal_handle.accepted:
      raise RuntimeError("Trajectory goal rejected by controller.")

    result_future = goal_handle.get_result_async()
    self._wait_future(result_future, 30.0)

  def _update_pose_from_tf_unlocked(self, timeout_s: float = 5.0) -> None:
    from rclpy.duration import Duration
    import rclpy

    if self._tf_buffer is None or self._node is None:
      return

    deadline = time.time() + timeout_s
    transform = None
    while time.time() < deadline and transform is None:
      try:
        transform = self._tf_buffer.lookup_transform(
          "base_link",
          "tool0",
          rclpy.time.Time(),
          timeout=Duration(seconds=0.2),
        )
      except Exception:
        time.sleep(0.05)

    if transform is None:
      raise RuntimeError("Could not read tool0 pose from TF.")

    t = transform.transform.translation
    q = transform.transform.rotation
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    rx = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    ry = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    rz = math.atan2(siny_cosp, cosy_cosp)

    self._pose = TcpPose(t.x, t.y, t.z, rx, ry, rz)

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
    if self._status.connected:
      self._status.driver_state = "idle"

  def _shutdown_ros_unlocked(self) -> None:
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
      self._node.destroy_node()
      self._node = None
    if self._driver is not None:
      self._driver.stop()
      self._driver = None

  def shutdown(self) -> None:
    with self._lock:
      self._shutdown_ros_unlocked()
      self._status.connected = False
      self._status.driver_state = "disconnected"
      self._connecting = False
