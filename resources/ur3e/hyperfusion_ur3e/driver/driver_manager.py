"""Launch and monitor the UR ROS 2 driver (ur_robot_driver) subprocess."""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from typing import Optional

from hyperfusion_ur3e import PKG_ROOT

HW_JOINT_STATES_TOPIC = "/joint_states"
STAMPER_BIN = PKG_ROOT / "venv" / "bin" / "joint_states_stamper"

STALE_PROCESS_PATTERNS = [
    "ur_control.launch.py",
    "venv/bin/joint_states_stamper",
    "/controller_manager/ros2_control_node",
    "/robot_state_publisher/robot_state_publisher",
    "/ur_robot_driver/trajectory_until_node",
    "/controller_manager/spawner",
]


class Ur3eRosDriverManager:
    """Starts ur_control.launch.py in a subprocess."""

    def __init__(
        self,
        *,
        ros_distro: str = "jazzy",
        ur_type: str = "ur3e",
        robot_ip: str = "192.168.0.10",
        use_mock_hardware: bool = True,
    ) -> None:
        self.ros_distro = ros_distro
        self.ur_type = ur_type
        self.robot_ip = robot_ip
        self.use_mock_hardware = use_mock_hardware
        self._process: Optional[subprocess.Popen] = None
        self._stamper_process: Optional[subprocess.Popen] = None

    @property
    def running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def start(self, min_settle_s: float = 8.0, fail_fast_s: float = 10.0) -> None:
        if self.running:
            self._verify_controller_manager()
            self.ensure_joint_states_stamper()
            return

        mock_flag = "true" if self.use_mock_hardware else "false"
        launch_cmd = (
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            "ros2 launch ur_robot_driver ur_control.launch.py "
            f"ur_type:={self.ur_type} "
            f"robot_ip:={self.robot_ip} "
            f"use_mock_hardware:={mock_flag} "
            "headless_mode:=true "
            "launch_rviz:=false "
            "controller_spawner_timeout:=120"
        )

        self._process = subprocess.Popen(
            ["bash", "-lc", launch_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

        self._wait_for_process_start(fail_fast_s)
        self._wait_for_settle(min_settle_s)
        self._verify_controller_manager()
        self.ensure_joint_states_stamper()

    def ensure_joint_states_stamper(self) -> None:
        """Start hardware joint_states -> /joint_states republisher for MoveIt/RViz."""
        if self._stamper_process is not None and self._stamper_process.poll() is None:
            return
        if self._stamper_running():
            return

        if not STAMPER_BIN.is_file():
            raise RuntimeError(
                f"joint_states_stamper not installed at {STAMPER_BIN}. "
                "Run ./install_env.sh in resources/ur3e."
            )

        stamper_cmd = (
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            f"exec {STAMPER_BIN.as_posix()}"
        )
        self._stamper_process = subprocess.Popen(
            ["bash", "-lc", stamper_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )
        time.sleep(0.5)
        if self._stamper_process.poll() is not None:
            stderr_tail = self._read_stderr_tail(self._stamper_process)
            self._stamper_process = None
            detail = "joint_states_stamper exited during startup."
            if stderr_tail:
                detail = f"{detail}\n{stderr_tail}"
            raise RuntimeError(detail)

        sys.stderr.write(
            f"UR3e driver: started joint_states_stamper ({HW_JOINT_STATES_TOPIC} -> /joint_states_stamped)\n"
        )

    @staticmethod
    def ensure_joint_states_stamper_for_distro(ros_distro: str) -> None:
        """Ensure stamper is running when attaching to an external ur_control launch."""
        helper = Ur3eRosDriverManager(ros_distro=ros_distro)
        helper.ensure_joint_states_stamper()

    def stop(self) -> None:
        self.stop_joint_states_stamper()
        if self._process is None:
            return
        if self._process.poll() is None:
            try:
                os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._process = None

    def stop_joint_states_stamper(self) -> None:
        if self._stamper_process is not None and self._stamper_process.poll() is None:
            try:
                os.killpg(os.getpgid(self._stamper_process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._stamper_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self._stamper_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._stamper_process = None

    @staticmethod
    def _stamper_running() -> bool:
        proc = subprocess.run(
            ["pgrep", "-f", "venv/bin/joint_states_stamper"],
            capture_output=True,
            text=True,
        )
        return proc.returncode == 0

    @staticmethod
    def stop_stale_launches() -> None:
        """Terminate leftover UR ROS processes from prior partial launches."""
        try:
            for pattern in STALE_PROCESS_PATTERNS:
                subprocess.run(
                    ["pkill", "-f", pattern],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            time.sleep(1.0)
        except Exception:
            pass

    def _wait_for_process_start(self, fail_fast_s: float) -> None:
        fail_deadline = time.time() + fail_fast_s
        while time.time() < fail_deadline:
            if self._process is not None and self._process.poll() is not None:
                detail = f"UR driver exited during startup (code={self._process.returncode})."
                stderr_tail = self._read_stderr_tail(self._process)
                if stderr_tail:
                    detail = f"{detail}\n{stderr_tail}"
                raise RuntimeError(detail)
            time.sleep(0.25)

    def _wait_for_settle(self, min_settle_s: float) -> None:
        settle_deadline = time.time() + min_settle_s
        while time.time() < settle_deadline:
            if self._process is not None and self._process.poll() is not None:
                raise RuntimeError(
                    f"UR driver exited during startup (code={self._process.returncode})."
                )
            time.sleep(0.25)

    def _verify_controller_manager(self) -> None:
        proc = subprocess.run(
            [
                "bash",
                "-lc",
                f"source /opt/ros/{self.ros_distro}/setup.bash && timeout 30 ros2 control list_controllers",
            ],
            capture_output=True,
            text=True,
        )
        if proc.returncode == 0:
            return

        detail = (proc.stderr or proc.stdout or "").strip()
        if "undefined symbol" in detail:
            detail = (
                f"{detail}\n"
                "ROS 2 Jazzy packages are ABI-mismatched. Run: "
                "sudo apt update && sudo apt install --only-upgrade "
                "ros-jazzy-controller-manager ros-jazzy-controller-manager-msgs "
                "ros-jazzy-diagnostic-updater ros-jazzy-force-torque-sensor-broadcaster "
                "ros-jazzy-hardware-interface ros-jazzy-joint-state-broadcaster "
                "ros-jazzy-joint-trajectory-controller ros-jazzy-ros2-control ros-jazzy-ros2-controllers"
            )
        raise RuntimeError(f"UR controller manager is not available.\n{detail[-2000:]}")

    @staticmethod
    def _read_stderr_tail(process: subprocess.Popen) -> str:
        if process.stderr is None:
            return ""
        try:
            return process.stderr.read().decode("utf-8", errors="replace").strip()[-2000:]
        except Exception:
            return ""
