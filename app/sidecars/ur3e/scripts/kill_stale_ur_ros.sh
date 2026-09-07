#!/usr/bin/bash
# Kill leftover UR driver / ROS 2 processes from partial launches (HyperFusion WSL).
# Usage: kill_stale_ur_ros.sh [--keep-sidecar]
set -e

KEEP_SIDECAR=0
if [[ "${1:-}" == "--keep-sidecar" ]]; then
  KEEP_SIDECAR=1
fi

export ROS_LOCALHOST_ONLY=1
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +e
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  timeout 5 ros2 daemon stop 2>/dev/null || true
  set -e
fi

patterns=(
  "ur_control.launch.py"
  "hyperfusion_ur_control.launch.py"
  "hyperfusion_ur_rsp.launch.py"
  "venv/bin/joint_states_stamper"
  "/controller_manager/ros2_control_node"
  "/robot_state_publisher/robot_state_publisher"
  "/ur_robot_driver/dashboard_client"
  "/ur_robot_driver/controller_stopper_node"
  "/ur_robot_driver/urscript_interface"
  "/ur_robot_driver/trajectory_until_node"
  "/controller_manager/spawner"
  "move_group"
  "hyperfusion_ur3e_moveit"
)

if [[ "${KEEP_SIDECAR}" -eq 0 ]]; then
  patterns+=("ur3e_server" "hyperfusion_ur3e.sidecar.server")
fi

for pattern in "${patterns[@]}"; do
  pkill -f "${pattern}" 2>/dev/null || true
done

sleep 2
