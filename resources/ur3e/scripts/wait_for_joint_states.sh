#!/usr/bin/env bash
# Waits until /joint_states is publishing (required for MoveIt execution).
# Note: omit bash -u (nounset) — ROS setup.bash references unset vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

ROS_DISTRO="${1:-jazzy}"
TIMEOUT_S="${2:-120}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "UR3e MoveIt: waiting for /joint_states (timeout ${TIMEOUT_S}s)…" >&2

deadline=$((SECONDS + TIMEOUT_S))
while (( SECONDS < deadline )); do
  if ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --timeout 2 >/dev/null 2>&1; then
    echo "UR3e MoveIt: /joint_states is live." >&2
    # Best-effort controller check — must not block launch (list_controllers can hang).
    timeout 3 ros2 control list_controllers 2>/dev/null \
      | grep -E "joint_state_broadcaster|scaled_joint_trajectory_controller" >&2 || true
    exit 0
  fi
  sleep 1
done

echo "UR3e MoveIt: ERROR — /joint_states not publishing after ${TIMEOUT_S}s." >&2
echo "  Connect the robot in HyperFusion first (sidecar runs joint_states_stamper) and confirm:" >&2
echo "    ros2 topic echo /joint_states --once" >&2
echo "    ros2 topic echo /joint_state_broadcaster/joint_states --once" >&2
echo "    ros2 control list_controllers" >&2
exit 1
