#!/usr/bin/env bash
# Headless move_group for scripted scan planning (no RViz).
set -eo pipefail

ROS_DISTRO="${1:-jazzy}"
UR_TYPE="${2:-ur3e}"

if [[ -n "${HYPERFUSION_UR3E_REPO:-}" ]]; then
  SCRIPT_DIR="${HYPERFUSION_UR3E_REPO}/scripts"
elif [[ -n "${BASH_SOURCE[0]:-}" && "${BASH_SOURCE[0]}" != "-" && -f "${BASH_SOURCE[0]}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  SCRIPT_DIR=""
  for candidate in /mnt/*/Pototypy/HyperFusion/app/sidecars/ur3e/scripts \
                   /mnt/*/HyperFusion/app/sidecars/ur3e/scripts; do
    if [[ -f "${candidate}/wait_for_joint_states.sh" ]]; then
      SCRIPT_DIR="${candidate}"
      break
    fi
  done
  if [[ -z "${SCRIPT_DIR}" ]]; then
    echo "UR3e MoveIt headless: ERROR — cannot locate app/sidecars/ur3e/scripts." >&2
    exit 1
  fi
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_LOCALHOST_ONLY=1
export HYPERFUSION_UR3E_SERVER_PORT="${HYPERFUSION_UR3E_SERVER_PORT:-8766}"

"${SCRIPT_DIR}/wait_for_joint_states.sh" "${ROS_DISTRO}" 120

MOVEIT_LAUNCH="${SCRIPT_DIR}/../launch/hyperfusion_moveit.launch.py"
exec ros2 launch "${MOVEIT_LAUNCH}" "ur_type:=${UR_TYPE}" "launch_rviz:=false"
