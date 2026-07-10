#!/usr/bin/env bash
# Launches MoveIt + RViz and exits when the RViz window is closed.
# Note: omit bash -u (nounset) — ROS setup.bash references unset vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

ROS_DISTRO="${1:-jazzy}"
UR_TYPE="${2:-ur3e}"

if [[ -n "${HYPERFUSION_UR3E_REPO:-}" ]]; then
  SCRIPT_DIR="${HYPERFUSION_UR3E_REPO}/scripts"
elif [[ -n "${BASH_SOURCE[0]:-}" && "${BASH_SOURCE[0]}" != "-" && -f "${BASH_SOURCE[0]}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  # Piped via `sed … | bash -s` — BASH_SOURCE is not the script path.
  SCRIPT_DIR=""
  for candidate in /mnt/*/Pototypy/HyperFusion/resources/ur3e/scripts \
                   /mnt/*/HyperFusion/resources/ur3e/scripts; do
    if [[ -f "${candidate}/wait_for_joint_states.sh" ]]; then
      SCRIPT_DIR="${candidate}"
      break
    fi
  done
  if [[ -z "${SCRIPT_DIR}" ]]; then
    echo "UR3e MoveIt: ERROR — cannot locate resources/ur3e/scripts (set HYPERFUSION_UR3E_REPO)." >&2
    exit 1
  fi
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_LOCALHOST_ONLY=1

"${SCRIPT_DIR}/wait_for_joint_states.sh" "${ROS_DISTRO}" 120

echo "UR3e MoveIt: waiting for /joint_states_stamped (timeout 30s)…" >&2
stamped_deadline=$((SECONDS + 30))
PROBE="${HYPERFUSION_UR3E_REPO}/scripts/probe_joint_states.py"
PY="${HYPERFUSION_UR3E_REPO}/venv/bin/python"
while (( SECONDS < stamped_deadline )); do
  if [[ -x "${PY}" && -f "${PROBE}" ]] && timeout 8 "${PY}" "${PROBE}" 6; then
    echo "UR3e MoveIt: /joint_states_stamped is live." >&2
    break
  fi
  sleep 1
done

echo "UR3e MoveIt: stopping any existing move_group (prevents duplicate /move_action)…" >&2
pkill -f "moveit_ros_move_group/[m]ove_group" 2>/dev/null || true
sleep 1

echo "UR3e MoveIt: starting MoveIt + RViz (ur_type=${UR_TYPE})…" >&2

MOVEIT_LAUNCH="${SCRIPT_DIR}/../launch/hyperfusion_moveit.launch.py"
ros2 launch "${MOVEIT_LAUNCH}" "ur_type:=${UR_TYPE}" 2>&1 &
launch_pid=$!

cleanup() {
    if kill -0 "$launch_pid" 2>/dev/null; then
        kill "$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# Wait for RViz to appear (MoveIt can take a while on cold start).
for i in $(seq 1 180); do
    if pgrep -x rviz2 >/dev/null 2>&1; then
        echo "UR3e MoveIt: RViz window is up (MoveIt ready)." >&2
        break
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
        echo "UR3e MoveIt: ERROR — MoveIt launch exited before RViz started." >&2
        wait "$launch_pid"
        exit $?
    fi
    if (( i % 15 == 0 )); then
        echo "UR3e MoveIt: still waiting for RViz (${i}s)…" >&2
    fi
    sleep 1
done

if ! pgrep -x rviz2 >/dev/null 2>&1; then
    echo "UR3e MoveIt: ERROR — RViz did not start within 180s." >&2
    cleanup
    exit 1
fi

# When RViz exits, tear down the launch process so wsl.exe returns.
while kill -0 "$launch_pid" 2>/dev/null; do
    if ! pgrep -x rviz2 >/dev/null 2>&1; then
        cleanup
        exit 0
    fi
    sleep 1
done

wait "$launch_pid"
