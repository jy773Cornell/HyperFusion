#!/usr/bin/env bash
# Waits until /joint_states is publishing (required for RViz / MoveIt).
# 1) Sidecar HTTP /joints (fast, no ROS CLI)
# 2) rclpy probe (no ros2 daemon)
# Note: omit bash -u (nounset) — ROS setup.bash references unset vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

ROS_DISTRO="${1:-jazzy}"
TIMEOUT_S="${2:-120}"
SIDECAR_PORT="${HYPERFUSION_UR3E_SERVER_PORT:-8766}"

export ROS_LOCALHOST_ONLY=1
source "/opt/ros/${ROS_DISTRO}/setup.bash"

_sidecar_joints_live() {
  command -v curl >/dev/null 2>&1 || return 1
  curl -sf --max-time 3 "http://127.0.0.1:${SIDECAR_PORT}/joints" 2>/dev/null \
    | grep -q '"ok"[[:space:]]*:[[:space:]]*true'
}

_rclpy_probe() {
  local wait_s="${1:-6}"
  local probe=""
  if [[ -n "${HYPERFUSION_UR3E_REPO:-}" ]]; then
    probe="${HYPERFUSION_UR3E_REPO}/scripts/probe_joint_states.py"
  fi
  if [[ ! -f "${probe}" ]]; then
    return 1
  fi
  local py="${HYPERFUSION_UR3E_REPO}/venv/bin/python"
  if [[ ! -x "${py}" ]]; then
    py="python3"
  fi
  timeout "$(( wait_s + 2 ))" "${py}" "${probe}" "${wait_s}"
}

echo "UR3e MoveIt: waiting for /joint_states (timeout ${TIMEOUT_S}s)…" >&2

deadline=$((SECONDS + TIMEOUT_S))
while (( SECONDS < deadline )); do
  if _sidecar_joints_live; then
    echo "UR3e MoveIt: /joint_states is live (sidecar /joints)." >&2
    exit 0
  fi
  if _rclpy_probe 6; then
    echo "UR3e MoveIt: /joint_states is live (rclpy probe)." >&2
    exit 0
  fi
  elapsed=$((TIMEOUT_S - (deadline - SECONDS)))
  if (( elapsed > 0 && elapsed % 10 == 0 )); then
    echo "UR3e MoveIt: still waiting for /joint_states (${elapsed}s)…" >&2
  fi
  sleep 1
done

echo "UR3e MoveIt: ERROR — /joint_states not publishing after ${TIMEOUT_S}s." >&2
echo "  Connect the robot in HyperFusion first (sidecar runs joint_states_stamper) and confirm:" >&2
echo "    curl -s http://127.0.0.1:${SIDECAR_PORT}/joints" >&2
echo "    export ROS_LOCALHOST_ONLY=1 && ros2 topic hz /joint_states --window 3" >&2
exit 1
