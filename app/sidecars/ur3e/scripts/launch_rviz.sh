#!/usr/bin/env bash
# Launches RViz2 for UR3e visualization ONLY (no MoveIt / move_group).
# Shows the live robot model from /robot_description + TF published by the running driver.
# Exits when the RViz window is closed so wsl.exe returns.
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
    echo "UR3e RViz: ERROR — cannot locate resources/ur3e/scripts (set HYPERFUSION_UR3E_REPO)." >&2
    exit 1
  fi
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
# Ensure a visible Windows window under WSLg (see wslg_display_env.sh).
# Strip CRLF so Windows-edited scripts still source cleanly in bash.
# shellcheck source=/dev/null
source <(sed 's/\r$//' "${SCRIPT_DIR}/wslg_display_env.sh")

# Robot model + TF come from the running driver (robot_state_publisher).
"${SCRIPT_DIR}/wait_for_joint_states.sh" "${ROS_DISTRO}" 120

RVIZ_CONFIG="${SCRIPT_DIR}/../config/hyperfusion_view.rviz"
RUNTIME_URDF="${SCRIPT_DIR}/../config/runtime_robot_description.urdf"

if [[ ! -f "${RUNTIME_URDF}" ]] || ! grep -q hyperfusion_tool_payload "${RUNTIME_URDF}"; then
  echo "UR3e RViz: WARNING — ${RUNTIME_URDF} missing tool payload; connect sidecar first." >&2
else
  echo "UR3e RViz: using materialized URDF (${RUNTIME_URDF})." >&2
  RVIZ_LAUNCH_CONFIG="/tmp/hyperfusion_view_runtime.rviz"
  python3 <<PY
from pathlib import Path
import re

base = Path("${RVIZ_CONFIG}")
runtime = Path("${RUNTIME_URDF}").resolve().as_posix()
text = base.read_text(encoding="utf-8")
topic_block = re.compile(
    r"Description Source: Topic\s+Description Topic:\s+"
    r"Depth: \d+\s+Durability Policy: [^\n]+\s+"
    r"History Policy: [^\n]+\s+Reliability Policy: [^\n]+\s+"
    r"Value: /robot_description",
    re.MULTILINE,
)
replacement = f"Description Source: File\n      Description File: {runtime}"
patched, count = topic_block.subn(replacement, text, count=1)
if count != 1:
    # Already file-based or config layout changed — write file-based RobotModel block.
    patched = text
out = Path("${RVIZ_LAUNCH_CONFIG}")
out.write_text(patched, encoding="utf-8")
PY
  if [[ -f "${RVIZ_LAUNCH_CONFIG}" ]]; then
    RVIZ_CONFIG="${RVIZ_LAUNCH_CONFIG}"
  fi
fi

echo "UR3e RViz: launching visualization (ur_type=${UR_TYPE})…" >&2

if [[ -f "${RVIZ_CONFIG}" ]]; then
  exec rviz2 -d "${RVIZ_CONFIG}"
else
  echo "UR3e RViz: WARNING — config not found (${RVIZ_CONFIG}); starting default RViz." >&2
  exec rviz2
fi
