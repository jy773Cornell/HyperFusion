#!/usr/bin/env bash
# Mock UR3e sidecar for batch semi-ring search (current ur_tool_payload.stl).
set -eo pipefail
cd "$(dirname "$0")/.."
export PYTHONUNBUFFERED=1
export ROS_LOCALHOST_ONLY=1
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
LOG="${1:-/mnt/d/Pototypy/HyperFusion/app/preset/mvs_scan_plans/semi/_mock_sidecar.log}"
mkdir -p "$(dirname "$LOG")"
exec ./venv/bin/ur3e_server \
  --host 0.0.0.0 --port 8766 \
  --robot-ip 192.168.1.10 --reverse-ip 192.168.1.20 \
  --dashboard-port 29999 --rtde-port 30004 \
  --max-linear-speed 0.05 --max-linear-accel 0.3 --max-joint-velocity-deg 40 \
  --tool-payload-radius-mm 80.0 --tool-payload-shape mesh \
  --tool-payload-mesh-file ur_tool_payload.stl \
  --tool-tcp-x-mm 0.715 --tool-tcp-y-mm -54.197 --tool-tcp-z-mm 73.755 \
  --tool-tcp-roll-deg -1.9138 --tool-tcp-pitch-deg 0.7450 --tool-tcp-yaw-deg 0.2868 \
  --use-mock-hardware --ros-distro jazzy --ur-type ur3e --prestart-driver \
  --initial-joint-deg 90,-180,145,-55,90,-90 \
  --ceiling-mount-height-mm 629.0 \
  --mount-roll-deg 180 --mount-pitch-deg 0 --mount-yaw-deg 0 \
  --mount-offset-x-mm 0 --mount-offset-y-mm 0 \
  --workspace-boundary-enabled \
  --workspace-length-mm 900 --workspace-width-mm 600 --workspace-height-mm 529 \
  --workspace-ceiling-clearance-mm 40 \
  >"$LOG" 2>&1
