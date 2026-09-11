#!/usr/bin/env bash
# Isolated mock planner. Args: PORT ROS_DOMAIN_ID UR_REVERSE_PORT
# Does not call kill_stale. Safe to run several at once after a clean stop.
set -eo pipefail
PORT="${1:?port}"
DOMAIN="${2:?ros-domain}"
UR_REVERSE="${3:?ur-reverse-port}"
INIT_DEG="${4:-90,-180,145,-55,90,-90}"
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID="${DOMAIN}"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
export HYPERFUSION_SKIP_STALE_KILL=1
export HYPERFUSION_UR3E_SERVER_PORT="${PORT}"
export HYPERFUSION_UR3E_RUNTIME="/mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e/config/runtime_p${PORT}"
export HYPERFUSION_UR_REVERSE_PORT="${UR_REVERSE}"
export HYPERFUSION_UR_SCRIPT_SENDER_PORT="$((UR_REVERSE + 1))"
export HYPERFUSION_UR_TRAJECTORY_PORT="$((UR_REVERSE + 2))"
export HYPERFUSION_UR_SCRIPT_COMMAND_PORT="$((UR_REVERSE + 3))"
mkdir -p "${HYPERFUSION_UR3E_RUNTIME}"
exec ./venv/bin/python -m hyperfusion_ur3e.sidecar.server \
  --host 0.0.0.0 --port "${PORT}" \
  --robot-ip 192.168.1.10 \
  --reverse-ip 192.168.1.20 \
  --dashboard-port 29999 \
  --rtde-port 30004 \
  --max-linear-speed 0.05 \
  --max-linear-accel 0.3 \
  --max-joint-velocity-deg 40 \
  --tool-payload-radius-mm 60 \
  --tool-payload-shape mesh \
  --tool-payload-mesh-file bfs_dlp_payload.stl \
  --tool-tcp-x-mm 0.372 \
  --tool-tcp-y-mm 57.104 \
  --tool-tcp-z-mm 27.4994 \
  --tool-tcp-roll-deg 25 \
  --tool-tcp-pitch-deg 0 \
  --tool-tcp-yaw-deg 0 \
  --use-mock-hardware \
  --ros-distro jazzy \
  --ur-type ur3e \
  --prestart-driver \
  --initial-joint-deg "${INIT_DEG}" \
  --ceiling-mount-height-mm 870 \
  --mount-roll-deg 180 \
  --mount-pitch-deg 0 \
  --mount-yaw-deg 0 \
  --mount-offset-x-mm 0 \
  --mount-offset-y-mm 0 \
  --workspace-boundary-enabled \
  --workspace-length-mm 900 \
  --workspace-width-mm 600 \
  --workspace-height-mm 720 \
  --workspace-ceiling-clearance-mm 40
