#!/usr/bin/env bash
# Isolated mock 8768. Home wrist_3 = +90. Mesh bfs_dlp_payload.stl. No 8766. No kill_stale.
set -eo pipefail
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
export HYPERFUSION_SKIP_STALE_KILL=1
export HYPERFUSION_UR3E_SERVER_PORT=8768
export HYPERFUSION_UR3E_RUNTIME=/mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e/config/runtime_p8768
export HYPERFUSION_UR_REVERSE_PORT=50101
export HYPERFUSION_UR_SCRIPT_SENDER_PORT=50102
export HYPERFUSION_UR_TRAJECTORY_PORT=50103
export HYPERFUSION_UR_SCRIPT_COMMAND_PORT=50104
mkdir -p "$HYPERFUSION_UR3E_RUNTIME"
exec ./venv/bin/python -m hyperfusion_ur3e.sidecar.server \
  --host 0.0.0.0 --port 8768 \
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
  --tool-tcp-roll-deg 0 \
  --tool-tcp-pitch-deg 0 \
  --tool-tcp-yaw-deg 0 \
  --use-mock-hardware \
  --ros-distro jazzy \
  --ur-type ur3e \
  --prestart-driver \
  --initial-joint-deg 90,-180,145,-55,90,90 \
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
