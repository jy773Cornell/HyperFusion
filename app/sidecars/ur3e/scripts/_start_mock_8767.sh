#!/usr/bin/env bash
# Mock planner on 8767 for batch semi-ring search. Do not use 8766. No kill_stale.
set -eo pipefail
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e
# ROS setup.bash is not `set -u` safe.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONUNBUFFERED=1
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export HYPERFUSION_UR3E_SERVER_PORT=8767
exec ./venv/bin/python -m hyperfusion_ur3e.sidecar.server \
  --host 0.0.0.0 --port 8767 \
  --robot-ip 192.168.1.10 \
  --reverse-ip 192.168.1.20 \
  --dashboard-port 29999 \
  --rtde-port 30004 \
  --max-linear-speed 0.05 \
  --max-linear-accel 0.3 \
  --max-joint-velocity-deg 40 \
  --tool-payload-radius-mm 80 \
  --tool-payload-shape mesh \
  --tool-payload-mesh-file ur_tool_payload.stl \
  --tool-tcp-x-mm 0.715 \
  --tool-tcp-y-mm -54.197 \
  --tool-tcp-z-mm 73.755 \
  --tool-tcp-roll-deg -1.9138 \
  --tool-tcp-pitch-deg 0.7450 \
  --tool-tcp-yaw-deg 0.2868 \
  --use-mock-hardware \
  --ros-distro jazzy \
  --ur-type ur3e \
  --prestart-driver \
  --initial-joint-deg 90,-180,145,-55,90,-90 \
  --ceiling-mount-height-mm 629 \
  --mount-roll-deg 180 \
  --mount-pitch-deg 0 \
  --mount-yaw-deg 0 \
  --mount-offset-x-mm 0 \
  --mount-offset-y-mm 0 \
  --workspace-boundary-enabled \
  --workspace-length-mm 900 \
  --workspace-width-mm 600 \
  --workspace-height-mm 529 \
  --workspace-ceiling-clearance-mm 40
