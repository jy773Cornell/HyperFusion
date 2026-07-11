#!/usr/bin/env bash
# Quick check: does live /robot_state_publisher robot_description include tool payload?
set -eo pipefail
ROS_DISTRO="${1:-jazzy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1

echo "=== nodes ==="
ros2 node list 2>/dev/null | grep -i robot_state || true

echo "=== param (grep link) ==="
timeout 15 ros2 param get /robot_state_publisher robot_description 2>/dev/null \
  | grep -o 'hyperfusion_tool_payload' | head -3 || echo "(no link in param output)"

echo "=== topic (grep link) ==="
timeout 15 ros2 topic echo /robot_description --once 2>/dev/null \
  | grep -o 'hyperfusion_tool_payload' | head -3 || echo "(no link in topic output)"

echo "=== xacro preflight ==="
REPO="${HYPERFUSION_UR3E_REPO:-/mnt/d/Pototypy/HyperFusion/resources/ur3e}"
xacro "${REPO}/urdf/hyperfusion_ur3e.urdf.xacro" \
  ur_type:=ur3e name:=ur3e use_mock_hardware:=false ceiling_mount:=true \
  tool_payload_enabled:=true tool_payload_shape:=hemisphere tool_payload_radius_m:=0.10 \
  "tool_payload_mesh_dir:=${REPO}/urdf/meshes/" 2>/dev/null \
  | grep -c hyperfusion_tool_payload || echo 0
