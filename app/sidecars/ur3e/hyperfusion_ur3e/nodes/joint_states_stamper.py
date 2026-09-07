#!/usr/bin/env python3
"""
Republish joint states with valid ROS timestamps for RViz and MoveIt.

Mock UR hardware can publish JointState messages with header.stamp = 0.
robot_state_publisher ignores those for TF, so RViz shows only the base link.
This node reads the broadcaster local topic and republishes stamped states to
/joint_states (RViz + robot_state_publisher + move_group) and /joint_states_stamped.

Positions are normalized into MoveIt / ur_description joint branches (e.g. wrist_2
-270° → 90°). Raw hardware angles remain on /joint_state_broadcaster/joint_states
for trajectory execute anchoring in scan_planner.
"""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from hyperfusion_ur3e.joint_angles import (
    CANONICAL_JOINT_NAMES,
    MOVEIT_JOINT_LIMITS_RAD,
    home_joints_deg_from_env,
    home_joints_rad_by_name,
    joint_delta_rad,
    stabilize_joint_reading,
    wrap_joint_for_stream,
)

# ros2_control joint_state_broadcaster with use_local_topics:=true (HyperFusion config).
INPUT_TOPIC = "/joint_state_broadcaster/joint_states"
OUTPUT_JOINT_STATES_TOPIC = "/joint_states"
OUTPUT_STAMPED_TOPIC = "/joint_states_stamped"


class JointStatesStamper(Node):
  def __init__(self) -> None:
    super().__init__("hyperfusion_joint_states_stamper")
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    self._pub_joint_states = self.create_publisher(JointState, OUTPUT_JOINT_STATES_TOPIC, sensor_qos)
    self._pub_stamped = self.create_publisher(JointState, OUTPUT_STAMPED_TOPIC, sensor_qos)
    self._sub = self.create_subscription(JointState, INPUT_TOPIC, self._on_joint_state, sensor_qos)
    self._last_wrapped: dict[str, float] = {}
    self._home_ref_rad = home_joints_rad_by_name(home_joints_deg_from_env() or [])
    if self._home_ref_rad:
      home_deg = home_joints_deg_from_env() or []
      self.get_logger().info(
          f"Branch reference from HYPERFUSION_HOME_JOINTS_DEG: {home_deg}"
      )
    self.get_logger().info(
        f"Republishing {INPUT_TOPIC} -> {OUTPUT_JOINT_STATES_TOPIC}, {OUTPUT_STAMPED_TOPIC}"
    )

  def _branch_reference(self, name: str, raw: float) -> float:
    if name in self._last_wrapped:
      return self._last_wrapped[name]
    if name in self._home_ref_rad:
      return self._home_ref_rad[name]
    return float(raw)

  def _on_joint_state(self, msg: JointState) -> None:
    if not msg.name:
      return

    wrapped_positions: list[float] = []
    for name, raw in zip(msg.name, msg.position):
      reference = self._branch_reference(name, float(raw))
      wrapped = wrap_joint_for_stream(name, float(raw), reference)
      # Follow hardware principalize (±2π) spins: if raw is already inside MoveIt
      # limits and wrap-equal to the tracked branch but ~one turn away, snap to raw
      # so MoveIt start matches RTDE (avoids home reject -4).
      try:
        jidx = CANONICAL_JOINT_NAMES.index(name)
        limits = MOVEIT_JOINT_LIMITS_RAD[jidx]
        raw_f = float(raw)
        if (
          limits is not None
          and limits[0] - 1e-9 <= raw_f <= limits[1] + 1e-9
          and abs(joint_delta_rad(wrapped, raw_f)) < 1e-4
          and abs(wrapped - raw_f) > (math.pi * 0.5)
        ):
          wrapped = raw_f
      except ValueError:
        pass
      previous = self._last_wrapped.get(name, wrapped)
      wrapped = stabilize_joint_reading(previous, wrapped)
      self._last_wrapped[name] = wrapped
      wrapped_positions.append(wrapped)

    out = JointState()
    out.header.stamp = self.get_clock().now().to_msg()
    out.header.frame_id = msg.header.frame_id
    out.name = list(msg.name)
    out.position = wrapped_positions
    out.velocity = list(msg.velocity)
    out.effort = list(msg.effort)
    self._pub_joint_states.publish(out)
    self._pub_stamped.publish(out)


def main() -> int:
  rclpy.init(args=sys.argv)
  node = JointStatesStamper()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
