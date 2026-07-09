#!/usr/bin/env python3
"""
Republish joint states with valid ROS timestamps for RViz and MoveIt.

Mock UR hardware can publish JointState messages with header.stamp = 0.
robot_state_publisher ignores those for TF, so RViz shows only the base link.
This node reads the broadcaster local topic and republishes stamped states to
/joint_states (RViz + robot_state_publisher) and /joint_states_stamped (MoveIt).
Joint angles are wrapped for branch continuity so MoveIt never sees equivalent
but out-of-branch values (e.g. 209 deg vs -151 deg on shoulder_lift).
"""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# ros2_control joint_state_broadcaster with use_local_topics:=true (HyperFusion config).
INPUT_TOPIC = "/joint_state_broadcaster/joint_states"
OUTPUT_JOINT_STATES_TOPIC = "/joint_states"
OUTPUT_STAMPED_TOPIC = "/joint_states_stamped"

TWO_PI = 2.0 * math.pi
JOINT_LIMITS_RAD = {
    "shoulder_pan_joint": (-TWO_PI, TWO_PI),
    "shoulder_lift_joint": (-TWO_PI, TWO_PI),
    "elbow_joint": (-math.pi, math.pi),
    "wrist_1_joint": (-TWO_PI, TWO_PI),
    "wrist_2_joint": (-TWO_PI, TWO_PI),
}


def _joint_delta_rad(a: float, b: float) -> float:
    delta = float(b) - float(a)
    while delta > math.pi:
        delta -= TWO_PI
    while delta < -math.pi:
        delta += TWO_PI
    return delta


def _wrap_joint(name: str, value: float, reference: float) -> float:
    limits = JOINT_LIMITS_RAD.get(name)
    if limits is None:
        wrapped = float(value)
        while wrapped > math.pi:
            wrapped -= TWO_PI
        while wrapped <= -math.pi:
            wrapped += TWO_PI
        if abs(_joint_delta_rad(reference, wrapped)) < 1e-6:
            return float(reference)
        return wrapped

    lo, hi = limits
    base = float(value)
    best = base
    best_dist = abs(_joint_delta_rad(reference, base))
    for step in range(-4, 5):
        candidate = base + (step * TWO_PI)
        if candidate < lo - 1e-9 or candidate > hi + 1e-9:
            continue
        dist = abs(_joint_delta_rad(reference, candidate))
        if dist < best_dist - 1e-9:
            best = candidate
            best_dist = dist
    if abs(_joint_delta_rad(reference, best)) < 1e-6:
        return float(reference)
    return best


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
    self.get_logger().info(
        f"Republishing {INPUT_TOPIC} -> {OUTPUT_JOINT_STATES_TOPIC}, {OUTPUT_STAMPED_TOPIC}"
    )

  def _on_joint_state(self, msg: JointState) -> None:
    if not msg.name:
      return

    wrapped_positions: list[float] = []
    for name, raw in zip(msg.name, msg.position):
      reference = self._last_wrapped.get(name, float(raw))
      wrapped = _wrap_joint(name, float(raw), reference)
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
