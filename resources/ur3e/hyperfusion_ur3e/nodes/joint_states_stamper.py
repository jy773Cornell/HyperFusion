#!/usr/bin/env python3
"""
Republish joint states with valid ROS timestamps for MoveIt execution.

Mock UR hardware can publish JointState messages with header.stamp = 0.
MoveIt rejects those; this node publishes /joint_states_stamped for MoveIt.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

INPUT_TOPIC = "/joint_states"
OUTPUT_TOPIC = "/joint_states_stamped"


class JointStatesStamper(Node):
  def __init__(self) -> None:
    super().__init__("hyperfusion_joint_states_stamper")
    self._pub = self.create_publisher(JointState, OUTPUT_TOPIC, 10)
    self._sub = self.create_subscription(JointState, INPUT_TOPIC, self._on_joint_state, 10)
    self.get_logger().info(f"Republishing {INPUT_TOPIC} -> {OUTPUT_TOPIC} with wall-clock stamps")

  def _on_joint_state(self, msg: JointState) -> None:
    if not msg.name:
      return

    out = JointState()
    out.header.stamp = self.get_clock().now().to_msg()
    out.header.frame_id = msg.header.frame_id
    out.name = list(msg.name)
    out.position = list(msg.position)
    out.velocity = list(msg.velocity)
    out.effort = list(msg.effort)
    self._pub.publish(out)


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
