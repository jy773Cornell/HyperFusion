#!/usr/bin/env python3
"""Probe /joint_states (and fallbacks) via rclpy — avoids ros2 CLI daemon hangs on WSL."""
from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

TOPICS = (
    "/joint_states",
    "/joint_states_stamped",
    "/joint_state_broadcaster/joint_states",
)


class JointStatesProbe(Node):
    def __init__(self) -> None:
        super().__init__("hyperfusion_joint_states_probe")
        self._got = False
        for topic in TOPICS:
            self.create_subscription(JointState, topic, self._on_msg, 10)

    def _on_msg(self, msg: JointState) -> None:
        if msg.name:
            self._got = True


def main() -> int:
    timeout_s = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = JointStatesProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.time() + max(1.0, timeout_s)
    try:
        while time.time() < deadline and not node._got:
            executor.spin_once(timeout_sec=0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node._got else 1


if __name__ == "__main__":
    raise SystemExit(main())
