"""Shared bridge data types."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List


@dataclass
class TcpPose:
    x: float = 0.3
    y: float = 0.0
    z: float = 0.4
    rx: float = math.pi
    ry: float = 0.0
    rz: float = 0.0

    def as_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    @classmethod
    def from_list(cls, values: List[float]) -> "TcpPose":
        if len(values) != 6:
            raise ValueError("TCP pose must have 6 values [x,y,z,rx,ry,rz]")
        return cls(*[float(v) for v in values])


@dataclass
class RobotStatus:
    connected: bool = False
    driver_state: str = "disconnected"
    robot_ip: str = ""
    fault: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "connected": self.connected,
            "driver_state": self.driver_state,
            "robot_ip": self.robot_ip,
            "fault": self.fault or None,
        }
