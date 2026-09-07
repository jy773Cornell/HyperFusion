#!/usr/bin/env python3
"""Run one direct-only pan move via sidecar HTTP (WSL)."""
from __future__ import annotations

import json
import math
import sys
import urllib.request

BASE = "http://127.0.0.1:8766"
DELTA_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0


def get_json(path: str) -> dict:
    with urllib.request.urlopen(BASE + path, timeout=30) as resp:
        return json.load(resp)


def post_json(path: str, body: dict, timeout: float = 180.0) -> dict:
    req = urllib.request.Request(
        BASE + path,
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.load(resp)


def main() -> int:
    health = get_json("/health")
    print("health:", json.dumps(health, indent=2))
    if not health.get("robot_connected"):
        print("Robot not connected — run connect first.", file=sys.stderr)
        return 1

    joints = get_json("/joints")["positions"]
    pan_deg = math.degrees(joints[0])
    print(f"current pan: {pan_deg:.1f} deg")
    target = list(joints)
    target[0] = joints[0] + math.radians(DELTA_DEG)
    print(f"target pan: {math.degrees(target[0]):.1f} deg (+{DELTA_DEG} deg)")

    body = {
        "joints": target,
        "direct_only": True,
        "home_joints_deg": [160, 0, -90, 0, 90, 180],
        "workspace": {
            "enabled": True,
            "length_m": 0.8,
            "width_m": 0.6,
            "height_m": 0.6,
        },
    }
    print("execute_scan_waypoint…")
    result = post_json("/execute_scan_waypoint", body)
    print(json.dumps(result, indent=2))

    after = get_json("/joints")["positions"]
    print(f"after pan: {math.degrees(after[0]):.1f} deg")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
