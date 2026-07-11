#!/usr/bin/env python3
"""Test MoveIt direct_only moves at several pan deltas."""
from __future__ import annotations

import json
import math
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8766"
DELTAS = [float(a) for a in sys.argv[1:]] or [3.0, -3.0, 5.0]


def get(path: str) -> dict:
    with urllib.request.urlopen(BASE + path, timeout=30) as resp:
        return json.load(resp)


def post(path: str, body: dict) -> dict:
    req = urllib.request.Request(
        BASE + path,
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=180) as resp:
        return json.load(resp)


def main() -> int:
    for delta in DELTAS:
        joints = get("/joints")["positions"]
        pan0 = math.degrees(joints[0])
        target = list(joints)
        target[0] += math.radians(delta)
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
        t0 = time.time()
        result = post("/execute_scan_waypoint", body)
        time.sleep(4.0)
        pan1 = math.degrees(get("/joints")["positions"][0])
        moved = pan1 - pan0
        print(
            f"delta={delta:+.1f} ok={result.get('ok')} "
            f"api_s={time.time() - t0:.1f} pan {pan0:.1f}->{pan1:.1f} moved={moved:.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
