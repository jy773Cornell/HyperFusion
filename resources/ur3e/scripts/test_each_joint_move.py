#!/usr/bin/env python3
"""Test MoveIt direct_only move on each joint individually."""
from __future__ import annotations

import json
import math
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8766"
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
DEFAULT_DELTA_DEG = 5.0


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


def deg_list(rad: list[float]) -> list[float]:
    return [round(math.degrees(v), 1) for v in rad]


def move_joint(joint_index: int, delta_deg: float, settle_s: float = 4.0) -> dict:
    before = get("/joints")["positions"]
    target = list(before)
    target[joint_index] += math.radians(delta_deg)
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
    try:
        result = post("/execute_scan_waypoint", body)
    except urllib.error.HTTPError as exc:
        payload = exc.read().decode("utf-8", errors="replace")
        return {
            "ok": False,
            "error": f"HTTP {exc.code}: {payload}",
            "joint_index": joint_index,
            "joint_name": JOINT_NAMES[joint_index],
            "delta_deg": delta_deg,
        }
    time.sleep(settle_s)
    after = get("/joints")["positions"]
    moved_deg = [
        round(math.degrees(after[i] - before[i]), 1) for i in range(6)
    ]
    return {
        "ok": bool(result.get("ok")),
        "executed": result.get("executed"),
        "error": result.get("error"),
        "moveit_error_code": result.get("moveit_error_code"),
        "api_s": round(time.time() - t0, 1),
        "joint_index": joint_index,
        "joint_name": JOINT_NAMES[joint_index],
        "delta_deg": delta_deg,
        "before_deg": deg_list(before),
        "after_deg": deg_list(after),
        "moved_deg": moved_deg,
        "primary_moved_deg": moved_deg[joint_index],
    }


def main() -> int:
    delta = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DELTA_DEG
    health = get("/health")
    print("health:", json.dumps(health, indent=2))
    if not health.get("robot_connected"):
        print("Robot not connected.", file=sys.stderr)
        return 1

    print(f"\n=== Per-joint MoveIt test (delta={delta:+.1f} deg each) ===\n")
    results: list[dict] = []
    for index, name in enumerate(JOINT_NAMES):
        print(f"--- {name} ({index}) +{delta} deg ---")
        row = move_joint(index, delta)
        results.append(row)
        status = "OK" if row.get("ok") and abs(row.get("primary_moved_deg", 0) - delta) < 1.5 else "FAIL"
        print(
            f"{status}: ok={row.get('ok')} moved={row.get('primary_moved_deg')} "
            f"target_delta={delta:+.1f} error={row.get('error')} code={row.get('moveit_error_code')}"
        )
        print(f"  before={row.get('before_deg')}")
        print(f"  after ={row.get('after_deg')}")
        print()

    print("=== Summary ===")
    for row in results:
        moved = row.get("primary_moved_deg", 0)
        delta_req = row.get("delta_deg", 0)
        ok = row.get("ok") and abs(moved - delta_req) < 1.5
        mark = "PASS" if ok else "FAIL"
        print(
            f"{mark} {row['joint_name']}: moved {moved:+.1f} deg "
            f"(wanted {delta_req:+.1f}), error={row.get('error')}"
        )

    failed = sum(
        1
        for row in results
        if not row.get("ok") or abs(row.get("primary_moved_deg", 0) - row.get("delta_deg", 0)) >= 1.5
    )
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
