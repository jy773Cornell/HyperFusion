#!/usr/bin/env python3
"""Connect (if needed) and poll sidecar status."""
from __future__ import annotations

import json
import time
import urllib.request

BASE = "http://127.0.0.1:8766"


def get(path: str) -> dict:
    with urllib.request.urlopen(BASE + path, timeout=10) as resp:
        return json.load(resp)


def post(path: str, body: dict | None = None) -> dict:
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(
        BASE + path,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.load(resp)


def main() -> None:
    health = get("/health")
    print("health:", json.dumps(health, indent=2))
    if not health.get("robot_connected"):
        print("Starting connect… (press Play on External Control if prompted)")
        print(post("/connect/start", {"ip": "192.168.1.10"}))
    for i in range(40):
        status = get("/connect/status")
        print(f"poll {i}: phase={status.get('phase')} connected={status.get('connected')} msg={status.get('message')}")
        if status.get("connected"):
            break
        time.sleep(3)


if __name__ == "__main__":
    main()
