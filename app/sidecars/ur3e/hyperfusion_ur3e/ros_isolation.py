"""Isolate parallel mock UR/MoveIt stacks by ROS_DOMAIN_ID (WSL sidecar)."""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Iterable, List


def skip_stale_kill() -> bool:
    return os.environ.get("HYPERFUSION_SKIP_STALE_KILL", "").strip().lower() in (
        "1",
        "true",
        "yes",
    )


def ros_domain_id() -> str:
    raw = os.environ.get("ROS_DOMAIN_ID", "0").strip()
    return raw or "0"


def bash_domain_exports() -> str:
    """Prefix for bash -lc launch strings so children stay on this domain."""
    domain = ros_domain_id()
    return (
        f"export ROS_DOMAIN_ID='{domain}' && "
        "export ROS_LOCALHOST_ONLY=1 && "
        "export ROS2CLI_DISABLE_DAEMON=1 && "
    )


def ur_driver_ports() -> dict[str, int]:
    """Host TCP ports for ur_robot_driver. Offset per parallel mock via env."""
    reverse = int(os.environ.get("HYPERFUSION_UR_REVERSE_PORT", "50001") or "50001")
    return {
        "reverse_port": reverse,
        "script_sender_port": int(
            os.environ.get("HYPERFUSION_UR_SCRIPT_SENDER_PORT", str(reverse + 1))
        ),
        "trajectory_port": int(
            os.environ.get("HYPERFUSION_UR_TRAJECTORY_PORT", str(reverse + 2))
        ),
        "script_command_port": int(
            os.environ.get("HYPERFUSION_UR_SCRIPT_COMMAND_PORT", str(reverse + 3))
        ),
    }


def proc_environ(pid: int) -> dict[str, str]:
    try:
        raw = Path(f"/proc/{pid}/environ").read_bytes()
    except OSError:
        return {}
    out: dict[str, str] = {}
    for part in raw.split(b"\0"):
        if not part or b"=" not in part:
            continue
        key, value = part.split(b"=", 1)
        out[key.decode("utf-8", "replace")] = value.decode("utf-8", "replace")
    return out


def pgrep_af(pattern: str) -> List[tuple[int, str]]:
    proc = subprocess.run(
        ["pgrep", "-af", pattern],
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        return []
    rows: List[tuple[int, str]] = []
    for line in (proc.stdout or "").splitlines():
        line = line.strip()
        if not line:
            continue
        pid_s, _, cmd = line.partition(" ")
        try:
            rows.append((int(pid_s), cmd))
        except ValueError:
            continue
    return rows


def pids_in_current_domain(pattern: str) -> List[int]:
    domain = ros_domain_id()
    found: List[int] = []
    for pid, _cmd in pgrep_af(pattern):
        env = proc_environ(pid)
        if env.get("ROS_DOMAIN_ID", "0") == domain:
            found.append(pid)
    return found


def kill_pids(pids: Iterable[int], *, wait_s: float = 0.5) -> None:
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            continue
    if wait_s > 0:
        time.sleep(wait_s)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            continue
