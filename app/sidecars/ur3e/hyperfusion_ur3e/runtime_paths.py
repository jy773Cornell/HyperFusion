"""Writable runtime dir for generated UR3e sidecar files (WSL).

Install trees such as C:\\HyperFusion are often not writable from WSL
([Errno 13] on initial_positions.yaml). Prefer the package config/ folder
when we can write there; otherwise use ~/.cache/hyperfusion_ur3e.
"""
from __future__ import annotations

import os
from pathlib import Path

from hyperfusion_ur3e import PKG_ROOT


def _can_write_dir(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / ".hf_write_probe"
        probe.write_text("ok", encoding="utf-8")
        probe.unlink(missing_ok=True)
        return True
    except OSError:
        return False


def runtime_dir() -> Path:
    """Directory for generated yaml/urdf/logs. Always writable."""
    env = os.environ.get("HYPERFUSION_UR3E_RUNTIME", "").strip()
    if env:
        chosen = Path(env).expanduser()
        chosen.mkdir(parents=True, exist_ok=True)
        return chosen.resolve()
    pkg_config = PKG_ROOT / "config"
    if _can_write_dir(pkg_config):
        return pkg_config.resolve()
    cache_root = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"))
    chosen = cache_root / "hyperfusion_ur3e"
    chosen.mkdir(parents=True, exist_ok=True)
    return chosen.resolve()


def initial_positions_path() -> Path:
    return runtime_dir() / "initial_positions.yaml"


def runtime_urdf_path() -> Path:
    return runtime_dir() / "runtime_robot_description.urdf"


def driver_stderr_log_path() -> Path:
    return runtime_dir() / "ur_robot_driver_stderr.log"
