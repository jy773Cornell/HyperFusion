# Camera pose helpers for FPP depth fusion (sidecar / depth_fusion).
# Resolves camera optical c2w in base_link from JSON + optional hand-eye YAML.
"""Load flange→camera hand–eye and build OpenCV camera c2w poses."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml


def load_flange_T_camera_yaml(path: Path) -> np.ndarray:
    """Read calibrate_bfs flange_T_camera.yaml (park / top-level T)."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Bad hand-eye YAML: {path}")
    t = data.get("T")
    if t is None and isinstance(data.get("methods"), dict):
        park = data["methods"].get("park") or data["methods"].get("tsai")
        if isinstance(park, dict):
            t = park.get("flange_T_camera") or park.get("T")
    if t is None:
        raise KeyError(f"No T / flange_T_camera in {path}")
    T = np.asarray(t, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"Expected 4x4 T in {path}, got {T.shape}")
    return T


def load_json_c2w(meta: dict) -> np.ndarray:
    ext = meta.get("extrinsics") or {}
    if ext.get("convention") == "camera_to_base_link_opencv" and "R" in ext and "t" in ext:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = np.asarray(ext["R"], dtype=np.float64)
        T[:3, 3] = np.asarray(ext["t"], dtype=np.float64).reshape(3)
        return T
    T_gl = np.asarray(meta["transform_matrix"], dtype=np.float64)
    flip = np.diag([1.0, -1.0, -1.0, 1.0])
    return T_gl @ flip


def load_flange_c2w(meta: dict) -> np.ndarray | None:
    bt = meta.get("base_T_flange")
    if not isinstance(bt, dict) or "T" not in bt:
        return None
    T = np.asarray(bt["T"], dtype=np.float64)
    return T if T.shape == (4, 4) else None


def resolve_camera_c2w(
    meta: dict,
    *,
    tool0_T_camera: np.ndarray | None,
    pose_mode: str,
) -> tuple[np.ndarray, str]:
    """Return (c2w_camera_opencv, source_tag)."""
    T_json = load_json_c2w(meta)
    T_flange = load_flange_c2w(meta)

    if pose_mode == "json":
        return T_json, "json_extrinsics"

    if pose_mode == "flange_camera":
        if T_flange is None or tool0_T_camera is None:
            raise RuntimeError("flange_camera needs base_T_flange + hand-eye")
        return T_flange @ tool0_T_camera, "flange@hand_eye"

    # auto: prefer flange @ hand-eye when both exist (correct for lens-specific cal)
    if T_flange is not None and tool0_T_camera is not None:
        return T_flange @ tool0_T_camera, "auto_flange@hand_eye"
    return T_json, "json_extrinsics_fallback"


def load_pose_meta(burst: Path, stem: str) -> dict:
    path = Path(burst) / f"{stem}.json"
    return json.loads(path.read_text(encoding="utf-8"))
