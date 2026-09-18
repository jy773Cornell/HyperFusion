# FPP MVS path helpers (sidecar / fpp / fpp_mvs). Offline only.
"""Resolve datafolder / multiview / processed layout and detect FPP bursts."""

from __future__ import annotations

import json
from pathlib import Path


def resolve_datafolder(input_path: Path) -> tuple[Path, Path]:
    """Return (datafolder, multiview_burst).

    Accepts either the dataset/cluster root or its ``multiview/`` folder.
    """
    p = Path(input_path).resolve()
    if p.name.lower() == "multiview" and p.is_dir():
        return p.parent, p
    multiview = p / "multiview"
    if multiview.is_dir():
        return p, multiview
    # Flat burst folder (standalone execute) — treat as both
    if any(p.glob("*.tif")) or any(p.glob("*.tiff")):
        return p, p
    raise FileNotFoundError(
        f"Expected dataset root with multiview/ or a multiview burst folder: {p}"
    )


def processed_root(burst: Path) -> Path:
    """Return the processing root inside the MVS burst folder."""
    return Path(burst) / "processed"


def decode_root(burst: Path) -> Path:
    return processed_root(burst) / "decode"


def fusion_root(burst: Path) -> Path:
    return processed_root(burst) / "fusion"


def has_fpp_burst(burst: Path, *, sample: int = 40) -> bool:
    """True if pose JSONs look like an FPP MVS capture (fpp_pattern / fpp_step_index)."""
    burst = Path(burst)
    if not burst.is_dir():
        return False
    n = 0
    for jpath in sorted(burst.glob("*.json")):
        if jpath.name.lower() == "transforms.json":
            continue
        try:
            meta = json.loads(jpath.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if "fpp_pattern" in meta or "fpp_step_index" in meta:
            return True
        n += 1
        if n >= sample:
            break
    return False


def default_hand_eye(burst: Path) -> Path | None:
    """Search dataset-local BFS calibration, with legacy fallbacks."""
    burst = Path(burst).resolve()
    # …/clusters/<name>/multiview → parents[1] = clusters
    for base in (burst.parents[1] if len(burst.parents) > 1 else burst.parent, burst.parent):
        for rel in (
            Path("camera_cal") / "bfs_cal" / "results" / "flange_T_camera.yaml",
            Path("camera_cal") / "bfs_cal" / "ba" / "results" / "flange_T_camera.yaml",
            Path("camera_cal") / "ba" / "results" / "flange_T_camera.yaml",
        ):
            cand = base / rel
            if cand.is_file():
                return cand
    return None


def default_stereo(burst: Path) -> Path | None:
    """Find dataset-local camera/projector stereo before app defaults."""
    burst = Path(burst).resolve()
    for base in (burst.parents[1] if len(burst.parents) > 1 else burst.parent, burst.parent):
        for rel in (
            Path("camera_cal") / "dlp_cal" / "results" / "camera_projector_stereo.yaml",
            Path("camera_cal") / "fpp_cal" / "results" / "camera_projector_stereo.yaml",
        ):
            cand = base / rel
            if cand.is_file():
                return cand
    return None
