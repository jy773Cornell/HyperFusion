# Default FPP calib paths under app/calibration/multiview (sidecar).
# Offline only — GUI does not load these yet.
"""Resolved paths to stereo YAML and checkerboard bursts."""
from __future__ import annotations

from pathlib import Path

# fpp_depth/ -> sidecars/fpp/ -> sidecars/ -> app/
_APP = Path(__file__).resolve().parents[3]
_CAL = _APP / "calibration" / "multiview"
# Prefer dlp_cal (current); fall back to legacy fpp_cal name.
_DLP_CAL = _CAL / "dlp_cal"
_FPP_CAL = _CAL / "fpp_cal"
FPP_CAL = _DLP_CAL if (_DLP_CAL / "results").is_dir() else _FPP_CAL
DEFAULT_STEREO_YAML = FPP_CAL / "results" / "camera_projector_stereo.yaml"
DEFAULT_CHECKERBOARD = FPP_CAL / "checkerboard"
DEFAULT_BOARD_YAML = _CAL / "bfs_cal" / "board.yaml"


def resolve_stereo_yaml(explicit: Path | None = None) -> Path | None:
    """Return an existing stereo YAML, or None if missing."""
    if explicit is not None:
        path = Path(explicit)
        if not path.is_file():
            raise FileNotFoundError(f"Stereo YAML not found: {path}")
        return path
    for cand in (
        _DLP_CAL / "results" / "camera_projector_stereo.yaml",
        _FPP_CAL / "results" / "camera_projector_stereo.yaml",
    ):
        if cand.is_file():
            return cand
    return None
