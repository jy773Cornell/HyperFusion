# Default FPP calib paths under app/calibration/multiview/fpp_cal (sidecar).
# Offline only — GUI does not load these yet.
"""Resolved paths to stereo YAML and checkerboard bursts."""
from __future__ import annotations

from pathlib import Path

# hyperfusion_fpp/ -> fpp/ -> sidecars/ -> app/
_APP = Path(__file__).resolve().parents[3]
FPP_CAL = _APP / "calibration" / "multiview" / "fpp_cal"
DEFAULT_STEREO_YAML = FPP_CAL / "results" / "camera_projector_stereo.yaml"
DEFAULT_CHECKERBOARD = FPP_CAL / "checkerboard"
DEFAULT_BOARD_YAML = _APP / "calibration" / "multiview" / "bfs_cal" / "board.yaml"


def resolve_stereo_yaml(explicit: Path | None = None) -> Path | None:
    """Return an existing stereo YAML, or None if missing."""
    if explicit is not None:
        path = Path(explicit)
        if not path.is_file():
            raise FileNotFoundError(f"Stereo YAML not found: {path}")
        return path
    if DEFAULT_STEREO_YAML.is_file():
        return DEFAULT_STEREO_YAML
    return None
