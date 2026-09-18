# Pixel depth from FPP decode (sidecar / offline). Metric camera Z in millimetres.
"""Pixel-level FPP maps: camera Z (mm) from stereo, or projector u (px)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .decode import DecodeResult
from .geometry import StereoGeometry, triangulate_maps
from .paths import resolve_stereo_yaml


@dataclass
class DepthResult:
    """Stereo mode: depth is camera Z in mm. Else depth is projector u in px."""

    depth: np.ndarray
    mask: np.ndarray
    mode: str
    units: str


def load_stereo(path: Path | str | None = None) -> StereoGeometry | None:
    """Load camera_projector_stereo.yaml (default under fpp_cal/results)."""
    resolved = resolve_stereo_yaml(Path(path) if path is not None else None)
    if resolved is None:
        return None
    return StereoGeometry.load(resolved)


def depth_from_decode(
    decoded: DecodeResult,
    stereo: StereoGeometry | None = None,
    *,
    stride: int = 1,
) -> DepthResult:
    """If stereo is set, triangulate camera Z in millimetres. Else projector u (px).

    ``stride`` > 1 skips pixels (faster, sparse). Use 1 for full depth maps.
    """
    mask = decoded.mask.copy()
    if stereo is not None:
        _xyz_m, z_m = triangulate_maps(stereo, decoded, stride=max(1, int(stride)))
        z_mm = (z_m * 1000.0).astype(np.float32)
        valid = mask & np.isfinite(z_mm)
        return DepthResult(depth=z_mm, mask=valid, mode="camera_z_mm", units="mm")
    u = np.where(mask, decoded.projector_u, np.nan).astype(np.float32)
    return DepthResult(depth=u, mask=mask, mode="projector_u_px", units="px")
