# Edge-aware depth filtering (sidecar / depth_fusion / fpp_mesh_refine).
# Bilateral-style filter on valid FPP depth. No robot I/O.
"""Edge-aware smoothing of raw depth maps."""

from __future__ import annotations

import numpy as np

from .frames import DepthFrame

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit("opencv required") from exc


def edge_aware_filter_depth(
    depth_m: np.ndarray,
    *,
    d_sigma_m: float = 0.004,
    space_sigma_px: float = 3.0,
    diameter: int = 7,
) -> np.ndarray:
    """Bilateral filter on depth; NaNs preserved via fill→filter→restore."""
    out = depth_m.astype(np.float32).copy()
    valid = np.isfinite(out)
    if int(valid.sum()) < 100:
        return out
    fill = out.copy()
    med = float(np.nanmedian(out))
    fill[~valid] = med
    # OpenCV bilateral expects 8-bit or float; scale depth to mm-ish range for sigma
    # Work in millimetres for numerically nicer sigmas.
    fill_mm = fill * 1000.0
    d_sigma = float(d_sigma_m) * 1000.0
    filt = cv2.bilateralFilter(
        fill_mm,
        d=int(diameter),
        sigmaColor=d_sigma,
        sigmaSpace=float(space_sigma_px),
    )
    out[valid] = (filt[valid] * 1.0e-3).astype(np.float32)
    out[~valid] = np.nan
    return out


def filter_frames(
    frames: list[DepthFrame],
    *,
    d_sigma_m: float = 0.004,
    space_sigma_px: float = 3.0,
    diameter: int = 7,
) -> list[DepthFrame]:
    for fr in frames:
        fr.depth_m = edge_aware_filter_depth(
            fr.depth_m,
            d_sigma_m=d_sigma_m,
            space_sigma_px=space_sigma_px,
            diameter=diameter,
        )
        fr.mask = np.isfinite(fr.depth_m)
    return frames
