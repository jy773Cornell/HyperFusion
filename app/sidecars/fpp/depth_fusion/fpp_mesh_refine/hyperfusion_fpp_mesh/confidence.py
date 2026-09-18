# Depth confidence maps (sidecar / depth_fusion / fpp_mesh_refine).
# Combines modulation, local depth smoothness, and edge strength. No robot I/O.
"""Estimate per-pixel depth confidence in [0, 1]."""

from __future__ import annotations

import numpy as np

from .frames import DepthFrame

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit("opencv required") from exc


def estimate_confidence(
    depth_m: np.ndarray,
    *,
    modulation: np.ndarray | None,
    mask: np.ndarray,
) -> np.ndarray:
    """Confidence from modulation × smoothness × (1 - edge)."""
    h, w = depth_m.shape
    conf = np.zeros((h, w), dtype=np.float32)
    valid = mask & np.isfinite(depth_m)
    if int(valid.sum()) < 50:
        return conf

    # Modulation term
    if modulation is not None and modulation.shape == depth_m.shape:
        mod = np.clip(modulation.astype(np.float32), 0.0, 1.0)
        # typical fruit mod ~0.1–0.6; soft ramp
        t_mod = np.clip((mod - 0.08) / 0.35, 0.0, 1.0)
    else:
        t_mod = np.ones_like(depth_m, dtype=np.float32)

    # Smoothness: low local depth variance → high conf
    fill = depth_m.copy()
    med = float(np.nanmedian(depth_m))
    fill[~valid] = med
    blur = cv2.GaussianBlur(fill, (5, 5), 0)
    resid = np.abs(fill - blur)
    t_smooth = np.exp(-resid / 0.004).astype(np.float32)  # 4 mm scale

    # Edge: high image gradient of depth → lower conf (uncertain at silhouettes)
    gx = cv2.Sobel(fill, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(fill, cv2.CV_32F, 0, 1, ksize=3)
    grad = np.sqrt(gx * gx + gy * gy)
    t_edge = np.exp(-grad / 0.02).astype(np.float32)  # 20 mm/px soft

    conf = (t_mod * t_smooth * t_edge).astype(np.float32)
    conf[~valid] = 0.0
    return conf


def confidence_frames(frames: list[DepthFrame]) -> list[DepthFrame]:
    for fr in frames:
        fr.conf = estimate_confidence(fr.depth_m, modulation=fr.modulation, mask=fr.mask)
        # Soft gate: drop very low confidence
        weak = (fr.conf is not None) and (fr.conf < 0.05)
        if isinstance(weak, np.ndarray):
            fr.depth_m = fr.depth_m.copy()
            fr.depth_m[weak] = np.nan
            fr.mask = np.isfinite(fr.depth_m)
            fr.conf = fr.conf.copy()
            fr.conf[~fr.mask] = 0.0
    return frames
