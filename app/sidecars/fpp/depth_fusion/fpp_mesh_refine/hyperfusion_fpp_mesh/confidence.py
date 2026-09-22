# Depth confidence maps (sidecar / depth_fusion / fpp_mesh_refine).
# Per-view C_FPP: modulation, phase, smoothness, edges, mask boundary. No robot I/O.
"""Estimate per-pixel depth confidence in [0, 1]."""

from __future__ import annotations

import numpy as np

from .frames import DepthFrame

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit("opencv required") from exc


def _mask_interior_term(mask: np.ndarray, *, falloff_px: float = 4.0) -> np.ndarray:
    """Down-weight pixels near the FPP mask boundary (unwrap / silhouette fringe)."""
    valid_u8 = (mask.astype(np.uint8) * 255)
    # Distance to nearest invalid (0) pixel; interior is far from boundary.
    dist = cv2.distanceTransform(valid_u8, cv2.DIST_L2, 3).astype(np.float32)
    if float(falloff_px) <= 1.0e-6:
        return np.ones(mask.shape, dtype=np.float32)
    return np.clip(dist / float(falloff_px), 0.0, 1.0)


def _discontinuity_term(
    depth_m: np.ndarray,
    valid: np.ndarray,
    *,
    jump_m: float = 0.008,
) -> np.ndarray:
    """Down-weight pixels next to large depth jumps (occlusion / multi-path edges)."""
    fill = depth_m.copy()
    med = float(np.nanmedian(depth_m[valid])) if int(valid.sum()) else 0.0
    if not np.isfinite(med):
        med = 0.25
    fill[~valid] = med
    # Max absolute difference to 4-neighbors.
    up = np.abs(fill - np.roll(fill, 1, axis=0))
    down = np.abs(fill - np.roll(fill, -1, axis=0))
    left = np.abs(fill - np.roll(fill, 1, axis=1))
    right = np.abs(fill - np.roll(fill, -1, axis=1))
    jump = np.maximum(np.maximum(up, down), np.maximum(left, right))
    # Soft: full keep below ~jump_m/2, near-zero above ~2·jump_m.
    scale = max(1.0e-4, float(jump_m))
    return np.exp(-np.maximum(jump - 0.5 * scale, 0.0) / scale).astype(np.float32)


def estimate_confidence(
    depth_m: np.ndarray,
    *,
    modulation: np.ndarray | None,
    phase_quality: np.ndarray | None,
    mask: np.ndarray,
    boundary_falloff_px: float = 4.0,
    discontinuity_jump_mm: float = 8.0,
) -> np.ndarray:
    """Per-view ``C_FPP`` in [0, 1] for soft union scoring / densify weights.

    Terms (multiplied):
      modulation · phase_quality · smoothness · edge · mask_interior · discontinuity
    """
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

    # Four-step phase amplitude. Decode already hard-gates at 0.04; use a
    # soft ramp so marginal phase estimates contribute less during fusion.
    if phase_quality is not None and phase_quality.shape == depth_m.shape:
        quality = np.maximum(phase_quality.astype(np.float32), 0.0)
        t_phase = np.clip((quality - 0.04) / 0.25, 0.0, 1.0)
    else:
        t_phase = np.ones_like(depth_m, dtype=np.float32)

    # Smoothness: low local depth variance → high conf
    fill = depth_m.copy()
    med = float(np.nanmedian(depth_m[valid]))
    if not np.isfinite(med):
        med = 0.25
    fill[~valid] = med
    blur = cv2.GaussianBlur(fill, (5, 5), 0)
    resid = np.abs(fill - blur)
    t_smooth = np.exp(-resid / 0.004).astype(np.float32)  # 4 mm scale

    # Edge: high image gradient of depth → lower conf (uncertain at silhouettes)
    gx = cv2.Sobel(fill, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(fill, cv2.CV_32F, 0, 1, ksize=3)
    grad = np.sqrt(gx * gx + gy * gy)
    t_edge = np.exp(-grad / 0.02).astype(np.float32)  # 20 mm/px soft

    # Mask interior: fringe near decode-mask border is less trusted
    t_interior = _mask_interior_term(valid, falloff_px=float(boundary_falloff_px))

    # Neighbor depth jumps (occlusion / layer edges)
    t_disc = _discontinuity_term(
        depth_m, valid, jump_m=float(discontinuity_jump_mm) * 1.0e-3
    )

    conf = (
        t_mod * t_phase * t_smooth * t_edge * t_interior * t_disc
    ).astype(np.float32)
    conf[~valid] = 0.0
    return conf


def confidence_frames(frames: list[DepthFrame]) -> list[DepthFrame]:
    for fr in frames:
        fr.conf = estimate_confidence(
            fr.depth_m,
            modulation=fr.modulation,
            phase_quality=fr.phase_quality,
            mask=fr.mask,
        )
        # Soft gate: drop very low confidence
        weak = (fr.conf is not None) and (fr.conf < 0.05)
        if isinstance(weak, np.ndarray):
            fr.depth_m = fr.depth_m.copy()
            fr.depth_m[weak] = np.nan
            fr.mask = np.isfinite(fr.depth_m)
            fr.conf = fr.conf.copy()
            fr.conf[~fr.mask] = 0.0
        if isinstance(fr.meta, dict) and fr.conf is not None:
            fr.meta["c_fpp"] = fr.conf
            fr.meta["c_fpp_terms"] = (
                "modulation*phase*smooth*edge*mask_interior*discontinuity"
            )
    return frames
