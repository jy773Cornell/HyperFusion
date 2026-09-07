"""Pixel-level FPP maps: projector u and optional Δu vs the plane homography.

No millimetre depth until a non-degenerate projector P exists.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .calibrate import FppCalibration, apply_u_homography
from .decode import DecodeResult


@dataclass
class DepthResult:
    depth_m: np.ndarray
    disparity_u: np.ndarray
    mask: np.ndarray
    mode: str


def depth_from_decode(
    decoded: DecodeResult,
    calib: FppCalibration | None,
    meta: dict | None = None,
) -> DepthResult:
    del meta
    mask = decoded.mask.copy()
    u = np.where(mask, decoded.projector_u, np.nan).astype(np.float32)
    disparity = np.full(mask.shape, np.nan, dtype=np.float32)
    mode = "projector_u_px"
    if calib is not None:
        vs, us = np.nonzero(mask & np.isfinite(decoded.projector_u))
        h = calib.u_homography
        if vs.size > 0 and h is not None and np.asarray(h).size == 6:
            xy = np.stack([us.astype(np.float64), vs.astype(np.float64)], axis=1)
            plane_u = apply_u_homography(np.asarray(h, dtype=np.float64), xy)
            disparity[vs, us] = (decoded.projector_u[vs, us] - plane_u).astype(np.float32)
            mode = "projector_u_px+du"
        elif calib.u_reference.shape == decoded.projector_u.shape:
            both = mask & np.isfinite(decoded.projector_u) & np.isfinite(calib.u_reference)
            disparity[both] = (decoded.projector_u[both] - calib.u_reference[both]).astype(
                np.float32
            )
            mode = "projector_u_px+du"
    return DepthResult(depth_m=u, disparity_u=disparity, mask=mask, mode=mode)


def remove_plane_background(
    depth: DepthResult,
    decoded: DecodeResult,
    *,
    min_abs_disparity_px: float = 0.0,
    open_px: int = 3,
    close_px: int = 9,
) -> DepthResult:
    """Optional: drop the card. Default 0 keeps the full lit patch."""
    if min_abs_disparity_px <= 0.0:
        return depth
    d = depth.disparity_u
    valid = depth.mask & np.isfinite(d)
    if int(valid.sum()) < 1000:
        return depth

    near = valid & (np.abs(d) < float(min_abs_disparity_px))
    far = valid & (np.abs(d) >= float(min_abs_disparity_px))
    fg = far if int(far.sum()) < int(near.sum()) else near

    mask_u8 = (fg.astype(np.uint8)) * 255
    if open_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px))
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, k)
    if close_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, k)
    mask = mask_u8 > 0
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    if num > 2:
        keep = np.zeros(mask.shape, dtype=bool)
        for i in range(1, num):
            width = int(stats[i, cv2.CC_STAT_WIDTH])
            height = int(stats[i, cv2.CC_STAT_HEIGHT])
            area = float(stats[i, cv2.CC_STAT_AREA])
            long_side = max(width, height)
            short_side = min(width, height)
            skinny_bar = long_side >= 350 and short_side / float(long_side) < 0.20
            if skinny_bar or area < 8000.0:
                continue
            keep |= labels == i
        if keep.any():
            mask = keep

    decoded.mask = mask
    decoded.projector_u = np.where(mask, decoded.projector_u, np.nan).astype(np.float32)
    decoded.wrapped_phase = np.where(mask, decoded.wrapped_phase, np.nan).astype(np.float32)
    return DepthResult(
        depth_m=np.where(mask, depth.depth_m, np.nan).astype(np.float32),
        disparity_u=np.where(mask, d, np.nan).astype(np.float32),
        mask=mask,
        mode=f"{depth.mode}+fg",
    )
