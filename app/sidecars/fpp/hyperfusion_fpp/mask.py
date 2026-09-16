"""Illumination mask: DLP patch is smaller than the BFS FOV."""

from __future__ import annotations

import numpy as np
import cv2

from .capture import FppBurst


def illumination_mask(
    burst: FppBurst,
    *,
    min_modulation: float = 0.15,
    open_px: int = 3,
    close_px: int = 9,
) -> np.ndarray:
    """True where the projector actually hits the scene.

    Uses White−Black (albedo / ambient) so unlit FOV is dropped before decode.
    """
    black = burst.image(0)
    white = burst.image(1)
    modulation = np.clip(white - black, 0.0, 1.0)
    mask = modulation >= float(min_modulation)

    mask_u8 = (mask.astype(np.uint8)) * 255
    if open_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px))
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, k)
    if close_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, k)
    return mask_u8 > 0


def normalize_in_mask(
    image: np.ndarray,
    black: np.ndarray,
    white: np.ndarray,
    mask: np.ndarray,
    eps: float = 1.0e-3,
) -> np.ndarray:
    denom = np.maximum(white - black, eps)
    norm = (image - black) / denom
    out = np.full(image.shape, np.nan, dtype=np.float32)
    valid = mask & np.isfinite(norm)
    out[valid] = np.clip(norm[valid], 0.0, 1.0)
    return out
