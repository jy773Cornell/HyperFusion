# Chip mask helpers for phase correction and fused ROI export (backend/offline).
from __future__ import annotations

from pathlib import Path

import numpy as np

from src.utils.resample import shift_mask_to_canvas


def sw_mask_after_refine(
    sw_mask_before: np.ndarray,
    dx_px: float,
    dy_px: float,
    *,
    scale: float = 1.0,
    center_x: float | None = None,
    center_y: float | None = None,
) -> np.ndarray:
    height, width = sw_mask_before.shape
    if abs(scale - 1.0) < 1e-6 and abs(dx_px) < 1e-6 and abs(dy_px) < 1e-6:
        return sw_mask_before
    if abs(scale - 1.0) >= 1e-6:
        from src.utils.resample import similarity_warp_mask

        if center_x is None or center_y is None:
            from src.phase_correction import mask_centroid

            center_x, center_y = mask_centroid(sw_mask_before)
        return similarity_warp_mask(sw_mask_before, scale, dx_px, dy_px, center_x, center_y)
    return shift_mask_to_canvas(sw_mask_before, dx_px, dy_px, width, height)


def pc_chip_mask(fx_mask: np.ndarray, sw_mask: np.ndarray, dilate_px: int = 8) -> np.ndarray:
    import cv2

    union = np.maximum(fx_mask > 0, sw_mask > 0).astype(np.uint8)
    if dilate_px > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px * 2 + 1, dilate_px * 2 + 1))
        union = cv2.dilate(union, kernel, iterations=1)
    return union


def fused_chip_mask(
    fx_mask: np.ndarray,
    sw_mask: np.ndarray,
    *,
    mode: str = "intersection",
) -> np.ndarray:
    fx_bool = fx_mask > 0
    sw_bool = sw_mask > 0
    if mode == "intersection":
        return (fx_bool & sw_bool).astype(np.uint8)
    if mode == "union":
        return (fx_bool | sw_bool).astype(np.uint8)
    if mode == "fx":
        return fx_bool.astype(np.uint8)
    raise ValueError(f"Unknown fused chip mask mode: {mode}")


def save_roi_mask(mask: np.ndarray, npy_path: Path, png_path: Path) -> None:
    from PIL import Image

    binary = (mask > 0).astype(np.uint8)
    np.save(npy_path, binary)
    Image.fromarray(binary * 255, mode="L").save(png_path)
