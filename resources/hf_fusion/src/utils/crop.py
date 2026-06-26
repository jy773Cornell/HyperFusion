# Object-aware crop box computation for dual-camera fusion alignment.
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CropRect:
    x0: int
    y0: int
    x1: int
    y1: int

    @property
    def width(self) -> int:
        return self.x1 - self.x0

    @property
    def height(self) -> int:
        return self.y1 - self.y0

    def is_valid(self) -> bool:
        return self.width > 0 and self.height > 0

    def clamp(self, canvas_width: int, canvas_height: int) -> CropRect:
        x0 = max(0, min(self.x0, canvas_width))
        y0 = max(0, min(self.y0, canvas_height))
        x1 = max(x0, min(self.x1, canvas_width))
        y1 = max(y0, min(self.y1, canvas_height))
        return CropRect(x0, y0, x1, y1)

    def intersect(self, other: CropRect) -> CropRect | None:
        x0 = max(self.x0, other.x0)
        y0 = max(self.y0, other.y0)
        x1 = min(self.x1, other.x1)
        y1 = min(self.y1, other.y1)
        if x1 <= x0 or y1 <= y0:
            return None
        return CropRect(x0, y0, x1, y1)

    def expand(self, margin_px: int) -> CropRect:
        return CropRect(self.x0 - margin_px, self.y0 - margin_px, self.x1 + margin_px, self.y1 + margin_px)

    def as_dict(self) -> dict[str, int]:
        return {"x0": self.x0, "y0": self.y0, "x1": self.x1, "y1": self.y1}

    def size_mm(self, mm_per_pixel: float) -> dict[str, float]:
        return {
            "cross_track": round(self.width * mm_per_pixel, 2),
            "along_scan": round(self.height * mm_per_pixel, 2),
        }


def fov_intersection_rect(
    fx_width: int,
    fx_height: int,
    sw_width: int,
    sw_height: int,
    shift_dx_px: float,
    shift_dy_px: float,
) -> CropRect:
    """Valid overlap of FX canvas and upsampled SWIR extent (shift defaults to origin)."""
    x0 = max(0, int(np.floor(shift_dx_px)))
    y0 = max(0, int(np.floor(shift_dy_px)))
    x1 = min(fx_width, int(np.ceil(shift_dx_px + sw_width)))
    y1 = min(fx_height, int(np.ceil(shift_dy_px + sw_height)))
    return CropRect(x0, y0, max(x0, x1), max(y0, y1))


def union_masks(masks: list[np.ndarray]) -> np.ndarray:
    if not masks:
        raise ValueError("Need at least one mask for union")
    union = np.zeros_like(masks[0], dtype=np.uint8)
    for mask in masks:
        union = np.maximum(union, (mask > 0).astype(np.uint8))
    return union


def mask_bounding_box(masks: list[np.ndarray]) -> CropRect | None:
    if not masks:
        return None
    ys: list[np.ndarray] = []
    xs: list[np.ndarray] = []
    for mask in masks:
        foreground = np.nonzero(mask > 0)
        if foreground[0].size == 0:
            continue
        ys.append(foreground[0])
        xs.append(foreground[1])
    if not xs:
        return None
    y_all = np.concatenate(ys)
    x_all = np.concatenate(xs)
    return CropRect(int(x_all.min()), int(y_all.min()), int(x_all.max()) + 1, int(y_all.max()) + 1)


def compute_roi_crop(
    object_masks: list[np.ndarray],
    fov_rect: CropRect,
    margin_mm: float,
    mm_per_pixel: float,
    canvas_width: int,
    canvas_height: int,
) -> CropRect:
    object_bbox = mask_bounding_box(object_masks)
    if object_bbox is None:
        raise ValueError("No foreground pixels in object masks")
    margin_px = int(np.ceil(margin_mm / mm_per_pixel))
    expanded = object_bbox.expand(margin_px).clamp(canvas_width, canvas_height)
    fov_clamped = fov_rect.clamp(canvas_width, canvas_height)
    crop = expanded.intersect(fov_clamped)
    if crop is None or not crop.is_valid():
        raise ValueError("ROI crop does not intersect valid FOV overlap")
    return crop


def crop_image(image: np.ndarray, rect: CropRect) -> np.ndarray:
    if image.ndim == 2:
        return image[rect.y0 : rect.y1, rect.x0 : rect.x1].copy()
    if image.ndim == 3:
        return image[rect.y0 : rect.y1, rect.x0 : rect.x1, :].copy()
    raise ValueError(f"Unsupported image shape {image.shape}")
