# Ground-grid resampling for dual-camera fusion (RGB, masks, future HSI bands).
from __future__ import annotations

from enum import Enum

import cv2
import numpy as np


class RgbInterpolation(str, Enum):
    BILINEAR = "bilinear"
    BICUBIC = "bicubic"
    LANCZOS = "lanczos"


_RGB_CV2 = {
    RgbInterpolation.BILINEAR: cv2.INTER_LINEAR,
    RgbInterpolation.BICUBIC: cv2.INTER_CUBIC,
    RgbInterpolation.LANCZOS: cv2.INTER_LANCZOS4,
}


def target_size_px(
    source_width: int,
    source_height: int,
    source_mm_per_pixel: float,
    target_mm_per_pixel: float,
) -> tuple[int, int]:
    scale = source_mm_per_pixel / target_mm_per_pixel
    return round(source_width * scale), round(source_height * scale)


def upsample_rgb(
    rgb: np.ndarray,
    target_width: int,
    target_height: int,
    method: RgbInterpolation = RgbInterpolation.BICUBIC,
) -> np.ndarray:
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"Expected HxWx3 RGB array, got shape {rgb.shape}")
    interpolation = _RGB_CV2[method]
    return cv2.resize(rgb, (target_width, target_height), interpolation=interpolation)


def upsample_mask(
    mask: np.ndarray,
    target_width: int,
    target_height: int,
) -> np.ndarray:
    if mask.ndim != 2:
        raise ValueError(f"Expected HxW mask array, got shape {mask.shape}")
    resized = cv2.resize(mask, (target_width, target_height), interpolation=cv2.INTER_NEAREST)
    return (resized > 0).astype(np.uint8)


def shift_mask_to_canvas(
    mask: np.ndarray,
    dx_px: float,
    dy_px: float,
    canvas_width: int,
    canvas_height: int,
) -> np.ndarray:
    matrix = np.float32([[1.0, 0.0, dx_px], [0.0, 1.0, dy_px]])
    shifted = cv2.warpAffine(
        mask,
        matrix,
        (canvas_width, canvas_height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return (shifted > 0).astype(np.uint8)


def shift_rgb_to_canvas(
    rgb: np.ndarray,
    dx_px: float,
    dy_px: float,
    canvas_width: int,
    canvas_height: int,
) -> np.ndarray:
    matrix = np.float32([[1.0, 0.0, dx_px], [0.0, 1.0, dy_px]])
    return cv2.warpAffine(
        rgb,
        matrix,
        (canvas_width, canvas_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )


def upsample_hsi_cube(
    cube: np.ndarray,
    target_width: int,
    target_height: int,
) -> np.ndarray:
    """Upsample (lines, samples, bands) spatially with bilinear interpolation per band."""
    if cube.ndim != 3:
        raise ValueError(f"Expected HxWxB cube, got shape {cube.shape}")
    lines, samples, bands = cube.shape
    out = np.zeros((target_height, target_width, bands), dtype=np.float32)
    for band in range(bands):
        out[:, :, band] = cv2.resize(
            cube[:, :, band],
            (target_width, target_height),
            interpolation=cv2.INTER_LINEAR,
        )
    return out


def shift_hsi_to_canvas(
    cube: np.ndarray,
    dx_px: float,
    dy_px: float,
    canvas_width: int,
    canvas_height: int,
) -> np.ndarray:
    """Shift (lines, samples, bands) onto a fixed canvas with zero border fill."""
    if cube.ndim != 3:
        raise ValueError(f"Expected HxWxB cube, got shape {cube.shape}")
    _, _, bands = cube.shape
    matrix = np.float32([[1.0, 0.0, dx_px], [0.0, 1.0, dy_px]])
    out = np.zeros((canvas_height, canvas_width, bands), dtype=np.float32)
    for band in range(bands):
        out[:, :, band] = cv2.warpAffine(
            cube[:, :, band],
            matrix,
            (canvas_width, canvas_height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0.0,
        )
    return out


def shift_hsi_cube(cube: np.ndarray, dx_px: float, dy_px: float) -> np.ndarray:
    """Apply sub-pixel translation to a (lines, samples, bands) cube in place dimensions."""
    if cube.ndim != 3:
        raise ValueError(f"Expected HxWxB cube, got shape {cube.shape}")
    height, width, bands = cube.shape
    matrix = np.float32([[1.0, 0.0, dx_px], [0.0, 1.0, dy_px]])
    out = np.zeros((height, width, bands), dtype=np.float32)
    for band in range(bands):
        out[:, :, band] = cv2.warpAffine(
            cube[:, :, band],
            matrix,
            (width, height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0.0,
        )
    return out


def similarity_matrix(
    center_x: float,
    center_y: float,
    scale: float,
    dx_px: float = 0.0,
    dy_px: float = 0.0,
) -> np.ndarray:
    """2x3 affine for uniform scale about (center_x, center_y) plus translation."""
    matrix = cv2.getRotationMatrix2D((center_x, center_y), 0.0, scale)
    matrix = matrix.astype(np.float32)
    matrix[0, 2] += dx_px
    matrix[1, 2] += dy_px
    return matrix


def similarity_warp_hsi_cube(
    cube: np.ndarray,
    scale: float,
    dx_px: float,
    dy_px: float,
    center_x: float,
    center_y: float,
) -> np.ndarray:
    """Uniform scale + translation on (lines, samples, bands)."""
    if cube.ndim != 3:
        raise ValueError(f"Expected HxWxB cube, got shape {cube.shape}")
    height, width, bands = cube.shape
    matrix = similarity_matrix(center_x, center_y, scale, dx_px, dy_px)
    out = np.zeros((height, width, bands), dtype=np.float32)
    for band in range(bands):
        out[:, :, band] = cv2.warpAffine(
            cube[:, :, band],
            matrix,
            (width, height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0.0,
        )
    return out


def similarity_warp_mask(
    mask: np.ndarray,
    scale: float,
    dx_px: float,
    dy_px: float,
    center_x: float,
    center_y: float,
) -> np.ndarray:
    """Uniform scale + translation on a binary mask (nearest-neighbor)."""
    if mask.ndim != 2:
        raise ValueError(f"Expected HxW mask, got shape {mask.shape}")
    height, width = mask.shape
    matrix = similarity_matrix(center_x, center_y, scale, dx_px, dy_px)
    warped = cv2.warpAffine(
        mask.astype(np.uint8),
        matrix,
        (width, height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return (warped > 0).astype(np.uint8)


def crop_hsi_cube(cube: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> np.ndarray:
    return cube[y0:y1, x0:x1, :].copy()
