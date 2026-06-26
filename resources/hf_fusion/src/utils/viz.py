# Visualization helpers for fusion alignment QA.
from __future__ import annotations

import cv2
import numpy as np
from PIL import Image, ImageDraw

from src.utils.dataset import LoadedCameraRgb, ObjectCentroid


def _draw_mask_contour(image_bgr: np.ndarray, mask: np.ndarray, color_bgr: tuple[int, int, int], thickness: int) -> None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(image_bgr, contours, -1, color_bgr, thickness, lineType=cv2.LINE_AA)


def mask_overlap_metrics(
    fx_mask: np.ndarray,
    sw_mask: np.ndarray,
    mm_per_pixel: float,
) -> dict[str, float]:
    """Intersection / union overlap between two binary masks on the reference grid."""
    fx_bool = fx_mask > 0
    sw_bool = sw_mask > 0
    intersection = int(np.count_nonzero(fx_bool & sw_bool))
    union = int(np.count_nonzero(fx_bool | sw_bool))
    fx_area = int(np.count_nonzero(fx_bool))
    sw_area = int(np.count_nonzero(sw_bool))
    px_area_mm2 = mm_per_pixel**2
    return {
        "iou_percent": round(100.0 * intersection / union, 1) if union else 0.0,
        "fx_coverage_percent": round(100.0 * intersection / fx_area, 1) if fx_area else 0.0,
        "sw_coverage_percent": round(100.0 * intersection / sw_area, 1) if sw_area else 0.0,
        "intersection_area_mm2": round(intersection * px_area_mm2, 2),
        "union_area_mm2": round(union * px_area_mm2, 2),
    }


def draw_mask_overlap_outline(
    rgb: np.ndarray,
    fx_mask: np.ndarray,
    sw_mask: np.ndarray,
    metrics: dict[str, float],
    title: str = "",
) -> Image.Image:
    """FX/SWIR mask contours on cropped RGB with overlap percentages."""
    canvas = cv2.cvtColor(rgb.copy(), cv2.COLOR_RGB2BGR)
    intersection = ((fx_mask > 0) & (sw_mask > 0)).astype(np.uint8)
    if np.any(intersection):
        tint = np.array([0, 200, 255], dtype=np.float32)
        canvas[intersection > 0] = (canvas[intersection > 0].astype(np.float32) * 0.55 + tint * 0.45).astype(
            np.uint8
        )
    _draw_mask_contour(canvas, fx_mask, (64, 255, 64), thickness=3)
    _draw_mask_contour(canvas, sw_mask, (64, 64, 255), thickness=2)

    img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    header = title or "Mask overlap after shift"
    draw.text(
        (8, 8),
        (
            f"{header}\n"
            f"IoU {metrics['iou_percent']:.1f}%  "
            f"FX {metrics['fx_coverage_percent']:.1f}%  "
            f"SWIR {metrics['sw_coverage_percent']:.1f}%\n"
            f"intersection {metrics['intersection_area_mm2']:.1f} mm²  "
            f"union {metrics['union_area_mm2']:.1f} mm²"
        ),
        fill=(255, 255, 0),
    )
    draw.text((8, img.height - 22), "green=FX  red=SWIR  cyan=overlap", fill=(255, 255, 0))
    return img


def draw_centroids(camera: LoadedCameraRgb, title: str) -> Image.Image:
    img = Image.fromarray(camera.rgb.copy())
    draw = ImageDraw.Draw(img)
    colors = [(255, 64, 64), (64, 255, 64), (64, 128, 255), (255, 200, 64)]
    for index, cen in enumerate(camera.centroids):
        color = colors[index % len(colors)]
        x, y = cen.cx_px, cen.cy_px
        r = 8
        draw.ellipse((x - r, y - r, x + r, y + r), outline=color, width=3)
        draw.line((x - 14, y, x + 14, y), fill=color, width=2)
        draw.line((x, y - 14, x, y + 14), fill=color, width=2)
        label = f"roi{cen.roi} ({cen.cx_mm:.1f},{cen.cy_mm:.1f})mm"
        draw.text((x + 12, y - 12), label, fill=color)
    draw.text((8, 8), title, fill=(255, 255, 0))
    return img


def save_centroid_table(centroids: list[ObjectCentroid]) -> list[dict]:
    rows = []
    for cen in sorted(centroids, key=lambda c: c.cx_mm):
        rows.append(
            {
                "roi": cen.roi,
                "label": cen.label,
                "score": round(cen.score, 4),
                "pixel_count": cen.pixel_count,
                "cx_px": round(cen.cx_px, 2),
                "cy_px": round(cen.cy_px, 2),
                "cx_mm": round(cen.cx_mm, 2),
                "cy_mm": round(cen.cy_mm, 2),
            }
        )
    return rows


def blend_rgb_overlay(fx_rgb: np.ndarray, sw_rgb: np.ndarray, alpha: float) -> np.ndarray:
    """Blend FX and SWIR RGB crops for QA overlays."""
    blended = fx_rgb.astype(np.float32) * (1.0 - alpha) + sw_rgb.astype(np.float32) * alpha
    return np.clip(blended, 0, 255).astype(np.uint8)
