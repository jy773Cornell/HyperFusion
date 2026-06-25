# Visualization helpers for fusion alignment QA.
from __future__ import annotations

import cv2
import numpy as np
from PIL import Image, ImageDraw

from hf_fusion.dataset import LoadedCameraRgb, ObjectCentroid

_PAIR_COLORS_BGR = [
    (64, 64, 255),
    (64, 255, 64),
    (255, 128, 64),
    (64, 200, 255),
]


def _draw_mask_contour(image_bgr: np.ndarray, mask: np.ndarray, color_bgr: tuple[int, int, int], thickness: int) -> None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(image_bgr, contours, -1, color_bgr, thickness, lineType=cv2.LINE_AA)


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


def draw_matched_shift_qa(
    fx_rgb: np.ndarray,
    pairs: list,
    shift_dx_px: float,
    shift_dy_px: float,
    fx_mm_per_pixel: float,
    title: str,
    fx_masks_by_roi: dict[int, np.ndarray] | None = None,
    sw_shifted_masks_by_roi: dict[int, np.ndarray] | None = None,
) -> Image.Image:
    """Draw FX/SWIR centroids and optional mask outlines after shift on the FX reference grid."""
    canvas = cv2.cvtColor(fx_rgb.copy(), cv2.COLOR_RGB2BGR)

    if fx_masks_by_roi and sw_shifted_masks_by_roi:
        for index, pair in enumerate(pairs):
            color = _PAIR_COLORS_BGR[index % len(_PAIR_COLORS_BGR)]
            fx_mask = fx_masks_by_roi.get(pair.fx_roi)
            sw_mask = sw_shifted_masks_by_roi.get(pair.sw_roi)
            if fx_mask is not None:
                _draw_mask_contour(canvas, fx_mask, color, thickness=3)
            if sw_mask is not None:
                _draw_mask_contour(canvas, sw_mask, color, thickness=1)

    img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    colors_rgb = [(255, 64, 64), (64, 255, 64), (64, 128, 255), (255, 200, 64)]
    for index, pair in enumerate(pairs):
        color = colors_rgb[index % len(colors_rgb)]
        fx_x = pair.fx_mm[0] / fx_mm_per_pixel
        fx_y = pair.fx_mm[1] / fx_mm_per_pixel
        sw_x = pair.sw_mm[0] / fx_mm_per_pixel
        sw_y = pair.sw_mm[1] / fx_mm_per_pixel
        sw_shifted_x = sw_x + shift_dx_px
        sw_shifted_y = sw_y + shift_dy_px

        draw.line((sw_x, sw_y, sw_shifted_x, sw_shifted_y), fill=(180, 180, 180), width=2)
        draw.ellipse((sw_x - 5, sw_y - 5, sw_x + 5, sw_y + 5), outline=(180, 180, 180), width=2)
        draw.ellipse((sw_shifted_x - 5, sw_shifted_y - 5, sw_shifted_x + 5, sw_shifted_y + 5), outline=color, width=2)
        draw.ellipse((fx_x - 7, fx_y - 7, fx_x + 7, fx_y + 7), outline=color, width=3)
        draw.line((fx_x - 10, fx_y, fx_x + 10, fx_y), fill=color, width=2)
        draw.line((fx_x, fx_y - 10, fx_x, fx_y + 10), fill=color, width=2)
        label = f"#{pair.pair_index} fx{pair.fx_roi}<->sw{pair.sw_roi}"
        draw.text((fx_x + 10, fx_y - 14), label, fill=color)

    legend = "thick outline=FX  thin outline=SWIR after shift"
    if fx_masks_by_roi is None:
        legend = "gray=SWIR centroid  color=SWIR+shift vs FX cross"
    draw.text(
        (8, 8),
        f"{title}\nshift=({shift_dx_px:.1f},{shift_dy_px:.1f})px {legend}",
        fill=(255, 255, 0),
    )
    return img
