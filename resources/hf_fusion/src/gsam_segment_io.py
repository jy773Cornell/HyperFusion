# GSAM mask NMS, z-order, overlay, and segmentation rewrite (backend/offline).
from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from src.gsam_client import win_to_wsl
from src.utils.roi_spectra import spectrum_y_axis_label, write_segmentation_roi_spectra


def mask_iou(a: np.ndarray, b: np.ndarray) -> float:
    aa = a > 0
    bb = b > 0
    inter = int(np.logical_and(aa, bb).sum())
    union = int(np.logical_or(aa, bb).sum())
    return float(inter) / float(union) if union else 0.0


def nms_keep_indices(masks: list[np.ndarray], scores: list[float], iou_thresh: float) -> list[int]:
    order = sorted(range(len(masks)), key=lambda i: scores[i], reverse=True)
    keep: list[int] = []
    for idx in order:
        if all(mask_iou(masks[idx], masks[k]) < iou_thresh for k in keep):
            keep.append(idx)
    return keep


def z_order_sort_indices(boxes_xyxy: np.ndarray) -> list[int]:
    # Right-to-left within rows, top-to-bottom across rows (tray z-order).
    count = int(boxes_xyxy.shape[0])
    if count <= 1:
        return list(range(count))
    x_tl = boxes_xyxy[:, 0].tolist()
    y_tl = boxes_xyxy[:, 1].tolist()
    heights = np.maximum(1.0, boxes_xyxy[:, 3] - boxes_xyxy[:, 1])
    row_tol = max(10.0, 0.5 * float(np.median(heights)))
    indices_by_y = sorted(range(count), key=lambda i: y_tl[i])
    rows: list[list[int]] = []
    current_row: list[int] = []
    current_row_y: float | None = None
    for idx in indices_by_y:
        y = y_tl[idx]
        if current_row_y is None:
            current_row = [idx]
            current_row_y = y
            continue
        if abs(y - current_row_y) <= row_tol:
            current_row.append(idx)
            current_row_y = float(np.mean([y_tl[j] for j in current_row]))
        else:
            rows.append(current_row)
            current_row = [idx]
            current_row_y = y
    if current_row:
        rows.append(current_row)
    for row in rows:
        row.sort(key=lambda i: x_tl[i], reverse=True)
    ordered: list[int] = []
    for row in rows:
        ordered.extend(row)
    return ordered


def mask_bbox_xyxy(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.nonzero(mask > 0)
    if ys.size == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def write_detection_overlay(
    rgb: np.ndarray,
    masks: list[np.ndarray],
    scores: list[float],
    labels: list[str],
    path: Path,
) -> None:
    palette = [
        (255, 64, 64),
        (64, 220, 64),
        (64, 128, 255),
        (255, 200, 32),
        (255, 64, 220),
        (32, 220, 220),
    ]
    tint = np.zeros_like(rgb, dtype=np.float32)
    weight = np.zeros(rgb.shape[:2], dtype=np.float32)
    for roi, mask in enumerate(masks):
        binary = mask > 0
        if not binary.any():
            continue
        color = np.array(palette[roi % len(palette)], dtype=np.float32)
        tint[binary] += color
        weight[binary] += 1.0
    used = weight > 0
    blended = rgb.astype(np.float32)
    blended[used] = blended[used] * 0.55 + (tint[used] / weight[used, None]) * 0.45
    canvas = Image.fromarray(np.clip(blended, 0, 255).astype(np.uint8))
    draw = ImageDraw.Draw(canvas)
    for roi, mask in enumerate(masks, start=1):
        ys, xs = np.nonzero(mask > 0)
        if ys.size == 0:
            continue
        x0, y0 = int(xs.min()), int(ys.min())
        x1, y1 = int(xs.max()), int(ys.max())
        color = palette[(roi - 1) % len(palette)]
        draw.rectangle([x0, y0, x1, y1], outline=color, width=2)
        cx, cy = int(xs.mean()), int(ys.mean())
        draw.text((cx - 8, cy - 6), f"{roi:02d}", fill=(255, 255, 255))
    _ = scores, labels
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path)


def write_roi_csv(
    seg_dir: Path, image_name: str, cube: np.ndarray, wavelengths: np.ndarray, detections: list
) -> None:
    mode_name = seg_dir.parent.parent.parent.name
    write_segmentation_roi_spectra(
        seg_dir,
        image_name=image_name,
        cube=cube,
        wavelengths=wavelengths,
        detections=detections,
        y_axis_label=spectrum_y_axis_label(mode_name),
        copy_csv_to_preprocessed=False,
    )


def rewrite_segmentation_from_masks(
    seg_dir: Path,
    rgb: np.ndarray,
    masks: list[np.ndarray],
    scores: list[float],
    labels: list[str],
    image_name: str,
    prompt: str,
    cube: np.ndarray,
    wavelengths: np.ndarray,
) -> None:
    height, width = rgb.shape[:2]
    masks_dir = seg_dir / "masks"
    rgb_dir = seg_dir / "segmented_rgb"
    if masks_dir.exists():
        shutil.rmtree(masks_dir)
    if rgb_dir.exists():
        shutil.rmtree(rgb_dir)
    masks_dir.mkdir(parents=True)
    rgb_dir.mkdir(parents=True)

    stack = np.zeros((height, width), dtype=np.uint8)
    detections = []
    for roi, (mask, score, label) in enumerate(zip(masks, scores, labels), start=1):
        binary = (mask > 0).astype(np.uint8)
        np.save(masks_dir / f"mask_{roi:03d}.npy", binary)
        Image.fromarray(binary * 255, mode="L").save(masks_dir / f"mask_{roi:03d}.png")
        stack = np.maximum(stack, binary * 255)
        ys, xs = np.nonzero(binary)
        if ys.size:
            bounds = {
                "x": int(xs.min()),
                "y": int(ys.min()),
                "width": int(xs.max() - xs.min() + 1),
                "height": int(ys.max() - ys.min() + 1),
            }
            crop = rgb[bounds["y"] : bounds["y"] + bounds["height"], bounds["x"] : bounds["x"] + bounds["width"]]
            alpha = binary[bounds["y"] : bounds["y"] + bounds["height"], bounds["x"] : bounds["x"] + bounds["width"]] * 255
            rgba = np.dstack((crop, alpha.astype(np.uint8)))
            Image.fromarray(rgba, mode="RGBA").save(rgb_dir / f"roi_{roi:03d}.png")
        else:
            bounds = {"x": 0, "y": 0, "width": 0, "height": 0}
        npy_path = masks_dir / f"mask_{roi:03d}.npy"
        png_path = masks_dir / f"mask_{roi:03d}.png"
        detections.append(
            {
                "roi": roi,
                "label": label,
                "score": float(score),
                "pixel_count": int(binary.sum()),
                "mask_png": win_to_wsl(png_path),
                "mask_npy": win_to_wsl(npy_path),
                "segmented_rgb_png": win_to_wsl(rgb_dir / f"roi_{roi:03d}.png"),
                "segmented_rgb_bounds": bounds,
            }
        )
    Image.fromarray(stack, mode="L").save(masks_dir / "stack_mask.png")
    write_detection_overlay(rgb, [m > 0 for m in masks], scores, labels, seg_dir / "overlay.png")
    man = {
        "ok": True,
        "image": image_name,
        "prompt": prompt,
        "detection_count": len(detections),
        "detections": detections,
    }
    (seg_dir / "segmentation_results.json").write_text(json.dumps(man, indent=2), encoding="utf-8")
    write_roi_csv(seg_dir, image_name, cube, wavelengths, detections)
