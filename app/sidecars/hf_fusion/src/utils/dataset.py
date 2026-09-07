# Session paths and ENVI/GSAM asset loading for fusion alignment.
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image


@dataclass
class ObjectCentroid:
    roi: int
    label: str
    score: float
    pixel_count: int
    cx_px: float
    cy_px: float
    cx_mm: float
    cy_mm: float
    mask_path: Path


@dataclass
class LoadedCameraRgb:
    camera: str
    rgb: np.ndarray
    width: int
    height: int
    mm_per_pixel: float
    centroids: list[ObjectCentroid]


def resolve_rgb_path(session_dir: Path, mode: str, camera: str) -> Path:
    pre = session_dir / mode / camera / "preprocessed"
    matches = sorted(pre.glob("*_rgb.png"))
    if not matches:
        raise FileNotFoundError(f"No RGB PNG under {pre}")
    return matches[0]


def load_segmentation(session_dir: Path, mode: str, camera: str) -> dict:
    path = session_dir / mode / camera / "preprocessed" / "segmentation" / "segmentation_results.json"
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def centroid_from_mask(mask: np.ndarray) -> tuple[float, float]:
    ys, xs = np.nonzero(mask > 0)
    if xs.size == 0:
        raise ValueError("Empty mask")
    return float(xs.mean()), float(ys.mean())


def resolve_mask_path(session_dir: Path, mode: str, camera: str, roi: int, det: dict | None = None) -> Path:
    if det is not None:
        mask_file = Path(det["mask_npy"])
        if mask_file.is_file():
            return mask_file
    return (
        session_dir
        / mode
        / camera
        / "preprocessed"
        / "segmentation"
        / "masks"
        / f"mask_{roi:03d}.npy"
    )


def load_rgb_array(session_dir: Path, mode: str, camera: str) -> tuple[np.ndarray, int, int]:
    rgb_path = resolve_rgb_path(session_dir, mode, camera)
    image = Image.open(rgb_path).convert("RGB")
    rgb = np.asarray(image)
    height, width = rgb.shape[:2]
    return rgb, width, height


def centroids_from_masks(
    session_dir: Path,
    mode: str,
    camera: str,
    mm_per_pixel: float,
    masks_by_roi: dict[int, np.ndarray],
) -> list[ObjectCentroid]:
    seg = load_segmentation(session_dir, mode, camera)
    centroids: list[ObjectCentroid] = []
    for det in seg.get("detections", []):
        roi = int(det["roi"])
        if roi not in masks_by_roi:
            raise KeyError(f"Missing mask for roi {roi} ({camera})")
        mask = masks_by_roi[roi]
        cx_px, cy_px = centroid_from_mask(mask)
        mask_path = resolve_mask_path(session_dir, mode, camera, roi, det)
        centroids.append(
            ObjectCentroid(
                roi=roi,
                label=str(det.get("label", "")),
                score=float(det.get("score", 0.0)),
                pixel_count=int(np.count_nonzero(mask > 0)),
                cx_px=cx_px,
                cy_px=cy_px,
                cx_mm=cx_px * mm_per_pixel,
                cy_mm=cy_px * mm_per_pixel,
                mask_path=mask_path,
            )
        )
    return centroids


def load_camera_rgb(session_dir: Path, mode: str, camera: str, mm_per_pixel: float) -> LoadedCameraRgb:
    rgb_path = resolve_rgb_path(session_dir, mode, camera)
    seg = load_segmentation(session_dir, mode, camera)
    image = Image.open(rgb_path).convert("RGB")
    rgb = np.asarray(image)
    height, width = rgb.shape[:2]

    centroids: list[ObjectCentroid] = []
    for det in seg.get("detections", []):
        roi = int(det["roi"])
        mask_file = resolve_mask_path(session_dir, mode, camera, roi, det)
        mask = np.load(mask_file)
        cx_px, cy_px = centroid_from_mask(mask)
        centroids.append(
            ObjectCentroid(
                roi=int(det["roi"]),
                label=str(det.get("label", "")),
                score=float(det.get("score", 0.0)),
                pixel_count=int(det.get("pixel_count", 0)),
                cx_px=cx_px,
                cy_px=cy_px,
                cx_mm=cx_px * mm_per_pixel,
                cy_mm=cy_px * mm_per_pixel,
                mask_path=mask_file,
            )
        )

    return LoadedCameraRgb(
        camera=camera,
        rgb=rgb,
        width=width,
        height=height,
        mm_per_pixel=mm_per_pixel,
        centroids=centroids,
    )


def fusion_dir(session_dir: Path, mode: str) -> Path:
    """Top-level fusion output directory for a capture session."""
    out = session_dir / mode / "fusion"
    out.mkdir(parents=True, exist_ok=True)
    return out


def fusion_metadata_dir(session_dir: Path, mode: str) -> Path:
    """Session-level fusion metadata (alignment.json, centroid QA, …)."""
    out = fusion_dir(session_dir, mode) / "metadata"
    out.mkdir(parents=True, exist_ok=True)
    return out


def load_fx_segmentation_mask(session: Path, mode: str, roi: int) -> np.ndarray:
    """Load a single FX10e chip segmentation mask."""
    path = session / mode / "fx10e" / "preprocessed" / "segmentation" / "masks" / f"mask_{roi:03d}.npy"
    return (np.load(path) > 0).astype(np.uint8)


def upsample_swir_to_fx_grid(
    session: Path,
    mode: str,
    cal: Any,
    rgb_method: Any,
) -> tuple[np.ndarray, int, int, dict[int, np.ndarray]]:
    """Upsample SWIR RGB and segmentation masks to the FX10e ground grid."""
    from src.utils.resample import target_size_px, upsample_mask, upsample_rgb

    sw_rgb_native, native_w, native_h = load_rgb_array(session, mode, "swir3")
    target_w, target_h = target_size_px(native_w, native_h, cal.swir3_mm_per_pixel, cal.fx10e_mm_per_pixel)
    sw_rgb_up = upsample_rgb(sw_rgb_native, target_w, target_h, method=rgb_method)

    seg = load_segmentation(session, mode, "swir3")
    masks_up: dict[int, np.ndarray] = {}
    for det in seg.get("detections", []):
        roi = int(det["roi"])
        mask_native = np.load(resolve_mask_path(session, mode, "swir3", roi, det))
        masks_up[roi] = upsample_mask(mask_native, target_w, target_h)

    return sw_rgb_up, target_w, target_h, masks_up
