# Session paths and ENVI/GSAM asset loading for fusion alignment.
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image


@dataclass(frozen=True)
class CameraStreamPaths:
    camera: str
    preprocessed: Path
    rgb_png: Path
    segmentation_json: Path
    masks_dir: Path


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


def stream_paths(session_dir: Path, mode: str, camera: str) -> CameraStreamPaths:
    preprocessed = session_dir / mode / camera / "preprocessed"
    return CameraStreamPaths(
        camera=camera,
        preprocessed=preprocessed,
        rgb_png=preprocessed / f"{session_dir.name}_rgb.png".replace(session_dir.name, _dataset_stem(session_dir, mode, camera)),
        segmentation_json=preprocessed / "segmentation" / "segmentation_results.json",
        masks_dir=preprocessed / "segmentation" / "masks",
    )


def _dataset_stem(session_dir: Path, mode: str, camera: str) -> str:
    pre = session_dir / mode / camera / "preprocessed"
    for path in sorted(pre.glob("*_rgb.png")):
        return path.stem.replace("_rgb", "")
    return session_dir.name


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


def load_camera_rgb(session_dir: Path, mode: str, camera: str, mm_per_pixel: float) -> LoadedCameraRgb:
    rgb_path = resolve_rgb_path(session_dir, mode, camera)
    seg = load_segmentation(session_dir, mode, camera)
    image = Image.open(rgb_path).convert("RGB")
    rgb = np.asarray(image)
    height, width = rgb.shape[:2]

    centroids: list[ObjectCentroid] = []
    for det in seg.get("detections", []):
        mask_file = Path(det["mask_npy"])
        if not mask_file.is_file():
            mask_file = session_dir / mode / camera / "preprocessed" / "segmentation" / "masks" / f"mask_{det['roi']:03d}.npy"
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


def fusion_step_dir(session_dir: Path, mode: str, step: str) -> Path:
    out = session_dir / mode / "fusion" / step
    out.mkdir(parents=True, exist_ok=True)
    return out
