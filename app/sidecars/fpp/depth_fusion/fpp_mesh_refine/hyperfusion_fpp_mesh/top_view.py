# Orthographic top-view depth raster for cleaned FPP clouds (sidecar / depth fusion).
# Uses the robot-pose/tray workspace frame; no hardware I/O.
"""Rasterize the topmost surface height above the sample stage."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc


@dataclass
class TopViewResult:
    depth_path: Path
    preview_path: Path
    width_px: int
    height_px: int
    resolution_mm: float
    valid_pixels: int
    min_height_mm: float
    max_height_mm: float


def rasterize_top_view(
    cloud: o3d.geometry.PointCloud,
    workspace: dict,
    out_dir: Path,
    *,
    resolution_m: float = 0.0005,
    name_prefix: str = "top_view",
) -> TopViewResult:
    """Write topmost stage-relative height in mm; invalid pixels are NaN."""
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.size == 0:
        raise ValueError("Cannot rasterize an empty point cloud")

    center = np.asarray(workspace["center_world_m"], dtype=np.float64)
    up = np.asarray(workspace["up_world"], dtype=np.float64)
    axis_u = np.asarray(workspace["axis_u_world"], dtype=np.float64)
    axis_v = np.asarray(workspace["axis_v_world"], dtype=np.float64)
    width_m = float(workspace["width_m"])
    depth_m = float(workspace["depth_m"])
    resolution_m = float(resolution_m)
    if resolution_m <= 0.0:
        raise ValueError("Top-view resolution must be positive")

    width_px = max(1, int(np.ceil(width_m / resolution_m)))
    height_px = width_px
    relative = points - center
    u = relative @ axis_u
    v = relative @ axis_v
    height = relative @ up
    half_width = 0.5 * width_m
    valid = (
        (u >= -half_width)
        & (u < half_width)
        & (v >= -half_width)
        & (v < half_width)
        & (height >= 0.0)
        & (height <= depth_m)
    )
    if not np.any(valid):
        raise ValueError("No cloud points fall inside the top-view workspace")

    columns = np.floor((u[valid] + half_width) / resolution_m).astype(np.int64)
    # Positive stage-v appears at the top of the image.
    rows = np.floor((half_width - v[valid]) / resolution_m).astype(np.int64)
    columns = np.clip(columns, 0, width_px - 1)
    rows = np.clip(rows, 0, height_px - 1)
    flat_index = rows * width_px + columns
    flat_height = np.full(width_px * height_px, -np.inf, dtype=np.float32)
    np.maximum.at(flat_height, flat_index, height[valid].astype(np.float32))
    depth_mm = flat_height.reshape(height_px, width_px) * np.float32(1000.0)
    depth_mm[~np.isfinite(depth_mm)] = np.nan

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    depth_path = out_dir / f"{name_prefix}_depth_mm.npy"
    preview_path = out_dir / f"{name_prefix}_depth.png"
    np.save(depth_path, depth_mm)

    finite = depth_mm[np.isfinite(depth_mm)]
    lo, hi = np.percentile(finite, (1.0, 99.0))
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        lo = float(np.min(finite))
        hi = float(np.max(finite)) + 1.0
    scaled = np.zeros(depth_mm.shape, dtype=np.uint8)
    finite_mask = np.isfinite(depth_mm)
    scaled[finite_mask] = np.clip(
        (depth_mm[finite_mask] - lo) * (255.0 / (hi - lo)),
        0.0,
        255.0,
    ).astype(np.uint8)
    preview = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    preview[~finite_mask] = 0
    if not cv2.imwrite(str(preview_path), preview):
        raise OSError(f"Could not write {preview_path}")

    return TopViewResult(
        depth_path=depth_path,
        preview_path=preview_path,
        width_px=width_px,
        height_px=height_px,
        resolution_mm=resolution_m * 1000.0,
        valid_pixels=int(finite.size),
        min_height_mm=float(np.min(finite)),
        max_height_mm=float(np.max(finite)),
    )
