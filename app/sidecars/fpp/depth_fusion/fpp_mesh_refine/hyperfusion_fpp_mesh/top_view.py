# Orthographic top-view distance raster for cleaned FPP clouds.
"""Rasterize the topmost distance from a fitted flat background surface."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .tray import _fit_upright_plane

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
    reference_plane: list[float]
    reference_plane_source: str


def rasterize_top_view(
    cloud: o3d.geometry.PointCloud,
    workspace: dict,
    out_dir: Path,
    *,
    resolution_m: float = 0.0005,
    canvas_size_px: int | None = 800,
    name_prefix: str = "top_view",
) -> TopViewResult:
    """Write topmost perpendicular distance from a flat reference plane.

    Invalid pixels are NaN. ``top_view_depth_mm.npy`` is retained as a
    compatibility alias for the explicit distance-from-plane map.
    """
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.size == 0:
        raise ValueError("Cannot rasterize an empty point cloud")

    center = np.asarray(workspace["center_world_m"], dtype=np.float64)
    workspace_up = np.asarray(workspace["up_world"], dtype=np.float64)
    workspace_up /= np.linalg.norm(workspace_up)
    axis_u = np.asarray(workspace["axis_u_world"], dtype=np.float64)
    workspace_axis_v = np.asarray(workspace["axis_v_world"], dtype=np.float64)
    width_m = float(workspace["width_m"])
    depth_m = float(workspace["depth_m"])
    resolution_m = float(resolution_m)
    if resolution_m <= 0.0:
        raise ValueError("Top-view resolution must be positive")

    plane_source = "workspace_plane"
    raw_plane = workspace.get("plane")
    if raw_plane is None:
        sample_stride = max(1, points.shape[0] // 250000)
        fitted, _upright, _reason = _fit_upright_plane(
            points[::sample_stride],
            plane_dist_m=0.0025,
            min_upright=float(np.cos(np.deg2rad(25.0))),
        )
        raw_plane = fitted
        plane_source = (
            "ransac_flat_background" if fitted is not None else "workspace_fallback"
        )

    if raw_plane is None:
        up = workspace_up
        plane = np.r_[up, -float(np.dot(up, center))]
    else:
        plane = np.asarray(raw_plane, dtype=np.float64).reshape(4)
        norm = float(np.linalg.norm(plane[:3]))
        if norm < 1.0e-12:
            raise ValueError("Reference plane has an invalid normal")
        plane /= norm
        signed = points @ plane[:3] + float(plane[3])
        away_from_plane = signed[np.abs(signed) > 0.005]
        if away_from_plane.size:
            # Orient the normal toward the observed object side. The flat
            # background can be either above or below the object in base_link.
            if float(np.median(away_from_plane)) < 0.0:
                plane *= -1.0
        elif float(np.dot(plane[:3], workspace_up)) < 0.0:
            plane *= -1.0
        up = plane[:3]
        # Put the raster origin on the fitted plane.
        center = center - (float(np.dot(up, center)) + float(plane[3])) * up

    axis_u = axis_u - float(np.dot(axis_u, up)) * up
    if float(np.linalg.norm(axis_u)) < 1.0e-9:
        axis_u = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        axis_u -= float(np.dot(axis_u, up)) * up
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(up, axis_u)
    axis_v /= np.linalg.norm(axis_v)
    if float(np.dot(axis_v, workspace_axis_v)) < 0.0:
        axis_v *= -1.0

    relative = points - center
    u = relative @ axis_u
    v = relative @ axis_v
    distance = points @ up + float(plane[3])
    if canvas_size_px is not None:
        # Fixed, object-centred output for stable downstream arrays.
        canvas_px = int(canvas_size_px)
        if canvas_px <= 0:
            raise ValueError("canvas_size_px must be positive")
        center = center + float(np.median(u)) * axis_u + float(np.median(v)) * axis_v
        relative = points - center
        u = relative @ axis_u
        v = relative @ axis_v
        width_px = canvas_px
        height_px = canvas_px
        width_m = float(canvas_px) * resolution_m
        half_width = 0.5 * width_m
    else:
        half_width = max(float(np.max(np.abs(u))), float(np.max(np.abs(v)))) + resolution_m
        width_m = 2.0 * half_width
        width_px = max(1, int(np.ceil(width_m / resolution_m)))
        height_px = width_px
    valid = (
        (u >= -half_width)
        & (u < half_width)
        & (v >= -half_width)
        & (v < half_width)
        & (distance >= 0.0)
    )
    if not np.any(valid):
        raise ValueError("No cloud points fall above the flat reference plane")

    columns = np.floor((u[valid] + half_width) / resolution_m).astype(np.int64)
    # Vertical flip from the legacy map: positive stage-v now points down.
    rows = np.floor((v[valid] + half_width) / resolution_m).astype(np.int64)
    columns = np.clip(columns, 0, width_px - 1)
    rows = np.clip(rows, 0, height_px - 1)
    flat_index = rows * width_px + columns
    flat_height = np.full(width_px * height_px, -np.inf, dtype=np.float32)
    np.maximum.at(flat_height, flat_index, distance[valid].astype(np.float32))
    distance_mm = flat_height.reshape(height_px, width_px) * np.float32(1000.0)
    distance_mm[~np.isfinite(distance_mm)] = np.nan

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    depth_path = out_dir / f"{name_prefix}_depth_mm.npy"
    preview_path = out_dir / f"{name_prefix}_depth.png"
    np.save(depth_path, distance_mm)

    finite = distance_mm[np.isfinite(distance_mm)]
    lo, hi = np.percentile(finite, (1.0, 99.0))
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        lo = float(np.min(finite))
        hi = float(np.max(finite)) + 1.0
    scaled = np.zeros(distance_mm.shape, dtype=np.uint8)
    finite_mask = np.isfinite(distance_mm)
    scaled[finite_mask] = np.clip(
        (distance_mm[finite_mask] - lo) * (255.0 / (hi - lo)),
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
        reference_plane=plane.tolist(),
        reference_plane_source=plane_source,
    )

def remove_flat_background(
    cloud: o3d.geometry.PointCloud,
    workspace: dict,
    *,
    distance_m: float = 0.003,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """Remove the dominant near-horizontal tray/background plane from a cloud."""
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.shape[0] < 200:
        return cloud, {"skipped": True, "reason": "too_few_points", "n_in": int(points.shape[0]), "n_out": int(points.shape[0])}
    up = np.asarray(workspace.get("up_world", [0.0, 0.0, 1.0]), dtype=np.float64)
    up /= np.linalg.norm(up)
    sample_stride = max(1, points.shape[0] // 250000)
    plane, upright, reason = _fit_upright_plane(
        points[::sample_stride],
        plane_dist_m=float(distance_m),
        min_upright=float(np.cos(np.deg2rad(25.0))),
    )
    if plane is None:
        return cloud, {
            "skipped": True,
            "reason": reason or "no_flat_plane",
            "upright": upright,
            "n_in": int(points.shape[0]),
            "n_out": int(points.shape[0]),
        }
    plane = np.asarray(plane, dtype=np.float64)
    plane /= np.linalg.norm(plane[:3])
    signed = points @ plane[:3] + float(plane[3])
    remove = np.abs(signed) <= float(distance_m)
    cleaned = cloud.select_by_index(np.flatnonzero(~remove).tolist())
    return cleaned, {
        "skipped": False,
        "plane": plane.tolist(),
        "upright": float(abs(np.dot(plane[:3], up))),
        "distance_mm": float(distance_m) * 1000.0,
        "n_in": int(points.shape[0]),
        "n_removed": int(remove.sum()),
        "n_out": int(len(cleaned.points)),
    }