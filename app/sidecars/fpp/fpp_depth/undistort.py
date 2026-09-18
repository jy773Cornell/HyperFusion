"""Remap an FPP burst into a pinhole grid (adapter).

Uses BFS K/D from the still JSON. Same newK convention as calibrate_bfs
(getOptimalNewCameraMatrix alpha=0). No hardware I/O.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .capture import FppBurst, camera_intrinsics


@dataclass
class UndistortMaps:
    camera_k: np.ndarray
    camera_d: np.ndarray
    new_k: np.ndarray
    map_x: np.ndarray
    map_y: np.ndarray
    roi_xywh: tuple[int, int, int, int]


def build_undistort_maps(
    meta: dict,
    size_wh: tuple[int, int],
    *,
    alpha: float = 0.0,
) -> UndistortMaps | None:
    k_dist = camera_intrinsics(meta)
    if k_dist is None:
        return None
    k, dist = k_dist
    if dist.size < 4 or not np.any(np.abs(dist) > 1.0e-12):
        return None
    w, h = int(size_wh[0]), int(size_wh[1])
    new_k, roi = cv2.getOptimalNewCameraMatrix(k, dist, (w, h), float(alpha))
    map_x, map_y = cv2.initUndistortRectifyMap(k, dist, None, new_k, (w, h), cv2.CV_32FC1)
    x, y, rw, rh = [int(v) for v in roi]
    return UndistortMaps(
        camera_k=np.asarray(k, dtype=np.float64),
        camera_d=np.asarray(dist, dtype=np.float64),
        new_k=np.asarray(new_k, dtype=np.float64),
        map_x=map_x,
        map_y=map_y,
        roi_xywh=(x, y, rw, rh),
    )


def remap_image(image: np.ndarray, maps: UndistortMaps) -> np.ndarray:
    return cv2.remap(
        image.astype(np.float32, copy=False),
        maps.map_x,
        maps.map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0.0,
    )


def write_new_k_into_meta(meta: dict, new_k: np.ndarray) -> None:
    """Point later K reads at the undistorted pinhole (D cleared)."""
    inner = meta.get("intrinsics") if isinstance(meta.get("intrinsics"), dict) else meta
    inner["fx"] = float(new_k[0, 0])
    inner["fy"] = float(new_k[1, 1])
    inner["cx"] = float(new_k[0, 2])
    inner["cy"] = float(new_k[1, 2])
    inner["distortion"] = []


def undistort_burst(burst: FppBurst, *, alpha: float = 0.0) -> UndistortMaps | None:
    """Remap every frame in place. Returns None if JSON has no usable D."""
    first = burst.image(0)
    if first.size == 0:
        return None
    h, w = first.shape[:2]
    maps = build_undistort_maps(burst.first_meta(), (w, h), alpha=alpha)
    if maps is None:
        return None
    for frame in burst.frames:
        frame.image = remap_image(frame.image, maps)
        if frame.meta:
            write_new_k_into_meta(frame.meta, maps.new_k)
    return maps
