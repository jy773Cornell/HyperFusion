# Chessboard corners on an FPP white frame (sidecar / offline).
# Same 10×7 board as bfs_cal/board.yaml (square_size_mm). No robot motion.
"""Detect inner corners and sample projector (u, v) at those pixels."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from .decode import DecodeResult


def load_board(path: Path | str) -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    cols = int(data["inner_corners_x"])
    rows = int(data["inner_corners_y"])
    square_m = float(data["square_size_mm"]) * 0.001
    data["pattern_size"] = (cols, rows)
    data["square_m"] = square_m
    data["n_corners"] = cols * rows
    return data


def object_points_for(board: dict[str, Any], pattern: tuple[int, int]) -> np.ndarray:
    cols, rows = pattern
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= float(board["square_m"])
    return obj


def gray_u8(image: np.ndarray) -> np.ndarray:
    arr = np.asarray(image)
    if arr.ndim == 3:
        arr = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
    if arr.dtype != np.uint8:
        mx = float(arr.max()) if arr.size else 1.0
        if mx > 1.5:
            arr = np.clip(arr, 0, 255).astype(np.uint8)
        else:
            arr = np.clip(arr * 255.0, 0, 255).astype(np.uint8)
    return arr


def detect_corners(gray: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    scale = 4 if min(gray.shape[:2]) >= 2000 else 1
    search = gray
    if scale > 1:
        search = cv2.resize(
            gray,
            (gray.shape[1] // scale, gray.shape[0] // scale),
            interpolation=cv2.INTER_AREA,
        )
    ok, corners = cv2.findChessboardCorners(search, pattern, flags)
    if not ok and scale > 1:
        ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
        scale = 1
    if not ok and hasattr(cv2, "findChessboardCornersSB"):
        sb = cv2.findChessboardCornersSB(
            gray,
            pattern,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )
        if sb[0]:
            return sb[1].astype(np.float32)
        return None
    if not ok:
        return None
    corners = corners.astype(np.float32)
    if scale > 1:
        corners *= float(scale)
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)


def detect_with_swap(gray: np.ndarray, pattern: tuple[int, int]) -> tuple[tuple[int, int], np.ndarray] | None:
    for candidate in (pattern, (pattern[1], pattern[0])):
        corners = detect_corners(gray, candidate)
        if corners is not None and len(corners) == candidate[0] * candidate[1]:
            return candidate, corners
    return None


def detect_on_white(
    white: np.ndarray,
    black: np.ndarray | None,
    pattern: tuple[int, int],
) -> tuple[tuple[int, int], np.ndarray] | None:
    """Try the white frame, then white−black (uneven DLP light)."""
    found = detect_with_swap(gray_u8(white), pattern)
    if found is not None:
        return found
    if black is None:
        return None
    w = np.asarray(white, dtype=np.float32)
    b = np.asarray(black, dtype=np.float32)
    if w.max() > 1.5:
        w = w / 255.0
        b = b / 255.0
    diff = np.clip(w - b, 0.0, 1.0)
    peak = float(diff.max()) if diff.size else 0.0
    if peak < 1.0e-6:
        return None
    return detect_with_swap((diff / peak * 255.0).astype(np.uint8), pattern)


def sample_map(values: np.ndarray, xy: np.ndarray) -> np.ndarray:
    """Bilinear sample *values* at camera pixels xy (N,2) = (u, v)."""
    h, w = values.shape[:2]
    x = np.asarray(xy[:, 0], dtype=np.float64)
    y = np.asarray(xy[:, 1], dtype=np.float64)
    x0 = np.floor(x).astype(np.int32)
    y0 = np.floor(y).astype(np.int32)
    x1 = x0 + 1
    y1 = y0 + 1
    out = np.full(x.shape, np.nan, dtype=np.float64)
    ok = (x0 >= 0) & (y0 >= 0) & (x1 < w) & (y1 < h)
    if not np.any(ok):
        return out
    xa = x[ok] - x0[ok]
    ya = y[ok] - y0[ok]
    v00 = values[y0[ok], x0[ok]].astype(np.float64)
    v01 = values[y0[ok], x1[ok]].astype(np.float64)
    v10 = values[y1[ok], x0[ok]].astype(np.float64)
    v11 = values[y1[ok], x1[ok]].astype(np.float64)
    out[ok] = (
        v00 * (1.0 - xa) * (1.0 - ya)
        + v01 * xa * (1.0 - ya)
        + v10 * (1.0 - xa) * ya
        + v11 * xa * ya
    )
    return out


def corners_in_patch(
    decoded: DecodeResult,
    corners: np.ndarray,
    *,
    neighborhood: int = 2,
) -> np.ndarray:
    """True if that corner and a small window sit on the lit mask with finite u,v."""
    xy = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    mask = decoded.mask
    u = decoded.projector_u
    v = decoded.projector_v
    h, w = mask.shape
    ok = np.zeros(xy.shape[0], dtype=bool)
    rad = int(max(0, neighborhood))
    for i, (cx, cy) in enumerate(xy):
        col = int(round(cx))
        row = int(round(cy))
        if col < 0 or row < 0 or col >= w or row >= h:
            continue
        r0, r1 = max(0, row - rad), min(h, row + rad + 1)
        c0, c1 = max(0, col - rad), min(w, col + rad + 1)
        patch = mask[r0:r1, c0:c1]
        if patch.size == 0 or not bool(patch.all()):
            continue
        uu = sample_map(u, np.array([[cx, cy]], dtype=np.float64))[0]
        vv = sample_map(v, np.array([[cx, cy]], dtype=np.float64))[0]
        ok[i] = np.isfinite(uu) and np.isfinite(vv)
    return ok
