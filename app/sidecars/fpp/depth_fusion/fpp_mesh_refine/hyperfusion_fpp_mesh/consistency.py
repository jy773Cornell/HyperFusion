# Multi-view depth consistency (sidecar / depth_fusion / fpp_mesh_refine).
# Project depth into neighbor views; reject outliers. No robot I/O.
"""Cross-view depth consistency / outlier rejection."""

from __future__ import annotations

import numpy as np

from .frames import DepthFrame


def _project_world_to_depth(
    xyz_w: np.ndarray,
    fr: DepthFrame,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (u, v, z_cam) for world points in fr's camera."""
    w2c = np.linalg.inv(fr.c2w)
    hom = np.hstack([xyz_w, np.ones((xyz_w.shape[0], 1), dtype=np.float64)])
    cam = (w2c @ hom.T).T[:, :3]
    z = cam[:, 2]
    fx, fy = fr.K[0, 0], fr.K[1, 1]
    cx, cy = fr.K[0, 2], fr.K[1, 2]
    u = fx * (cam[:, 0] / np.maximum(z, 1e-8)) + cx
    v = fy * (cam[:, 1] / np.maximum(z, 1e-8)) + cy
    return u, v, z


def _sample_depth(depth: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    h, w = depth.shape
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    out = np.full(u.shape, np.nan, dtype=np.float32)
    ok = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    out[ok] = depth[vi[ok], ui[ok]]
    return out


def backproject(fr: DepthFrame, *, stride: int = 2, conf_min: float = 0.1) -> np.ndarray:
    ys, xs = np.where(np.isfinite(fr.depth_m))
    if fr.conf is not None:
        keep = fr.conf[ys, xs] >= conf_min
        ys, xs = ys[keep], xs[keep]
    if stride > 1:
        ys, xs = ys[::stride], xs[::stride]
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    z = fr.depth_m[ys, xs].astype(np.float64)
    fx, fy = fr.K[0, 0], fr.K[1, 1]
    cx, cy = fr.K[0, 2], fr.K[1, 2]
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    cam = np.stack([x, y, z], axis=1)
    return (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]


def consistency_reject(
    frames: list[DepthFrame],
    *,
    max_dz_m: float = 0.006,
    min_views: int = 2,
    sample_stride: int = 3,
) -> dict:
    """Keep a pixel only if ≥ min_views other frames agree within max_dz_m.

    For sparse 6-view fruit, min_views=1 is often safer (at least one agree).
    """
    n = len(frames)
    stats = {"checked": 0, "kept": 0, "killed": 0}
    if n < 2:
        return stats

    # Precompute nothing heavy — per frame, sample its pixels and test against others
    for i, fr in enumerate(frames):
        ys, xs = np.where(np.isfinite(fr.depth_m))
        if sample_stride > 1:
            ys, xs = ys[::sample_stride], xs[::sample_stride]
        if xs.size == 0:
            continue
        z = fr.depth_m[ys, xs].astype(np.float64)
        fx, fy = fr.K[0, 0], fr.K[1, 1]
        cx, cy = fr.K[0, 2], fr.K[1, 2]
        x = (xs.astype(np.float64) - cx) * z / fx
        y = (ys.astype(np.float64) - cy) * z / fy
        cam = np.stack([x, y, z], axis=1)
        world = (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]

        agree = np.zeros(xs.shape[0], dtype=np.int32)
        for j, other in enumerate(frames):
            if j == i:
                continue
            u, v, z_pred = _project_world_to_depth(world, other)
            z_obs = _sample_depth(other.depth_m, u, v)
            ok = np.isfinite(z_obs) & (z_pred > 0.05) & (np.abs(z_obs - z_pred) <= float(max_dz_m))
            agree += ok.astype(np.int32)

        keep = agree >= int(min_views)
        stats["checked"] += int(xs.size)
        stats["kept"] += int(keep.sum())
        stats["killed"] += int((~keep).sum())

        # Kill inconsistent (full-res: also kill neighborhood via marking those pixels)
        fr.depth_m = fr.depth_m.copy()
        fr.depth_m[ys[~keep], xs[~keep]] = np.nan
        if fr.conf is not None:
            fr.conf = fr.conf.copy()
            fr.conf[ys[~keep], xs[~keep]] = 0.0
        fr.mask = np.isfinite(fr.depth_m)

    return stats
