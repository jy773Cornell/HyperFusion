# Tray plane crop (sidecar / depth_fusion / fpp_mesh_refine).
# Optional RANSAC removal of flat background before fuse. No robot I/O.
"""Remove dominant tray plane from depth frames."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .frames import DepthFrame

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc


@dataclass
class TrayStats:
    n_before: int
    n_after: int
    n_removed: int
    plane: list[float] | None


def remove_tray(
    frames: list[DepthFrame],
    *,
    plane_dist_m: float = 0.008,
    keep_band_m: float = 0.012,
) -> TrayStats:
    chunks: list[np.ndarray] = []
    for fr in frames:
        ys, xs = np.where(np.isfinite(fr.depth_m))
        if xs.size == 0:
            continue
        ys, xs = ys[::4], xs[::4]
        z = fr.depth_m[ys, xs].astype(np.float64)
        fx, fy = fr.K[0, 0], fr.K[1, 1]
        cx, cy = fr.K[0, 2], fr.K[1, 2]
        x = (xs.astype(np.float64) - cx) * z / fx
        y = (ys.astype(np.float64) - cy) * z / fy
        cam = np.stack([x, y, z], axis=1)
        world = (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]
        chunks.append(world)
    n_before = int(sum(np.count_nonzero(np.isfinite(f.depth_m)) for f in frames))
    if not chunks:
        return TrayStats(n_before, n_before, 0, None)

    pts = np.concatenate(chunks, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    o3d.utility.random.seed(0)
    model, _inl = pcd.segment_plane(
        distance_threshold=float(plane_dist_m),
        ransac_n=3,
        num_iterations=1000,
    )
    plane = np.asarray(model, dtype=np.float64)
    a, b, c, d = plane

    n_removed = 0
    for fr in frames:
        ys, xs = np.where(np.isfinite(fr.depth_m))
        if xs.size == 0:
            continue
        z = fr.depth_m[ys, xs].astype(np.float64)
        fx, fy = fr.K[0, 0], fr.K[1, 1]
        cx, cy = fr.K[0, 2], fr.K[1, 2]
        x = (xs.astype(np.float64) - cx) * z / fx
        y = (ys.astype(np.float64) - cy) * z / fy
        cam = np.stack([x, y, z], axis=1)
        world = (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]
        dist = np.abs(a * world[:, 0] + b * world[:, 1] + c * world[:, 2] + d) / (
            np.sqrt(a * a + b * b + c * c) + 1e-12
        )
        kill = dist <= float(keep_band_m)
        n_removed += int(kill.sum())
        fr.depth_m = fr.depth_m.copy()
        fr.depth_m[ys[kill], xs[kill]] = np.nan
        if fr.conf is not None:
            fr.conf = fr.conf.copy()
            fr.conf[ys[kill], xs[kill]] = 0.0
        fr.mask = np.isfinite(fr.depth_m)

    n_after = int(sum(np.count_nonzero(np.isfinite(f.depth_m)) for f in frames))
    return TrayStats(n_before, n_after, n_removed, plane.tolist())
