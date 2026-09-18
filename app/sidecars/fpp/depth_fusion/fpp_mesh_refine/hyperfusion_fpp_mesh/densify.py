# Dense point-cloud build from refined FPP depth (sidecar / fpp / depth_fusion).
# Back-projects confident pixels to world, then light voxel merge. No robot I/O.
"""Extra densify step: metric dense cloud from multi-view depth."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .frames import DepthFrame

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc


@dataclass
class DenseCloudResult:
    cloud: o3d.geometry.PointCloud
    n_raw: int
    n_after_voxel: int
    voxel_m: float


def densify_from_depth(
    frames: list[DepthFrame],
    *,
    conf_min: float = 0.10,
    pixel_stride: int = 1,
    voxel_m: float = 0.0005,
) -> DenseCloudResult:
    """Merge back-projected depth into a dense world cloud.

    ``voxel_m`` only merges duplicates (default 0.5 mm). Use 0 to keep every pixel.
    """
    pts_list: list[np.ndarray] = []
    col_list: list[np.ndarray] = []
    for fr in frames:
        ys, xs = np.where(np.isfinite(fr.depth_m))
        if fr.conf is not None:
            keep = fr.conf[ys, xs] >= float(conf_min)
            ys, xs = ys[keep], xs[keep]
        if int(pixel_stride) > 1:
            ys, xs = ys[::pixel_stride], xs[::pixel_stride]
        if xs.size == 0:
            continue
        z = fr.depth_m[ys, xs].astype(np.float64)
        fx, fy = fr.K[0, 0], fr.K[1, 1]
        cx, cy = fr.K[0, 2], fr.K[1, 2]
        x = (xs.astype(np.float64) - cx) * z / fx
        y = (ys.astype(np.float64) - cy) * z / fy
        cam = np.stack([x, y, z], axis=1)
        world = (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]
        rgb = fr.color[ys, xs].astype(np.float64) / 255.0
        pts_list.append(world)
        col_list.append(rgb)

    if not pts_list:
        raise RuntimeError("densify: no valid depth pixels")

    pts = np.concatenate(pts_list, axis=0)
    cols = np.concatenate(col_list, axis=0)
    n_raw = int(pts.shape[0])

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts)
    cloud.colors = o3d.utility.Vector3dVector(np.clip(cols, 0.0, 1.0))

    if float(voxel_m) > 0.0:
        cloud = cloud.voxel_down_sample(float(voxel_m))

    return DenseCloudResult(
        cloud=cloud,
        n_raw=n_raw,
        n_after_voxel=int(len(cloud.points)),
        voxel_m=float(voxel_m),
    )
