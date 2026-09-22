# Tray plane crop (sidecar / depth_fusion / fpp_mesh_refine).
# Optional RANSAC removal of flat background before fuse. No robot I/O.
"""Remove dominant tray plane from depth frames.

Prefers nearly horizontal planes in ``base_link`` (+Z up). A near-vertical
RANSAC hit (grape flank / wall) must not become the stage model — that warps
the workspace box and deletes the real tray.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .frames import DepthFrame

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc

# World +Z (base_link). Tray / stage should be roughly level.
_UP_WORLD = np.array([0.0, 0.0, 1.0], dtype=np.float64)
# Reject planes steeper than ~45° from horizontal (|n·ẑ| < cos45°).
_MIN_UPRIGHT = float(np.cos(np.deg2rad(45.0)))


@dataclass
class TrayStats:
    n_before: int
    n_after: int
    n_removed: int
    plane: list[float] | None
    upright: float | None = None
    skipped_reason: str | None = None


def _backproject_world(fr: DepthFrame, *, stride: int = 1) -> np.ndarray:
    ys, xs = np.where(np.isfinite(fr.depth_m))
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if stride > 1:
        ys, xs = ys[::stride], xs[::stride]
    z = fr.depth_m[ys, xs].astype(np.float64)
    fx, fy = fr.K[0, 0], fr.K[1, 1]
    cx, cy = fr.K[0, 2], fr.K[1, 2]
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    cam = np.stack([x, y, z], axis=1)
    return (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]


def _fit_upright_plane(
    pts: np.ndarray,
    *,
    plane_dist_m: float,
    min_upright: float = _MIN_UPRIGHT,
    max_peels: int = 6,
) -> tuple[np.ndarray | None, float | None, str | None]:
    """RANSAC with peel: keep best near-horizontal plane (base_link +Z)."""
    if pts.shape[0] < 200:
        return None, None, "too_few_points"

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    remaining = pcd
    candidates: list[tuple[np.ndarray, int, float]] = []

    for _ in range(int(max_peels)):
        if len(remaining.points) < 200:
            break
        o3d.utility.random.seed(0)
        model, inl = remaining.segment_plane(
            distance_threshold=float(plane_dist_m),
            ransac_n=3,
            num_iterations=1000,
        )
        plane = np.asarray(model, dtype=np.float64)
        nrm = float(np.linalg.norm(plane[:3]))
        if nrm < 1.0e-12 or len(inl) < 100:
            break
        upright = float(abs(np.dot(plane[:3] / nrm, _UP_WORLD)))
        candidates.append((plane, int(len(inl)), upright))
        remaining = remaining.select_by_index(inl, invert=True)

    if not candidates:
        return None, None, "ransac_failed"

    upright_ok = [c for c in candidates if c[2] >= float(min_upright)]
    if upright_ok:
        plane, _n_inl, upright = max(upright_ok, key=lambda c: c[1])
        return plane, upright, None

    # No level plane — do not carve with a vertical hit (destroys stage/workspace).
    best = max(candidates, key=lambda c: c[2])
    return None, float(best[2]), f"no_upright_plane_max={best[2]:.3f}"


def remove_tray(
    frames: list[DepthFrame],
    *,
    plane_dist_m: float = 0.008,
    keep_band_m: float = 0.012,
    min_upright: float = _MIN_UPRIGHT,
) -> TrayStats:
    chunks: list[np.ndarray] = []
    for fr in frames:
        world = _backproject_world(fr, stride=4)
        if world.shape[0]:
            chunks.append(world)
    n_before = int(sum(np.count_nonzero(np.isfinite(f.depth_m)) for f in frames))
    if not chunks:
        return TrayStats(n_before, n_before, 0, None, None, "no_points")

    pts = np.concatenate(chunks, axis=0)
    plane, upright, skip = _fit_upright_plane(
        pts,
        plane_dist_m=float(plane_dist_m),
        min_upright=float(min_upright),
    )
    if plane is None:
        print(
            f"  tray skipped ({skip}); leaving depth intact "
            f"(best |n·ẑ|={upright})",
            flush=True,
        )
        return TrayStats(n_before, n_before, 0, None, upright, skip)

    a, b, c, d = plane
    n_removed = 0
    for fr in frames:
        ys, xs = np.where(np.isfinite(fr.depth_m))
        if xs.size == 0:
            continue
        world = _backproject_world(fr, stride=1)
        # _backproject uses same where() order as ys,xs above
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
    print(
        f"  tray plane upright |n·ẑ|={upright:.3f} removed={n_removed}",
        flush=True,
    )
    return TrayStats(n_before, n_after, n_removed, plane.tolist(), upright, None)
