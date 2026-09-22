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
    n_after_support: int
    n_after_voxel: int
    n_dropped_by_support: int
    n_single_view: int
    n_multi_view: int
    voxel_m: float
    support_voxel_m: float
    support_min_views: int
    confidence_weighted: bool
    weight_source: str


def _weighted_voxel_downsample(
    points: np.ndarray,
    colors: np.ndarray,
    weights: np.ndarray,
    voxel_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Average each voxel using per-pixel reconstruction confidence / fusion score."""
    voxel_index = np.floor(points / float(voxel_m)).astype(np.int64)
    _, inverse = np.unique(voxel_index, axis=0, return_inverse=True)
    n_voxels = int(inverse.max()) + 1
    weight_sum = np.bincount(inverse, weights=weights, minlength=n_voxels)
    safe_weight = np.maximum(weight_sum, 1.0e-12)
    out_points = np.column_stack(
        [
            np.bincount(inverse, weights=weights * points[:, axis], minlength=n_voxels)
            / safe_weight
            for axis in range(3)
        ]
    )
    out_colors = np.column_stack(
        [
            np.bincount(inverse, weights=weights * colors[:, axis], minlength=n_voxels)
            / safe_weight
            for axis in range(3)
        ]
    )
    return out_points, out_colors


def _multiview_support_mask(
    points: np.ndarray,
    view_ids: np.ndarray,
    *,
    support_voxel_m: float,
    min_views: int,
) -> np.ndarray:
    """Keep raw samples whose coarse spatial cell is observed by enough views."""
    cells = np.floor(points / float(support_voxel_m)).astype(np.int64)
    _, cell_inverse = np.unique(cells, axis=0, return_inverse=True)
    pairs = np.column_stack((cell_inverse, view_ids))
    unique_pairs = np.unique(pairs, axis=0)
    support = np.bincount(
        unique_pairs[:, 0],
        minlength=int(cell_inverse.max()) + 1,
    )
    return support[cell_inverse] >= int(min_views)


def _per_sample_view_support(
    points: np.ndarray,
    view_ids: np.ndarray,
    *,
    support_voxel_m: float,
) -> np.ndarray:
    """Return how many distinct views hit each sample's support cell."""
    cells = np.floor(points / float(support_voxel_m)).astype(np.int64)
    _, cell_inverse = np.unique(cells, axis=0, return_inverse=True)
    pairs = np.column_stack((cell_inverse, view_ids))
    unique_pairs = np.unique(pairs, axis=0)
    support = np.bincount(
        unique_pairs[:, 0],
        minlength=int(cell_inverse.max()) + 1,
    )
    return support[cell_inverse].astype(np.int32)


def _pixel_weights(
    fr: DepthFrame,
    ys: np.ndarray,
    xs: np.ndarray,
    *,
    weight_source: str,
) -> tuple[np.ndarray, str]:
    """Pick densify weights from fusion_score (preferred) or conf."""
    source = str(weight_source or "auto").strip().lower()
    score_map = fr.meta.get("fusion_score") if isinstance(fr.meta, dict) else None

    if source in ("auto", "score") and isinstance(score_map, np.ndarray):
        if score_map.shape == fr.depth_m.shape:
            # Soft scores can exceed 1 when multiple views AGREE — that is intentional.
            w = np.clip(score_map[ys, xs].astype(np.float64), 0.05, 3.0)
            return w, "fusion_score"

    if source == "score":
        # Requested score but missing — fall back.
        pass

    if fr.conf is not None:
        return np.clip(fr.conf[ys, xs].astype(np.float64), 0.05, 1.0), "conf"
    return np.ones(xs.size, dtype=np.float64), "uniform"


def densify_from_depth(
    frames: list[DepthFrame],
    *,
    conf_min: float = 0.10,
    pixel_stride: int = 1,
    voxel_m: float = 0.0005,
    support_voxel_m: float = 0.002,
    support_min_views: int = 2,
    weight_source: str = "auto",
) -> DenseCloudResult:
    """Merge back-projected depth into a dense world cloud.

    ``support_min_views=1`` keeps single-view cells (union). Higher values are the
    legacy intersection gate.

    ``weight_source``: ``auto`` uses ``meta['fusion_score']`` when present, else conf.
    """
    pts_list: list[np.ndarray] = []
    col_list: list[np.ndarray] = []
    weight_list: list[np.ndarray] = []
    view_list: list[np.ndarray] = []
    used_sources: set[str] = set()

    for view_id, fr in enumerate(frames):
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
        weights, src = _pixel_weights(fr, ys, xs, weight_source=weight_source)
        used_sources.add(src)
        pts_list.append(world)
        col_list.append(rgb)
        weight_list.append(weights)
        view_list.append(np.full(xs.size, view_id, dtype=np.int32))

    if not pts_list:
        raise RuntimeError("densify: no valid depth pixels")

    pts = np.concatenate(pts_list, axis=0)
    cols = np.concatenate(col_list, axis=0)
    weights = np.concatenate(weight_list, axis=0)
    view_ids = np.concatenate(view_list, axis=0)
    n_raw = int(pts.shape[0])

    if int(support_min_views) > 1:
        supported = _multiview_support_mask(
            pts,
            view_ids,
            support_voxel_m=float(support_voxel_m),
            min_views=int(support_min_views),
        )
        pts = pts[supported]
        cols = cols[supported]
        weights = weights[supported]
        view_ids = view_ids[supported]

    n_after_support = int(pts.shape[0])
    n_dropped_by_support = int(n_raw - n_after_support)
    if n_after_support == 0:
        raise RuntimeError(
            "densify: multi-view support removed every point; "
            "check calibration or reduce support_min_views"
        )

    view_support = _per_sample_view_support(
        pts, view_ids, support_voxel_m=float(support_voxel_m)
    )
    n_single_view = int((view_support <= 1).sum())
    n_multi_view = int((view_support >= 2).sum())

    if float(voxel_m) > 0.0:
        pts, cols = _weighted_voxel_downsample(pts, cols, weights, float(voxel_m))

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(pts)
    cloud.colors = o3d.utility.Vector3dVector(np.clip(cols, 0.0, 1.0))

    if "fusion_score" in used_sources:
        weight_label = "fusion_score"
    elif "conf" in used_sources:
        weight_label = "conf"
    else:
        weight_label = "uniform"

    return DenseCloudResult(
        cloud=cloud,
        n_raw=n_raw,
        n_after_support=n_after_support,
        n_after_voxel=int(len(cloud.points)),
        n_dropped_by_support=n_dropped_by_support,
        n_single_view=n_single_view,
        n_multi_view=n_multi_view,
        voxel_m=float(voxel_m),
        support_voxel_m=float(support_voxel_m),
        support_min_views=int(support_min_views),
        confidence_weighted=True,
        weight_source=weight_label,
    )
