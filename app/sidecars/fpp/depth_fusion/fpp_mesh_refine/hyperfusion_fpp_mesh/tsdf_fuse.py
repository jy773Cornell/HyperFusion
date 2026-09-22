# Optional weighted TSDF from refined FPP frames (sidecar / fpp_mesh_refine).
# Integrates valid depth only (NaN/0 = unknown). No free-space carving. No robot I/O.
"""Score-gated Open3D TSDF of multi-view FPP depth."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .frames import DepthFrame

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc


@dataclass
class TsdfFuseResult:
    cloud: o3d.geometry.PointCloud
    mesh: o3d.geometry.TriangleMesh
    n_frames: int
    n_pixels_integrated: int
    voxel_m: float
    trunc_m: float
    weight_source: str
    conf_min: float


def _frame_weight_map(fr: DepthFrame, *, weight_source: str) -> tuple[np.ndarray, str]:
    source = str(weight_source or "auto").strip().lower()
    score = fr.meta.get("fusion_score") if isinstance(fr.meta, dict) else None
    if source in ("auto", "score") and isinstance(score, np.ndarray):
        if score.shape == fr.depth_m.shape:
            return np.clip(score.astype(np.float32), 0.0, 3.0), "fusion_score"
    if fr.conf is not None and fr.conf.shape == fr.depth_m.shape:
        return np.clip(fr.conf.astype(np.float32), 0.0, 1.0), "conf"
    return np.ones(fr.depth_m.shape, dtype=np.float32), "uniform"


def fuse_frames_tsdf(
    frames: list[DepthFrame],
    *,
    voxel_m: float = 0.0015,
    trunc_m: float = 0.008,
    conf_min: float = 0.12,
    weight_source: str = "auto",
    depth_trunc_m: float | None = None,
) -> TsdfFuseResult:
    """Integrate refined frames into a TSDF volume.

    Open3D's ScalableTSDFVolume has no per-pixel weight channel, so soft scores
    are applied as a **gate**: pixels with weight < ``conf_min`` are written as
    depth=0 (unknown) and do **not** carve free space. NOT_OBSERVED / weak fringe
    therefore cannot erase good geometry from other views.
    """
    if not frames:
        raise RuntimeError("tsdf: no frames")

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=float(voxel_m),
        sdf_trunc=float(trunc_m),
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    used_sources: set[str] = set()
    n_pix = 0
    trunc = float(depth_trunc_m) if depth_trunc_m is not None else 10.0

    for fr in frames:
        weights, src = _frame_weight_map(fr, weight_source=weight_source)
        used_sources.add(src)
        valid = (
            np.isfinite(fr.depth_m)
            & fr.mask
            & (weights >= float(conf_min))
            & (fr.depth_m > 1.0e-4)
        )
        depth_o3d = np.zeros(fr.depth_m.shape, dtype=np.float32)
        depth_o3d[valid] = fr.depth_m[valid].astype(np.float32)
        count = int(valid.sum())
        if count < 200:
            continue
        n_pix += count

        color = fr.color
        if color is None or color.shape[:2] != fr.depth_m.shape:
            color = np.full(
                (fr.depth_m.shape[0], fr.depth_m.shape[1], 3), 180, dtype=np.uint8
            )
        else:
            color = np.ascontiguousarray(color)
            if color.dtype != np.uint8:
                color = (
                    np.clip(color, 0, 255).astype(np.uint8)
                    if float(np.max(color)) > 1.5
                    else np.clip(color * 255.0, 0, 255).astype(np.uint8)
                )

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color),
            o3d.geometry.Image(depth_o3d),
            depth_scale=1.0,
            depth_trunc=float(trunc),
            convert_rgb_to_intensity=False,
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            int(fr.depth_m.shape[1]),
            int(fr.depth_m.shape[0]),
            float(fr.K[0, 0]),
            float(fr.K[1, 1]),
            float(fr.K[0, 2]),
            float(fr.K[1, 2]),
        )
        volume.integrate(rgbd, intrinsic, np.linalg.inv(fr.c2w))

    if n_pix == 0:
        raise RuntimeError(
            "tsdf: no pixels passed the score/conf gate; "
            "lower conf_min or check refined depth"
        )

    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    cloud = volume.extract_point_cloud()

    if "fusion_score" in used_sources:
        weight_label = "fusion_score"
    elif "conf" in used_sources:
        weight_label = "conf"
    else:
        weight_label = "uniform"

    return TsdfFuseResult(
        cloud=cloud,
        mesh=mesh,
        n_frames=len(frames),
        n_pixels_integrated=n_pix,
        voxel_m=float(voxel_m),
        trunc_m=float(trunc_m),
        weight_source=weight_label,
        conf_min=float(conf_min),
    )
