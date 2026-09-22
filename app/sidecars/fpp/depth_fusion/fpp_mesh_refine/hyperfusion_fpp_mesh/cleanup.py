# Point-cloud cleanup for dense FPP back-project (sidecar / depth_fusion / fpp_mesh_refine).
# ROI/component → SOR → ROR → optional light MLS. No robot I/O.
"""
Cleanup pipeline on the fused dense cloud::

  multi-view back-project
  → ROI + connected-component removal
  → Statistical Outlier Removal (SOR)
  → Radius Outlier Removal (ROR)
  → optional light MLS smoothing
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc


# Intersection-era cleanup is aggressive (assumes multi-view support already
# removed lonely noise). Union keeps 1-view surfaces, so knobs must be milder.
_CLEANUP_PRESETS: dict[str, dict[str, float | int]] = {
    "intersection": {
        "sor_k": 25,
        "sor_std": 1.75,
        "ror_min_neighbors": 12,
        "ror_radius_scale": 2.5,
        "component_min_points": 800,
        "final_component_eps_m": 0.0015,
        "final_component_min_points": 500,
        "surface_max_normal_angle_deg": 55.0,
    },
    "union": {
        "sor_k": 20,
        "sor_std": 2.50,
        "ror_min_neighbors": 6,
        "ror_radius_scale": 3.5,
        "component_min_points": 300,
        "final_component_eps_m": 0.0025,
        "final_component_min_points": 200,
        "surface_max_normal_angle_deg": 75.0,
    },
}


def resolve_cleanup_defaults(
    fusion_mode: str,
    *,
    sor_k: int | None = None,
    sor_std: float | None = None,
    ror_min_neighbors: int | None = None,
    component_min_points: int | None = None,
    final_component_eps_m: float | None = None,
    final_component_min_points: int | None = None,
    surface_max_normal_angle_deg: float | None = None,
) -> dict[str, float | int | str]:
    """Pick cleanup knobs for fusion mode; explicit values always win."""
    mode = str(fusion_mode or "intersection").strip().lower()
    if mode not in _CLEANUP_PRESETS:
        mode = "intersection"
    base = dict(_CLEANUP_PRESETS[mode])
    if sor_k is not None:
        base["sor_k"] = int(sor_k)
    if sor_std is not None:
        base["sor_std"] = float(sor_std)
    if ror_min_neighbors is not None:
        base["ror_min_neighbors"] = int(ror_min_neighbors)
    if component_min_points is not None:
        base["component_min_points"] = int(component_min_points)
    if final_component_eps_m is not None:
        base["final_component_eps_m"] = float(final_component_eps_m)
    if final_component_min_points is not None:
        base["final_component_min_points"] = int(final_component_min_points)
    if surface_max_normal_angle_deg is not None:
        base["surface_max_normal_angle_deg"] = float(surface_max_normal_angle_deg)
    base["preset"] = mode
    return base


@dataclass
class CleanupStats:
    n_in: int
    n_after_roi: int
    n_after_sor: int
    n_after_ror: int
    n_after_surface: int
    n_after_final_components: int
    n_out: int
    n_components_kept: int = 1
    mean_nn_m: float | None = None
    steps: dict = field(default_factory=dict)


def _estimate_mean_nn(cloud: o3d.geometry.PointCloud, *, k: int = 8, sample: int = 5000) -> float:
    pts = np.asarray(cloud.points)
    if pts.shape[0] < k + 1:
        return 0.002
    rng = np.random.default_rng(0)
    idx = rng.choice(pts.shape[0], size=min(sample, pts.shape[0]), replace=False)
    tree = o3d.geometry.KDTreeFlann(cloud)
    dists: list[float] = []
    for i in idx:
        _, _, d2 = tree.search_knn_vector_3d(cloud.points[i], k + 1)
        if len(d2) > 1:
            dists.append(float(np.mean(np.sqrt(d2[1:]))))
    return float(np.median(dists)) if dists else 0.002


def roi_component_filter(
    cloud: o3d.geometry.PointCloud,
    *,
    percentile_crop: bool = True,
    percentile_lo: float = 1.0,
    percentile_hi: float = 99.0,
    margin_m: float = 0.005,
    eps_m: float | None = None,
    min_points: int = 500,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """Optionally percentile-crop, then keep significant DBSCAN components."""
    pts = np.asarray(cloud.points)
    n0 = int(pts.shape[0])
    if n0 == 0:
        return cloud, {"n_in": 0, "n_out": 0}

    if percentile_crop:
        lo = np.percentile(pts, percentile_lo, axis=0) - margin_m
        hi = np.percentile(pts, percentile_hi, axis=0) + margin_m
        inside = np.all((pts >= lo) & (pts <= hi), axis=1)
        cropped = cloud.select_by_index(np.where(inside)[0].tolist())
    else:
        cropped = cloud

    if eps_m is None:
        eps_m = max(0.003, 3.0 * _estimate_mean_nn(cropped))

    labels = np.array(cropped.cluster_dbscan(eps=float(eps_m), min_points=20, print_progress=False))
    if labels.size == 0 or int(labels.max()) < 0:
        return cropped, {
            "n_in": n0,
            "n_after_roi": int(len(cropped.points)),
            "n_out": int(len(cropped.points)),
            "eps_m": float(eps_m),
            "n_components_kept": 0,
        }

    # Keep components with enough points; prefer largest if only noise otherwise
    counts: dict[int, int] = {}
    for lab in labels:
        if lab < 0:
            continue
        counts[int(lab)] = counts.get(int(lab), 0) + 1
    if not counts:
        return cropped, {"n_in": n0, "n_out": int(len(cropped.points)), "eps_m": float(eps_m)}

    keep_labs = {lab for lab, c in counts.items() if c >= int(min_points)}
    if not keep_labs:
        keep_labs = {max(counts, key=counts.get)}  # type: ignore[arg-type]

    keep_idx = [i for i, lab in enumerate(labels) if int(lab) in keep_labs]
    out = cropped.select_by_index(keep_idx)
    return out, {
        "n_in": n0,
        "n_after_roi": int(len(cropped.points)),
        "n_out": int(len(out.points)),
        "eps_m": float(eps_m),
        "n_components_kept": int(len(keep_labs)),
        "component_sizes": {str(k): int(v) for k, v in sorted(counts.items(), key=lambda x: -x[1])[:8]},
    }


def physical_workspace_filter(
    cloud: o3d.geometry.PointCloud,
    *,
    tray_plane: list[float],
    camera_c2w: list[np.ndarray],
    width_m: float = 0.300,
    depth_m: float = 0.300,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """Crop a stage-centered box above the tray, derived from robot camera poses."""
    pts = np.asarray(cloud.points)
    n0 = int(pts.shape[0])
    plane = np.asarray(tray_plane, dtype=np.float64).reshape(4)
    normal_norm = float(np.linalg.norm(plane[:3]))
    if n0 == 0 or normal_norm < 1.0e-9 or not camera_c2w:
        return cloud, {"n_in": n0, "n_out": n0, "skipped": True}

    normal = plane[:3] / normal_norm
    offset = float(plane[3]) / normal_norm
    cameras = np.asarray([pose[:3, 3] for pose in camera_c2w], dtype=np.float64)
    camera_side = cameras @ normal + offset
    # "Above" means the tray side containing the robot cameras.
    up = normal if float(np.median(camera_side)) >= 0.0 else -normal

    intersections: list[np.ndarray] = []
    for pose in camera_c2w:
        origin = pose[:3, 3]
        forward = pose[:3, 2]
        denom = float(np.dot(normal, forward))
        if abs(denom) < 1.0e-6:
            continue
        ray_t = -float(np.dot(normal, origin) + offset) / denom
        if ray_t > 0.0:
            intersections.append(origin + ray_t * forward)
    if intersections:
        center = np.median(np.asarray(intersections), axis=0)
    else:
        center = np.median(cameras, axis=0)
        center = center - (float(np.dot(normal, center)) + offset) * normal

    axis_u = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    axis_u -= float(np.dot(axis_u, up)) * up
    if float(np.linalg.norm(axis_u)) < 1.0e-6:
        axis_u = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis_u -= float(np.dot(axis_u, up)) * up
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(up, axis_u)
    axis_v /= np.linalg.norm(axis_v)

    relative = pts - center
    u = relative @ axis_u
    v = relative @ axis_v
    height = relative @ up
    half_width = 0.5 * float(width_m)
    inside = (
        (np.abs(u) <= half_width)
        & (np.abs(v) <= half_width)
        & (height >= 0.0)
        & (height <= float(depth_m))
    )
    out = cloud.select_by_index(np.flatnonzero(inside).tolist())
    return out, {
        "n_in": n0,
        "n_out": int(len(out.points)),
        "width_m": float(width_m),
        "depth_m": float(depth_m),
        "center_world_m": center.tolist(),
        "up_world": up.tolist(),
        "axis_u_world": axis_u.tolist(),
        "axis_v_world": axis_v.tolist(),
        "plane": plane.tolist(),
        "mode": "tray_plane",
    }


def object_gravity_workspace_filter(
    cloud: o3d.geometry.PointCloud,
    *,
    camera_c2w: list[np.ndarray] | None,
    width_m: float = 0.300,
    depth_m: float = 0.300,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """Object-centered ROI when FPP has no tray (ambient-lit object only).

    Up is ``base_link`` +Z. XY center follows camera look-at at the object
    median depth. Height for top-view is relative to the object base (5th
    percentile Z), not a fitted tray plane. Box is ``width_m`` × ``width_m``
    × ``depth_m`` (depth = height along +Z).
    """
    pts = np.asarray(cloud.points)
    n0 = int(pts.shape[0])
    if n0 == 0:
        return cloud, {"n_in": 0, "n_out": 0, "skipped": True, "mode": "object_gravity"}

    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    z_obj = float(np.median(pts[:, 2]))
    center_xy = np.median(pts[:, :2], axis=0).astype(np.float64)
    if camera_c2w:
        hits: list[np.ndarray] = []
        for pose in camera_c2w:
            origin = np.asarray(pose[:3, 3], dtype=np.float64)
            forward = np.asarray(pose[:3, 2], dtype=np.float64)
            fz = float(forward[2])
            if abs(fz) < 1.0e-6:
                continue
            ray_t = (z_obj - float(origin[2])) / fz
            if ray_t > 0.0:
                hits.append(origin + ray_t * forward)
        if hits:
            hit = np.median(np.asarray(hits), axis=0)
            center_xy = hit[:2]

    z_base = float(np.percentile(pts[:, 2], 5.0))
    center = np.array([center_xy[0], center_xy[1], z_base], dtype=np.float64)
    span = float(depth_m)

    axis_u = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    axis_u -= float(np.dot(axis_u, up)) * up
    if float(np.linalg.norm(axis_u)) < 1.0e-6:
        axis_u = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis_u -= float(np.dot(axis_u, up)) * up
    axis_u /= np.linalg.norm(axis_u)
    axis_v = np.cross(up, axis_u)
    axis_v /= np.linalg.norm(axis_v)

    relative = pts - center
    u = relative @ axis_u
    v = relative @ axis_v
    height = relative @ up
    half_width = 0.5 * float(width_m)
    inside = (
        (np.abs(u) <= half_width)
        & (np.abs(v) <= half_width)
        & (height >= 0.0)
        & (height <= span)
    )
    out = cloud.select_by_index(np.flatnonzero(inside).tolist())
    return out, {
        "n_in": n0,
        "n_out": int(len(out.points)),
        "width_m": float(width_m),
        "depth_m": span,
        "center_world_m": center.tolist(),
        "up_world": up.tolist(),
        "axis_u_world": axis_u.tolist(),
        "axis_v_world": axis_v.tolist(),
        "mode": "object_gravity",
        "gravity_fallback": True,
        "z_base_m": z_base,
    }


def statistical_outlier_removal(
    cloud: o3d.geometry.PointCloud,
    *,
    nb_neighbors: int = 25,
    std_ratio: float = 1.75,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """SOR: remove points with mean kNN distance > μ + α·σ."""
    n0 = int(len(cloud.points))
    if n0 < nb_neighbors + 1:
        return cloud, {"n_in": n0, "n_out": n0, "nb_neighbors": nb_neighbors, "std_ratio": std_ratio}
    cleaned, ind = cloud.remove_statistical_outlier(
        nb_neighbors=int(nb_neighbors),
        std_ratio=float(std_ratio),
    )
    return cleaned, {
        "n_in": n0,
        "n_out": int(len(cleaned.points)),
        "n_removed": n0 - int(len(cleaned.points)),
        "nb_neighbors": int(nb_neighbors),
        "std_ratio": float(std_ratio),
    }


def radius_outlier_removal(
    cloud: o3d.geometry.PointCloud,
    *,
    radius_m: float | None = None,
    min_neighbors: int = 12,
    radius_scale: float = 2.5,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """ROR: remove points with too few neighbors inside radius."""
    n0 = int(len(cloud.points))
    if n0 == 0:
        return cloud, {"n_in": 0, "n_out": 0}
    if radius_m is None:
        # Scale × mean spacing — larger scale / fewer neighbors = milder (union).
        radius_m = max(0.002, float(radius_scale) * _estimate_mean_nn(cloud))
    cleaned, ind = cloud.remove_radius_outlier(
        nb_points=int(min_neighbors),
        radius=float(radius_m),
    )
    return cleaned, {
        "n_in": n0,
        "n_out": int(len(cleaned.points)),
        "n_removed": n0 - int(len(cleaned.points)),
        "radius_m": float(radius_m),
        "min_neighbors": int(min_neighbors),
        "radius_scale": float(radius_scale),
    }


def multiscale_surface_filter(
    cloud: o3d.geometry.PointCloud,
    *,
    max_normal_angle_deg: float = 55.0,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """Reject points whose fine- and coarse-scale normals strongly disagree."""
    n0 = int(len(cloud.points))
    if n0 < 50:
        return cloud, {"n_in": n0, "n_out": n0, "skipped": True}

    spacing = _estimate_mean_nn(cloud)
    fine_radius = max(0.002, 3.0 * spacing)
    coarse_radius = max(0.004, 6.0 * spacing)
    fine_cloud = o3d.geometry.PointCloud(cloud)
    coarse_cloud = o3d.geometry.PointCloud(cloud)
    fine_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=fine_radius, max_nn=30)
    )
    coarse_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=coarse_radius, max_nn=60)
    )
    fine = np.asarray(fine_cloud.normals)
    coarse = np.asarray(coarse_cloud.normals)
    agreement = np.abs(np.einsum("ij,ij->i", fine, coarse))
    threshold = float(np.cos(np.deg2rad(max_normal_angle_deg)))
    keep = np.isfinite(agreement) & (agreement >= threshold)
    out = cloud.select_by_index(np.flatnonzero(keep).tolist())
    if len(out.points) > 0:
        out.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=coarse_radius, max_nn=60
            )
        )
        # Sign consistency is useful downstream; filtering itself is sign-invariant.
        try:
            out.orient_normals_consistent_tangent_plane(20)
        except RuntimeError:
            # Filtering remains valid because agreement used abs(dot).
            pass
    return out, {
        "n_in": n0,
        "n_out": int(len(out.points)),
        "n_removed": n0 - int(len(out.points)),
        "mean_nn_m": float(spacing),
        "fine_radius_m": float(fine_radius),
        "coarse_radius_m": float(coarse_radius),
        "max_normal_angle_deg": float(max_normal_angle_deg),
    }


def light_mls_smooth(
    cloud: o3d.geometry.PointCloud,
    *,
    search_radius_m: float | None = None,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """Mild MLS smoothing — keep berry valleys (small radius)."""
    n0 = int(len(cloud.points))
    if n0 < 50:
        return cloud, {"n_in": n0, "n_out": n0, "skipped": True}
    if search_radius_m is None:
        search_radius_m = max(0.002, 2.0 * _estimate_mean_nn(cloud))
    try:
        # Open3D MLS via MovingLeastSquares if available
        mls = o3d.geometry.PointCloud(cloud)
        # Fallback: estimate normals + slight position pull via neighbor mean (very mild)
        mls.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=float(search_radius_m), max_nn=30
            )
        )
        # Use Open3D's optional reconstruct if present; else neighbor average
        tree = o3d.geometry.KDTreeFlann(mls)
        pts = np.asarray(mls.points)
        out = pts.copy()
        r = float(search_radius_m)
        for i in range(pts.shape[0]):
            k, idx, _ = tree.search_radius_vector_3d(mls.points[i], r)
            if k >= 4:
                nb = pts[list(idx)]
                # 70% original + 30% neighbor mean — light only
                out[i] = 0.70 * pts[i] + 0.30 * nb.mean(axis=0)
        mls.points = o3d.utility.Vector3dVector(out)
        return mls, {
            "n_in": n0,
            "n_out": int(len(mls.points)),
            "search_radius_m": float(search_radius_m),
            "blend": 0.30,
        }
    except Exception as exc:  # noqa: BLE001
        return cloud, {"n_in": n0, "n_out": n0, "error": str(exc)}


def cleanup_dense_cloud(
    cloud: o3d.geometry.PointCloud,
    *,
    do_roi: bool = True,
    do_sor: bool = True,
    do_ror: bool = True,
    do_surface: bool = True,
    do_final_components: bool = True,
    do_smooth: bool = False,
    tray_plane: list[float] | None = None,
    camera_c2w: list[np.ndarray] | None = None,
    workspace_width_m: float = 0.300,
    workspace_depth_m: float = 0.300,
    sor_k: int = 25,
    sor_std: float = 1.75,
    ror_min_neighbors: int = 12,
    ror_radius_m: float | None = None,
    ror_radius_scale: float = 2.5,
    component_min_points: int = 800,
    final_component_eps_m: float = 0.0015,
    final_component_min_points: int = 500,
    surface_max_normal_angle_deg: float = 55.0,
) -> tuple[o3d.geometry.PointCloud, CleanupStats]:
    """Run physical ROI/components, SOR, ROR, normals, and optional smoothing."""
    n_in = int(len(cloud.points))
    stats = CleanupStats(
        n_in=n_in,
        n_after_roi=n_in,
        n_after_sor=n_in,
        n_after_ror=n_in,
        n_after_surface=n_in,
        n_after_final_components=n_in,
        n_out=n_in,
    )
    cur = cloud

    if do_roi:
        if tray_plane is not None and camera_c2w:
            cur, workspace_stats = physical_workspace_filter(
                cur,
                tray_plane=tray_plane,
                camera_c2w=camera_c2w,
                width_m=float(workspace_width_m),
                depth_m=float(workspace_depth_m),
            )
            stats.steps["workspace"] = workspace_stats
            percentile_crop = False
        elif camera_c2w is not None or len(cur.points) > 0:
            # No tray in decode (typical with ambient object-only lighting).
            cur, workspace_stats = object_gravity_workspace_filter(
                cur,
                camera_c2w=camera_c2w,
                width_m=float(workspace_width_m),
                depth_m=float(workspace_depth_m),
            )
            stats.steps["workspace"] = workspace_stats
            percentile_crop = False
        else:
            percentile_crop = True
        cur, s = roi_component_filter(
            cur,
            percentile_crop=percentile_crop,
            min_points=int(component_min_points),
        )
        stats.n_after_roi = int(len(cur.points))
        stats.n_components_kept = int(s.get("n_components_kept", 1))
        stats.steps["components"] = s

    if do_sor and len(cur.points) > 0:
        cur, s = statistical_outlier_removal(cur, nb_neighbors=sor_k, std_ratio=sor_std)
        stats.n_after_sor = int(len(cur.points))
        stats.steps["sor"] = s
    else:
        stats.n_after_sor = int(len(cur.points))

    if do_ror and len(cur.points) > 0:
        cur, s = radius_outlier_removal(
            cur,
            radius_m=ror_radius_m,
            min_neighbors=ror_min_neighbors,
            radius_scale=float(ror_radius_scale),
        )
        stats.n_after_ror = int(len(cur.points))
        stats.steps["ror"] = s
    else:
        stats.n_after_ror = int(len(cur.points))

    if do_surface and len(cur.points) > 0:
        cur, s = multiscale_surface_filter(
            cur, max_normal_angle_deg=float(surface_max_normal_angle_deg)
        )
        stats.n_after_surface = int(len(cur.points))
        stats.steps["surface"] = s
    else:
        stats.n_after_surface = int(len(cur.points))

    if do_final_components and len(cur.points) > 0:
        cur, s = roi_component_filter(
            cur,
            percentile_crop=False,
            eps_m=float(final_component_eps_m),
            min_points=int(final_component_min_points),
        )
        stats.n_after_final_components = int(len(cur.points))
        stats.steps["final_components"] = s
    else:
        stats.n_after_final_components = int(len(cur.points))

    if do_smooth and len(cur.points) > 0:
        cur, s = light_mls_smooth(cur)
        stats.steps["smooth"] = s

    stats.n_out = int(len(cur.points))
    stats.mean_nn_m = _estimate_mean_nn(cur) if stats.n_out > 0 else None
    return cur, stats
