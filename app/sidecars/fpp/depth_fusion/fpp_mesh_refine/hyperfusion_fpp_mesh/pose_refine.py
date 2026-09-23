# Constrained multi-view pose refinement (sidecar / depth_fusion / fpp_mesh_refine).
# Grow a seed cloud and ICP each frame onto it; skip ICP when already aligned. No robot I/O.
"""Refine camera poses slightly so depth clouds agree better."""

from __future__ import annotations

import numpy as np

from .frames import DepthFrame

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc


def _frame_cloud(fr: DepthFrame, *, stride: int = 4, conf_min: float = 0.15) -> o3d.geometry.PointCloud:
    depth = fr.depth_m
    ys, xs = np.where(np.isfinite(depth))
    if fr.conf is not None:
        keep = fr.conf[ys, xs] >= float(conf_min)
        ys, xs = ys[keep], xs[keep]
    if xs.size == 0:
        return o3d.geometry.PointCloud()
    if stride > 1:
        ys, xs = ys[::stride], xs[::stride]
    z = depth[ys, xs].astype(np.float64)
    fx, fy = fr.K[0, 0], fr.K[1, 1]
    cx, cy = fr.K[0, 2], fr.K[1, 2]
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    cam = np.stack([x, y, z], axis=1)
    world = (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(world)
    return pcd


def _se3_delta(T_new: np.ndarray, T_old: np.ndarray) -> tuple[float, float]:
    """Return (trans_m, rot_deg) of T_new relative to T_old."""
    d = np.linalg.inv(T_old) @ T_new
    t = float(np.linalg.norm(d[:3, 3]))
    R = d[:3, :3]
    cosang = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    ang = float(np.degrees(np.arccos(cosang)))
    return t, ang


def _median_nn_m(src: o3d.geometry.PointCloud, tgt: o3d.geometry.PointCloud) -> float:
    if len(src.points) == 0 or len(tgt.points) == 0:
        return 1.0e9
    d = np.asarray(src.compute_point_cloud_distance(tgt), dtype=np.float64)
    if d.size == 0:
        return 1.0e9
    return float(np.median(d))


def _ensure_normals(cloud: o3d.geometry.PointCloud, voxel_m: float) -> o3d.geometry.PointCloud:
    if len(cloud.points) == 0:
        return cloud
    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=float(voxel_m) * 3.0, max_nn=30)
    )
    return cloud


def refine_poses_constrained(
    frames: list[DepthFrame],
    *,
    max_trans_m: float = 0.020,
    max_rot_deg: float = 4.0,
    icp_thresh_m: float = 0.020,
    voxel_m: float = 0.0025,
    already_aligned_nn_m: float = 0.004,
) -> dict:
    """Align frames to a growing multi-view seed cloud (not a single pin).

    Aligning every pin to ``00000`` alone fails for side / second-ring views
    with little overlap — ICP slides tens of centimetres. Hand–eye poses are
    often already within a few mm of the fused seed; in that case we keep them.
    """
    if len(frames) < 2:
        return {"refined": 0, "rejected": 0, "skipped_aligned": 0, "deltas": []}

    clouds = [_frame_cloud(fr) for fr in frames]
    for i, c in enumerate(clouds):
        if len(c.points) > 0:
            clouds[i] = c.voxel_down_sample(voxel_m)
            clouds[i] = _ensure_normals(clouds[i], voxel_m)

    # Prefer a dense early frame as the seed (usually look-down 00000).
    order = sorted(
        (i for i, c in enumerate(clouds) if len(c.points) >= 80),
        key=lambda i: -len(clouds[i].points),
    )
    if not order:
        return {"refined": 0, "rejected": 0, "skipped_aligned": 0, "deltas": []}

    seed_i = order[0]
    # Put seed first, then remaining by descending size (stable coverage growth).
    order = [seed_i] + [i for i in order if i != seed_i]

    accum = clouds[seed_i]
    refined = 0
    rejected = 0
    skipped_aligned = 0
    deltas: list[dict] = []

    for i in order:
        fr = frames[i]
        if i == seed_i:
            deltas.append(
                {
                    "stem": fr.stem,
                    "trans_mm": 0.0,
                    "rot_deg": 0.0,
                    "fitness": 1.0,
                    "median_nn_mm": 0.0,
                    "accepted": True,
                    "role": "seed",
                }
            )
            continue
        if len(clouds[i].points) < 80:
            continue

        med_nn = _median_nn_m(clouds[i], accum)
        row = {
            "stem": fr.stem,
            "trans_mm": 0.0,
            "rot_deg": 0.0,
            "fitness": 1.0,
            "median_nn_mm": med_nn * 1000.0,
            "accepted": True,
            "role": "already_aligned",
        }

        # Already sits on the fused object — do not ICP (avoids catastrophic slides).
        if med_nn <= float(already_aligned_nn_m):
            skipped_aligned += 1
            deltas.append(row)
            accum += clouds[i]
            accum = accum.voxel_down_sample(voxel_m)
            accum = _ensure_normals(accum, voxel_m)
            continue

        result = o3d.pipelines.registration.registration_icp(
            clouds[i],
            accum,
            float(icp_thresh_m),
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80),
        )
        T_delta = np.asarray(result.transformation, dtype=np.float64)
        T_new = T_delta @ fr.c2w
        t_m, r_deg = _se3_delta(T_new, fr.c2w)
        row.update(
            {
                "trans_mm": t_m * 1000.0,
                "rot_deg": r_deg,
                "fitness": float(result.fitness),
                "inlier_rmse_mm": float(result.inlier_rmse) * 1000.0,
                "accepted": False,
                "role": "align_to_seed",
            }
        )
        ok = (
            t_m <= float(max_trans_m)
            and r_deg <= float(max_rot_deg)
            and result.fitness > 0.25
            and float(result.inlier_rmse) < 0.008
        )
        if ok:
            fr.c2w = T_new
            clouds[i] = clouds[i].transform(T_delta)
            row["accepted"] = True
            refined += 1
        else:
            # Keep hand–eye pose; still add geometry so later pins see coverage.
            rejected += 1
        deltas.append(row)
        accum += clouds[i]
        accum = accum.voxel_down_sample(voxel_m)
        accum = _ensure_normals(accum, voxel_m)

    return {
        "refined": refined,
        "rejected": rejected,
        "skipped_aligned": skipped_aligned,
        "deltas": deltas,
        "seed": frames[seed_i].stem,
        "max_trans_mm": float(max_trans_m) * 1000.0,
        "max_rot_deg": float(max_rot_deg),
        "already_aligned_nn_mm": float(already_aligned_nn_m) * 1000.0,
    }
