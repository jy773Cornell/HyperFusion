# Constrained multi-view pose refinement (sidecar / depth_fusion / fpp_mesh_refine).
# Small SE(3) ICP of each frame cloud to a reference; translation/rotation capped. No robot I/O.
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


def refine_poses_constrained(
    frames: list[DepthFrame],
    *,
    max_trans_m: float = 0.003,
    max_rot_deg: float = 0.5,
    icp_thresh_m: float = 0.008,
    voxel_m: float = 0.003,
) -> dict:
    """Align each frame (except ref0) to the union of others via point-to-plane ICP.

    Rejects updates that exceed translation/rotation caps (keeps BA pose).
    """
    if len(frames) < 2:
        return {"refined": 0, "rejected": 0, "deltas": []}

    clouds = [_frame_cloud(fr) for fr in frames]
    for i, c in enumerate(clouds):
        if len(c.points) > 0:
            clouds[i] = c.voxel_down_sample(voxel_m)
            clouds[i].estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_m * 3.0, max_nn=30)
            )

    refined = 0
    rejected = 0
    deltas: list[dict] = []
    for i, fr in enumerate(frames):
        if len(clouds[i].points) < 200:
            continue
        # Reference = merge all other clouds
        others = [clouds[j] for j in range(len(clouds)) if j != i and len(clouds[j].points) > 0]
        if not others:
            continue
        ref = others[0]
        for o in others[1:]:
            ref += o
        ref = ref.voxel_down_sample(voxel_m)
        if len(ref.points) < 200:
            continue
        ref.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_m * 3.0, max_nn=30)
        )

        result = o3d.pipelines.registration.registration_icp(
            clouds[i],
            ref,
            float(icp_thresh_m),
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40),
        )
        T_delta = np.asarray(result.transformation, dtype=np.float64)
        # Apply delta in world: new_c2w = T_delta @ old_c2w
        # (cloud was in world; ICP maps source→target in world)
        T_new = T_delta @ fr.c2w
        t_m, r_deg = _se3_delta(T_new, fr.c2w)
        row = {
            "stem": fr.stem,
            "trans_mm": t_m * 1000.0,
            "rot_deg": r_deg,
            "fitness": float(result.fitness),
            "accepted": False,
        }
        if t_m <= float(max_trans_m) and r_deg <= float(max_rot_deg) and result.fitness > 0.1:
            fr.c2w = T_new
            row["accepted"] = True
            refined += 1
        else:
            rejected += 1
        deltas.append(row)

    return {"refined": refined, "rejected": rejected, "deltas": deltas}
