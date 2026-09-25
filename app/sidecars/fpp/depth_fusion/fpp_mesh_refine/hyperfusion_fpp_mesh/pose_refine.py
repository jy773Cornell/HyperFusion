# Robust multi-view pose refinement (sidecar / depth_fusion / fpp_mesh_refine).
# Multi-scale pair ICP → globally optimized pose graph. No robot I/O.
"""Correct residual robot/hand-eye pose error before depth fusion."""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from .frames import DepthFrame

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc


@dataclass
class _PairRegistration:
    source: int
    target: int
    transformation: np.ndarray
    information: np.ndarray
    fitness: float
    rmse_m: float
    trans_m: float
    rot_deg: float
    initial_fitness: float
    adjacent: bool


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


def _transform_delta(T: np.ndarray) -> tuple[float, float]:
    """Return translation metres and rotation degrees for a rigid transform."""
    mat = np.asarray(T, dtype=np.float64).reshape(4, 4)
    trans_m = float(np.linalg.norm(mat[:3, 3]))
    cosang = np.clip((np.trace(mat[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)
    return trans_m, float(np.degrees(np.arccos(cosang)))


def _downsample_with_normals(
    cloud: o3d.geometry.PointCloud, voxel_m: float
) -> o3d.geometry.PointCloud:
    down = cloud.voxel_down_sample(float(voxel_m))
    return _ensure_normals(down, float(voxel_m))


def _register_pair_multiscale(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    *,
    source_index: int,
    target_index: int,
    adjacent: bool,
    voxel_scales_m: tuple[float, ...],
    correspondence_scales_m: tuple[float, ...],
    iterations: tuple[int, ...],
) -> _PairRegistration:
    """Robust coarse-to-fine point-to-plane ICP from source to target."""
    coarse_eval = o3d.pipelines.registration.evaluate_registration(
        source, target, float(correspondence_scales_m[0]), np.eye(4)
    )
    transform = np.eye(4, dtype=np.float64)
    final_result = None
    final_source = source
    final_target = target
    for voxel_m, max_corr_m, max_iteration in zip(
        voxel_scales_m, correspondence_scales_m, iterations
    ):
        src = _downsample_with_normals(source, float(voxel_m))
        tgt = _downsample_with_normals(target, float(voxel_m))
        if len(src.points) < 80 or len(tgt.points) < 80:
            continue
        loss = o3d.pipelines.registration.TukeyLoss(
            k=max(float(max_corr_m) * 0.5, float(voxel_m))
        )
        estimate = o3d.pipelines.registration.TransformationEstimationPointToPlane(
            loss
        )
        final_result = o3d.pipelines.registration.registration_icp(
            src,
            tgt,
            float(max_corr_m),
            transform,
            estimate,
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1.0e-7,
                relative_rmse=1.0e-7,
                max_iteration=int(max_iteration),
            ),
        )
        transform = np.asarray(final_result.transformation, dtype=np.float64)
        final_source = src
        final_target = tgt
    if final_result is None:
        raise RuntimeError("pair registration has too few points")

    fine_corr = float(correspondence_scales_m[-1])
    final_eval = o3d.pipelines.registration.evaluate_registration(
        final_source, final_target, fine_corr, transform
    )
    information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        final_source, final_target, fine_corr, transform
    )
    trans_m, rot_deg = _transform_delta(transform)
    return _PairRegistration(
        source=int(source_index),
        target=int(target_index),
        transformation=transform,
        information=np.asarray(information, dtype=np.float64),
        fitness=float(final_eval.fitness),
        rmse_m=float(final_eval.inlier_rmse),
        trans_m=trans_m,
        rot_deg=rot_deg,
        initial_fitness=float(coarse_eval.fitness),
        adjacent=bool(adjacent),
    )


def _cloud_union(
    clouds: list[o3d.geometry.PointCloud], skip: int, voxel_m: float
) -> o3d.geometry.PointCloud:
    union = o3d.geometry.PointCloud()
    for index, cloud in enumerate(clouds):
        if index != skip:
            union += cloud
    return union.voxel_down_sample(float(voxel_m))


def _connected_indices(
    valid: list[int], pairs: list[_PairRegistration], start: int
) -> set[int]:
    """Return the pose-graph component reachable from the start node."""
    adjacency = {index: set() for index in valid}
    for pair in pairs:
        adjacency[pair.source].add(pair.target)
        adjacency[pair.target].add(pair.source)
    visited: set[int] = set()
    pending = [start]
    while pending:
        current = pending.pop()
        if current in visited:
            continue
        visited.add(current)
        pending.extend(adjacency[current] - visited)
    return visited


def refine_poses_constrained(
    frames: list[DepthFrame],
    *,
    max_trans_m: float = 0.030,
    max_rot_deg: float = 6.0,
    voxel_scales_m: tuple[float, ...] = (0.0050, 0.0025, 0.00125),
    correspondence_scales_m: tuple[float, ...] = (0.0150, 0.0075, 0.0030),
    iterations: tuple[int, ...] = (50, 35, 25),
    min_initial_fitness: float = 0.20,
    min_final_fitness: float = 0.30,
    max_final_rmse_m: float = 0.0030,
    loop_closure_preference: float = 0.20,
) -> dict:
    """Globally refine poses using robust pairwise ICP and a pose graph."""
    n_frames = len(frames)
    if n_frames < 2:
        return {
            "method": "multiscale_robust_pose_graph",
            "refined": 0,
            "rejected": 0,
            "deltas": [],
            "edges": [],
        }
    if not (
        len(voxel_scales_m)
        == len(correspondence_scales_m)
        == len(iterations)
    ):
        raise ValueError("pose-refine scale and iteration lists must have equal length")

    clouds = [_frame_cloud(frame) for frame in frames]
    valid = [index for index, cloud in enumerate(clouds) if len(cloud.points) >= 80]
    if len(valid) < 2:
        return {
            "method": "multiscale_robust_pose_graph",
            "refined": 0,
            "rejected": 0,
            "deltas": [],
            "edges": [],
            "reason": "fewer_than_two_clouds",
        }

    reference = max(valid, key=lambda index: (len(clouds[index].points), -index))
    pair_results: list[_PairRegistration] = []
    edge_rows: list[dict] = []
    for a_pos, source_i in enumerate(valid):
        for target_i in valid[a_pos + 1 :]:
            adjacent = target_i == source_i + 1
            initial = o3d.pipelines.registration.evaluate_registration(
                clouds[source_i],
                clouds[target_i],
                float(correspondence_scales_m[0]),
                np.eye(4),
            )
            if not adjacent and float(initial.fitness) < float(min_initial_fitness):
                edge_rows.append(
                    {
                        "source": frames[source_i].stem,
                        "target": frames[target_i].stem,
                        "adjacent": False,
                        "accepted": False,
                        "reason": "low_initial_overlap",
                        "initial_fitness": float(initial.fitness),
                    }
                )
                continue
            try:
                pair = _register_pair_multiscale(
                    clouds[source_i],
                    clouds[target_i],
                    source_index=source_i,
                    target_index=target_i,
                    adjacent=adjacent,
                    voxel_scales_m=voxel_scales_m,
                    correspondence_scales_m=correspondence_scales_m,
                    iterations=iterations,
                )
            except RuntimeError as exc:
                edge_rows.append(
                    {
                        "source": frames[source_i].stem,
                        "target": frames[target_i].stem,
                        "adjacent": adjacent,
                        "accepted": False,
                        "reason": str(exc),
                    }
                )
                continue

            accepted = (
                pair.fitness >= float(min_final_fitness)
                and pair.rmse_m <= float(max_final_rmse_m)
                and pair.rot_deg <= float(max_rot_deg) * 2.0
            )
            row = {
                "source": frames[source_i].stem,
                "target": frames[target_i].stem,
                "adjacent": adjacent,
                "uncertain": not adjacent,
                "accepted": bool(accepted),
                "initial_fitness": pair.initial_fitness,
                "fitness": pair.fitness,
                "rmse_mm": pair.rmse_m * 1000.0,
                "transform_translation_mm": pair.trans_m * 1000.0,
                "rot_deg": pair.rot_deg,
            }
            if not accepted:
                row["reason"] = "quality_or_motion_gate"
            edge_rows.append(row)
            if accepted:
                pair_results.append(pair)

    constrained = _connected_indices(valid, pair_results, reference)

    pose_graph = o3d.pipelines.registration.PoseGraph()
    local_index = {global_index: local for local, global_index in enumerate(valid)}
    for _ in valid:
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))
    for pair in pair_results:
        pose_graph.edges.append(
            o3d.pipelines.registration.PoseGraphEdge(
                local_index[pair.source],
                local_index[pair.target],
                pair.transformation,
                pair.information,
                uncertain=not pair.adjacent,
            )
        )

    if len(constrained) != len(valid):
        return {
            "method": "multiscale_robust_pose_graph",
            "refined": 0,
            "rejected": len(valid) - 1,
            "deltas": [],
            "edges": edge_rows,
            "reference": frames[reference].stem,
            "reason": "pose_graph_not_connected",
        }

    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=float(correspondence_scales_m[-1]),
        edge_prune_threshold=0.25,
        preference_loop_closure=float(loop_closure_preference),
        reference_node=int(local_index[reference]),
    )
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option,
    )

    corrections = [np.eye(4, dtype=np.float64) for _ in frames]
    for global_index, local in local_index.items():
        corrections[global_index] = np.asarray(
            pose_graph.nodes[local].pose, dtype=np.float64
        ).copy()
    corrected_clouds: list[o3d.geometry.PointCloud] = []
    for cloud, correction in zip(clouds, corrections):
        moved = copy.deepcopy(cloud)
        moved.transform(correction)
        corrected_clouds.append(moved)

    refined = 0
    rejected = 0
    deltas: list[dict] = []
    validation_voxel_m = float(voxel_scales_m[1])
    for index, frame in enumerate(frames):
        correction = corrections[index]
        # A left-multiplied world correction rotates about the world origin, so
        # its raw translation can be tens of millimetres even for a small camera
        # correction. Gate the physically meaningful camera-local pose delta.
        corrected_c2w = correction @ frame.c2w
        trans_m, rot_deg = _se3_delta(corrected_c2w, frame.c2w)
        before_target = _cloud_union(clouds, index, validation_voxel_m)
        after_target = _cloud_union(corrected_clouds, index, validation_voxel_m)
        before_nn = _median_nn_m(clouds[index], before_target)
        after_nn = _median_nn_m(corrected_clouds[index], after_target)
        is_reference = index == reference
        accepted = is_reference or (
            index in constrained
            and trans_m <= float(max_trans_m)
            and rot_deg <= float(max_rot_deg)
            and after_nn <= before_nn * 1.05
        )
        if accepted:
            if not is_reference:
                frame.c2w = corrected_c2w
                refined += 1
        else:
            rejected += 1
        deltas.append(
            {
                "stem": frame.stem,
                "trans_mm": trans_m * 1000.0,
                "rot_deg": rot_deg,
                "median_nn_before_mm": before_nn * 1000.0,
                "median_nn_after_mm": after_nn * 1000.0,
                "accepted": bool(accepted),
                "role": "reference" if is_reference else "pose_graph",
                "proposed_world_correction": correction.tolist(),
                "applied_camera_to_world": frame.c2w.tolist(),
            }
        )

    return {
        "method": "multiscale_robust_pose_graph",
        "refined": refined,
        "rejected": rejected,
        "skipped_aligned": 0,
        "deltas": deltas,
        "edges": edge_rows,
        "accepted_edges": len(pair_results),
        "reference": frames[reference].stem,
        "max_trans_mm": float(max_trans_m) * 1000.0,
        "max_rot_deg": float(max_rot_deg),
        "voxel_scales_mm": [float(value) * 1000.0 for value in voxel_scales_m],
        "correspondence_scales_mm": [
            float(value) * 1000.0 for value in correspondence_scales_m
        ],
        "loop_closure_preference": float(loop_closure_preference),
        "pose_convention": "camera_to_world_metres",
        "optimized_poses": [
            {"stem": frame.stem, "camera_to_world": frame.c2w.tolist()}
            for frame in frames
        ],
    }
