# Classical FPP dense-cloud refine (sidecar / fpp / depth_fusion / fpp_mesh_refine).
# Filter → conf → pose → consistency → densify → cleanup. No robot I/O.
"""Run the end-to-end FPP mesh refine pipeline with per-stage timings."""

from __future__ import annotations

import json
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

import numpy as np

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d required") from exc

from .cleanup import cleanup_dense_cloud, resolve_cleanup_defaults
from .confidence import confidence_frames
from .consistency import consistency_reject
from .densify import densify_from_depth
from .filter import filter_frames
from .frames import load_raw_frames
from .pose_refine import refine_poses_constrained
from .top_view import rasterize_top_view
from .tray import remove_tray
from .tsdf_fuse import fuse_frames_tsdf


def resolve_fusion_mode_defaults(
    fusion_mode: str,
    *,
    support_min_views: int | None = None,
) -> tuple[str, int]:
    """Normalize fusion mode and apply mode-specific support defaults.

    ``intersection`` (legacy): densify requires ≥2 views per 2 mm cell.
    ``union``: densify allows 1-view cells; voxel merge weights use fusion_score when present.
    Explicit ``support_min_views`` always wins.
    """
    mode = str(fusion_mode or "intersection").strip().lower()
    if mode not in ("intersection", "union"):
        raise ValueError(
            f"fusion_mode must be 'intersection' or 'union', got {fusion_mode!r}"
        )
    if support_min_views is not None:
        return mode, max(1, int(support_min_views))
    return mode, (1 if mode == "union" else 2)


@contextmanager
def _timed(store: dict[str, float], name: str) -> Iterator[None]:
    t0 = time.perf_counter()
    try:
        yield
    finally:
        store[name] = round(time.perf_counter() - t0, 3)


def run_pipeline(
    burst: Path,
    decode_root: Path,
    out_dir: Path,
    *,
    tool0_T_camera: np.ndarray | None,
    pose_mode: str = "flange_camera",
    mode: str = "sweep",
    poses: list[str] | None = None,
    z_min_m: float = 0.15,
    z_max_m: float = 0.50,
    min_modulation: float = 0.12,
    stride: int = 2,
    remove_tray_plane: bool = False,
    tray_band_m: float = 0.012,
    do_pose_refine: bool = True,
    do_consistency: bool = True,
    consistency_min_views: int = 1,
    consistency_max_dz_m: float = 0.006,
    consistency_min_keep_ratio: float = 0.05,
    voxel_m: float = 0.0015,
    trunc_m: float = 0.008,
    conf_min: float = 0.12,
    densify: bool = True,
    dense_voxel_m: float = 0.0005,
    dense_pixel_stride: int = 1,
    support_voxel_m: float = 0.002,
    support_min_views: int | None = None,
    fusion_mode: str = "union",
    score_lambda_agree: float = 0.15,
    score_lambda_contradict: float = 0.40,
    score_keep_threshold: float = 0.12,
    enable_tsdf: bool = False,
    tsdf_voxel_m: float | None = None,
    tsdf_trunc_m: float | None = None,
    cleanup_dense: bool = True,
    sor_k: int | None = None,
    sor_std: float | None = None,
    ror_min_neighbors: int | None = None,
    ror_radius_mm: float | None = None,
    component_min_points: int | None = None,
    workspace_width_m: float = 0.300,
    workspace_depth_m: float = 0.300,
    surface_filter: bool = True,
    final_component_eps_m: float | None = None,
    final_component_min_points: int | None = None,
    top_view: bool = True,
    top_view_resolution_mm: float = 0.5,
    light_smooth: bool = False,
) -> dict[str, Any]:
    fusion_mode, support_min_views = resolve_fusion_mode_defaults(
        fusion_mode, support_min_views=support_min_views
    )
    cleanup_knobs = resolve_cleanup_defaults(
        fusion_mode,
        sor_k=sor_k,
        sor_std=sor_std,
        ror_min_neighbors=ror_min_neighbors,
        component_min_points=component_min_points,
        final_component_eps_m=final_component_eps_m,
        final_component_min_points=final_component_min_points,
    )

    t0 = time.perf_counter()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # Keep one geometry product. Remove names emitted by older pipeline versions.
    for legacy_name in (
        "grape_cloud_dense.ply",
        "grape_cloud_dense_clean.ply",
        "grape_cloud.ply",
        "grape_mesh.ply",
        "grape_mesh_from_dense_clean.ply",
        "apex_top_view_depth_mm.npy",
        "apex_top_view_depth.png",
    ):
        (out_dir / legacy_name).unlink(missing_ok=True)

    stages: dict[str, Any] = {}
    timings: dict[str, float] = {}
    stages["fusion_config"] = {
        "fusion_mode": fusion_mode,
        "support_min_views": int(support_min_views),
        "support_voxel_m": float(support_voxel_m),
        "consistency_min_views": int(consistency_min_views),
        "consistency_max_dz_m": float(consistency_max_dz_m),
        "consistency_keep_mode": "soft" if fusion_mode == "union" else "hard",
        "score_lambda_agree": float(score_lambda_agree),
        "score_lambda_contradict": float(score_lambda_contradict),
        "score_keep_threshold": float(score_keep_threshold),
        "conf_min": float(conf_min),
        "note": (
            "union: soft score keep + densify 1-view cells + milder cleanup"
            if fusion_mode == "union"
            else "intersection: hard agree≥min_views + multi-view densify support"
        ),
        "cleanup_preset": cleanup_knobs["preset"],
        "cleanup": {
            "sor_k": int(cleanup_knobs["sor_k"]),
            "sor_std": float(cleanup_knobs["sor_std"]),
            "ror_min_neighbors": int(cleanup_knobs["ror_min_neighbors"]),
            "ror_radius_scale": float(cleanup_knobs["ror_radius_scale"]),
            "component_min_points": int(cleanup_knobs["component_min_points"]),
            "final_component_eps_m": float(cleanup_knobs["final_component_eps_m"]),
            "final_component_min_points": int(
                cleanup_knobs["final_component_min_points"]
            ),
            "surface_max_normal_angle_deg": float(
                cleanup_knobs["surface_max_normal_angle_deg"]
            ),
        },
    }
    print(
        f"[0] fusion_mode={fusion_mode} support_min_views={support_min_views} "
        f"consistency={'soft-score' if fusion_mode == 'union' else f'hard agree≥{consistency_min_views}'} "
        f"cleanup={cleanup_knobs['preset']}",
        flush=True,
    )

    print("[1] load raw depth maps…", flush=True)
    with _timed(timings, "load"):
        frames = load_raw_frames(
            burst,
            decode_root,
            tool0_T_camera=tool0_T_camera,
            pose_mode=pose_mode,
            mode=mode,
            poses=poses,
            z_min_m=z_min_m,
            z_max_m=z_max_m,
            min_modulation=min_modulation,
            stride=max(1, int(stride)),
        )
    stages["load"] = {"n_frames": len(frames), "stems": [f.stem for f in frames]}

    tray_plane: list[float] | None = None
    if remove_tray_plane:
        print("[1b] remove tray plane (optional)…", flush=True)
        with _timed(timings, "tray"):
            tray = remove_tray(frames, keep_band_m=float(tray_band_m))
        stages["tray"] = {
            "n_before": tray.n_before,
            "n_after": tray.n_after,
            "n_removed": tray.n_removed,
            "plane": tray.plane,
            "upright": tray.upright,
            "skipped_reason": tray.skipped_reason,
        }
        tray_plane = tray.plane
        print(f"  tray {tray.n_before} -> {tray.n_after} px", flush=True)
    else:
        timings["tray"] = 0.0
        stages["tray"] = {
            "skipped": True,
            "note": "object-only FPP; workspace uses base_link +Z (no tray reference)",
        }
        print("[1b] tray crop skipped (object-only / no tray reference)", flush=True)

    print("[2] edge-aware depth filter…", flush=True)
    with _timed(timings, "filter"):
        filter_frames(frames)
    stages["filter"] = {"ok": True}

    print("[3] depth confidence…", flush=True)
    with _timed(timings, "confidence"):
        confidence_frames(frames)
    stages["confidence"] = {
        "mean": [
            float(np.nanmean(f.conf[f.mask])) if f.conf is not None and f.mask.any() else 0.0
            for f in frames
        ]
    }

    if do_pose_refine:
        print("[4] robust pose-graph refine…", flush=True)
        with _timed(timings, "pose_refine"):
            pose_stats = refine_poses_constrained(frames)
        stages["pose_refine"] = pose_stats
        optimized_poses = pose_stats.get("optimized_poses")
        if optimized_poses:
            optimized_poses_path = out_dir / "optimized_poses.json"
            optimized_poses_path.write_text(
                json.dumps(
                    {
                        "method": pose_stats.get("method"),
                        "reference": pose_stats.get("reference"),
                        "pose_convention": pose_stats.get("pose_convention"),
                        "poses": optimized_poses,
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
            pose_stats["optimized_poses_path"] = str(optimized_poses_path)
        print(
            f"  method={pose_stats.get('method', 'legacy')} "
            f"refined={pose_stats.get('refined', 0)} "
            f"rejected={pose_stats.get('rejected', 0)} "
            f"already_aligned={pose_stats.get('skipped_aligned', 0)} "
            f"reference={pose_stats.get('reference', pose_stats.get('seed'))}",
            flush=True,
        )
    else:
        timings["pose_refine"] = 0.0
        stages["pose_refine"] = {"skipped": True}

    if do_consistency:
        keep_mode = "soft" if fusion_mode == "union" else "hard"
        print(f"[5] multi-view consistency ({keep_mode})…", flush=True)
        with _timed(timings, "consistency"):
            cons = consistency_reject(
                frames,
                max_dz_m=float(consistency_max_dz_m),
                min_views=int(consistency_min_views),
                keep_mode=keep_mode,
                lambda_agree=float(score_lambda_agree),
                lambda_contradict=float(score_lambda_contradict),
                keep_threshold=float(score_keep_threshold),
            )
        stages["consistency"] = cons
        labels = cons.get("labels") or {}
        if keep_mode == "soft":
            print(
                f"  kept={cons['kept']} killed={cons['killed']} "
                f"({100.0 * float(cons.get('kill_ratio', 0.0)):.1f}% removed by "
                f"score≥{score_keep_threshold:g}; "
                f"λ_a={score_lambda_agree:g} λ_c={score_lambda_contradict:g})",
                flush=True,
            )
            score_stats = cons.get("score") or {}
            print(
                f"  mean score kept={float(score_stats.get('mean_kept', 0.0)):.3f} "
                f"killed={float(score_stats.get('mean_killed', 0.0)):.3f}",
                flush=True,
            )
        else:
            print(
                f"  kept={cons['kept']} killed={cons['killed']} "
                f"({100.0 * float(cons.get('kill_ratio', 0.0)):.1f}% removed by "
                f"agree≥{consistency_min_views})",
                flush=True,
            )
        print(
            f"  labels (pixel×view): AGREE={labels.get('AGREE', 0)} "
            f"NOT_OBSERVED={labels.get('NOT_OBSERVED', 0)} "
            f"OCCLUDED={labels.get('OCCLUDED', 0)} "
            f"CONTRADICT={labels.get('CONTRADICT', 0)}",
            flush=True,
        )
        checked = int(cons.get("checked", 0))
        keep_ratio = float(cons.get("keep_ratio", 0.0))
        # Soft mode keeps more by design — only abort on catastrophic loss.
        abort_ratio = (
            float(consistency_min_keep_ratio) * 0.5
            if keep_mode == "soft"
            else float(consistency_min_keep_ratio)
        )
        if checked > 0 and keep_ratio < abort_ratio:
            raise RuntimeError(
                "Multi-view geometry rejected: only "
                f"{keep_ratio:.1%} of sampled depth survived consistency "
                f"(minimum {abort_ratio:.1%}, mode={keep_mode}). "
                "Check camera intrinsics, hand-eye, and FPP stereo calibration."
            )
    else:
        timings["consistency"] = 0.0
        stages["consistency"] = {"skipped": True}

    point_cloud_path = None
    point_cloud_points = None
    final_cloud: o3d.geometry.PointCloud | None = None
    workspace_info: dict[str, Any] | None = None
    if densify:
        print("[6] densify point cloud (back-project refined depth)…", flush=True)
        with _timed(timings, "densify"):
            dense = densify_from_depth(
                frames,
                conf_min=float(conf_min),
                pixel_stride=max(1, int(dense_pixel_stride)),
                voxel_m=float(dense_voxel_m),
                support_voxel_m=float(support_voxel_m),
                support_min_views=int(support_min_views),
                weight_source="auto",
            )
        drop_pct = (
            100.0 * float(dense.n_dropped_by_support) / float(dense.n_raw)
            if dense.n_raw > 0
            else 0.0
        )
        stages["densify"] = {
            "n_raw": dense.n_raw,
            "n_after_support": dense.n_after_support,
            "n_dropped_by_support": dense.n_dropped_by_support,
            "support_drop_ratio": round(drop_pct / 100.0, 4),
            "n_after_voxel": dense.n_after_voxel,
            "n_single_view": dense.n_single_view,
            "n_multi_view": dense.n_multi_view,
            "voxel_m": dense.voxel_m,
            "support_voxel_m": dense.support_voxel_m,
            "support_min_views": dense.support_min_views,
            "confidence_weighted": dense.confidence_weighted,
            "weight_source": dense.weight_source,
            "fusion_mode": fusion_mode,
        }
        print(
            f"  dense {dense.n_raw:,} -> support {dense.n_after_support:,} "
            f"(dropped {dense.n_dropped_by_support:,} / {drop_pct:.1f}% by "
            f"≥{dense.support_min_views}-view rule) "
            f"-> {dense.n_after_voxel:,} pts "
            f"(1-view={dense.n_single_view:,}, multi={dense.n_multi_view:,}, "
            f"w={dense.weight_source}, "
            f"support={dense.support_voxel_m * 1000:.2f} mm, "
            f"voxel={dense.voxel_m * 1000:.2f} mm, mode={fusion_mode})",
            flush=True,
        )

        if cleanup_dense:
            print(
                f"[6b] dense cleanup ({cleanup_knobs['preset']}): "
                "physical ROI/component -> SOR -> ROR "
                "-> surface -> tight components"
                + (" -> light smooth" if light_smooth else ""),
                flush=True,
            )
            with _timed(timings, "cleanup"):
                ror_r = None if ror_radius_mm is None else float(ror_radius_mm) * 1.0e-3
                cleaned, cstats = cleanup_dense_cloud(
                    dense.cloud,
                    do_roi=True,
                    do_sor=True,
                    do_ror=True,
                    do_surface=bool(surface_filter),
                    do_final_components=True,
                    do_smooth=bool(light_smooth),
                    tray_plane=tray_plane,
                    camera_c2w=[frame.c2w for frame in frames],
                    workspace_width_m=float(workspace_width_m),
                    workspace_depth_m=float(workspace_depth_m),
                    sor_k=int(cleanup_knobs["sor_k"]),
                    sor_std=float(cleanup_knobs["sor_std"]),
                    ror_min_neighbors=int(cleanup_knobs["ror_min_neighbors"]),
                    ror_radius_m=ror_r,
                    ror_radius_scale=float(cleanup_knobs["ror_radius_scale"]),
                    component_min_points=int(cleanup_knobs["component_min_points"]),
                    final_component_eps_m=float(
                        cleanup_knobs["final_component_eps_m"]
                    ),
                    final_component_min_points=int(
                        cleanup_knobs["final_component_min_points"]
                    ),
                    surface_max_normal_angle_deg=float(
                        cleanup_knobs["surface_max_normal_angle_deg"]
                    ),
                )
                point_cloud_path = out_dir / "dense_point_cloud.ply"
                o3d.io.write_point_cloud(str(point_cloud_path), cleaned)
                point_cloud_points = int(len(cleaned.points))
                final_cloud = cleaned
                workspace_info = cstats.steps.get("workspace")
            stages["cleanup"] = {
                "preset": cleanup_knobs["preset"],
                "knobs": {
                    "sor_k": int(cleanup_knobs["sor_k"]),
                    "sor_std": float(cleanup_knobs["sor_std"]),
                    "ror_min_neighbors": int(cleanup_knobs["ror_min_neighbors"]),
                    "ror_radius_scale": float(cleanup_knobs["ror_radius_scale"]),
                    "component_min_points": int(cleanup_knobs["component_min_points"]),
                    "final_component_eps_m": float(
                        cleanup_knobs["final_component_eps_m"]
                    ),
                    "final_component_min_points": int(
                        cleanup_knobs["final_component_min_points"]
                    ),
                    "surface_max_normal_angle_deg": float(
                        cleanup_knobs["surface_max_normal_angle_deg"]
                    ),
                },
                "n_in": cstats.n_in,
                "n_after_roi": cstats.n_after_roi,
                "n_after_sor": cstats.n_after_sor,
                "n_after_ror": cstats.n_after_ror,
                "n_after_surface": cstats.n_after_surface,
                "n_after_final_components": cstats.n_after_final_components,
                "n_out": cstats.n_out,
                "n_components_kept": cstats.n_components_kept,
                "mean_nn_m": cstats.mean_nn_m,
                "steps": cstats.steps,
                "path": str(point_cloud_path),
            }
            print(
                f"  cleanup {cstats.n_in:,} -> {cstats.n_out:,} pts "
                f"(roi={cstats.n_after_roi:,}, sor={cstats.n_after_sor:,}, "
                f"ror={cstats.n_after_ror:,}, surface={cstats.n_after_surface:,}, "
                f"components={cstats.n_after_final_components:,}, "
                f"preset={cleanup_knobs['preset']})",
                flush=True,
            )
        else:
            with _timed(timings, "write_point_cloud"):
                point_cloud_path = out_dir / "dense_point_cloud.ply"
                o3d.io.write_point_cloud(str(point_cloud_path), dense.cloud)
                point_cloud_points = int(len(dense.cloud.points))
                final_cloud = dense.cloud
            timings["cleanup"] = 0.0
            stages["cleanup"] = {"skipped": True, "path": str(point_cloud_path)}
    else:
        timings["densify"] = 0.0
        timings["cleanup"] = 0.0
        stages["densify"] = {"skipped": True}
        stages["cleanup"] = {"skipped": True}

    if enable_tsdf:
        print("[6c] optional weighted TSDF (score-gated, no free-space carve)…", flush=True)
        try:
            with _timed(timings, "tsdf"):
                tsdf_voxel = (
                    float(tsdf_voxel_m) if tsdf_voxel_m is not None else float(voxel_m)
                )
                tsdf_trunc = (
                    float(tsdf_trunc_m) if tsdf_trunc_m is not None else float(trunc_m)
                )
                tsdf = fuse_frames_tsdf(
                    frames,
                    voxel_m=tsdf_voxel,
                    trunc_m=tsdf_trunc,
                    conf_min=float(conf_min),
                    weight_source="auto",
                    depth_trunc_m=float(z_max_m),
                )
                tsdf_cloud_path = out_dir / "tsdf_cloud.ply"
                tsdf_mesh_path = out_dir / "tsdf_mesh.ply"
                o3d.io.write_point_cloud(str(tsdf_cloud_path), tsdf.cloud)
                o3d.io.write_triangle_mesh(str(tsdf_mesh_path), tsdf.mesh)
            stages["tsdf"] = {
                "ok": True,
                "n_frames": tsdf.n_frames,
                "n_pixels_integrated": tsdf.n_pixels_integrated,
                "voxel_m": tsdf.voxel_m,
                "trunc_m": tsdf.trunc_m,
                "weight_source": tsdf.weight_source,
                "conf_min": tsdf.conf_min,
                "cloud": str(tsdf_cloud_path),
                "mesh": str(tsdf_mesh_path),
                "cloud_points": int(len(tsdf.cloud.points)),
                "mesh_vertices": int(len(tsdf.mesh.vertices)),
                "note": (
                    "Secondary product. Primary geometry remains dense_point_cloud.ply. "
                    "Depth=0 for low-score pixels (unknown; no free-space carve)."
                ),
            }
            print(
                f"  tsdf {tsdf.n_pixels_integrated:,} px -> "
                f"{int(len(tsdf.cloud.points)):,} pts / "
                f"{int(len(tsdf.mesh.vertices)):,} verts "
                f"(voxel={tsdf.voxel_m * 1000:.2f} mm, trunc={tsdf.trunc_m * 1000:.2f} mm, "
                f"w={tsdf.weight_source}, gate≥{tsdf.conf_min:.2f})",
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001 — optional path
            timings.setdefault("tsdf", 0.0)
            stages["tsdf"] = {"ok": False, "error": str(exc)}
            print(f"  tsdf skipped: {exc}", flush=True)
    else:
        timings["tsdf"] = 0.0
        stages["tsdf"] = {"skipped": True}

    if top_view and final_cloud is not None and workspace_info is not None:
        print("[7] top-view depth map...", flush=True)
        with _timed(timings, "top_view"):
            top = rasterize_top_view(
                final_cloud,
                workspace_info,
                out_dir,
                resolution_m=float(top_view_resolution_mm) * 1.0e-3,
            )
        stages["top_view"] = {
            "depth_mm": str(top.depth_path),
            "preview": str(top.preview_path),
            "size_px": [top.width_px, top.height_px],
            "resolution_mm": top.resolution_mm,
            "valid_pixels": top.valid_pixels,
            "min_height_mm": top.min_height_mm,
            "max_height_mm": top.max_height_mm,
            "meaning": (
                "topmost height above object base (base_link +Z); no tray"
                if str(workspace_info.get("mode", "")) == "object_gravity"
                else "topmost surface height above fitted sample stage"
            ),
            "workspace_mode": workspace_info.get("mode"),
        }
    else:
        timings["top_view"] = 0.0
        stages["top_view"] = {"skipped": True}

    pipeline_steps = [
        "load",
        "tray_crop",
        "edge_aware_filter",
        "confidence",
        "pose_refine",
        "consistency",
        "densify_backproject",
        "physical_roi_sor_ror_surface_tight_components",
        "optional_weighted_tsdf",
        "top_view_depth",
    ]

    summary = {
        "ok": True,
        "method": "fpp_mesh_refine",
        "fusion_mode": fusion_mode,
        "pipeline": pipeline_steps,
        "burst": str(burst),
        "decode": str(decode_root),
        "out": str(out_dir),
        "pose_mode": pose_mode,
        "mode": mode,
        "stages": stages,
        "stage_timings_s": timings,
        "point_cloud": str(point_cloud_path) if point_cloud_path else None,
        "point_cloud_points": point_cloud_points,
        "elapsed_s": round(time.perf_counter() - t0, 3),
        "drop_accounting": {
            "consistency_killed": (stages.get("consistency") or {}).get("killed"),
            "consistency_kill_ratio": (stages.get("consistency") or {}).get("kill_ratio"),
            "densify_dropped_by_support": (stages.get("densify") or {}).get(
                "n_dropped_by_support"
            ),
            "densify_support_drop_ratio": (stages.get("densify") or {}).get(
                "support_drop_ratio"
            ),
        },
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(
        f"Done points={summary.get('point_cloud_points')} "
        f"t={summary['elapsed_s']}s -> {point_cloud_path}",
        flush=True,
    )
    return summary
