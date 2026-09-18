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

from .cleanup import cleanup_dense_cloud
from .confidence import confidence_frames
from .consistency import consistency_reject
from .densify import densify_from_depth
from .filter import filter_frames
from .frames import load_raw_frames
from .pose_refine import refine_poses_constrained
from .tray import remove_tray


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
    z_min_m: float = 0.35,
    z_max_m: float = 0.60,
    min_modulation: float = 0.12,
    stride: int = 2,
    remove_tray_plane: bool = True,
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
    cleanup_dense: bool = True,
    sor_k: int = 25,
    sor_std: float = 1.75,
    ror_min_neighbors: int = 12,
    ror_radius_mm: float | None = None,
    component_min_points: int = 800,
    light_smooth: bool = False,
) -> dict[str, Any]:
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
    ):
        (out_dir / legacy_name).unlink(missing_ok=True)

    stages: dict[str, Any] = {}
    timings: dict[str, float] = {}

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

    if remove_tray_plane:
        print("[1b] remove tray plane…", flush=True)
        with _timed(timings, "tray"):
            tray = remove_tray(frames, keep_band_m=float(tray_band_m))
        stages["tray"] = {
            "n_before": tray.n_before,
            "n_after": tray.n_after,
            "n_removed": tray.n_removed,
            "plane": tray.plane,
        }
        print(f"  tray {tray.n_before} -> {tray.n_after} px", flush=True)
    else:
        timings["tray"] = 0.0
        stages["tray"] = {"skipped": True}

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
        print("[4] constrained pose refine…", flush=True)
        with _timed(timings, "pose_refine"):
            pose_stats = refine_poses_constrained(frames)
        stages["pose_refine"] = pose_stats
        print(
            f"  refined={pose_stats['refined']} rejected={pose_stats['rejected']}",
            flush=True,
        )
    else:
        timings["pose_refine"] = 0.0
        stages["pose_refine"] = {"skipped": True}

    if do_consistency:
        print("[5] multi-view consistency…", flush=True)
        with _timed(timings, "consistency"):
            cons = consistency_reject(
                frames,
                max_dz_m=float(consistency_max_dz_m),
                min_views=int(consistency_min_views),
            )
        stages["consistency"] = cons
        print(f"  kept={cons['kept']} killed={cons['killed']}", flush=True)
        checked = int(cons.get("checked", 0))
        keep_ratio = float(cons.get("kept", 0)) / checked if checked > 0 else 0.0
        stages["consistency"]["keep_ratio"] = keep_ratio
        if checked > 0 and keep_ratio < float(consistency_min_keep_ratio):
            raise RuntimeError(
                "Multi-view geometry rejected: only "
                f"{keep_ratio:.1%} of sampled depth agrees across sweep views "
                f"(minimum {float(consistency_min_keep_ratio):.1%}). "
                "Check camera intrinsics, hand-eye, and FPP stereo calibration."
            )
    else:
        timings["consistency"] = 0.0
        stages["consistency"] = {"skipped": True}

    point_cloud_path = None
    point_cloud_points = None
    if densify:
        print("[6] densify point cloud (back-project refined depth)…", flush=True)
        with _timed(timings, "densify"):
            dense = densify_from_depth(
                frames,
                conf_min=float(conf_min),
                pixel_stride=max(1, int(dense_pixel_stride)),
                voxel_m=float(dense_voxel_m),
            )
        stages["densify"] = {
            "n_raw": dense.n_raw,
            "n_after_voxel": dense.n_after_voxel,
            "voxel_m": dense.voxel_m,
        }
        print(
            f"  dense {dense.n_raw:,} -> {dense.n_after_voxel:,} pts "
            f"(voxel={dense.voxel_m * 1000:.2f} mm)",
            flush=True,
        )

        if cleanup_dense:
            print(
                "[6b] dense cleanup: ROI/component -> SOR -> ROR"
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
                    do_smooth=bool(light_smooth),
                    sor_k=int(sor_k),
                    sor_std=float(sor_std),
                    ror_min_neighbors=int(ror_min_neighbors),
                    ror_radius_m=ror_r,
                    component_min_points=int(component_min_points),
                )
                point_cloud_path = out_dir / "dense_point_cloud.ply"
                o3d.io.write_point_cloud(str(point_cloud_path), cleaned)
                point_cloud_points = int(len(cleaned.points))
            stages["cleanup"] = {
                "n_in": cstats.n_in,
                "n_after_roi": cstats.n_after_roi,
                "n_after_sor": cstats.n_after_sor,
                "n_after_ror": cstats.n_after_ror,
                "n_out": cstats.n_out,
                "n_components_kept": cstats.n_components_kept,
                "mean_nn_m": cstats.mean_nn_m,
                "steps": cstats.steps,
                "path": str(point_cloud_path),
            }
            print(
                f"  cleanup {cstats.n_in:,} -> {cstats.n_out:,} pts "
                f"(roi={cstats.n_after_roi:,}, sor={cstats.n_after_sor:,}, "
                f"ror={cstats.n_after_ror:,})",
                flush=True,
            )
        else:
            with _timed(timings, "write_point_cloud"):
                point_cloud_path = out_dir / "dense_point_cloud.ply"
                o3d.io.write_point_cloud(str(point_cloud_path), dense.cloud)
                point_cloud_points = int(len(dense.cloud.points))
            timings["cleanup"] = 0.0
            stages["cleanup"] = {"skipped": True, "path": str(point_cloud_path)}
    else:
        timings["densify"] = 0.0
        timings["cleanup"] = 0.0
        stages["densify"] = {"skipped": True}
        stages["cleanup"] = {"skipped": True}

    pipeline_steps = [
        "load",
        "tray_crop",
        "edge_aware_filter",
        "confidence",
        "pose_refine",
        "consistency",
        "densify_backproject",
        "roi_sor_ror_cleanup",
    ]

    summary = {
        "ok": True,
        "method": "fpp_mesh_refine",
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
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(
        f"Done points={summary.get('point_cloud_points')} "
        f"t={summary['elapsed_s']}s -> {point_cloud_path}",
        flush=True,
    )
    return summary
