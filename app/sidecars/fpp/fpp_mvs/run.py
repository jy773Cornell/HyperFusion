# End-to-end FPP MVS: decode → dense clean cloud (sidecar / fpp / fpp_mvs).
# Writes multiview/processed/{decode,fusion} + metadata.json. No robot I/O.
"""Orchestrate decode + classical fusion with per-stage timings."""

from __future__ import annotations

import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
FPP = HERE.parent
DF = FPP / "depth_fusion"
MESH = DF / "fpp_mesh_refine"
for p in (FPP, DF, MESH, HERE):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from fpp_depth.paths import DEFAULT_STEREO_YAML, resolve_stereo_yaml  # noqa: E402
from fpp_mvs.decode_stage import decode_mvs_burst  # noqa: E402
from fpp_mvs.paths import (  # noqa: E402
    decode_root,
    default_hand_eye,
    default_stereo,
    fusion_root,
    has_fpp_burst,
    processed_root,
    resolve_datafolder,
)
from hyperfusion_depth_fusion.poses import load_flange_T_camera_yaml  # noqa: E402
from hyperfusion_fpp_mesh.pipeline import run_pipeline  # noqa: E402


def _utc_now() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def run_fpp_mvs_pipeline(
    input_path: Path,
    *,
    stereo: Path | None = None,
    no_stereo: bool = False,
    hand_eye: Path | None = None,
    pose_mode: str = "auto",
    mode: str = "sweep",
    channel: str = "auto",
    min_modulation: float = 0.15,
    min_phase_quality: float = 0.04,
    skip_decode: bool = False,
    skip_fusion: bool = False,
    require_fpp: bool = True,
) -> dict[str, Any]:
    t_all = time.perf_counter()
    datafolder, burst = resolve_datafolder(input_path)
    if mode != "sweep":
        raise ValueError(
            "The unified FPP MVS pipeline currently fuses sweep views only "
            "(00026…00156); mode must be 'sweep'."
        )

    if require_fpp and not has_fpp_burst(burst):
        return {
            "ok": False,
            "skipped": True,
            "reason": "no_fpp_burst",
            "datafolder": str(datafolder),
            "burst": str(burst),
            "elapsed_s": 0.0,
        }

    proc = processed_root(burst)
    dec = decode_root(burst)
    fus = fusion_root(burst)
    proc.mkdir(parents=True, exist_ok=True)

    stages: dict[str, Any] = {}
    stereo_path: Path | None = None
    if not no_stereo:
        if stereo is not None:
            stereo_path = resolve_stereo_yaml(stereo)
        else:
            stereo_path = default_stereo(burst)
            if stereo_path is None:
                stereo_path = resolve_stereo_yaml(None)
        if stereo_path is None:
            raise FileNotFoundError(
                "No dataset camera_cal stereo or app stereo calibration found. "
                f"App default checked: {DEFAULT_STEREO_YAML}. Pass --stereo PATH."
            )

    # --- decode ---
    if skip_decode:
        stages["decode"] = {"skipped": True, "elapsed_s": 0.0, "decode_root": str(dec)}
        print("[decode] skipped", flush=True)
    else:
        print(f"[decode] -> {dec}", flush=True)
        stages["decode"] = decode_mvs_burst(
            burst,
            dec,
            stereo_yaml=stereo_path,
            channel=channel,
            min_modulation=min_modulation,
            min_phase_quality=min_phase_quality,
        )

    # --- fusion (dense clean cloud) ---
    he_path = hand_eye
    tool0 = None
    if not skip_fusion:
        if pose_mode != "json":
            if he_path is None:
                he_path = default_hand_eye(burst)
            if he_path is not None and Path(he_path).is_file():
                tool0 = load_flange_T_camera_yaml(Path(he_path))
                print(f"[fusion] hand-eye: {he_path}", flush=True)
            elif pose_mode == "flange_camera":
                raise FileNotFoundError(
                    "Need --hand-eye flange_T_camera.yaml (or --pose-mode json)"
                )
            else:
                he_path = None
                print("[fusion] no dataset hand-eye; using scan JSON/app pose", flush=True)

        print(f"[fusion] -> {fus}", flush=True)
        t_fus = time.perf_counter()
        fusion_summary = run_pipeline(
            burst,
            dec,
            fus,
            tool0_T_camera=tool0,
            pose_mode=pose_mode,
            mode=mode,
            densify=True,
            cleanup_dense=True,
            min_modulation=float(min_modulation),
        )
        fusion_wall = round(time.perf_counter() - t_fus, 3)
        # Prefer pipeline stage timings when present
        stage_timings = fusion_summary.get("stage_timings_s") or {}
        stages["fusion"] = {
            "ok": bool(fusion_summary.get("ok")),
            "elapsed_s": fusion_wall,
            "stage_timings_s": stage_timings,
            "point_cloud": fusion_summary.get("point_cloud"),
            "point_cloud_points": fusion_summary.get("point_cloud_points"),
            "out": str(fus),
            "summary": str(fus / "summary.json"),
        }
    else:
        stages["fusion"] = {"skipped": True, "elapsed_s": 0.0}

    total = round(time.perf_counter() - t_all, 3)
    metadata: dict[str, Any] = {
        "ok": True,
        "schema": "hyperfusion.fpp_mvs.processed.v1",
        "created_utc": _utc_now(),
        "datafolder": str(datafolder),
        "burst": str(burst),
        "processed": str(proc),
        "decode": str(dec),
        "fusion": str(fus),
        "stereo": str(stereo_path) if stereo_path else None,
        "hand_eye": str(he_path) if he_path else None,
        "calibration_source": {
            "stereo": (
                "dataset_camera_cal"
                if stereo_path is not None
                and "camera_cal" in {part.lower() for part in stereo_path.parts}
                else "app_calibration"
            ),
            "pose": "dataset_camera_cal" if he_path else "scan_json_app_parameters",
        },
        "pose_mode": pose_mode,
        "mode": mode,
        "fused_stems": (
            fusion_summary.get("stages", {}).get("load", {}).get("stems")
            if not skip_fusion
            else None
        ),
        "channel": channel,
        "pipeline": [
            "decode_fpp_depth",
            "tray_crop",
            "edge_aware_filter",
            "confidence",
            "pose_refine",
            "consistency",
            "densify_backproject",
            "roi_sor_ror_cleanup",
        ],
        "stages": {
            "decode": {
                "elapsed_s": stages.get("decode", {}).get("elapsed_s"),
                "n_pins": stages.get("decode", {}).get("n_pins"),
                "skipped": stages.get("decode", {}).get("skipped", False),
                "pin_timings_s": stages.get("decode", {}).get("pin_timings_s"),
            },
            "fusion": {
                "elapsed_s": stages.get("fusion", {}).get("elapsed_s"),
                "stage_timings_s": stages.get("fusion", {}).get("stage_timings_s"),
                "skipped": stages.get("fusion", {}).get("skipped", False),
                "point_cloud_points": stages.get("fusion", {}).get("point_cloud_points"),
                "point_cloud": stages.get("fusion", {}).get("point_cloud"),
            },
        },
        "elapsed_s": total,
        "primary_cloud": stages.get("fusion", {}).get("point_cloud"),
        "detail": stages,
    }
    meta_path = proc / "metadata.json"
    meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(
        f"[done] t={total:.2f}s decode={metadata['stages']['decode']['elapsed_s']}s "
        f"fusion={metadata['stages']['fusion']['elapsed_s']}s -> {meta_path}",
        flush=True,
    )
    metadata["metadata"] = str(meta_path)
    return metadata
