# Decode stage for FPP MVS (sidecar / fpp / fpp_mvs). Writes processed/decode/<stem>/.
"""Per-pin FPP decode → metric depth under a flat decode root."""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

from fpp_depth.capture import list_burst_jobs, load_burst
from fpp_depth.decode import decode_burst
from fpp_depth.depth import depth_from_decode
from fpp_depth.geometry import StereoGeometry
from fpp_depth.io_maps import write_decode_maps, write_depth_maps
from fpp_depth.undistort import undistort_burst


def decode_mvs_burst(
    burst: Path,
    decode_out: Path,
    *,
    stereo_yaml: Path | None,
    channel: str = "auto",
    min_modulation: float = 0.15,
    min_phase_quality: float = 0.04,
) -> dict[str, Any]:
    """Decode every FPP pin burst into ``decode_out/<start_stem>/``."""
    t0 = time.perf_counter()
    burst = Path(burst).resolve()
    decode_out = Path(decode_out).resolve()
    decode_out.mkdir(parents=True, exist_ok=True)

    jobs = list_burst_jobs(burst)
    if not jobs:
        raise FileNotFoundError(f"No FPP pin bursts in {burst}")

    stereo = StereoGeometry.load(Path(stereo_yaml)) if stereo_yaml is not None else None
    pin_results: list[dict[str, Any]] = []
    pin_timings: list[dict[str, Any]] = []

    for burst_dir, start in jobs:
        tp = time.perf_counter()
        loaded = load_burst(burst_dir, start=start, channel=channel)
        undistort_maps = undistort_burst(loaded)
        decoded = decode_burst(
            loaded,
            min_modulation=min_modulation,
            min_phase_quality=min_phase_quality,
        )
        effective_stereo = stereo
        if stereo is not None and undistort_maps is not None:
            # The scan may carry a newer BFS K/D than the checkerboard stereo
            # calibration. R/t remain camera→projector, but triangulation and
            # later back-projection must use this burst's undistorted pinhole K.
            effective_stereo = replace(
                stereo,
                camera_k=undistort_maps.new_k.copy(),
                camera_d=np.zeros(5, dtype=np.float64),
                undistorted=True,
            )
        depth = depth_from_decode(decoded, stereo=effective_stereo)
        dest = decode_out / start.stem
        write_decode_maps(dest, decoded, stem="fpp")
        write_depth_maps(dest, depth, stem="fpp")
        camera_k = (
            effective_stereo.camera_k
            if effective_stereo is not None
            else (
                undistort_maps.new_k
                if undistort_maps is not None
                else None
            )
        )
        if camera_k is not None:
            np.save(dest / "fpp_camera_k.npy", np.asarray(camera_k, dtype=np.float64))
        elapsed = round(time.perf_counter() - tp, 3)
        summary: dict[str, Any] = {
            "stem": start.stem,
            "start": start.name,
            "decode_channel": loaded.decode_channel,
            "valid_px": int(depth.mask.sum()),
            "mode": depth.mode,
            "out": str(dest),
            "elapsed_s": elapsed,
        }
        if depth.units == "mm":
            finite = depth.depth[np.isfinite(depth.depth)]
            if finite.size:
                summary["z_mm"] = {
                    "med": float(np.median(finite)),
                    "p05": float(np.percentile(finite, 5)),
                    "p95": float(np.percentile(finite, 95)),
                }
        pin_results.append(summary)
        pin_timings.append({"stem": start.stem, "elapsed_s": elapsed})
        print(
            f"  decode {start.stem}: valid={summary['valid_px']:,} px  t={elapsed:.2f}s",
            flush=True,
        )

    total = round(time.perf_counter() - t0, 3)
    return {
        "ok": True,
        "n_pins": len(pin_results),
        "stereo": str(stereo_yaml) if stereo_yaml else None,
        "decode_root": str(decode_out),
        "pins": pin_results,
        "elapsed_s": total,
        "pin_timings_s": pin_timings,
    }
