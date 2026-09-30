"""Canonical per-pin FPP decode stage used by every sidecar entry point.

Loads and undistorts one burst, decodes PSP, triangulates depth, and writes the
standard ``fpp_*`` maps. This module performs no hardware I/O.
"""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

from .capture import load_burst
from .decode import decode_burst
from .depth import depth_from_decode
from .geometry import StereoGeometry
from .io_maps import write_decode_maps, write_depth_maps
from .undistort import UndistortMaps, undistort_burst


def stereo_for_undistorted_burst(
    stereo: StereoGeometry | None,
    undistort_maps: UndistortMaps | None,
) -> StereoGeometry | None:
    """Return geometry whose camera model matches the decoded image grid."""
    if stereo is None or undistort_maps is None:
        return stereo
    return replace(
        stereo,
        camera_k=undistort_maps.new_k.copy(),
        camera_d=np.zeros(5, dtype=np.float64),
        undistorted=True,
    )


def decode_pin(
    burst_dir: Path,
    start: Path,
    dest: Path,
    *,
    stereo: StereoGeometry | None,
    channel: str = "auto",
    min_modulation: float = 0.15,
    min_phase_quality: float = 0.04,
) -> dict[str, Any]:
    """Decode one FPP pin burst and write its canonical output directory."""
    started = time.perf_counter()
    burst_dir = Path(burst_dir).resolve()
    start = Path(start)
    dest = Path(dest).resolve()

    loaded = load_burst(burst_dir, start=start, channel=channel)
    undistort_maps = undistort_burst(loaded)
    effective_stereo = stereo_for_undistorted_burst(stereo, undistort_maps)
    decoded = decode_burst(
        loaded,
        min_modulation=float(min_modulation),
        min_phase_quality=float(min_phase_quality),
    )
    depth = depth_from_decode(decoded, stereo=effective_stereo)

    write_decode_maps(dest, decoded, stem="fpp")
    write_depth_maps(dest, depth, stem="fpp")

    camera_k = (
        effective_stereo.camera_k
        if effective_stereo is not None
        else (undistort_maps.new_k if undistort_maps is not None else None)
    )
    camera_k_path: Path | None = None
    if camera_k is not None:
        camera_k_path = dest / "fpp_camera_k.npy"
        np.save(camera_k_path, np.asarray(camera_k, dtype=np.float64))

    elapsed = round(time.perf_counter() - started, 3)
    summary: dict[str, Any] = {
        "stem": start.stem,
        "start": start.name,
        "burst_dir": str(burst_dir),
        "decode_channel": loaded.decode_channel,
        "min_modulation": float(min_modulation),
        "min_phase_quality": float(min_phase_quality),
        "valid_px": int(depth.mask.sum()),
        "mode": depth.mode,
        "units": depth.units,
        "out": str(dest),
        "camera_k": str(camera_k_path) if camera_k_path is not None else None,
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
    return summary
