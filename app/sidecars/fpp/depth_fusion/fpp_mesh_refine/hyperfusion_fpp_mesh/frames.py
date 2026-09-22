# Load FPP decode RGB-D frames for mesh refine (sidecar / depth_fusion / fpp_mesh_refine).
# No robot I/O.
"""Load calibrated per-pose depth (+ optional modulation / white RGB)."""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_DF = Path(__file__).resolve().parents[2]
if str(_DF) not in sys.path:
    sys.path.insert(0, str(_DF))

from hyperfusion_depth_fusion.poses import load_pose_meta, resolve_camera_c2w  # noqa: E402
from hyperfusion_depth_fusion.tsdf import (  # noqa: E402
    _load_color_undistorted,
    discover_pose_dirs,
    pinhole_for_depth,
)


@dataclass
class DepthFrame:
    stem: str
    depth_m: np.ndarray  # HxW float32, NaN invalid
    mask: np.ndarray  # HxW bool
    modulation: np.ndarray | None
    color: np.ndarray  # HxW x3 uint8
    K: np.ndarray  # 3x3
    c2w: np.ndarray  # 4x4
    pose_source: str
    conf: np.ndarray | None = None  # HxW float32 [0,1]
    phase_quality: np.ndarray | None = None  # HxW PSP fringe quality
    meta: dict = field(default_factory=dict)


def select_stems(all_stems: list[str], *, mode: str, poses: list[str] | None) -> list[str]:
    if poses:
        want = {p.strip() for p in poses if p.strip()}
        return [s for s in all_stems if s in want]
    if mode == "sweep":
        return [s for s in all_stems if s != "00000"]
    if mode == "apex":
        return [s for s in all_stems if s == "00000"][:1] or all_stems[:1]
    return list(all_stems)


def load_raw_frames(
    burst: Path,
    decode_root: Path,
    *,
    tool0_T_camera: np.ndarray | None,
    pose_mode: str,
    mode: str = "sweep",
    poses: list[str] | None = None,
    z_min_m: float = 0.35,
    z_max_m: float = 0.60,
    min_modulation: float = 0.0,
    stride: int = 1,
) -> list[DepthFrame]:
    """Load raw FPP depth maps (pre-filter)."""
    dirs = discover_pose_dirs(decode_root)
    stems = select_stems([d.name for d in dirs], mode=mode, poses=poses)
    want = set(stems)
    frames: list[DepthFrame] = []
    for pose_dir in dirs:
        stem = pose_dir.name
        if stem not in want:
            continue
        try:
            meta = load_pose_meta(burst, stem)
        except FileNotFoundError:
            continue
        depth_mm = np.load(pose_dir / "fpp_depth.npy").astype(np.float32)
        mask = np.load(pose_dir / "fpp_mask.npy").astype(bool)
        if depth_mm.shape != mask.shape:
            continue
        mod = None
        mod_path = pose_dir / "fpp_modulation.npy"
        if mod_path.is_file():
            mod = np.load(mod_path).astype(np.float32)
            if mod.shape == mask.shape and float(min_modulation) > 0.0:
                mask = mask & (mod >= float(min_modulation))
        phase_quality = None
        phase_quality_path = pose_dir / "fpp_phase_quality.npy"
        if phase_quality_path.is_file():
            phase_quality = np.load(phase_quality_path).astype(np.float32)
            if phase_quality.shape != mask.shape:
                phase_quality = None

        depth_m = np.full(depth_mm.shape, np.nan, dtype=np.float32)
        valid = mask & np.isfinite(depth_mm)
        depth_m[valid] = depth_mm[valid] * 1.0e-3
        depth_m[valid & ((depth_m < z_min_m) | (depth_m > z_max_m))] = np.nan
        mask = np.isfinite(depth_m)

        color = _load_color_undistorted(burst, stem, depth_mm.shape, meta)
        decoded_k_path = pose_dir / "fpp_camera_k.npy"
        if decoded_k_path.is_file():
            K = np.load(decoded_k_path).astype(np.float64).reshape(3, 3)
        else:
            pin = pinhole_for_depth(meta, (depth_mm.shape[1], depth_mm.shape[0]))
            K = np.asarray(pin.intrinsic_matrix, dtype=np.float64)
        c2w, src = resolve_camera_c2w(meta, tool0_T_camera=tool0_T_camera, pose_mode=pose_mode)

        if stride > 1:
            depth_m = depth_m[::stride, ::stride].copy()
            mask = mask[::stride, ::stride].copy()
            color = color[::stride, ::stride].copy()
            if mod is not None:
                mod = mod[::stride, ::stride].copy()
            if phase_quality is not None:
                phase_quality = phase_quality[::stride, ::stride].copy()
            K = K.copy()
            K[0, 0] /= float(stride)
            K[1, 1] /= float(stride)
            K[0, 2] /= float(stride)
            K[1, 2] /= float(stride)

        if int(mask.sum()) < 1000:
            continue
        frames.append(
            DepthFrame(
                stem=stem,
                depth_m=depth_m,
                mask=mask,
                modulation=mod,
                color=color,
                K=K,
                c2w=c2w.copy(),
                pose_source=src,
                phase_quality=phase_quality,
                meta=meta,
            )
        )
    if not frames:
        raise RuntimeError("No usable raw depth frames")
    return frames
