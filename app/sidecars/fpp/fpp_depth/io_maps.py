"""Write FPP maps (npy + preview PNG)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .decode import DecodeResult
from .depth import DepthResult


def _save_preview(path: Path, data: np.ndarray, mask: np.ndarray, title: str) -> None:
    """Preview PNG: invalid / unlit pixels are black (not white)."""
    vis = np.array(data, dtype=np.float32, copy=True)
    vis[~mask | ~np.isfinite(vis)] = np.nan
    finite = vis[np.isfinite(vis)]
    vmin = vmax = None
    if finite.size >= 32:
        vmin, vmax = np.percentile(finite, (2.0, 98.0))
        if not np.isfinite(vmin) or not np.isfinite(vmax) or vmin >= vmax:
            vmin = vmax = None
    cmap = plt.get_cmap("turbo").copy()
    cmap.set_bad(color="black")
    fig, ax = plt.subplots(figsize=(8, 6), facecolor="black")
    ax.set_facecolor("black")
    im = ax.imshow(vis, cmap=cmap, vmin=vmin, vmax=vmax)
    ax.set_title(title, color="white")
    ax.axis("off")
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.ax.yaxis.set_tick_params(color="white")
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color="white")
    fig.tight_layout()
    fig.savefig(path, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)


def write_decode_maps(out_dir: Path, decoded: DecodeResult, stem: str = "fpp") -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    np.save(out_dir / f"{stem}_mask.npy", decoded.mask)
    np.save(out_dir / f"{stem}_projector_u.npy", decoded.projector_u)
    np.save(out_dir / f"{stem}_projector_v.npy", decoded.projector_v)
    np.save(out_dir / f"{stem}_phase.npy", decoded.wrapped_phase)
    np.save(out_dir / f"{stem}_phase_v.npy", decoded.wrapped_phase_v)
    np.save(out_dir / f"{stem}_modulation.npy", decoded.modulation)
    np.save(out_dir / f"{stem}_phase_quality.npy", decoded.phase_quality)
    _save_preview(out_dir / f"{stem}_mask.png", decoded.mask.astype(np.float32), decoded.mask, "FPP mask")
    _save_preview(out_dir / f"{stem}_projector_u.png", decoded.projector_u, decoded.mask, "Projector u (px)")
    if np.isfinite(decoded.projector_v).any():
        _save_preview(out_dir / f"{stem}_projector_v.png", decoded.projector_v, decoded.mask, "Projector v (px)")
    _save_preview(out_dir / f"{stem}_phase.png", decoded.wrapped_phase, decoded.mask, "Wrapped u phase")


def write_depth_maps(out_dir: Path, depth: DepthResult, stem: str = "fpp") -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    np.save(out_dir / f"{stem}_depth.npy", depth.depth)
    if depth.units == "mm":
        _save_preview(out_dir / f"{stem}_depth.png", depth.depth, depth.mask, "Camera Z (mm)")
    else:
        _save_preview(
            out_dir / f"{stem}_depth.png",
            depth.depth,
            depth.mask,
            "Projector u (px)",
        )
