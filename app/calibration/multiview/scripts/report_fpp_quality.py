# Quality report for per-pose FPP plane calib (residual + preview PNGs).
# Scripts layer. Reads *_fpp_calib.npz only — no TIFF reload, no robot motion.
"""Write fpp_calib_quality.json and quality/*.png from existing pose npz files."""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIDECAR_FPP = ROOT.parents[1] / "sidecars" / "fpp"
DEFAULT_DIR = ROOT / "FPP"

if str(SIDECAR_FPP) not in sys.path:
    sys.path.insert(0, str(SIDECAR_FPP))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from hyperfusion_fpp.calibrate import (  # noqa: E402
    INDEX_NAME,
    FppCalibration,
    _intersect_plane_z,
    evaluate_calibration,
    pose_calib_path,
)


def _residual_map(calib: FppCalibration, step: int = 4) -> np.ndarray:
    """Homography residual on tray XY (not the degenerate 3×4 P)."""
    h, w = calib.mask.shape
    residual = np.full((h, w), np.nan, dtype=np.float32)
    if calib.camera_k is None:
        return residual
    vs, us = np.nonzero(calib.mask)
    if vs.size == 0:
        return residual
    keep = (vs % step == 0) & (us % step == 0)
    vs, us = vs[keep], us[keep]
    pixels = np.stack([us.astype(np.float64), vs.astype(np.float64)], axis=1)
    xyz = _intersect_plane_z(calib.camera_k, calib.camera_r, calib.camera_t, pixels, calib.plane_z_m)
    obs = calib.u_reference[vs, us].astype(np.float64)
    finite = np.isfinite(xyz).all(axis=1) & np.isfinite(obs)
    vs, us, xyz, obs = vs[finite], us[finite], xyz[finite], obs[finite]
    if xyz.shape[0] < 16:
        return residual
    x, y = xyz[:, 0], xyz[:, 1]
    ones = np.ones_like(obs)
    design = np.column_stack([x, y, ones, -obs * x, -obs * y, -obs])
    _, _, vt = np.linalg.svd(design, full_matrices=False)
    hh = vt[-1]
    den = hh[3] * x + hh[4] * y + hh[5]
    pred = (hh[0] * x + hh[1] * y + hh[2]) / den
    err = pred - obs
    ok = np.isfinite(err) & (np.abs(den) > 1.0e-12)
    residual[vs[ok], us[ok]] = err[ok].astype(np.float32)
    return residual


def _save_map(path: Path, data: np.ndarray, mask: np.ndarray, title: str, vmin=None, vmax=None) -> None:
    vis = np.array(data, dtype=np.float32, copy=True)
    vis[~mask | ~np.isfinite(vis)] = np.nan
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(vis, cmap="turbo", vmin=vmin, vmax=vmax)
    ax.set_title(title)
    ax.axis("off")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main() -> int:
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DIR
    index_path = src / INDEX_NAME
    if not index_path.is_file():
        raise FileNotFoundError(index_path)
    index = json.loads(index_path.read_text(encoding="utf-8"))
    preview = src / "quality"
    preview.mkdir(parents=True, exist_ok=True)

    rows = []
    preview_stems = set()
    n = len(index.get("poses") or [])
    if n:
        preview_stems.add(index["poses"][0]["stem"])
        preview_stems.add(index["poses"][n // 2]["stem"])
        preview_stems.add(index["poses"][-1]["stem"])

    for rec in index.get("poses") or []:
        npz = src / rec["calib"] if rec.get("calib") else pose_calib_path(src, rec["stem"])
        calib = FppCalibration.load(npz)
        q = evaluate_calibration(calib)
        q["stem"] = rec["stem"]
        q["start"] = rec.get("start")
        rows.append(q)
        print(
            f"{rec['stem']} rms={q['residual_rms_u']} med={q['residual_med_u']} "
            f"p95={q['residual_p95_u']} zcam={q['z_cam_med_m']}",
            flush=True,
        )
        if rec["stem"] in preview_stems:
            _save_map(preview / f"{rec['stem']}_mask.png", calib.mask.astype(np.float32), calib.mask, f"{rec['stem']} mask")
            _save_map(
                preview / f"{rec['stem']}_u.png",
                calib.u_reference,
                calib.mask,
                f"{rec['stem']} projector u (px)",
                vmin=0,
                vmax=1279,
            )
            res = _residual_map(calib)
            _save_map(
                preview / f"{rec['stem']}_residual_u.png",
                res,
                np.isfinite(res),
                f"{rec['stem']} u residual (px)",
                vmin=-8,
                vmax=8,
            )

    rms = [r["residual_rms_u"] for r in rows if r["residual_rms_u"] is not None]
    report = {
        "ok": True,
        "plane_z_m": index.get("plane_z_m"),
        "n_poses": len(rows),
        "residual_rms_u_median": float(np.median(rms)) if rms else None,
        "residual_rms_u_max": float(np.max(rms)) if rms else None,
        "poses": rows,
    }
    out_json = src / "fpp_calib_quality.json"
    out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")

    fig, ax = plt.subplots(figsize=(10, 4))
    xs = list(range(len(rows)))
    ys = [r["residual_rms_u"] if r["residual_rms_u"] is not None else np.nan for r in rows]
    ax.plot(xs, ys, "o-", ms=4)
    ax.set_xlabel("pose index (0 = apex)")
    ax.set_ylabel("plane homography residual RMS (projector px)")
    ax.set_title("FPP u-on-tray residual per pose")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(preview / "residual_rms_vs_pose.png", dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(xs, [r["valid_px"] / 1.0e6 for r in rows], "s-", ms=4, color="tab:green")
    ax.set_xlabel("pose index (0 = apex)")
    ax.set_ylabel("valid pixels (millions)")
    ax.set_title("FPP lit-patch coverage per pose")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(preview / "coverage_vs_pose.png", dpi=130)
    plt.close(fig)
    print(json.dumps({"ok": True, "quality": str(out_json), "preview": str(preview)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
