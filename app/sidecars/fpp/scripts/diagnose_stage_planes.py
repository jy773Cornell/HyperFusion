#!/usr/bin/env python3
# Offline stage-plane pose diagnostic for FPP MVS bursts (sidecar / fpp).
# Fits a dominant plane per pin in camera, transforms to base_link, reports
# normal spread / height agreement. Also tests common pose-chain mistakes.
"""Diagnose whether robot/hand-eye poses put a stationary stage into one plane.

Example::

  .\\.venv\\Scripts\\python.exe scripts\\diagnose_stage_planes.py ^
    --input C:\\Users\\jy773\\Downloads\\niagara
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
DF = HERE / "depth_fusion"
if str(DF) not in sys.path:
    sys.path.insert(0, str(DF))

from fpp_mvs.paths import resolve_datafolder  # noqa: E402
from hyperfusion_depth_fusion.poses import (  # noqa: E402
    load_flange_T_camera_yaml,
    load_json_c2w,
    load_flange_c2w,
)


def _fit_plane(pts: np.ndarray) -> tuple[np.ndarray, float]:
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    n = vt[-1]
    n = n / (np.linalg.norm(n) + 1e-12)
    return n, float(-n @ c)


def _ransac_plane(
    pts: np.ndarray, *, thr_m: float = 0.003, iters: int = 200, seed: int = 0
) -> tuple[np.ndarray, float, int]:
    rng = np.random.default_rng(seed)
    n_pts = len(pts)
    best: tuple[int, np.ndarray] | None = None
    for _ in range(iters):
        idx = rng.choice(n_pts, 3, replace=False)
        p0, p1, p2 = pts[idx]
        n = np.cross(p1 - p0, p2 - p0)
        ln = float(np.linalg.norm(n))
        if ln < 1e-9:
            continue
        n = n / ln
        d = float(-n @ p0)
        inl = np.abs(pts @ n + d) < thr_m
        score = int(inl.sum())
        if best is None or score > best[0]:
            best = (score, inl)
    assert best is not None
    n, d = _fit_plane(pts[best[1]])
    if n[2] > 0.0:
        n, d = -n, -d
    return n, d, best[0]


def _plane_to_base(
    n_c: np.ndarray, d_c: float, t_c2w: np.ndarray
) -> tuple[np.ndarray, float]:
    r = t_c2w[:3, :3]
    t = t_c2w[:3, 3]
    n_b = r @ n_c
    n_b = n_b / (np.linalg.norm(n_b) + 1e-12)
    d_b = d_c - float(n_b @ t)
    if n_b[2] < 0.0:
        n_b, d_b = -n_b, -d_b
    return n_b, d_b


def _load_cam_points(
    burst: Path, stem: str, *, stride: int = 8
) -> tuple[np.ndarray, dict, np.ndarray]:
    depth = np.load(burst / "processed" / "decode" / stem / "fpp_depth.npy").astype(
        np.float32
    )
    mask = np.load(burst / "processed" / "decode" / stem / "fpp_mask.npy").astype(bool)
    meta = json.loads((burst / f"{stem}.json").read_text(encoding="utf-8"))
    k = meta["intrinsics"]
    fx, fy, cx, cy = float(k["fx"]), float(k["fy"]), float(k["cx"]), float(k["cy"])
    valid = mask & np.isfinite(depth) & (depth > 0)
    med = float(np.nanmedian(depth[valid])) if np.any(valid) else 0.0
    scale = 0.001 if med > 10.0 else 1.0
    d = depth * scale
    h, w = d.shape
    us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    zz = d[vs, us]
    mm = mask[vs, us] & np.isfinite(zz) & (zz >= 0.15) & (zz <= 0.55)
    u = us[mm].astype(np.float64)
    v = vs[mm].astype(np.float64)
    z = zz[mm].astype(np.float64)
    x = (u - cx) / fx * z
    y = (v - cy) / fy * z
    pts = np.stack([x, y, z], axis=1)
    return pts, meta, load_json_c2w(meta)


def _pose_variants(
    meta: dict, t_he: np.ndarray | None
) -> dict[str, np.ndarray]:
    t_json = load_json_c2w(meta)
    out: dict[str, np.ndarray] = {"json": t_json}
    t_f = load_flange_c2w(meta)
    if t_f is not None and t_he is not None:
        out["flange@HE"] = t_f @ t_he
        out["flange@inv(HE)"] = t_f @ np.linalg.inv(t_he)
        out["HE@flange"] = t_he @ t_f
    t_gl = t_json.copy()
    t_gl[:3, :3] = t_gl[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    out["json*diag(1,-1,-1)"] = t_gl
    return out


def diagnose(burst: Path, *, hand_eye: Path | None) -> int:
    _, burst = resolve_datafolder(burst)
    decode = burst / "processed" / "decode"
    if not decode.is_dir():
        print(f"missing decode tree: {decode}", flush=True)
        return 1

    t_he = load_flange_T_camera_yaml(hand_eye) if hand_eye and hand_eye.is_file() else None
    stems = sorted(
        p.name
        for p in decode.iterdir()
        if p.is_dir() and (p / "fpp_depth.npy").is_file()
    )
    if not stems:
        print("no decode stems", flush=True)
        return 1

    print(f"burst={burst}", flush=True)
    print(f"stems={len(stems)} hand_eye={hand_eye}", flush=True)
    print(
        "stem  ang_vs_horiz°  n_base                         z0_mm   "
        "XY_mm     inl   | JSON≡HE?",
        flush=True,
    )

    n_bases: list[np.ndarray] = []
    z0s: list[float] = []
    for stem in stems:
        pts, meta, t_json = _load_cam_points(burst, stem)
        if len(pts) < 500:
            print(f"{stem}  skip (n={len(pts)})", flush=True)
            continue
        n_c, d_c, inl = _ransac_plane(pts)
        n_b, d_b = _plane_to_base(n_c, d_c, t_json)
        z0 = -d_b / (n_b[2] + 1e-12)
        n_exp = t_json[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        n_exp = n_exp / (np.linalg.norm(n_exp) + 1e-12)
        if n_exp[2] > 0.0:
            n_exp = -n_exp
        ang = math.degrees(math.acos(float(np.clip(n_c @ n_exp, -1.0, 1.0))))
        inl_mask = np.abs(pts @ n_c + d_c) < 0.003
        extent = (pts[inl_mask][:, :2].max(0) - pts[inl_mask][:, :2].min(0)) * 1000.0

        he_note = ""
        if t_he is not None:
            variants = _pose_variants(meta, t_he)
            dhe = variants.get("flange@HE")
            if dhe is not None:
                dR = math.degrees(
                    math.acos(
                        float(
                            np.clip(
                                0.5
                                * (
                                    np.trace(t_json[:3, :3].T @ dhe[:3, :3]) - 1.0
                                ),
                                -1.0,
                                1.0,
                            )
                        )
                    )
                )
                dt = float(np.linalg.norm(t_json[:3, 3] - dhe[:3, 3]) * 1000.0)
                he_note = f" | JSON vs HE {dR:.3f}° {dt:.2f}mm"

        print(
            f"{stem}  {ang:6.2f}  "
            f"({n_b[0]:+.3f},{n_b[1]:+.3f},{n_b[2]:+.3f})  "
            f"{z0 * 1000:7.1f}  {extent[0]:.0f}x{extent[1]:.0f}  {inl:6d}{he_note}",
            flush=True,
        )
        n_bases.append(n_b)
        z0s.append(z0)

    if len(n_bases) < 2:
        return 0

    ns = np.asarray(n_bases)
    ref = ns[0]
    angs = [
        math.degrees(math.acos(float(np.clip(n @ ref, -1.0, 1.0)))) for n in ns
    ]
    print(
        f"summary: max normal spread vs pin0 = {max(angs):.2f}°  "
        f"mean={float(np.mean(angs)):.2f}°  z0_std={float(np.std(z0s)) * 1000:.2f} mm",
        flush=True,
    )
    print(
        "readout: normals ≫1–2° → rotation/HE/chain; "
        "normals OK but z spreads → translation/units; "
        "both OK but object ghosts → need object edges / pose graph "
        "(plane alone cannot fix yaw about normal or XY).",
        flush=True,
    )

    if t_he is not None:
        print("\npose-chain variants (max normal spread ° / z0_std mm):", flush=True)
        # reload first stem set once
        views = []
        for stem in stems:
            pts, meta, _ = _load_cam_points(burst, stem)
            if len(pts) < 500:
                continue
            n_c, d_c, _ = _ransac_plane(pts)
            views.append((n_c, d_c, meta))
        for name in (
            "json",
            "flange@HE",
            "flange@inv(HE)",
            "HE@flange",
            "json*diag(1,-1,-1)",
        ):
            ns2: list[np.ndarray] = []
            zs2: list[float] = []
            for n_c, d_c, meta in views:
                t = _pose_variants(meta, t_he).get(name)
                if t is None:
                    continue
                n_b, d_b = _plane_to_base(n_c, d_c, t)
                ns2.append(n_b)
                zs2.append(-d_b / (n_b[2] + 1e-12))
            if len(ns2) < 2:
                continue
            ns2a = np.asarray(ns2)
            an = [
                math.degrees(math.acos(float(np.clip(n @ ns2a[0], -1.0, 1.0))))
                for n in ns2a
            ]
            print(
                f"  {name:22s}  max={max(an):5.2f}°  z_std={float(np.std(zs2)) * 1000:6.2f} mm",
                flush=True,
            )
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument(
        "--hand-eye",
        type=Path,
        default=Path(__file__).resolve().parents[3]
        / "calibration"
        / "multiview"
        / "bfs_cal"
        / "results"
        / "flange_T_camera.yaml",
    )
    args = p.parse_args()
    return diagnose(args.input, hand_eye=args.hand_eye)


if __name__ == "__main__":
    raise SystemExit(main())
