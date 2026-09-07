# Offline FPP plane calibration (HDMI 26-frame PSP burst on a flat surface).
# Distinct from calibrate_bfs.py (checkerboard K/D + hand-eye). No robot motion.
"""Fit one projector P / u-map per HDMI PSP burst (each robot pose).

Uses the decoder in app/sidecars/fpp. Pattern is smaller than the BFS FOV;
only the lit patch is used. Writes {stem}_fpp_calib.npz plus fpp_calib_index.json.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIDECAR_FPP = ROOT.parents[1] / "sidecars" / "fpp"
DEFAULT_INPUT = ROOT / "FPP"
DEFAULT_OUT = DEFAULT_INPUT
CFG_PATH = ROOT.parents[1] / "preset" / "hyperfusion.cfg"

if str(SIDECAR_FPP) not in sys.path:
    sys.path.insert(0, str(SIDECAR_FPP))

from hyperfusion_fpp.calibrate import (  # noqa: E402
    INDEX_NAME,
    evaluate_calibration,
    fit_calibration,
    pose_calib_path,
)
from hyperfusion_fpp.capture import list_burst_starts, load_burst  # noqa: E402
from hyperfusion_fpp.decode import decode_burst  # noqa: E402
from hyperfusion_fpp.io_maps import write_decode_maps  # noqa: E402
from hyperfusion_fpp.undistort import undistort_burst  # noqa: E402


def _plane_z_from_cfg(path: Path) -> float:
    """Tray Z in base_link ≈ ceiling_mount_height (not world Z=0)."""
    if not path.is_file():
        return 0.629
    text = path.read_text(encoding="utf-8")
    match = re.search(r"^\s*ceiling_mount_height_mm\s*=\s*([0-9.]+)", text, re.M)
    if not match:
        return 0.629
    return float(match.group(1)) * 0.001


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="FPP plane calibration, one fit per pose")
    p.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Folder with one or more FPP bursts")
    p.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT,
        help="Output folder (writes {stem}_fpp_calib.npz). A .npz path is used only for a single burst.",
    )
    p.add_argument(
        "--plane-z-m",
        type=float,
        default=None,
        help="Flat surface Z in base_link (default: ceiling_mount_height_mm from hyperfusion.cfg).",
    )
    p.add_argument(
        "--min-modulation",
        type=float,
        default=0.08,
        help="White-Black threshold. Unlit FOV is dropped.",
    )
    p.add_argument(
        "--scale-mm-per-u",
        type=float,
        default=None,
        help="Optional fallback: metres = (mm/u)*0.001 * (u - u_plane).",
    )
    p.add_argument("--preview", action="store_true", help="Write u/mask PNG+NPY previews per pose")
    p.add_argument(
        "--stem",
        action="append",
        default=None,
        help="Only these burst stems (repeatable). Example: --stem 00000",
    )
    p.add_argument(
        "--undistort",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Remap burst with BFS K/D before decode (default: on).",
    )
    p.add_argument(
        "--ransac-thresh-u",
        type=float,
        default=8.0,
        help="RANSAC inlier threshold in projector pixels.",
    )
    p.add_argument(
        "--min-valid-px",
        type=int,
        default=1000,
        help="Skip a pose if decode coverage is below this (cut-off / missing pattern).",
    )
    p.add_argument(
        "--raw",
        action="store_true",
        help="Skip RANSAC inlier filter — keep every lit pixel (preview the raw u map).",
    )
    return p.parse_args()


def _out_dir_and_single(out: Path, n_bursts: int) -> tuple[Path, Path | None]:
    out = Path(out)
    if out.suffix.lower() == ".npz":
        if n_bursts > 1:
            raise ValueError(
                f"{n_bursts} bursts found; --out must be a folder "
                f"(writes {{stem}}_fpp_calib.npz), not {out.name}."
            )
        return out.parent, out
    return out, None


def main() -> int:
    args = parse_args()
    src = Path(args.input)
    starts = list_burst_starts(src)
    if args.stem:
        want = {str(s) for s in args.stem}
        starts = [p for p in starts if p.stem in want]
    if not starts:
        raise FileNotFoundError(f"No FPP burst in {src}")
    plane_z = float(args.plane_z_m) if args.plane_z_m is not None else _plane_z_from_cfg(CFG_PATH)
    dest, single_npz = _out_dir_and_single(Path(args.out), len(starts))
    dest.mkdir(parents=True, exist_ok=True)

    poses: list[dict] = []
    for start in starts:
        rec: dict = {"start": start.name, "stem": start.stem, "ok": False}
        try:
            burst = load_burst(src, start=start)
            maps = undistort_burst(burst) if args.undistort else None
            if args.undistort and maps is None:
                print(f"{start.name}: no K/D in JSON — decode on raw", flush=True)
            decoded = decode_burst(burst, min_modulation=args.min_modulation)
            decode_px = int(decoded.mask.sum())
            rec["decode_px"] = decode_px
            if decode_px < int(args.min_valid_px):
                rec["skip_reason"] = f"pattern_cut_off_or_empty decode_px={decode_px}"
                poses.append(rec)
                print(
                    f"[{len(poses)}/{len(starts)}] {start.name} SKIP {rec['skip_reason']}",
                    flush=True,
                )
                continue
            meta = burst.first_meta()
            calib = fit_calibration(
                decoded,
                meta,
                plane_z_m=plane_z,
                ransac_thresh_u=args.ransac_thresh_u,
                undistorted=maps is not None,
                keep_all_pixels=bool(args.raw),
            )
            if args.scale_mm_per_u is not None:
                calib.depth_scale_m_per_u = float(args.scale_mm_per_u) * 0.001
            if int(calib.mask.sum()) < 16:
                rec["skip_reason"] = f"too_few_inliers inliers={int(calib.mask.sum())}"
                rec["inlier_count"] = int(calib.mask.sum())
                poses.append(rec)
                print(
                    f"[{len(poses)}/{len(starts)}] {start.name} SKIP {rec['skip_reason']}",
                    flush=True,
                )
                continue
            npz = single_npz if single_npz is not None else pose_calib_path(dest, start.stem)
            calib.save(npz)
            if args.preview:
                write_decode_maps(dest / "calib_preview" / start.stem, decoded, stem="plane")
            ext = meta.get("extrinsics") if isinstance(meta.get("extrinsics"), dict) else {}
            planned = (
                meta.get("planned_world_position_m")
                if isinstance(meta.get("planned_world_position_m"), dict)
                else {}
            )
            quality = evaluate_calibration(calib)
            rec.update(
                {
                    "ok": True,
                    "calib": npz.name,
                    "valid_px": int(calib.mask.sum()),
                    "has_projector_p": calib.projector_p is not None,
                    "residual_rms_u": quality["residual_rms_u"],
                    "residual_med_u": quality["residual_med_u"],
                    "residual_p95_u": quality["residual_p95_u"],
                    "z_cam_med_m": quality["z_cam_med_m"],
                    "undistorted": quality.get("undistorted"),
                    "inlier_count": quality.get("inlier_count"),
                    "residual_model": quality.get("residual_model"),
                    "camera_t": [float(x) for x in (ext.get("t") or [])],
                    "planned_world_position_m": {
                        "x_m": planned.get("x_m"),
                        "y_m": planned.get("y_m"),
                        "z_m": planned.get("z_m"),
                    },
                }
            )
        except Exception as exc:  # noqa: BLE001 — one cut-off/corrupt burst must not abort the set
            rec["skip_reason"] = str(exc)
        poses.append(rec)
        if rec.get("ok"):
            print(
                f"[{len(poses)}/{len(starts)}] {start.name} decode_px={rec['decode_px']} "
                f"inliers={rec['inlier_count']} rms={rec['residual_rms_u']} "
                f"med={rec['residual_med_u']} "
                f"undistort={rec['undistorted']} P={rec['has_projector_p']}",
                flush=True,
            )
        elif rec.get("skip_reason"):
            print(
                f"[{len(poses)}/{len(starts)}] {start.name} SKIP {rec['skip_reason']}",
                flush=True,
            )

    n_ok = sum(1 for r in poses if r.get("ok"))
    index = {
        "ok": n_ok > 0,
        "input": str(src),
        "plane_z_m": plane_z,
        "undistort": bool(args.undistort),
        "ransac_thresh_u": float(args.ransac_thresh_u),
        "min_valid_px": int(args.min_valid_px),
        "raw": bool(args.raw),
        "n_poses": len(poses),
        "n_ok": n_ok,
        "n_skipped": len(poses) - n_ok,
        "poses": poses,
    }
    (dest / INDEX_NAME).write_text(json.dumps(index, indent=2), encoding="utf-8")
    print(json.dumps(index, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001 — CLI
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise SystemExit(1)
