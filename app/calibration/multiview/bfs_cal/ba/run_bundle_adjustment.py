# Offline joint bundle adjustment for Multiview BFS (folder-local).
# Calibration layer only. Writes under ba/results — does not touch hyperfusion.cfg.
"""Joint BA: camera K/D + flange_T_camera + base_T_board (fixed board).

Adapts the multi-view FPP paper idea (unified reprojection BA) to HyperFusion's
eye-in-hand setup: one moving camera, fixed checkerboard, known base_T_flange.

Uses ``bfs_cal/checkerboard`` + ``bfs_cal/board.yaml``. Seeds from existing
``bfs_cal/results`` YAMLs when present. Optional DLP stereo refine is a second
pass when FPP bursts are available.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent
MULTIVIEW = ROOT.parent
BFS = MULTIVIEW / "bfs_cal"
DLP = MULTIVIEW / "dlp_cal"
DEFAULT_BOARD = BFS / "board.yaml"
DEFAULT_IMAGES = BFS / "checkerboard"
DEFAULT_SEED = BFS / "results"
DEFAULT_OUT = ROOT / "results"


def load_board(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    cols = int(data["inner_corners_x"])
    rows = int(data["inner_corners_y"])
    square_m = float(data["square_size_mm"]) * 0.001
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= square_m
    data["pattern_size"] = (cols, rows)
    data["object_points"] = obj
    return data


def rpy_zyx_deg_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    r, p, y = map(math.radians, (roll, pitch, yaw))
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def R_to_rpy_zyx_deg(R: np.ndarray) -> list[float]:
    sy = -float(R[2, 0])
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy > 1e-8:
        pitch = math.asin(max(-1.0, min(1.0, sy)))
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        pitch = math.asin(max(-1.0, min(1.0, sy)))
        roll = math.atan2(float(-R[0, 1]), float(R[1, 1]))
        yaw = 0.0
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def T_from_rt(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def rt_from_T(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    return rvec.reshape(3), T[:3, 3].copy()


def load_flange_T(meta: dict) -> np.ndarray | None:
    bt = meta.get("base_T_flange")
    if not isinstance(bt, dict) or "T" not in bt:
        return None
    T = np.asarray(bt["T"], dtype=np.float64)
    return T if T.shape == (4, 4) else None


def detect_views(
    images_dir: Path,
    board: dict[str, Any],
    cache_path: Path,
) -> list[dict[str, Any]]:
    if cache_path.is_file():
        data = np.load(cache_path, allow_pickle=True)
        views = list(data["views"])
        print(f"Loaded {len(views)} views from cache {cache_path.name}", flush=True)
        return views

    pattern = board["pattern_size"]
    obj = board["object_points"]
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    views: list[dict[str, Any]] = []
    images = sorted(
        {
            *images_dir.glob("*.tif"),
            *images_dir.glob("*.tiff"),
            *images_dir.glob("*.png"),
        }
    )
    for path in images:
        jpath = path.with_suffix(".json")
        if not jpath.is_file():
            continue
        meta = json.loads(jpath.read_text(encoding="utf-8-sig"))
        T_bf = load_flange_T(meta)
        if T_bf is None:
            continue
        img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if img is None:
            continue
        if img.ndim == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
        if gray.dtype != np.uint8:
            g = gray.astype(np.float32)
            g = g / (65535.0 if g.max() > 255 else max(float(g.max()), 1.0))
            gray = np.clip(g * 255.0, 0, 255).astype(np.uint8)
        ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
        if not ok:
            print(f"  SKIP no corners {path.name}", flush=True)
            continue
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4),
        )
        views.append(
            {
                "file": path.name,
                "stem": path.stem,
                "size": (int(gray.shape[1]), int(gray.shape[0])),
                "img_pts": corners.reshape(-1, 2).astype(np.float64),
                "obj_pts": obj.astype(np.float64),
                "base_T_flange": T_bf,
            }
        )
        print(f"  OK {path.name}", flush=True)
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache_path, views=np.array(views, dtype=object))
    return views


def average_SE3(Ts: list[np.ndarray]) -> np.ndarray:
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    R_stack = np.asarray([T[:3, :3] for T in Ts], dtype=np.float64)
    R_mean = np.mean(R_stack, axis=0)
    u, _, vt = np.linalg.svd(R_mean)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1
        R = u @ vt
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def pnp_cam_T_board(v: dict, K: np.ndarray, D: np.ndarray) -> np.ndarray | None:
    ok, rvec, tvec = cv2.solvePnP(
        v["obj_pts"].astype(np.float32),
        v["img_pts"].astype(np.float32),
        K,
        D,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    return T_from_rt(rvec.reshape(3), tvec.reshape(3))


def select_inlier_views(
    views: list[dict],
    K: np.ndarray,
    D: np.ndarray,
    T_fc: np.ndarray,
    *,
    max_board_err_mm: float,
) -> tuple[list[dict], np.ndarray, dict]:
    """Keep views whose implied base_T_board is near the median (fixed-board BA)."""
    boards: list[np.ndarray] = []
    kept_meta: list[dict] = []
    for v in views:
        T_cb = pnp_cam_T_board(v, K, D)
        if T_cb is None:
            continue
        T_bb = v["base_T_flange"] @ T_fc @ T_cb
        boards.append(T_bb)
        kept_meta.append(v)
    if len(boards) < 8:
        raise RuntimeError(f"Too few PnP boards: {len(boards)}")
    ts = np.asarray([T[:3, 3] for T in boards])
    med = np.median(ts, axis=0)
    err = np.linalg.norm(ts - med, axis=1) * 1000.0
    mask = err <= float(max_board_err_mm)
    # If too strict, keep best 60%
    if int(mask.sum()) < 12:
        thr = float(np.percentile(err, 60))
        mask = err <= thr
        max_board_err_mm = thr
    inliers = [kept_meta[i] for i, m in enumerate(mask) if m]
    T_bb = average_SE3([boards[i] for i, m in enumerate(mask) if m])
    stats = {
        "n_all": len(kept_meta),
        "n_inliers": len(inliers),
        "board_err_mm_median_all": float(np.median(err)),
        "board_err_mm_max_all": float(np.max(err)),
        "inlier_threshold_mm": float(max_board_err_mm),
        "board_err_mm_median_inliers": float(np.median(err[mask])),
    }
    return inliers, T_bb, stats


def seed_params(
    seed_dir: Path, views: list[dict], *, max_board_err_mm: float = 8.0
) -> tuple[np.ndarray, dict]:
    """x = [fx,fy,cx,cy,k1..k3, rvec_X(3), t_X(3), rvec_board(3), t_board(3)]."""
    K = np.eye(3)
    D = np.zeros(5)
    T_fc = np.eye(4)

    intr_p = seed_dir / "camera_intrinsics.yaml"
    he_p = seed_dir / "flange_T_camera.yaml"
    if intr_p.is_file():
        intr = yaml.safe_load(intr_p.read_text(encoding="utf-8"))
        K = np.asarray(intr["K"], dtype=np.float64)
        D = np.asarray(intr["D"], dtype=np.float64).reshape(-1)[:5]
    if he_p.is_file():
        he = yaml.safe_load(he_p.read_text(encoding="utf-8"))
        T_fc = np.asarray(he["T"], dtype=np.float64)

    # Always rebuild board from HE + PnP so seed residual is consistent.
    inliers, T_bb, sel = select_inlier_views(
        views, K, D, T_fc, max_board_err_mm=max_board_err_mm
    )
    rx, tx = rt_from_T(T_fc)
    rb, tb = rt_from_T(T_bb)
    x0 = np.concatenate(
        [
            [K[0, 0], K[1, 1], K[0, 2], K[1, 2]],
            D.reshape(-1)[:5],
            rx,
            tx,
            rb,
            tb,
        ]
    )
    return x0.astype(np.float64), {"inliers": inliers, "select": sel}


def unpack(x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    fx, fy, cx, cy = x[0:4]
    D = x[4:9].copy()
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    T_fc = T_from_rt(x[9:12], x[12:15])
    T_bb = T_from_rt(x[15:18], x[18:21])
    return K, D, T_fc, T_bb


def residuals(x: np.ndarray, views: list[dict]) -> np.ndarray:
    K, D, T_fc, T_bb = unpack(x)
    errs: list[np.ndarray] = []
    for v in views:
        T_bc = v["base_T_flange"] @ T_fc
        T_cb = np.linalg.inv(T_bc) @ T_bb
        rvec, tvec = rt_from_T(T_cb)
        proj, _ = cv2.projectPoints(
            v["obj_pts"].astype(np.float64),
            rvec,
            tvec,
            K,
            D,
        )
        e = (proj.reshape(-1, 2) - v["img_pts"]).reshape(-1)
        errs.append(e)
    return np.concatenate(errs)


def rms_px(x: np.ndarray, views: list[dict]) -> float:
    r = residuals(x, views)
    return float(np.sqrt(np.mean(r * r)))


def per_view_stats(x: np.ndarray, views: list[dict]) -> list[dict]:
    K, D, T_fc, T_bb = unpack(x)
    out = []
    for v in views:
        T_bc = v["base_T_flange"] @ T_fc
        T_cb = np.linalg.inv(T_bc) @ T_bb
        rvec, tvec = rt_from_T(T_cb)
        proj, _ = cv2.projectPoints(
            v["obj_pts"].astype(np.float64), rvec, tvec, K, D
        )
        e = np.linalg.norm(proj.reshape(-1, 2) - v["img_pts"], axis=1)
        out.append(
            {
                "file": v["file"],
                "rms_px": float(np.sqrt(np.mean(e * e))),
                "med_px": float(np.median(e)),
                "max_px": float(np.max(e)),
            }
        )
    return out


def holdout_pose_err(
    x: np.ndarray, views: list[dict]
) -> dict[str, float]:
    """Board pose consistency in base: compare each view's implied base_T_board to BA board."""
    K, D, T_fc, T_bb = unpack(x)
    d_t = []
    d_r = []
    for v in views:
        ok, rvec, tvec = cv2.solvePnP(
            v["obj_pts"].astype(np.float32),
            v["img_pts"].astype(np.float32),
            K,
            D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            continue
        T_cb = T_from_rt(rvec.reshape(3), tvec.reshape(3))
        T_bc = v["base_T_flange"] @ T_fc
        T_bb_i = T_bc @ T_cb
        dt = np.linalg.norm(T_bb_i[:3, 3] - T_bb[:3, 3]) * 1000.0
        R_err = T_bb_i[:3, :3] @ T_bb[:3, :3].T
        ang = math.degrees(
            math.acos(max(-1.0, min(1.0, 0.5 * (np.trace(R_err) - 1.0))))
        )
        d_t.append(dt)
        d_r.append(ang)
    return {
        "n": float(len(d_t)),
        "translation_err_mm_median": float(np.median(d_t)) if d_t else float("nan"),
        "rotation_err_deg_median": float(np.median(d_r)) if d_r else float("nan"),
    }


def write_results(
    out_dir: Path,
    x: np.ndarray,
    *,
    train: list[dict],
    hold: list[dict],
    seed_dir: Path,
    n_iter: int,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    K, D, T_fc, T_bb = unpack(x)
    rpy = R_to_rpy_zyx_deg(T_fc[:3, :3])
    xyz_mm = (T_fc[:3, 3] * 1000.0).tolist()

    train_rms = rms_px(x, train)
    hold_rms = rms_px(x, hold) if hold else float("nan")
    hold_pose = holdout_pose_err(x, hold) if hold else {"n": 0.0}

    intr = {
        "rms_px_train": train_rms,
        "rms_px_holdout": hold_rms,
        "width": train[0]["size"][0],
        "height": train[0]["size"][1],
        "K": K.tolist(),
        "D": D.tolist(),
        "model": "brown_conrady_k1_k2_p1_p2_k3",
        "hyperfusion_cfg": {
            "bfs_camera_fx": float(K[0, 0]),
            "bfs_camera_fy": float(K[1, 1]),
            "bfs_camera_cx": float(K[0, 2]),
            "bfs_camera_cy": float(K[1, 2]),
            "bfs_camera_distortion": ", ".join(f"{v:.8g}" for v in D.tolist()),
        },
        "n_train": len(train),
        "n_holdout": len(hold),
        "method": "joint_ba_fixed_board",
        "note": "Folder-local BA only — not written to hyperfusion.cfg.",
        "seed": str(seed_dir),
    }
    (out_dir / "camera_intrinsics.yaml").write_text(
        yaml.safe_dump(intr, sort_keys=False), encoding="utf-8"
    )

    he = {
        "method": "joint_ba",
        "T": T_fc.tolist(),
        "urdf_origin": {
            "xyz_m": T_fc[:3, 3].tolist(),
            "xyz_mm": xyz_mm,
            "rpy_deg": rpy,
        },
        "tool_tcp_suggestion": {
            "tool_tcp_x_mm": xyz_mm[0],
            "tool_tcp_y_mm": xyz_mm[1],
            "tool_tcp_z_mm": xyz_mm[2],
            "tool_tcp_roll_deg": rpy[0],
            "tool_tcp_pitch_deg": rpy[1],
            "tool_tcp_yaw_deg": rpy[2],
            "note": "Suggestion only — do not auto-copy into app cfg.",
        },
    }
    (out_dir / "flange_T_camera.yaml").write_text(
        yaml.safe_dump(he, sort_keys=False), encoding="utf-8"
    )

    board = {
        "T": T_bb.tolist(),
        "xyz_m": T_bb[:3, 3].tolist(),
        "xyz_mm": (T_bb[:3, 3] * 1000.0).tolist(),
        "rpy_deg": R_to_rpy_zyx_deg(T_bb[:3, :3]),
    }
    (out_dir / "base_T_board.yaml").write_text(
        yaml.safe_dump(board, sort_keys=False), encoding="utf-8"
    )

    report = {
        "ok": True,
        "method": "joint_ba_K_HE_board",
        "n_iter": n_iter,
        "train_rms_px": train_rms,
        "holdout_rms_px": hold_rms,
        "holdout_pose": hold_pose,
        "train_views": per_view_stats(x, train),
        "holdout_views": per_view_stats(x, hold) if hold else [],
        "K": K.tolist(),
        "D": D.tolist(),
        "flange_T_camera_xyz_mm": xyz_mm,
        "flange_T_camera_rpy_deg": rpy,
        "outputs": {
            "camera_intrinsics": str(out_dir / "camera_intrinsics.yaml"),
            "flange_T_camera": str(out_dir / "flange_T_camera.yaml"),
            "base_T_board": str(out_dir / "base_T_board.yaml"),
        },
    }
    (out_dir / "ba_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({k: report[k] for k in (
        "ok", "train_rms_px", "holdout_rms_px", "holdout_pose",
        "flange_T_camera_xyz_mm", "flange_T_camera_rpy_deg",
    )}, indent=2), flush=True)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--images", type=Path, default=DEFAULT_IMAGES)
    p.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    p.add_argument("--seed", type=Path, default=DEFAULT_SEED)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument("--holdout", type=int, default=8)
    p.add_argument("--max-views", type=int, default=0, help="0 = all")
    p.add_argument("--board-inlier-mm", type=float, default=8.0)
    p.add_argument("--ftol", type=float, default=1e-12)
    p.add_argument("--xtol", type=float, default=1e-12)
    p.add_argument("--max-nfev", type=int, default=200)
    p.add_argument("--rebuild-cache", action="store_true")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    board = load_board(args.board)
    cache = ROOT / "cache" / "bfs_corners.npz"
    if args.rebuild_cache and cache.is_file():
        cache.unlink()
    print(f"Detecting corners in {args.images}", flush=True)
    views = detect_views(args.images, board, cache)
    if len(views) < 8:
        raise SystemExit(f"Need >= 8 views with corners+flange, got {len(views)}")

    x0, seed_info = seed_params(
        args.seed, views, max_board_err_mm=float(args.board_inlier_mm)
    )
    views = seed_info["inliers"]
    print(f"inlier select: {seed_info['select']}", flush=True)

    if args.max_views and len(views) > args.max_views:
        idx = np.linspace(0, len(views) - 1, args.max_views).astype(int)
        views = [views[i] for i in idx]

    n_hold = min(int(args.holdout), max(0, len(views) // 5))
    train = views[:-n_hold] if n_hold else views
    hold = views[-n_hold:] if n_hold else []
    print(f"BA train={len(train)} holdout={len(hold)}", flush=True)
    print(f"seed train RMS={rms_px(x0, train):.4f} px", flush=True)

    w, h = train[0]["size"]
    lo = np.full_like(x0, -np.inf)
    hi = np.full_like(x0, np.inf)
    lo[0], hi[0] = x0[0] * 0.85, x0[0] * 1.15
    lo[1], hi[1] = x0[1] * 0.85, x0[1] * 1.15
    lo[2], hi[2] = 0.0, float(w)
    lo[3], hi[3] = 0.0, float(h)
    lo[12:15] = x0[12:15] - 0.05
    hi[12:15] = x0[12:15] + 0.05

    res = least_squares(
        residuals,
        x0,
        args=(train,),
        method="trf",
        loss="soft_l1",
        f_scale=1.0,
        bounds=(lo, hi),
        ftol=args.ftol,
        xtol=args.xtol,
        max_nfev=args.max_nfev,
        verbose=2,
    )
    print(
        f"done cost={res.cost:.6f} nfev={res.nfev} train_RMS={rms_px(res.x, train):.4f}",
        flush=True,
    )
    write_results(
        args.out,
        res.x,
        train=train,
        hold=hold,
        seed_dir=args.seed,
        n_iter=int(res.nfev),
    )
    # Append selection stats into report
    report_path = args.out / "ba_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["inlier_select"] = seed_info["select"]
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise
