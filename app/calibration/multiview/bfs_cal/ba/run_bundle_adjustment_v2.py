# Offline joint BA v2: K + HE + per-view board poses + soft shared-board prior.
# Folder-local only — writes ba/results, does not touch hyperfusion.cfg.
"""Bundle adjustment adapted from multi-view FPP papers for eye-in-hand data.

Unlike a rigid multi-camera rig, robot stills can have flange noise / slight board
motion. This BA therefore:

  * frees per-view ``camera_T_board`` (reprojection),
  * shares ``K,D`` and ``flange_T_camera``,
  * softly pulls ``base_T_flange @ flange_T_camera @ camera_T_board`` toward one board.

Outputs stay under ``ba/results/``.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from run_bundle_adjustment import (  # noqa: E402
    DEFAULT_BOARD,
    DEFAULT_IMAGES,
    DEFAULT_OUT,
    DEFAULT_SEED,
    R_to_rpy_zyx_deg,
    T_from_rt,
    detect_views,
    load_board,
    rt_from_T,
    average_SE3,
)

# Reuse corner cache from v1.


def pack0(K: np.ndarray, D: np.ndarray, T_fc: np.ndarray, cam_Ts: list[np.ndarray]) -> np.ndarray:
    rx, tx = rt_from_T(T_fc)
    parts = [[K[0, 0], K[1, 1], K[0, 2], K[1, 2]], D.reshape(-1)[:5], rx, tx]
    for T in cam_Ts:
        r, t = rt_from_T(T)
        parts.extend([r, t])
    return np.concatenate(parts).astype(np.float64)


def unpack(x: np.ndarray, n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
    fx, fy, cx, cy = x[0:4]
    D = x[4:9].copy()
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    T_fc = T_from_rt(x[9:12], x[12:15])
    cam_Ts = []
    o = 15
    for _ in range(n):
        cam_Ts.append(T_from_rt(x[o : o + 3], x[o + 3 : o + 6]))
        o += 6
    return K, D, T_fc, cam_Ts


def se3_log_t_mm_r_deg(T: np.ndarray) -> np.ndarray:
    t_mm = T[:3, 3] * 1000.0
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    r_deg = rvec.reshape(3) * (180.0 / math.pi)
    return np.concatenate([t_mm, r_deg])


def residuals(
    x: np.ndarray,
    views: list[dict],
    *,
    board_weight: float,
) -> np.ndarray:
    n = len(views)
    K, D, T_fc, cam_Ts = unpack(x, n)
    boards = [views[i]["base_T_flange"] @ T_fc @ cam_Ts[i] for i in range(n)]
    T_bb = average_SE3(boards)

    errs: list[np.ndarray] = []
    for i, v in enumerate(views):
        rvec, tvec = rt_from_T(cam_Ts[i])
        proj, _ = cv2.projectPoints(
            v["obj_pts"].astype(np.float64), rvec, tvec, K, D
        )
        e = (proj.reshape(-1, 2) - v["img_pts"]).reshape(-1)
        errs.append(e)
        # Soft shared-board prior (mm / deg), scaled
        dT = np.linalg.inv(T_bb) @ boards[i]
        errs.append(board_weight * se3_log_t_mm_r_deg(dT))
    return np.concatenate(errs)


def rms_reproj(x: np.ndarray, views: list[dict]) -> float:
    n = len(views)
    K, D, _, cam_Ts = unpack(x, n)
    es = []
    for i, v in enumerate(views):
        rvec, tvec = rt_from_T(cam_Ts[i])
        proj, _ = cv2.projectPoints(v["obj_pts"].astype(np.float64), rvec, tvec, K, D)
        e = proj.reshape(-1, 2) - v["img_pts"]
        es.append(e.reshape(-1))
    r = np.concatenate(es)
    return float(np.sqrt(np.mean(r * r)))


def board_spread_mm(x: np.ndarray, views: list[dict]) -> dict:
    n = len(views)
    _, _, T_fc, cam_Ts = unpack(x, n)
    boards = [views[i]["base_T_flange"] @ T_fc @ cam_Ts[i] for i in range(n)]
    T_bb = average_SE3(boards)
    err = [np.linalg.norm(T[:3, 3] - T_bb[:3, 3]) * 1000.0 for T in boards]
    return {
        "median_mm": float(np.median(err)),
        "max_mm": float(np.max(err)),
        "mean_mm": float(np.mean(err)),
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--images", type=Path, default=DEFAULT_IMAGES)
    p.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    p.add_argument("--seed", type=Path, default=DEFAULT_SEED)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument("--holdout", type=int, default=6)
    p.add_argument("--max-views", type=int, default=40)
    p.add_argument("--board-weight", type=float, default=0.15, help="soft board prior weight")
    p.add_argument("--max-nfev", type=int, default=120)
    args = p.parse_args()

    board = load_board(args.board)
    cache = ROOT / "cache" / "bfs_corners.npz"
    views = detect_views(args.images, board, cache)
    # Even spacing for tractable BA
    if args.max_views and len(views) > args.max_views:
        idx = np.linspace(0, len(views) - 1, args.max_views).astype(int)
        views = [views[i] for i in idx]

    n_hold = min(int(args.holdout), max(0, len(views) // 5))
    train = views[:-n_hold] if n_hold else views
    hold = views[-n_hold:] if n_hold else []

    intr = yaml.safe_load((args.seed / "camera_intrinsics.yaml").read_text(encoding="utf-8"))
    he = yaml.safe_load((args.seed / "flange_T_camera.yaml").read_text(encoding="utf-8"))
    K = np.asarray(intr["K"], float)
    D = np.asarray(intr["D"], float).reshape(-1)[:5]
    T_fc = np.asarray(he["T"], float)

    cam_Ts = []
    for v in train:
        ok, rvec, tvec = cv2.solvePnP(
            v["obj_pts"].astype(np.float32),
            v["img_pts"].astype(np.float32),
            K,
            D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError(f"PnP failed {v['file']}")
        cam_Ts.append(T_from_rt(rvec.reshape(3), tvec.reshape(3)))

    x0 = pack0(K, D, T_fc, cam_Ts)
    print(
        f"BA v2 train={len(train)} hold={len(hold)} "
        f"seed_reproj_RMS={rms_reproj(x0, train):.4f}px "
        f"seed_board_spread={board_spread_mm(x0, train)}",
        flush=True,
    )

    res = least_squares(
        residuals,
        x0,
        args=(train,),
        kwargs={"board_weight": float(args.board_weight)},
        method="trf",
        loss="soft_l1",
        f_scale=1.0,
        ftol=1e-12,
        xtol=1e-12,
        max_nfev=int(args.max_nfev),
        verbose=2,
    )
    x = res.x
    K2, D2, T_fc2, cam_Ts2 = unpack(x, len(train))
    boards = [train[i]["base_T_flange"] @ T_fc2 @ cam_Ts2[i] for i in range(len(train))]
    T_bb = average_SE3(boards)

    hold_rms = float("nan")
    if hold:
        # PnP hold with refined K, then board vs T_bb using refined HE
        es = []
        dmm = []
        for v in hold:
            ok, rvec, tvec = cv2.solvePnP(
                v["obj_pts"].astype(np.float32),
                v["img_pts"].astype(np.float32),
                K2,
                D2,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                continue
            proj, _ = cv2.projectPoints(
                v["obj_pts"].astype(np.float64), rvec, tvec, K2, D2
            )
            e = proj.reshape(-1, 2) - v["img_pts"]
            es.append(float(np.sqrt(np.mean(e * e))))
            T_cb = T_from_rt(rvec.reshape(3), tvec.reshape(3))
            T_bbi = v["base_T_flange"] @ T_fc2 @ T_cb
            dmm.append(float(np.linalg.norm(T_bbi[:3, 3] - T_bb[:3, 3]) * 1000.0))
        hold_rms = float(np.median(es)) if es else float("nan")
        hold_board = float(np.median(dmm)) if dmm else float("nan")
    else:
        hold_board = float("nan")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rpy = R_to_rpy_zyx_deg(T_fc2[:3, :3])
    xyz_mm = (T_fc2[:3, 3] * 1000.0).tolist()
    train_rms = rms_reproj(x, train)
    spread = board_spread_mm(x, train)

    intr_out = {
        "rms_px_train": train_rms,
        "rms_px_holdout_pnp": hold_rms,
        "width": train[0]["size"][0],
        "height": train[0]["size"][1],
        "K": K2.tolist(),
        "D": D2.tolist(),
        "method": "joint_ba_v2_per_view_board_soft_prior",
        "note": "Folder-local BA — not written to hyperfusion.cfg.",
        "hyperfusion_cfg": {
            "bfs_camera_fx": float(K2[0, 0]),
            "bfs_camera_fy": float(K2[1, 1]),
            "bfs_camera_cx": float(K2[0, 2]),
            "bfs_camera_cy": float(K2[1, 2]),
            "bfs_camera_distortion": ", ".join(f"{v:.8g}" for v in D2.tolist()),
        },
    }
    (out / "camera_intrinsics.yaml").write_text(
        yaml.safe_dump(intr_out, sort_keys=False), encoding="utf-8"
    )
    he_out = {
        "method": "joint_ba_v2",
        "T": T_fc2.tolist(),
        "urdf_origin": {"xyz_mm": xyz_mm, "rpy_deg": rpy},
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
    (out / "flange_T_camera.yaml").write_text(
        yaml.safe_dump(he_out, sort_keys=False), encoding="utf-8"
    )
    (out / "base_T_board.yaml").write_text(
        yaml.safe_dump(
            {
                "T": T_bb.tolist(),
                "xyz_mm": (T_bb[:3, 3] * 1000.0).tolist(),
                "rpy_deg": R_to_rpy_zyx_deg(T_bb[:3, :3]),
                "note": "Average of per-view boards after BA",
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    report = {
        "ok": True,
        "method": "joint_ba_v2_per_view_board_soft_prior",
        "n_train": len(train),
        "n_holdout": len(hold),
        "board_weight": float(args.board_weight),
        "nfev": int(res.nfev),
        "train_rms_px": train_rms,
        "holdout_pnp_rms_px_median": hold_rms,
        "train_board_spread_mm": spread,
        "holdout_board_err_mm_median": hold_board,
        "flange_T_camera_xyz_mm": xyz_mm,
        "flange_T_camera_rpy_deg": rpy,
        "K": K2.tolist(),
        "D": D2.tolist(),
        "note": (
            "v1 fixed-board BA failed because implied board poses scatter "
            f"~{spread['median_mm']:.0f} mm; v2 frees per-view boards with a soft prior."
        ),
    }
    (out / "ba_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: report[k] for k in (
        "ok", "train_rms_px", "holdout_pnp_rms_px_median",
        "train_board_spread_mm", "holdout_board_err_mm_median",
        "flange_T_camera_xyz_mm", "flange_T_camera_rpy_deg", "note",
    )}, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(json.dumps({"ok": False, "error": str(exc)}))
        raise
