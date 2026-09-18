# Offline BFS + UR3e calibration: detect, K/D, undistort preview, PnP, hand-eye.
# bfs_cal layer. No robot motion. Run from the multiview Windows venv with OpenCV.
"""Eye-in-hand calibration for HyperFusion BFS on UR3e (fixed checkerboard).

Intrinsics work on a folder of TIFFs. Hand-eye requires per-image JSON with
base_T_flange (written by the app after this capture-writer change).
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

ROOT = Path(__file__).resolve().parent
DEFAULT_BOARD = ROOT / "board.yaml"
DEFAULT_IMAGES = ROOT / "checkerboard"
DEFAULT_RESULTS = ROOT / "results"

HAND_EYE_METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

CAD_FLANGE_T_CAM_MM = np.array([0.0, -56.035, 81.825], dtype=np.float64)


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
    data["square_m"] = square_m
    return data


def list_images(folder: Path) -> list[Path]:
    """Flat folder or recursive (multiview_* / nested)."""
    files: list[Path] = []
    for ext in ("*.tif", "*.tiff", "*.TIF", "*.png", "*.jpg"):
        files.extend(folder.glob(ext))
        files.extend(folder.rglob(ext))
    # Prefer unique paths; skip decode/fusion junk if present.
    out: list[Path] = []
    seen: set[Path] = set()
    for p in sorted({q.resolve() for q in files}):
        parts = {x.lower() for x in p.parts}
        if parts & {"decode", "fusion", "rejected_no_corners", "dropped_worst"}:
            continue
        if p in seen:
            continue
        seen.add(p)
        out.append(p)
    return out


def load_prior_flange_T_camera_mm(
    xyz_mm: list[float], rpy_deg: list[float]
) -> np.ndarray:
    """App/CAD mount prior: same physical camera body → HE t must stay near this."""
    roll, pitch, yaw = [math.radians(float(v)) for v in rpy_deg]
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = np.asarray(xyz_mm, dtype=np.float64) * 0.001
    return T


def write_intrinsics_into_pose_jsons(
    images_dir: Path, K: np.ndarray, D: np.ndarray, *, source: str
) -> int:
    """Overwrite capture-time (often wrong-lens) intrinsics with fitted K/D."""
    n = 0
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    dist = [float(x) for x in np.asarray(D).reshape(-1)[:5]]
    for jpath in sorted(images_dir.glob("*.json")):
        if jpath.name.lower() in {"transforms.json", "board_check.json"}:
            continue
        meta = json.loads(jpath.read_text(encoding="utf-8"))
        if "intrinsics" not in meta or not isinstance(meta["intrinsics"], dict):
            meta["intrinsics"] = {}
        meta["intrinsics"].update(
            {
                "fx": fx,
                "fy": fy,
                "cx": cx,
                "cy": cy,
                "distortion": dist,
                "source": source,
            }
        )
        jpath.write_text(json.dumps(meta, indent=4) + "\n", encoding="utf-8")
        n += 1
    return n


def read_bgr(path: Path) -> np.ndarray:
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"Could not read image: {path}")
    return img


def detect_corners(gray: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    scale = 4 if min(gray.shape[:2]) >= 2000 else 1
    search = gray
    if scale > 1:
        search = cv2.resize(
            gray,
            (gray.shape[1] // scale, gray.shape[0] // scale),
            interpolation=cv2.INTER_AREA,
        )
    ok, corners = cv2.findChessboardCorners(search, pattern, flags)
    if not ok and scale > 1:
        ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
        scale = 1
    if not ok and hasattr(cv2, "findChessboardCornersSB"):
        sb = cv2.findChessboardCornersSB(
            gray,
            pattern,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )
        if sb[0]:
            return sb[1].astype(np.float32)
        return None
    if not ok:
        return None
    corners = corners.astype(np.float32)
    if scale > 1:
        corners *= float(scale)
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)


def detect_with_swap(gray: np.ndarray, pattern: tuple[int, int]) -> tuple[tuple[int, int], np.ndarray] | None:
    for candidate in (pattern, (pattern[1], pattern[0])):
        corners = detect_corners(gray, candidate)
        if corners is not None and len(corners) == candidate[0] * candidate[1]:
            return candidate, corners
    return None


def object_points_for(board: dict[str, Any], pattern: tuple[int, int]) -> np.ndarray:
    cols, rows = pattern
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj *= board["square_m"]
    return obj


def rotvec_to_R(rvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return R


def Rt_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def T_to_Rt(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[:3, :3].copy(), T[:3, 3].copy()


def invert_T(T: np.ndarray) -> np.ndarray:
    R, t = T_to_Rt(T)
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rotation_angle_deg(R: np.ndarray) -> float:
    c = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.degrees(math.acos(c)))


def average_SE3(Ts: list[np.ndarray]) -> np.ndarray:
    finite = [T for T in Ts if np.isfinite(T).all()]
    if not finite:
        raise ValueError("average_SE3: no finite transforms")
    ts = np.stack([T[:3, 3] for T in finite], axis=0)
    t_med = np.median(ts, axis=0)
    R_sum = np.zeros((3, 3), dtype=np.float64)
    for T in finite:
        R_sum += T[:3, :3]
    try:
        U, _, Vt = np.linalg.svd(R_sum)
    except np.linalg.LinAlgError:
        # Fall back to first finite rotation if SVD fails (degenerate set).
        return Rt_to_T(finite[0][:3, :3], t_med)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, 2] *= -1
        R_mean = U @ Vt
    return Rt_to_T(R_mean, t_med)


def load_flange_T(json_path: Path) -> np.ndarray | None:
    if not json_path.is_file():
        return None
    data = json.loads(json_path.read_text(encoding="utf-8"))
    flange = data.get("base_T_flange")
    if not isinstance(flange, dict):
        return None
    T = flange.get("T")
    if isinstance(T, list) and len(T) == 4:
        return np.asarray(T, dtype=np.float64)
    xyz = [flange.get("x_m"), flange.get("y_m"), flange.get("z_m")]
    rot = flange.get("rotvec_rad") or {}
    rvec = [rot.get("rx"), rot.get("ry"), rot.get("rz")]
    if any(v is None for v in xyz + rvec):
        return None
    R = rotvec_to_R(np.array(rvec, dtype=np.float64))
    return Rt_to_T(R, np.array(xyz, dtype=np.float64))


def write_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def detect_dataset(
    images: list[Path], board: dict[str, Any]
) -> tuple[list[dict[str, Any]], tuple[int, int]]:
    pattern0 = board["pattern_size"]
    frames: list[dict[str, Any]] = []
    size = (0, 0)
    used_pattern = pattern0
    for path in images:
        bgr = read_bgr(path)
        size = (bgr.shape[1], bgr.shape[0])
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        found = detect_with_swap(gray, pattern0)
        if found is None:
            print(f"  MISS  {path.name}")
            continue
        used_pattern, corners = found
        print(f"  OK    {path.name}  corners={len(corners)}  pattern={used_pattern}")
        frames.append(
            {
                "path": path,
                "corners": corners,
                "object": object_points_for(board, used_pattern),
                "flange_T": load_flange_T(path.with_suffix(".json")),
            }
        )
    return frames, size


def calibrate_intrinsics(
    frames: list[dict[str, Any]], size: tuple[int, int], args: argparse.Namespace
) -> dict[str, Any]:
    obj_pts = [f["object"] for f in frames]
    img_pts = [f["corners"] for f in frames]
    w, h = size
    flags = 0
    camera_matrix = None
    dist = None
    if args.use_nominal_k:
        # Hemisphere / similar-depth sets cannot separate fx from range.
        # Seed with BFS nominal K and keep fy = fx; still solve cx, cy, D.
        camera_matrix = np.array(
            [[4638.0, 0.0, 2048.0], [0.0, 4638.0, 1500.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        dist = np.zeros((1, 5), dtype=np.float64)
        flags = (
            cv2.CALIB_USE_INTRINSIC_GUESS
            | cv2.CALIB_FIX_ASPECT_RATIO
            | cv2.CALIB_FIX_PRINCIPAL_POINT
            | cv2.CALIB_FIX_K3
        )
        if args.fix_focal:
            flags |= cv2.CALIB_FIX_FOCAL_LENGTH
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, size, camera_matrix, dist, flags=flags
    )
    D = D.reshape(-1)
    if D.size < 5:
        D = np.pad(D, (0, 5 - D.size))
    D = D[:5]
    return {
        "rms_px": float(rms),
        "width": int(size[0]),
        "height": int(size[1]),
        "K": K.astype(np.float64),
        "D": D.astype(np.float64),
        "rvecs": rvecs,
        "tvecs": tvecs,
    }


def undistort_previews(
    frames: list[dict[str, Any]], K: np.ndarray, D: np.ndarray, out_dir: Path, count: int = 6
) -> dict[str, Any]:
    if not frames:
        return {}
    sample = frames[:: max(1, len(frames) // count)][:count]
    bgr0 = read_bgr(sample[0]["path"])
    h, w = bgr0.shape[:2]
    newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0.0)
    mapx, mapy = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_32FC1)
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for item in sample:
        raw = read_bgr(item["path"])
        und = cv2.remap(raw, mapx, mapy, cv2.INTER_LINEAR)
        before = out_dir / f"before_{item['path'].stem}.jpg"
        after = out_dir / f"after_{item['path'].stem}.jpg"
        cv2.imwrite(str(before), raw, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.imwrite(str(after), und, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        written.append({"before": str(before.name), "after": str(after.name)})
    x, y, rw, rh = [int(v) for v in roi]
    return {
        "newK": newK.astype(np.float64),
        "roi_xywh": [x, y, rw, rh],
        "preview_count": len(written),
        "files": written,
    }


def solve_pnp_frames(
    frames: list[dict[str, Any]], K: np.ndarray, D: np.ndarray
) -> list[dict[str, Any]]:
    out = []
    for item in frames:
        ok, rvec, tvec = cv2.solvePnP(
            item["object"], item["corners"], K, D, flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not ok:
            print(f"  PnP fail {item['path'].name}")
            continue
        proj, _ = cv2.projectPoints(item["object"], rvec, tvec, K, D)
        err = float(np.linalg.norm(proj.reshape(-1, 2) - item["corners"].reshape(-1, 2), axis=1).mean())
        cam_T_board = Rt_to_T(rotvec_to_R(rvec), tvec)
        out.append(
            {
                **item,
                "rvec": rvec.reshape(3),
                "tvec": tvec.reshape(3),
                "cam_T_board": cam_T_board,
                "pnp_rms_px": err,
            }
        )
    return out


def split_holdout(frames: list[dict[str, Any]], holdout: int) -> tuple[list, list]:
    if holdout <= 0 or holdout >= len(frames):
        return frames, []
    return frames[:-holdout], frames[-holdout:]


def run_hand_eye(
    fit: list[dict[str, Any]],
    *,
    prior_T: np.ndarray | None = None,
    max_prior_delta_mm: float = 80.0,
) -> dict[str, Any]:
    usable = [f for f in fit if f.get("flange_T") is not None]
    if len(usable) < 3:
        return {"ok": False, "error": f"need >= 3 flange poses, got {len(usable)}"}
    R_gripper2base = [f["flange_T"][:3, :3] for f in usable]
    t_gripper2base = [f["flange_T"][:3, 3].reshape(3, 1) for f in usable]
    R_target2cam = [f["cam_T_board"][:3, :3] for f in usable]
    t_target2cam = [f["cam_T_board"][:3, 3].reshape(3, 1) for f in usable]

    prior_t_mm = (
        prior_T[:3, 3] * 1000.0 if prior_T is not None else CAD_FLANGE_T_CAM_MM.copy()
    )

    methods = {}
    for name, flag in HAND_EYE_METHODS.items():
        try:
            R, t = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=flag
            )
        except cv2.error as exc:
            methods[name] = {"ok": False, "error": str(exc)}
            continue
        R = np.asarray(R, dtype=np.float64)
        t = np.asarray(t, dtype=np.float64).reshape(3)
        if not np.isfinite(R).all() or not np.isfinite(t).all():
            methods[name] = {"ok": False, "error": "non-finite R/t from calibrateHandEye"}
            continue
        flange_T_cam = Rt_to_T(R, t)
        board_Ts = [f["flange_T"] @ flange_T_cam @ f["cam_T_board"] for f in usable]
        try:
            mean_T = average_SE3(board_Ts)
        except (ValueError, np.linalg.LinAlgError) as exc:
            methods[name] = {"ok": False, "error": f"board average failed: {exc}"}
            continue
        t_err = []
        r_err = []
        for T in board_Ts:
            if not np.isfinite(T).all():
                continue
            t_err.append(np.linalg.norm(T[:3, 3] - mean_T[:3, 3]) * 1000.0)
            r_err.append(rotation_angle_deg(mean_T[:3, :3].T @ T[:3, :3]))
        if not t_err:
            methods[name] = {"ok": False, "error": "no finite board poses"}
            continue
        t_mm = flange_T_cam[:3, 3] * 1000.0
        cad_delta_mm = t_mm - CAD_FLANGE_T_CAM_MM
        prior_delta_mm = float(np.linalg.norm(t_mm - prior_t_mm))
        optical_z = R[:, 2]
        z_deg = float(math.degrees(math.acos(np.clip(optical_z[2], -1.0, 1.0))))
        methods[name] = {
            "ok": True,
            "flange_T_camera": flange_T_cam.tolist(),
            "t_cam_in_flange_mm": t_mm.tolist(),
            "cad_t_delta_mm": cad_delta_mm.tolist(),
            "prior_t_delta_mm": prior_delta_mm,
            "optical_z_vs_tool0_z_deg": z_deg,
            "base_T_board_spread_mm_median": float(np.median(t_err)),
            "base_T_board_spread_mm_max": float(np.max(t_err)),
            "base_T_board_spread_deg_median": float(np.median(r_err)),
            "base_T_board_spread_deg_max": float(np.max(r_err)),
            "n": len(usable),
        }

    ok_methods = {
        k: v
        for k, v in methods.items()
        if v.get("ok") and np.isfinite(v.get("prior_t_delta_mm", np.nan))
    }
    if not ok_methods:
        return {"ok": False, "error": "all hand-eye methods failed", "methods": methods}
    # Prefer physical mount agreement (lens change must not invent a new TCP).
    best = min(
        ok_methods.items(),
        key=lambda kv: (
            kv[1]["prior_t_delta_mm"],
            kv[1]["base_T_board_spread_mm_median"]
            + 10.0 * kv[1]["base_T_board_spread_deg_median"],
        ),
    )
    chosen = best[0]
    flange_T_cam = np.asarray(ok_methods[chosen]["flange_T_camera"], dtype=np.float64)
    used_prior = False
    note = None
    if prior_T is not None and float(ok_methods[chosen]["prior_t_delta_mm"]) > float(
        max_prior_delta_mm
    ):
        # Same camera body / mount: discard unphysical HE translation.
        flange_T_cam = prior_T.copy()
        used_prior = True
        note = (
            f"Rejected OpenCV HE '{chosen}' (|t-prior|="
            f"{ok_methods[chosen]['prior_t_delta_mm']:.1f} mm > {max_prior_delta_mm} mm). "
            "Kept app/CAD mount prior — lens-only change must not move tool0→camera by tens of cm."
        )
        print(f"WARNING: {note}")
    board_Ts = [f["flange_T"] @ flange_T_cam @ f["cam_T_board"] for f in usable]
    try:
        base_T_board = average_SE3(board_Ts)
    except (ValueError, np.linalg.LinAlgError) as exc:
        return {
            "ok": False,
            "error": f"base_T_board average failed: {exc}",
            "methods": methods,
        }
    return {
        "ok": True,
        "chosen_method": "prior_mount" if used_prior else chosen,
        "opencv_best_method": chosen,
        "used_mount_prior": used_prior,
        "note": note,
        "methods": methods,
        "flange_T_camera": flange_T_cam,
        "base_T_board": base_T_board,
        "n_fit": len(usable),
    }


def rpy_xyz_from_T(T: np.ndarray) -> dict[str, list[float]]:
    R, t = T_to_Rt(T)
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-8:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return {
        "xyz_m": t.tolist(),
        "xyz_mm": (t * 1000.0).tolist(),
        "rpy_rad": [roll, pitch, yaw],
        "rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
    }


def validate_holdout(
    hold: list[dict[str, Any]], flange_T_cam: np.ndarray, base_T_board: np.ndarray
) -> dict[str, Any]:
    rows = []
    for f in hold:
        if f.get("flange_T") is None:
            continue
        observed = f["flange_T"] @ flange_T_cam @ f["cam_T_board"]
        dt_mm = float(np.linalg.norm(observed[:3, 3] - base_T_board[:3, 3]) * 1000.0)
        dR = rotation_angle_deg(base_T_board[:3, :3].T @ observed[:3, :3])
        rows.append(
            {
                "file": f["path"].name,
                "translation_err_mm": dt_mm,
                "rotation_err_deg": dR,
                "pnp_rms_px": f["pnp_rms_px"],
            }
        )
    if not rows:
        return {"ok": False, "error": "no hold-out frames with flange TF"}
    return {
        "ok": True,
        "count": len(rows),
        "translation_err_mm_median": float(np.median([r["translation_err_mm"] for r in rows])),
        "rotation_err_deg_median": float(np.median([r["rotation_err_deg"] for r in rows])),
        "pnp_rms_px_median": float(np.median([r["pnp_rms_px"] for r in rows])),
        "frames": rows,
    }


def numpy_to_nested(K: np.ndarray) -> list:
    return np.asarray(K).tolist()


def run(args: argparse.Namespace) -> int:
    board = load_board(args.board)
    images = list_images(args.images)
    if args.max_images and args.max_images > 0:
        images = images[: args.max_images]
    if not images:
        print(f"No images in {args.images}", file=sys.stderr)
        return 1
    print(f"Board {board['pattern_size']} squares={board.get('squares_x')}x{board.get('squares_y')} "
          f"{board['square_size_mm']} mm")
    print(f"Images: {len(images)} in {args.images}")
    print("Note: pose JSON intrinsics are ignored for K/D (often wrong lens from capture cfg).")

    frames, size = detect_dataset(images, board)
    if len(frames) < 3:
        print(f"Need >= 3 detections, got {len(frames)}", file=sys.stderr)
        return 1

    fit, hold = split_holdout(frames, args.holdout)
    print(f"Fit {len(fit)}  hold-out {len(hold)}")

    intr = calibrate_intrinsics(fit, size, args)
    print(f"Intrinsics RMS {intr['rms_px']:.4f} px")
    if intr["rms_px"] >= 0.5:
        print("WARNING: RMS >= 0.5 px — do not copy K,D into hyperfusion.cfg")
    print("K =\n", intr["K"])
    print("D =", intr["D"])

    args.out.mkdir(parents=True, exist_ok=True)
    preview = undistort_previews(fit, intr["K"], intr["D"], args.out / "undistort_preview")
    passed = bool(intr["rms_px"] < 0.5)
    intr_yaml: dict[str, Any] = {
        "rms_px": intr["rms_px"],
        "width": intr["width"],
        "height": intr["height"],
        "K": numpy_to_nested(intr["K"]),
        "D": numpy_to_nested(intr["D"]),
        "model": "brown_conrady_k1_k2_p1_p2_k3",
        "newK": numpy_to_nested(preview.get("newK", intr["K"])),
        "roi_xywh": preview.get("roi_xywh"),
        "hyperfusion_cfg": {
            "bfs_camera_fx": float(intr["K"][0, 0]),
            "bfs_camera_fy": float(intr["K"][1, 1]),
            "bfs_camera_cx": float(intr["K"][0, 2]),
            "bfs_camera_cy": float(intr["K"][1, 2]),
            "bfs_camera_distortion": ", ".join(f"{v:.8g}" for v in intr["D"].tolist()),
        },
        "n_images": len(fit),
        "pass_rms_px_lt_0_5": passed,
        "note": "Fitted from checkerboard images; capture JSON K/D are not used as input.",
    }
    if not passed:
        intr_yaml["warning"] = (
            "RMS >= 0.5 px. K,D may still be used; recapture with close/far poses "
            "and board near image edges to tighten RMS."
        )
    write_yaml(args.out / "camera_intrinsics.yaml", intr_yaml)

    n_json = write_intrinsics_into_pose_jsons(
        Path(args.images),
        intr["K"],
        intr["D"],
        source=str((args.out / "camera_intrinsics.yaml").resolve()),
    )
    print(f"Wrote fitted K/D into {n_json} pose JSON files under {args.images}")

    pnp_fit = solve_pnp_frames(fit, intr["K"], intr["D"])
    pnp_hold = solve_pnp_frames(hold, intr["K"], intr["D"])
    if pnp_fit:
        print(f"PnP fit median RMS {np.median([f['pnp_rms_px'] for f in pnp_fit]):.4f} px")

    prior_T = load_prior_flange_T_camera_mm(args.prior_tcp_xyz_mm, args.prior_tcp_rpy_deg)
    print(
        "HE mount prior xyz_mm =",
        [round(v, 3) for v in args.prior_tcp_xyz_mm],
        "rpy_deg =",
        [round(v, 4) for v in args.prior_tcp_rpy_deg],
    )
    he = run_hand_eye(
        pnp_fit,
        prior_T=prior_T,
        max_prior_delta_mm=float(args.he_max_prior_delta_mm),
    )
    if not he.get("ok"):
        print(f"Hand-eye skipped: {he.get('error')}")
        write_yaml(args.out / "flange_T_camera.yaml", he)
        write_yaml(
            args.out / "validation.yaml",
            {"ok": False, "note": "hand-eye needs base_T_flange in per-image JSON"},
        )
        print(f"Wrote {args.out / 'camera_intrinsics.yaml'} and undistort_preview/")
        return 0

    flange_T_cam = he["flange_T_camera"]
    base_T_board = he["base_T_board"]
    write_yaml(
        args.out / "flange_T_camera.yaml",
        {
            "method": he["chosen_method"],
            "opencv_best_method": he.get("opencv_best_method"),
            "used_mount_prior": he.get("used_mount_prior", False),
            "note": he.get("note"),
            "T": numpy_to_nested(flange_T_cam),
            "urdf_origin": rpy_xyz_from_T(flange_T_cam),
            "cad_tool_tcp_mm": CAD_FLANGE_T_CAM_MM.tolist(),
            "prior_tool_tcp_mm": list(args.prior_tcp_xyz_mm),
            "prior_tool_tcp_rpy_deg": list(args.prior_tcp_rpy_deg),
            "methods": he["methods"],
            "n_fit": he["n_fit"],
        },
    )
    write_yaml(
        args.out / "base_T_board.yaml",
        {
            "parent_frame": "base_link",
            "child_frame": "board",
            "T": numpy_to_nested(base_T_board),
            "origin": rpy_xyz_from_T(base_T_board),
            "n_fit": he["n_fit"],
            "chosen_hand_eye": he["chosen_method"],
        },
    )
    val = validate_holdout(pnp_hold, flange_T_cam, base_T_board)
    write_yaml(args.out / "validation.yaml", val)
    print(f"Hand-eye method {he['chosen_method']}")
    print("flange t mm", (flange_T_cam[:3, 3] * 1000.0).tolist())
    if val.get("ok"):
        print(
            f"Hold-out median {val['translation_err_mm_median']:.2f} mm  "
            f"{val['rotation_err_deg_median']:.3f} deg  "
            f"{val['pnp_rms_px_median']:.3f} px"
        )
    print(f"Results in {args.out}")
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="BFS + UR3e checkerboard calibration")
    p.add_argument("--board", type=Path, default=DEFAULT_BOARD)
    p.add_argument("--images", type=Path, default=DEFAULT_IMAGES)
    p.add_argument("--out", type=Path, default=DEFAULT_RESULTS)
    p.add_argument("--holdout", type=int, default=6, help="last N frames held out (0 = none)")
    p.add_argument("--max-images", type=int, default=0, help="use only the first N images (0 = all)")
    p.add_argument(
        "--use-nominal-k",
        action="store_true",
        help="seed K with dummy fx=fy=4638 (do not use; free K is the measured camera)",
    )
    p.add_argument(
        "--fix-focal",
        action="store_true",
        help="with --use-nominal-k, do not move fx/fy (solve principal point + distortion only)",
    )
    p.add_argument(
        "--prior-tcp-xyz-mm",
        type=float,
        nargs=3,
        default=[0.693, -71.639, 114.199],
        metavar=("X", "Y", "Z"),
        help="Physical mount prior tool0→camera (mm). Default = hyperfusion.cfg tool_tcp.",
    )
    p.add_argument(
        "--prior-tcp-rpy-deg",
        type=float,
        nargs=3,
        default=[-0.3469, -0.2031, 0.1778],
        metavar=("ROLL", "PITCH", "YAW"),
        help="Physical mount prior RPY (deg). Default = hyperfusion.cfg tool_tcp.",
    )
    p.add_argument(
        "--he-max-prior-delta-mm",
        type=float,
        default=80.0,
        help="If OpenCV HE |t-prior| exceeds this, keep the mount prior instead.",
    )
    return p.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
