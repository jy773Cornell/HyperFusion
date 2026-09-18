# Camera–projector stereo from checkerboard FPP bursts (sidecar / offline).
# Metric millimetres; does not use a known tray Z. No robot motion.
"""Fit projector K and camera_T_projector from tilted-board bursts."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from .board_detect import (
    corners_in_patch,
    detect_on_white,
    object_points_for,
    sample_map,
)
from .capture import FppBurst, PROJECTOR_HEIGHT_PX, PROJECTOR_WIDTH_PX, camera_intrinsics
from .decode import DecodeResult


@dataclass
class StereoGeometry:
    camera_k: np.ndarray
    camera_d: np.ndarray
    projector_k: np.ndarray
    projector_d: np.ndarray
    R: np.ndarray
    t: np.ndarray
    camera_size: tuple[int, int]
    projector_size: tuple[int, int]
    undistorted: bool = True

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "camera_k": self.camera_k.astype(np.float64).tolist(),
            "camera_d": self.camera_d.reshape(-1).astype(np.float64).tolist(),
            "projector_k": self.projector_k.astype(np.float64).tolist(),
            "projector_d": self.projector_d.reshape(-1).astype(np.float64).tolist(),
            "R": self.R.astype(np.float64).tolist(),
            "t": self.t.reshape(-1).astype(np.float64).tolist(),
            "camera_size": [int(self.camera_size[0]), int(self.camera_size[1])],
            "projector_size": [int(self.projector_size[0]), int(self.projector_size[1])],
            "undistorted": bool(self.undistorted),
            "units": "metres",
            "R_meaning": "X_proj = R @ X_cam + t",
        }
        if path.suffix.lower() == ".json":
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            return
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        np.savez_compressed(
            path.with_suffix(".npz"),
            camera_k=self.camera_k,
            camera_d=self.camera_d,
            projector_k=self.projector_k,
            projector_d=self.projector_d,
            R=self.R,
            t=self.t,
            camera_size=np.asarray(self.camera_size, dtype=np.int32),
            projector_size=np.asarray(self.projector_size, dtype=np.int32),
            undistorted=np.uint8(1 if self.undistorted else 0),
        )

    @staticmethod
    def load(path: Path) -> "StereoGeometry":
        path = Path(path)
        npz = path if path.suffix.lower() == ".npz" else path.with_suffix(".npz")
        if npz.is_file():
            data = np.load(npz, allow_pickle=False)
            return StereoGeometry(
                camera_k=np.asarray(data["camera_k"], dtype=np.float64).reshape(3, 3),
                camera_d=np.asarray(data["camera_d"], dtype=np.float64).reshape(-1),
                projector_k=np.asarray(data["projector_k"], dtype=np.float64).reshape(3, 3),
                projector_d=np.asarray(data["projector_d"], dtype=np.float64).reshape(-1),
                R=np.asarray(data["R"], dtype=np.float64).reshape(3, 3),
                t=np.asarray(data["t"], dtype=np.float64).reshape(3),
                camera_size=tuple(int(x) for x in np.asarray(data["camera_size"]).reshape(-1)[:2]),
                projector_size=tuple(
                    int(x) for x in np.asarray(data["projector_size"]).reshape(-1)[:2]
                ),
                undistorted=bool(int(np.asarray(data["undistorted"])))
                if "undistorted" in data.files
                else True,
            )
        text = path.read_text(encoding="utf-8")
        payload = yaml.safe_load(text) if path.suffix.lower() in {".yaml", ".yml"} else json.loads(text)
        return StereoGeometry(
            camera_k=np.asarray(payload["camera_k"], dtype=np.float64).reshape(3, 3),
            camera_d=np.asarray(payload["camera_d"], dtype=np.float64).reshape(-1),
            projector_k=np.asarray(payload["projector_k"], dtype=np.float64).reshape(3, 3),
            projector_d=np.asarray(payload["projector_d"], dtype=np.float64).reshape(-1),
            R=np.asarray(payload["R"], dtype=np.float64).reshape(3, 3),
            t=np.asarray(payload["t"], dtype=np.float64).reshape(3),
            camera_size=tuple(int(x) for x in payload["camera_size"]),
            projector_size=tuple(int(x) for x in payload["projector_size"]),
            undistorted=bool(payload.get("undistorted", True)),
        )


def observe_burst(
    burst: FppBurst,
    decoded: DecodeResult,
    board: dict,
    *,
    require_all_in_patch: bool = True,
) -> tuple[dict | None, str]:
    """One Execute burst → corner / projector correspondences, or a skip reason."""
    if 1 not in burst.by_index or 0 not in burst.by_index:
        return None, "missing_black_or_white"
    found = detect_on_white(burst.image(1), burst.image(0), board["pattern_size"])
    if found is None:
        return None, "no_70_corners_on_white"
    pattern, corners = found
    in_patch = corners_in_patch(decoded, corners)
    n_dark = int((~in_patch).sum())
    if require_all_in_patch and n_dark:
        return None, f"corners_in_dark={n_dark}"
    uv = projector_uv_at_corners(decoded, corners)
    finite_uv = np.isfinite(uv).all(axis=1)
    if require_all_in_patch and not bool(finite_uv.all()):
        return None, f"nan_projector_uv={int((~finite_uv).sum())}"
    k_dist = camera_intrinsics(burst.first_meta())
    if k_dist is None:
        return None, "no_camera_K_in_json"
    k, dist = k_dist
    white = burst.image(1)
    h, w = white.shape[:2]
    xy = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    return {
        "stem": burst.frames[0].path.stem,
        "pattern": pattern,
        "object": object_points_for(board, pattern),
        "camera_xy": xy,
        "projector_uv": uv,
        "in_patch": in_patch,
        "n_corners": int(xy.shape[0]),
        "n_in_patch": int(in_patch.sum()),
        "camera_k": k,
        "camera_d": dist,
        "camera_size": (int(w), int(h)),
        "white": white,
    }, "ok"


def projector_uv_at_corners(decoded: DecodeResult, corners: np.ndarray) -> np.ndarray:
    xy = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    u = sample_map(decoded.projector_u, xy)
    v = sample_map(decoded.projector_v, xy)
    return np.stack([u, v], axis=1)


def _as_img_pts(xy: np.ndarray) -> np.ndarray:
    return np.asarray(xy, dtype=np.float32).reshape(-1, 1, 2)


def _mean_rotation(rots: list[np.ndarray]) -> np.ndarray:
    acc = np.zeros((3, 3), dtype=np.float64)
    for r in rots:
        acc += r
    u, _, vt = np.linalg.svd(acc)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1.0
        r = u @ vt
    return r


def compose_camera_to_projector(
    r_cam_board: np.ndarray,
    t_cam_board: np.ndarray,
    r_proj_board: np.ndarray,
    t_proj_board: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """X_proj = R @ X_cam + t from two board poses."""
    r = r_proj_board @ r_cam_board.T
    t = t_proj_board.reshape(3) - r @ t_cam_board.reshape(3)
    return r, t


def solve_board_pose(
    object_pts: np.ndarray,
    image_pts: np.ndarray,
    k: np.ndarray,
    dist: np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(object_pts, dtype=np.float32),
        _as_img_pts(image_pts),
        k,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None
    r, _ = cv2.Rodrigues(rvec)
    return r, tvec.reshape(3)


def fit_stereo(
    views: list[dict],
    camera_k: np.ndarray,
    camera_d: np.ndarray,
    camera_size: tuple[int, int],
    *,
    projector_size: tuple[int, int] = (PROJECTOR_WIDTH_PX, PROJECTOR_HEIGHT_PX),
    undistorted: bool = True,
) -> tuple[StereoGeometry, dict]:
    """views: object (N,3), camera_xy (N,2), projector_uv (N,2)."""
    if len(views) < 4:
        raise ValueError(f"Need >= 4 tilted-board bursts, got {len(views)}")
    obj = [np.asarray(v["object"], dtype=np.float32) for v in views]
    cam = [_as_img_pts(v["camera_xy"]) for v in views]
    proj = [_as_img_pts(v["projector_uv"]) for v in views]
    flags_p = (
        cv2.CALIB_ZERO_TANGENT_DIST
        | cv2.CALIB_FIX_K3
        | cv2.CALIB_FIX_K4
        | cv2.CALIB_FIX_K5
        | cv2.CALIB_FIX_K6
    )
    rms_p, kp, dp, rvecs_p, tvecs_p = cv2.calibrateCamera(
        obj, proj, projector_size, None, None, flags=flags_p
    )
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    stereo_ok = True
    try:
        rms_s, kc, dc, kp2, dp2, r, t, _e, _f = cv2.stereoCalibrate(
            obj,
            cam,
            proj,
            np.asarray(camera_k, dtype=np.float64),
            np.asarray(camera_d, dtype=np.float64).reshape(-1, 1),
            kp,
            dp,
            camera_size,
            flags=cv2.CALIB_FIX_INTRINSIC,
            criteria=criteria,
        )
        kp, dp = kp2, dp2
        camera_k = kc
        camera_d = dc.reshape(-1)
        r = np.asarray(r, dtype=np.float64)
        t = np.asarray(t, dtype=np.float64).reshape(3)
    except cv2.error:
        stereo_ok = False
        rms_s = float("nan")
        rots = []
        trans = []
        for view, rvec, tvec in zip(views, rvecs_p, tvecs_p):
            cam_pose = solve_board_pose(view["object"], view["camera_xy"], camera_k, camera_d)
            if cam_pose is None:
                continue
            rp, _ = cv2.Rodrigues(rvec)
            rots.append(compose_camera_to_projector(cam_pose[0], cam_pose[1], rp, tvec.reshape(3))[0])
            trans.append(compose_camera_to_projector(cam_pose[0], cam_pose[1], rp, tvec.reshape(3))[1])
        if len(rots) < 4:
            raise RuntimeError("stereoCalibrate failed and PnP compose had < 4 views")
        r = _mean_rotation(rots)
        t = np.median(np.stack(trans, axis=0), axis=0)

    geom = StereoGeometry(
        camera_k=np.asarray(camera_k, dtype=np.float64).reshape(3, 3),
        camera_d=np.asarray(camera_d, dtype=np.float64).reshape(-1),
        projector_k=np.asarray(kp, dtype=np.float64).reshape(3, 3),
        projector_d=np.asarray(dp, dtype=np.float64).reshape(-1),
        R=r,
        t=t,
        camera_size=camera_size,
        projector_size=projector_size,
        undistorted=undistorted,
    )
    stats = {
        "n_views": len(views),
        "projector_rms_px": float(rms_p),
        "stereo_rms_px": float(rms_s) if np.isfinite(rms_s) else None,
        "stereo_from": "stereoCalibrate" if stereo_ok else "pnp_compose",
        "baseline_mm": float(np.linalg.norm(t) * 1000.0),
    }
    return geom, stats


def projection_matrices(geom: StereoGeometry) -> tuple[np.ndarray, np.ndarray]:
    p_cam = geom.camera_k @ np.hstack([np.eye(3), np.zeros((3, 1))])
    p_proj = geom.projector_k @ np.hstack([geom.R, geom.t.reshape(3, 1)])
    return p_cam, p_proj


def triangulate_points(geom: StereoGeometry, camera_xy: np.ndarray, projector_uv: np.ndarray) -> np.ndarray:
    """Camera-frame XYZ (metres), shape (N,3). Invalid rows are NaN."""
    cam = np.asarray(camera_xy, dtype=np.float64).reshape(-1, 2)
    proj = np.asarray(projector_uv, dtype=np.float64).reshape(-1, 2)
    n = cam.shape[0]
    xyz = np.full((n, 3), np.nan, dtype=np.float64)
    finite = np.isfinite(cam).all(axis=1) & np.isfinite(proj).all(axis=1)
    if int(finite.sum()) < 1:
        return xyz
    p_cam, p_proj = projection_matrices(geom)
    hom = cv2.triangulatePoints(p_cam, p_proj, cam[finite].T, proj[finite].T)
    w = hom[3]
    good = np.abs(w) > 1.0e-12
    pts = np.full((int(finite.sum()), 3), np.nan, dtype=np.float64)
    pts[good] = (hom[:3, good] / w[good]).T
    xyz[finite] = pts
    return xyz


def triangulate_maps(
    geom: StereoGeometry,
    decoded: DecodeResult,
    *,
    stride: int = 4,
) -> tuple[np.ndarray, np.ndarray]:
    """Camera-frame XYZ (m) and Z-only depth on the decode grid. Unlit stays NaN."""
    h, w = decoded.mask.shape
    xyz = np.full((h, w, 3), np.nan, dtype=np.float32)
    depth = np.full((h, w), np.nan, dtype=np.float32)
    vs, us = np.nonzero(
        decoded.mask & np.isfinite(decoded.projector_u) & np.isfinite(decoded.projector_v)
    )
    if stride > 1:
        keep = (vs % stride == 0) & (us % stride == 0)
        vs, us = vs[keep], us[keep]
    if vs.size == 0:
        return xyz, depth
    cam = np.stack([us.astype(np.float64), vs.astype(np.float64)], axis=1)
    proj = np.stack(
        [decoded.projector_u[vs, us].astype(np.float64), decoded.projector_v[vs, us].astype(np.float64)],
        axis=1,
    )
    pts = triangulate_points(geom, cam, proj)
    xyz[vs, us] = pts.astype(np.float32)
    depth[vs, us] = pts[:, 2].astype(np.float32)
    return xyz, depth


def evaluate_holdout(geom: StereoGeometry, views: list[dict]) -> dict:
    """Triangulated corners vs camera PnP board points (mm)."""
    errors: list[float] = []
    reproj_u: list[float] = []
    per: list[dict] = []
    for view in views:
        pose = solve_board_pose(view["object"], view["camera_xy"], geom.camera_k, geom.camera_d)
        if pose is None:
            per.append({"stem": view.get("stem"), "ok": False, "reason": "pnp_fail"})
            continue
        r, t = pose
        xyz_pnp = (r @ np.asarray(view["object"], dtype=np.float64).T).T + t.reshape(1, 3)
        xyz_tri = triangulate_points(geom, view["camera_xy"], view["projector_uv"])
        err = np.linalg.norm(xyz_tri - xyz_pnp, axis=1) * 1000.0
        ok = np.isfinite(err)
        rms = float(np.sqrt(np.mean(err[ok] ** 2))) if ok.any() else None
        if ok.any():
            errors.extend(err[ok].tolist())
        _p_cam, p_proj = projection_matrices(geom)
        hom = np.hstack([xyz_tri, np.ones((xyz_tri.shape[0], 1))])
        proj_h = (p_proj @ hom.T).T
        pred = proj_h[:, :2] / proj_h[:, 2:3]
        duv = np.linalg.norm(pred - np.asarray(view["projector_uv"], dtype=np.float64), axis=1)
        duv_ok = np.isfinite(duv)
        if duv_ok.any():
            reproj_u.extend(duv[duv_ok].tolist())
        per.append(
            {
                "stem": view.get("stem"),
                "ok": True,
                "n": int(ok.sum()),
                "rms_mm": rms,
                "med_mm": float(np.median(err[ok])) if ok.any() else None,
                "max_mm": float(np.max(err[ok])) if ok.any() else None,
                "reproj_rms_uv": float(np.sqrt(np.mean(duv[duv_ok] ** 2))) if duv_ok.any() else None,
            }
        )
    out = {
        "n_views": len(views),
        "rms_mm": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        "med_mm": float(np.median(errors)) if errors else None,
        "p95_mm": float(np.percentile(errors, 95)) if errors else None,
        "max_mm": float(np.max(errors)) if errors else None,
        "reproj_rms_uv": float(np.sqrt(np.mean(np.square(reproj_u)))) if reproj_u else None,
        "views": per,
    }
    return out
