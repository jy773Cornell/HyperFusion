"""Plane calibration: projector column + camera rays → projector P.

Single-plane 3×4 P is degenerate. Primary model is a RANSAC 1D homography
from undistorted camera pixels (or tray XY) to projector u.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .capture import camera_extrinsics_rt, camera_intrinsics
from .decode import DecodeResult


@dataclass
class FppCalibration:
    projector_p: np.ndarray | None
    camera_k: np.ndarray | None
    camera_r: np.ndarray | None
    camera_t: np.ndarray | None
    plane_z_m: float
    u_reference: np.ndarray
    mask: np.ndarray
    depth_scale_m_per_u: float | None
    u_homography: np.ndarray | None = None
    undistorted: bool = False
    inlier_count: int = 0

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            projector_p=self.projector_p if self.projector_p is not None else np.array([]),
            camera_k=self.camera_k if self.camera_k is not None else np.array([]),
            camera_r=self.camera_r if self.camera_r is not None else np.array([]),
            camera_t=self.camera_t if self.camera_t is not None else np.array([]),
            plane_z_m=np.float64(self.plane_z_m),
            u_reference=self.u_reference,
            mask=self.mask,
            depth_scale_m_per_u=(
                np.float64(self.depth_scale_m_per_u)
                if self.depth_scale_m_per_u is not None
                else np.array([])
            ),
            u_homography=self.u_homography if self.u_homography is not None else np.array([]),
            undistorted=np.uint8(1 if self.undistorted else 0),
            inlier_count=np.int64(self.inlier_count),
        )

    @staticmethod
    def load(path: Path) -> "FppCalibration":
        data = np.load(Path(path), allow_pickle=False)

        def maybe(name: str, shape: tuple[int, ...]) -> np.ndarray | None:
            if name not in data.files:
                return None
            arr = np.asarray(data[name])
            if arr.size == 0:
                return None
            return arr.reshape(shape)

        scale_raw = np.asarray(data["depth_scale_m_per_u"])
        scale = float(scale_raw) if scale_raw.size == 1 else None
        undist = False
        if "undistorted" in data.files:
            undist = bool(int(np.asarray(data["undistorted"])))
        inliers = 0
        if "inlier_count" in data.files:
            inliers = int(np.asarray(data["inlier_count"]))
        return FppCalibration(
            projector_p=maybe("projector_p", (3, 4)),
            camera_k=maybe("camera_k", (3, 3)),
            camera_r=maybe("camera_r", (3, 3)),
            camera_t=maybe("camera_t", (3,)),
            plane_z_m=float(data["plane_z_m"]),
            u_reference=np.asarray(data["u_reference"], dtype=np.float32),
            mask=np.asarray(data["mask"], dtype=bool),
            depth_scale_m_per_u=scale,
            u_homography=maybe("u_homography", (6,)),
            undistorted=undist,
            inlier_count=inliers,
        )


def _intersect_plane_z(
    k: np.ndarray,
    r: np.ndarray,
    t: np.ndarray,
    pixels: np.ndarray,
    plane_z_m: float,
) -> np.ndarray:
    """Camera pixels (N,2) → base_link points on Z=plane_z_m."""
    kinv = np.linalg.inv(k)
    ones = np.ones((pixels.shape[0], 1), dtype=np.float64)
    dirs_cam = (kinv @ np.hstack([pixels, ones]).T).T
    dirs_base = (r @ dirs_cam.T).T
    denom = dirs_base[:, 2]
    lam = (plane_z_m - t[2]) / denom
    return t.reshape(1, 3) + lam[:, None] * dirs_base


def _dlt_projector(points_h: np.ndarray, u: np.ndarray) -> np.ndarray:
    """3×4 P with unused middle row. u ≈ (P X)_0 / (P X)_2. points_h is (N,4)."""
    a_rows = [np.concatenate([x, -ui * x]) for x, ui in zip(points_h, u)]
    a = np.asarray(a_rows, dtype=np.float64)
    _, _, vt = np.linalg.svd(a)
    vec = vt[-1]
    p0, p2 = vec[:4], vec[4:]
    p = np.vstack([p0, np.zeros(4), p2])
    if p[2, 3] < 0:
        p = -p
    return p


def _fit_u_homography(xy: np.ndarray, u: np.ndarray) -> np.ndarray | None:
    """u = (h0 x + h1 y + h2) / (h3 x + h4 y + h5). xy is (N,2)."""
    if xy.shape[0] < 8:
        return None
    x, y = xy[:, 0], xy[:, 1]
    ones = np.ones_like(u)
    design = np.column_stack([x, y, ones, -u * x, -u * y, -u])
    _, _, vt = np.linalg.svd(design, full_matrices=False)
    h = vt[-1]
    if abs(float(h[5])) > 1.0e-12:
        h = h / h[5]
    return h


def apply_u_homography(h: np.ndarray, xy: np.ndarray) -> np.ndarray:
    x, y = xy[:, 0], xy[:, 1]
    den = h[3] * x + h[4] * y + h[5]
    den = np.where(np.abs(den) < 1.0e-12, np.nan, den)
    return (h[0] * x + h[1] * y + h[2]) / den


def _ransac_u_homography(
    xy: np.ndarray,
    u: np.ndarray,
    *,
    thresh_u: float,
    iters: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray | None, np.ndarray]:
    n = int(u.size)
    if n < 16:
        return None, np.zeros(n, dtype=bool)
    sample_n = 8
    best_h: np.ndarray | None = None
    best_inliers = np.zeros(n, dtype=bool)
    best_count = 0
    for _ in range(int(iters)):
        pick = rng.choice(n, size=sample_n, replace=False)
        h = _fit_u_homography(xy[pick], u[pick])
        if h is None:
            continue
        pred = apply_u_homography(h, xy)
        err = np.abs(pred - u)
        inliers = np.isfinite(err) & (err < float(thresh_u))
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_h = h
            best_inliers = inliers
    if best_h is None or best_count < 16:
        return None, np.zeros(n, dtype=bool)
    h_refit = _fit_u_homography(xy[best_inliers], u[best_inliers])
    if h_refit is None:
        return best_h, best_inliers
    pred = apply_u_homography(h_refit, xy)
    err = np.abs(pred - u)
    inliers = np.isfinite(err) & (err < float(thresh_u))
    if int(inliers.sum()) >= 16:
        h_final = _fit_u_homography(xy[inliers], u[inliers])
        if h_final is not None:
            return h_final, inliers
    return h_refit, best_inliers


def fit_u_homography_ransac(
    xy: np.ndarray,
    u: np.ndarray,
    *,
    thresh_u: float = 8.0,
    iters: int = 60,
    seed: int = 0,
) -> tuple[np.ndarray | None, np.ndarray]:
    rng = np.random.default_rng(int(seed))
    return _ransac_u_homography(xy, u, thresh_u=thresh_u, iters=iters, rng=rng)


def fit_calibration(
    decoded: DecodeResult,
    meta: dict,
    *,
    plane_z_m: float,
    max_points: int = 40000,
    ransac_thresh_u: float = 8.0,
    ransac_iters: int = 400,
    undistorted: bool = False,
    keep_all_pixels: bool = False,
) -> FppCalibration:
    k_dist = camera_intrinsics(meta)
    rt = camera_extrinsics_rt(meta)
    u_ref = decoded.projector_u.copy()
    mask = decoded.mask.copy()

    projector_p = None
    camera_k = None
    camera_r = None
    camera_t = None
    u_h = None
    inlier_count = 0
    vs, us = np.nonzero(mask)
    if vs.size >= 16:
        if vs.size > max_points:
            pick = np.linspace(0, vs.size - 1, max_points, dtype=int)
            vs_fit, us_fit = vs[pick], us[pick]
        else:
            vs_fit, us_fit = vs, us
        pixels = np.stack([us_fit.astype(np.float64), vs_fit.astype(np.float64)], axis=1)
        u_samp = u_ref[vs_fit, us_fit].astype(np.float64)
        finite = np.isfinite(pixels).all(axis=1) & np.isfinite(u_samp)
        pixels = pixels[finite]
        u_samp = u_samp[finite]
        vs_fit, us_fit = vs_fit[finite], us_fit[finite]
        rng = np.random.default_rng(0)
        u_h, inliers = _ransac_u_homography(
            pixels,
            u_samp,
            thresh_u=ransac_thresh_u,
            iters=ransac_iters,
            rng=rng,
        )
        if u_h is not None and inliers.any():
            inlier_count = int(inliers.sum())
            if not keep_all_pixels:
                keep = np.zeros(mask.shape, dtype=bool)
                keep[vs_fit[inliers], us_fit[inliers]] = True
                if vs.size > max_points:
                    pred_all = apply_u_homography(
                        u_h,
                        np.stack([us.astype(np.float64), vs.astype(np.float64)], axis=1),
                    )
                    err_all = np.abs(pred_all - u_ref[vs, us].astype(np.float64))
                    ok_all = np.isfinite(err_all) & (err_all < float(ransac_thresh_u))
                    keep[vs[ok_all], us[ok_all]] = True
                    inlier_count = int(keep.sum())
                mask = keep
            else:
                inlier_count = int(mask.sum())
        if k_dist is not None and rt is not None:
            camera_k, _dist = k_dist
            camera_r, camera_t = rt
            vs_p, us_p = np.nonzero(mask)
            if vs_p.size >= 16:
                if vs_p.size > max_points:
                    pick = np.linspace(0, vs_p.size - 1, max_points, dtype=int)
                    vs_p, us_p = vs_p[pick], us_p[pick]
                pix = np.stack([us_p.astype(np.float64), vs_p.astype(np.float64)], axis=1)
                xyz = _intersect_plane_z(camera_k, camera_r, camera_t, pix, plane_z_m)
                u_p = u_ref[vs_p, us_p].astype(np.float64)
                finite_p = np.isfinite(xyz).all(axis=1) & np.isfinite(u_p)
                xyz = xyz[finite_p]
                u_p = u_p[finite_p]
                if xyz.shape[0] >= 16:
                    points_h = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
                    projector_p = _dlt_projector(points_h, u_p)

    return FppCalibration(
        projector_p=projector_p,
        camera_k=camera_k,
        camera_r=camera_r,
        camera_t=camera_t,
        plane_z_m=plane_z_m,
        u_reference=u_ref,
        mask=mask,
        depth_scale_m_per_u=None,
        u_homography=u_h,
        undistorted=undistorted,
        inlier_count=inlier_count,
    )


def evaluate_calibration(calib: FppCalibration, *, max_points: int = 80000) -> dict:
    """Coverage + 1D homography residual (single-plane 3×4 P is degenerate)."""
    mask = calib.mask
    vs, us = np.nonzero(mask)
    out: dict = {
        "valid_px": int(vs.size),
        "mask_frac": float(mask.mean()) if mask.size else 0.0,
        "has_projector_p": calib.projector_p is not None,
        "p_degenerate": False,
        "u_span": None,
        "residual_rms_u": None,
        "residual_med_u": None,
        "residual_p95_u": None,
        "z_cam_med_m": None,
        "residual_model": None,
        "undistorted": bool(calib.undistorted),
        "inlier_count": int(calib.inlier_count),
    }
    u = calib.u_reference
    if vs.size:
        uu = u[vs, us]
        if np.isfinite(uu).any():
            out["u_span"] = [float(np.nanmin(uu)), float(np.nanmax(uu))]
    if calib.projector_p is not None:
        p2 = calib.projector_p[2]
        # P row 2 ≈ k·(0,0,1,-plane_z) → den≈0 on the tray.
        plane_n = np.array([0.0, 0.0, 1.0, -float(calib.plane_z_m)])
        if np.linalg.norm(p2) > 0:
            align = abs(float(np.dot(p2 / np.linalg.norm(p2), plane_n / np.linalg.norm(plane_n))))
            out["p_degenerate"] = align > 0.98
    if vs.size < 16:
        return out
    if vs.size > max_points:
        pick = np.linspace(0, vs.size - 1, max_points, dtype=int)
        vs, us = vs[pick], us[pick]
    pixels = np.stack([us.astype(np.float64), vs.astype(np.float64)], axis=1)
    u_obs = u[vs, us].astype(np.float64)
    finite = np.isfinite(pixels).all(axis=1) & np.isfinite(u_obs)
    pixels = pixels[finite]
    u_obs = u_obs[finite]
    vs, us = vs[finite], us[finite]
    if pixels.shape[0] < 16:
        return out
    h = calib.u_homography if calib.u_homography is not None else _fit_u_homography(pixels, u_obs)
    if h is None:
        return out
    pred = apply_u_homography(h, pixels)
    err = pred - u_obs
    ok = np.isfinite(err)
    err = err[ok]
    if err.size == 0:
        return out
    out["residual_model"] = (
        "undistorted_pixel_to_u_homography" if calib.undistorted else "pixel_to_u_homography"
    )
    out["residual_rms_u"] = float(np.sqrt(np.mean(err * err)))
    out["residual_med_u"] = float(np.median(np.abs(err)))
    out["residual_p95_u"] = float(np.percentile(np.abs(err), 95))
    if calib.camera_k is None or calib.camera_r is None or calib.camera_t is None:
        return out
    xyz = _intersect_plane_z(calib.camera_k, calib.camera_r, calib.camera_t, pixels[ok], calib.plane_z_m)
    if xyz.size:
        zcam = (calib.camera_r.T @ (xyz - calib.camera_t).T).T[:, 2]
        out["z_cam_med_m"] = float(np.median(zcam[np.isfinite(zcam)])) if np.isfinite(zcam).any() else None
    return out


INDEX_NAME = "fpp_calib_index.json"


def pose_calib_path(out_dir: Path, stem: str) -> Path:
    return Path(out_dir) / f"{stem}_fpp_calib.npz"


def resolve_calibration(
    calib_arg: Path,
    start: Path,
    meta: dict,
    *,
    max_t_m: float = 0.025,
) -> tuple[FppCalibration, Path, str]:
    """Load a single .npz, or the matching per-pose file from a calib folder.

    Match order: `{stem}_fpp_calib.npz`, then nearest camera t in the index.
    """
    path = Path(calib_arg)
    if path.is_file():
        return FppCalibration.load(path), path, "file"
    if not path.is_dir():
        raise FileNotFoundError(f"FPP calib not found: {path}")

    exact = pose_calib_path(path, start.stem)
    if exact.is_file():
        return FppCalibration.load(exact), exact, "stem"

    index_path = path / INDEX_NAME
    if not index_path.is_file():
        raise FileNotFoundError(f"No {exact.name} or {INDEX_NAME} in {path}")

    index = json.loads(index_path.read_text(encoding="utf-8"))
    poses = index.get("poses") or []
    rt = camera_extrinsics_rt(meta)
    if rt is None or not poses:
        raise FileNotFoundError(f"Cannot match pose calib in {path}")
    t = rt[1]
    best = None
    best_dist = float("inf")
    for rec in poses:
        ct = rec.get("camera_t")
        rel = rec.get("calib")
        if not ct or not rel:
            continue
        dist = float(np.linalg.norm(np.asarray(ct, dtype=np.float64) - t))
        if dist < best_dist:
            best_dist = dist
            best = rec
    if best is None:
        raise FileNotFoundError(f"Empty pose index: {index_path}")
    if best_dist > max_t_m:
        raise FileNotFoundError(
            f"Nearest calib t is {best_dist * 1000:.1f} mm from this burst "
            f"(limit {max_t_m * 1000:.0f} mm). Recapture or pass the pose .npz."
        )
    chosen = path / str(best["calib"])
    return FppCalibration.load(chosen), chosen, f"nearest_t_{best_dist:.4f}m"
