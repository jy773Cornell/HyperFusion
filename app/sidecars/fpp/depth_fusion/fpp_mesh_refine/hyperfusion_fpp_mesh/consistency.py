# Multi-view depth consistency (sidecar / depth_fusion / fpp_mesh_refine).
# Project depth into neighbor views; label + reject outliers. No robot I/O.
"""Cross-view depth consistency / visibility labels / outlier rejection."""

from __future__ import annotations

import numpy as np

from .frames import DepthFrame

# Cross-view comparison labels (per source-pixel × other-view).
LABEL_AGREE = 0
LABEL_NOT_OBSERVED = 1
LABEL_OCCLUDED = 2
LABEL_CONTRADICT = 3

LABEL_NAMES = {
    LABEL_AGREE: "AGREE",
    LABEL_NOT_OBSERVED: "NOT_OBSERVED",
    LABEL_OCCLUDED: "OCCLUDED",
    LABEL_CONTRADICT: "CONTRADICT",
}


def _project_world_to_depth(
    xyz_w: np.ndarray,
    fr: DepthFrame,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (u, v, z_cam) for world points in fr's camera."""
    w2c = np.linalg.inv(fr.c2w)
    hom = np.hstack([xyz_w, np.ones((xyz_w.shape[0], 1), dtype=np.float64)])
    cam = (w2c @ hom.T).T[:, :3]
    z = cam[:, 2]
    fx, fy = fr.K[0, 0], fr.K[1, 1]
    cx, cy = fr.K[0, 2], fr.K[1, 2]
    u = fx * (cam[:, 0] / np.maximum(z, 1e-8)) + cx
    v = fy * (cam[:, 1] / np.maximum(z, 1e-8)) + cy
    return u, v, z


def _sample_depth(depth: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    h, w = depth.shape
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    out = np.full(u.shape, np.nan, dtype=np.float32)
    ok = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    out[ok] = depth[vi[ok], ui[ok]]
    return out


def _sample_mask(mask: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Sample bool mask; False outside image or where mask is False."""
    h, w = mask.shape
    ui = np.rint(u).astype(np.int32)
    vi = np.rint(v).astype(np.int32)
    out = np.zeros(u.shape, dtype=bool)
    ok = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    out[ok] = mask[vi[ok], ui[ok]]
    return out


def classify_cross_view(
    z_pred: np.ndarray,
    z_obs: np.ndarray,
    in_image: np.ndarray,
    fpp_valid: np.ndarray,
    *,
    max_dz_m: float,
    min_z_pred_m: float = 0.05,
) -> np.ndarray:
    """Classify each projection into AGREE / NOT_OBSERVED / OCCLUDED / CONTRADICT.

    Missing FPP measurement (outside image, invalid mask, NaN depth, behind camera)
    is NOT_OBSERVED — never treated as disagreement.
    """
    labels = np.full(z_pred.shape, LABEL_NOT_OBSERVED, dtype=np.int8)
    tau = float(max_dz_m)

    observed = (
        in_image
        & fpp_valid
        & np.isfinite(z_obs)
        & (z_pred > float(min_z_pred_m))
        & (z_obs > float(min_z_pred_m))
    )
    if not np.any(observed):
        return labels

    dz = z_obs.astype(np.float64) - z_pred.astype(np.float64)
    agree = observed & (np.abs(dz) <= tau)
    # Candidate behind the surface seen by the other camera → occluded / neutral.
    occluded = observed & (~agree) & (z_pred > (z_obs.astype(np.float64) + tau))
    # Candidate in front of the other camera's surface → contradiction.
    contradict = observed & (~agree) & (z_pred < (z_obs.astype(np.float64) - tau))

    labels[agree] = LABEL_AGREE
    labels[occluded] = LABEL_OCCLUDED
    labels[contradict] = LABEL_CONTRADICT
    return labels


def backproject(fr: DepthFrame, *, stride: int = 2, conf_min: float = 0.1) -> np.ndarray:
    ys, xs = np.where(np.isfinite(fr.depth_m))
    if fr.conf is not None:
        keep = fr.conf[ys, xs] >= conf_min
        ys, xs = ys[keep], xs[keep]
    if stride > 1:
        ys, xs = ys[::stride], xs[::stride]
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    z = fr.depth_m[ys, xs].astype(np.float64)
    fx, fy = fr.K[0, 0], fr.K[1, 1]
    cx, cy = fr.K[0, 2], fr.K[1, 2]
    x = (xs.astype(np.float64) - cx) * z / fx
    y = (ys.astype(np.float64) - cy) * z / fy
    cam = np.stack([x, y, z], axis=1)
    return (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]


def fusion_score(
    c_fpp: np.ndarray,
    n_agree: np.ndarray,
    n_contradict: np.ndarray,
    *,
    lambda_agree: float = 0.15,
    lambda_contradict: float = 0.40,
) -> np.ndarray:
    """Score = C_FPP + λ_a·N_agree − λ_c·N_contradict (NOT_OBSERVED/OCCLUDED ignored)."""
    return (
        c_fpp.astype(np.float32)
        + float(lambda_agree) * n_agree.astype(np.float32)
        - float(lambda_contradict) * n_contradict.astype(np.float32)
    )


def consistency_reject(
    frames: list[DepthFrame],
    *,
    max_dz_m: float = 0.006,
    min_views: int = 2,
    sample_stride: int = 1,
    keep_mode: str = "hard",
    lambda_agree: float = 0.15,
    lambda_contradict: float = 0.40,
    keep_threshold: float = 0.12,
) -> dict:
    """Label cross-view visibility, then keep/kill pixels.

    ``keep_mode``:
      hard — legacy: keep if N_agree ≥ min_views
      soft — keep if Score = C_FPP + λ_a·N_agree − λ_c·N_contradict ≥ threshold
             (NOT_OBSERVED / OCCLUDED never reduce the score)

    Labels stored on each frame under ``meta['visibility']``; soft scores under
    ``meta['fusion_score']``.
    """
    mode = str(keep_mode or "hard").strip().lower()
    if mode not in ("hard", "soft"):
        raise ValueError(f"keep_mode must be 'hard' or 'soft', got {keep_mode!r}")

    n = len(frames)
    stats: dict = {
        "checked": 0,
        "kept": 0,
        "killed": 0,
        "keep_mode": mode,
        "min_views": int(min_views),
        "max_dz_m": float(max_dz_m),
        "lambda_agree": float(lambda_agree),
        "lambda_contradict": float(lambda_contradict),
        "keep_threshold": float(keep_threshold),
        "labels": {
            "AGREE": 0,
            "NOT_OBSERVED": 0,
            "OCCLUDED": 0,
            "CONTRADICT": 0,
        },
        "score": {
            "mean_kept": 0.0,
            "mean_killed": 0.0,
        },
        "per_frame": [],
    }
    if n < 2:
        stats["keep_ratio"] = 1.0
        stats["kill_ratio"] = 0.0
        return stats

    score_kept_sum = 0.0
    score_killed_sum = 0.0
    score_kept_n = 0
    score_killed_n = 0

    for i, fr in enumerate(frames):
        ys, xs = np.where(np.isfinite(fr.depth_m))
        if sample_stride > 1:
            ys, xs = ys[::sample_stride], xs[::sample_stride]
        if xs.size == 0:
            continue

        z = fr.depth_m[ys, xs].astype(np.float64)
        fx, fy = fr.K[0, 0], fr.K[1, 1]
        cx, cy = fr.K[0, 2], fr.K[1, 2]
        x = (xs.astype(np.float64) - cx) * z / fx
        y = (ys.astype(np.float64) - cy) * z / fy
        cam = np.stack([x, y, z], axis=1)
        world = (cam @ fr.c2w[:3, :3].T) + fr.c2w[:3, 3]

        n_pix = int(xs.size)
        n_agree = np.zeros(n_pix, dtype=np.int16)
        n_not_obs = np.zeros(n_pix, dtype=np.int16)
        n_occluded = np.zeros(n_pix, dtype=np.int16)
        n_contradict = np.zeros(n_pix, dtype=np.int16)

        oh, ow = fr.depth_m.shape
        for j, other in enumerate(frames):
            if j == i:
                continue
            u, v, z_pred = _project_world_to_depth(world, other)
            h, w = other.depth_m.shape
            in_image = (u >= 0.0) & (u < float(w)) & (v >= 0.0) & (v < float(h)) & (
                z_pred > 0.05
            )
            z_obs = _sample_depth(other.depth_m, u, v)
            fpp_valid = _sample_mask(other.mask, u, v)
            labels = classify_cross_view(
                z_pred, z_obs, in_image, fpp_valid, max_dz_m=float(max_dz_m)
            )
            n_agree += (labels == LABEL_AGREE).astype(np.int16)
            n_not_obs += (labels == LABEL_NOT_OBSERVED).astype(np.int16)
            n_occluded += (labels == LABEL_OCCLUDED).astype(np.int16)
            n_contradict += (labels == LABEL_CONTRADICT).astype(np.int16)

        agree_map = np.zeros((oh, ow), dtype=np.int16)
        not_obs_map = np.zeros((oh, ow), dtype=np.int16)
        occ_map = np.zeros((oh, ow), dtype=np.int16)
        contra_map = np.zeros((oh, ow), dtype=np.int16)
        agree_map[ys, xs] = n_agree
        not_obs_map[ys, xs] = n_not_obs
        occ_map[ys, xs] = n_occluded
        contra_map[ys, xs] = n_contradict
        fr.meta["visibility"] = {
            "n_agree": agree_map,
            "n_not_observed": not_obs_map,
            "n_occluded": occ_map,
            "n_contradict": contra_map,
        }

        if fr.conf is not None:
            c_fpp = fr.conf[ys, xs].astype(np.float32)
        else:
            c_fpp = np.full(n_pix, 0.5, dtype=np.float32)

        scores = fusion_score(
            c_fpp,
            n_agree,
            n_contradict,
            lambda_agree=float(lambda_agree),
            lambda_contradict=float(lambda_contradict),
        )
        score_map = np.zeros((oh, ow), dtype=np.float32)
        score_map[ys, xs] = scores
        fr.meta["fusion_score"] = score_map

        if mode == "soft":
            keep = scores >= float(keep_threshold)
        else:
            keep = n_agree >= int(min_views)

        n_checked = n_pix
        n_kept = int(keep.sum())
        n_killed = int((~keep).sum())
        stats["checked"] += n_checked
        stats["kept"] += n_kept
        stats["killed"] += n_killed
        stats["labels"]["AGREE"] += int(n_agree.sum())
        stats["labels"]["NOT_OBSERVED"] += int(n_not_obs.sum())
        stats["labels"]["OCCLUDED"] += int(n_occluded.sum())
        stats["labels"]["CONTRADICT"] += int(n_contradict.sum())

        if n_kept:
            score_kept_sum += float(scores[keep].sum())
            score_kept_n += n_kept
        if n_killed:
            score_killed_sum += float(scores[~keep].sum())
            score_killed_n += n_killed

        stats["per_frame"].append(
            {
                "stem": fr.stem,
                "checked": n_checked,
                "kept": n_kept,
                "killed": n_killed,
                "AGREE": int(n_agree.sum()),
                "NOT_OBSERVED": int(n_not_obs.sum()),
                "OCCLUDED": int(n_occluded.sum()),
                "CONTRADICT": int(n_contradict.sum()),
                "mean_agree": float(n_agree.mean()) if n_pix else 0.0,
                "mean_contradict": float(n_contradict.mean()) if n_pix else 0.0,
                "mean_score": float(scores.mean()) if n_pix else 0.0,
                "mean_score_kept": float(scores[keep].mean()) if n_kept else 0.0,
            }
        )

        fr.depth_m = fr.depth_m.copy()
        fr.depth_m[ys[~keep], xs[~keep]] = np.nan
        if fr.conf is not None:
            fr.conf = fr.conf.copy()
            # Soft mode: raise conf toward fusion score for kept pixels (helps densify).
            if mode == "soft":
                fr.conf[ys[keep], xs[keep]] = np.clip(scores[keep], 0.05, 1.0)
            fr.conf[ys[~keep], xs[~keep]] = 0.0
        fr.mask = np.isfinite(fr.depth_m)

    checked = int(stats["checked"])
    stats["keep_ratio"] = float(stats["kept"]) / checked if checked > 0 else 0.0
    stats["kill_ratio"] = float(stats["killed"]) / checked if checked > 0 else 0.0
    stats["score"]["mean_kept"] = (
        score_kept_sum / score_kept_n if score_kept_n else 0.0
    )
    stats["score"]["mean_killed"] = (
        score_killed_sum / score_killed_n if score_killed_n else 0.0
    )
    return stats
