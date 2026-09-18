# Offline COLMAP SfM on BFS checkerboard stills vs robot/BA poses.
# camera_cal layer only. Writes under colmap/ — does not touch hyperfusion.cfg.
"""Run COLMAP on checkerboard TIFFs; compare camera poses to BA hand–eye.

Uses pycolmap. Aligns COLMAP world to base_link with a Sim(3) fit on camera
centers, then reports translation / rotation error vs T_base_camera =
base_T_flange @ flange_T_camera (BA).
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

try:
    import pycolmap
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pycolmap required: pip install pycolmap") from exc


def load_T_fc(path: Path) -> np.ndarray:
    he = yaml.safe_load(path.read_text(encoding="utf-8"))
    return np.asarray(he["T"], dtype=np.float64)


def load_K_from_ba(path: Path) -> tuple[np.ndarray, np.ndarray, int, int]:
    intr = yaml.safe_load(path.read_text(encoding="utf-8"))
    K = np.asarray(intr["K"], dtype=np.float64)
    D = np.asarray(intr["D"], dtype=np.float64).reshape(-1)[:5]
    w = int(intr.get("width") or 4096)
    h = int(intr.get("height") or 3000)
    return K, D, w, h


def load_flange(meta: dict) -> np.ndarray | None:
    flange = meta.get("base_T_flange")
    if not isinstance(flange, dict):
        return None
    T = flange.get("T")
    if isinstance(T, list) and len(T) == 4:
        return np.asarray(T, dtype=np.float64)
    return None


def invert_T(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rotation_angle_deg(R: np.ndarray) -> float:
    c = np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.degrees(math.acos(c)))


def prepare_images(src: Path, dst: Path) -> list[str]:
    """Convert TIFFs to JPG (COLMAP-friendly); return stem list in sorted order."""
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True, exist_ok=True)
    stems: list[str] = []
    for tif in sorted(src.glob("*.tif")) + sorted(src.glob("*.tiff")):
        img = cv2.imread(str(tif), cv2.IMREAD_COLOR)
        if img is None:
            print(f"SKIP unreadable {tif.name}", flush=True)
            continue
        out = dst / f"{tif.stem}.jpg"
        # Downscale a bit for speed/memory; poses still comparable after Sim(3).
        h, w = img.shape[:2]
        scale = 1.0
        if max(h, w) > 2048:
            scale = 2048.0 / max(h, w)
            img = cv2.resize(
                img,
                (int(round(w * scale)), int(round(h * scale))),
                interpolation=cv2.INTER_AREA,
            )
        cv2.imwrite(str(out), img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        stems.append(tif.stem)
        print(f"  prepared {out.name} scale={scale:.3f}", flush=True)
    return stems


def manual_camera_poses(
    images_dir: Path, T_fc: np.ndarray, stems: list[str]
) -> dict[str, np.ndarray]:
    """stem -> T_base_camera (camera pose in base_link)."""
    out: dict[str, np.ndarray] = {}
    for stem in stems:
        jpath = images_dir / f"{stem}.json"
        if not jpath.is_file():
            continue
        meta = json.loads(jpath.read_text(encoding="utf-8-sig"))
        T_bf = load_flange(meta)
        if T_bf is None:
            continue
        out[stem] = T_bf @ T_fc
    return out


def colmap_image_T_world(rec: Any, image_id: int) -> np.ndarray:
    """COLMAP stores world-to-camera; return T_world_camera (camera center frame)."""
    im = rec.images[image_id]
    # pycolmap 4.x: cam_from_world is a Rigid3d (world -> camera)
    cfw = im.cam_from_world()
    R = np.asarray(cfw.rotation.matrix(), dtype=np.float64)
    t = np.asarray(cfw.translation, dtype=np.float64).reshape(3)
    T_cw = np.eye(4, dtype=np.float64)
    T_cw[:3, :3] = R
    T_cw[:3, 3] = t
    return invert_T(T_cw)  # T_world_camera


def umeyama_sim3(
    src: np.ndarray, dst: np.ndarray
) -> tuple[float, np.ndarray, np.ndarray]:
    """Sim(3): dst ≈ s * R @ src + t. Points as (N,3)."""
    assert src.shape == dst.shape and src.shape[1] == 3
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    X = src - mu_s
    Y = dst - mu_d
    var_s = np.sum(X * X) / n
    cov = (Y.T @ X) / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    scale = float(np.trace(np.diag(D) @ S) / max(var_s, 1e-18))
    t = mu_d - scale * R @ mu_s
    return scale, R, t


def apply_sim3_to_pose(T_wc: np.ndarray, s: float, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Transform COLMAP T_world_camera into base: c_base = s R c_colmap + t, R_base = R R_colmap."""
    Rc = T_wc[:3, :3]
    cc = T_wc[:3, 3]
    Tout = np.eye(4, dtype=np.float64)
    Tout[:3, :3] = R @ Rc
    Tout[:3, 3] = s * (R @ cc) + t
    return Tout


def run_colmap(
    image_dir: Path,
    database: Path,
    sparse_dir: Path,
    *,
    K: np.ndarray | None,
    width: int,
    height: int,
    use_known_intrinsics: bool,
) -> Any:
    if database.is_file():
        database.unlink()
    if sparse_dir.exists():
        shutil.rmtree(sparse_dir)
    sparse_dir.mkdir(parents=True, exist_ok=True)

    # Feature extraction
    if use_known_intrinsics and K is not None:
        # Scale K to prepared image size if we downscaled: detect from first jpg
        jpgs = sorted(image_dir.glob("*.jpg"))
        im0 = cv2.imread(str(jpgs[0]), cv2.IMREAD_COLOR)
        assert im0 is not None
        h, w = im0.shape[:2]
        sx, sy = w / float(width), h / float(height)
        fx, fy, cx, cy = K[0, 0] * sx, K[1, 1] * sy, K[0, 2] * sx, K[1, 2] * sy
        # PINHOLE: fx, fy, cx, cy
        cam_mode = pycolmap.CameraMode.SINGLE
        reader = pycolmap.ImageReaderOptions()
        reader.camera_model = "PINHOLE"
        reader.camera_params = f"{fx},{fy},{cx},{cy}"
        print(
            f"COLMAP known PINHOLE fx/fy={fx:.2f}/{fy:.2f} cx/cy={cx:.2f}/{cy:.2f} "
            f"size={w}x{h}",
            flush=True,
        )
        pycolmap.extract_features(
            database_path=database,
            image_path=image_dir,
            camera_mode=cam_mode,
            reader_options=reader,
            # keep intrinsics fixed during BA later via mapper options if available
        )
    else:
        print("COLMAP free SIMPLE_RADIAL intrinsics", flush=True)
        pycolmap.extract_features(
            database_path=database,
            image_path=image_dir,
            camera_mode=pycolmap.CameraMode.SINGLE,
        )

    pycolmap.match_exhaustive(database_path=database)

    maps = pycolmap.incremental_mapping(
        database_path=database,
        image_path=image_dir,
        output_path=sparse_dir,
    )
    if not maps:
        raise RuntimeError("COLMAP produced no reconstruction")
    # Pick largest model
    best_id = max(maps.keys(), key=lambda i: maps[i].num_reg_images())
    rec = maps[best_id]
    print(
        f"COLMAP model {best_id}: images={rec.num_reg_images()} points={rec.num_points3D()}",
        flush=True,
    )
    # Write text export for inspection
    text_dir = sparse_dir / "0_text"
    text_dir.mkdir(parents=True, exist_ok=True)
    rec.write_text(text_dir)
    rec.write(sparse_dir / "0")
    return rec


def compare(
    rec: Any,
    manual: dict[str, np.ndarray],
    out_dir: Path,
) -> dict[str, Any]:
    # Build COLMAP poses keyed by stem
    colmap_poses: dict[str, np.ndarray] = {}
    for image_id, im in rec.images.items():
        stem = Path(im.name).stem
        colmap_poses[stem] = colmap_image_T_world(rec, image_id)

    common = sorted(set(manual) & set(colmap_poses))
    if len(common) < 3:
        raise RuntimeError(
            f"Need >=3 shared views for Sim(3); common={len(common)} "
            f"manual={len(manual)} colmap={len(colmap_poses)}"
        )

    src = np.stack([colmap_poses[s][:3, 3] for s in common], axis=0)
    dst = np.stack([manual[s][:3, 3] for s in common], axis=0)
    scale, R_align, t_align = umeyama_sim3(src, dst)

    per: list[dict[str, Any]] = []
    terr, rerr = [], []
    for stem in common:
        T_col_base = apply_sim3_to_pose(colmap_poses[stem], scale, R_align, t_align)
        T_man = manual[stem]
        # Compare camera centers and relative rotation
        dc = np.linalg.norm(T_col_base[:3, 3] - T_man[:3, 3]) * 1000.0
        R_delta = T_man[:3, :3].T @ T_col_base[:3, :3]
        dang = rotation_angle_deg(R_delta)
        terr.append(dc)
        rerr.append(dang)
        per.append(
            {
                "stem": stem,
                "center_err_mm": float(dc),
                "rot_err_deg": float(dang),
                "manual_center_m": T_man[:3, 3].tolist(),
                "colmap_aligned_center_m": T_col_base[:3, 3].tolist(),
            }
        )

    report = {
        "ok": True,
        "n_common": len(common),
        "n_manual": len(manual),
        "n_colmap": len(colmap_poses),
        "sim3": {
            "scale": float(scale),
            "R": R_align.tolist(),
            "t_m": t_align.tolist(),
            "note": "aligns COLMAP camera centers -> base_link (manual BA poses)",
        },
        "center_err_mm": {
            "median": float(np.median(terr)),
            "mean": float(np.mean(terr)),
            "p95": float(np.percentile(terr, 95)),
            "max": float(np.max(terr)),
        },
        "rot_err_deg": {
            "median": float(np.median(rerr)),
            "mean": float(np.mean(rerr)),
            "p95": float(np.percentile(rerr, 95)),
            "max": float(np.max(rerr)),
        },
        "per_view": per,
        "missing_in_colmap": sorted(set(manual) - set(colmap_poses)),
        "extra_in_colmap": sorted(set(colmap_poses) - set(manual)),
    }
    (out_dir / "compare_to_ba.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )

    # Also dump aligned COLMAP poses
    aligned = {
        stem: apply_sim3_to_pose(colmap_poses[stem], scale, R_align, t_align).tolist()
        for stem in common
    }
    (out_dir / "colmap_T_base_camera_aligned.json").write_text(
        json.dumps(aligned, indent=2) + "\n", encoding="utf-8"
    )
    manual_dump = {stem: manual[stem].tolist() for stem in common}
    (out_dir / "manual_ba_T_base_camera.json").write_text(
        json.dumps(manual_dump, indent=2) + "\n", encoding="utf-8"
    )
    return report


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--images",
        type=Path,
        required=True,
        help="bfs_cal/checkerboard (TIFF + JSON)",
    )
    p.add_argument(
        "--ba",
        type=Path,
        required=True,
        help="ba/results with camera_intrinsics.yaml + flange_T_camera.yaml",
    )
    p.add_argument("--out", type=Path, required=True, help="camera_cal/colmap")
    p.add_argument(
        "--free-intrinsics",
        action="store_true",
        help="Do not lock COLMAP to BA K (default: use BA PINHOLE)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    img_prep = out / "images_jpg"
    db = out / "database.db"
    sparse = out / "sparse"

    print("=== prepare images ===", flush=True)
    stems = prepare_images(Path(args.images), img_prep)
    if len(stems) < 5:
        raise SystemExit(f"Need >=5 images, got {len(stems)}")

    K, _D, w, h = load_K_from_ba(Path(args.ba) / "camera_intrinsics.yaml")
    T_fc = load_T_fc(Path(args.ba) / "flange_T_camera.yaml")
    manual = manual_camera_poses(Path(args.images), T_fc, stems)
    print(f"manual poses with flange JSON: {len(manual)}/{len(stems)}", flush=True)

    print("=== COLMAP SfM ===", flush=True)
    rec = run_colmap(
        img_prep,
        db,
        sparse,
        K=K,
        width=w,
        height=h,
        use_known_intrinsics=not args.free_intrinsics,
    )

    print("=== compare to BA poses ===", flush=True)
    report = compare(rec, manual, out)
    print(
        json.dumps(
            {
                "n_common": report["n_common"],
                "center_err_mm": report["center_err_mm"],
                "rot_err_deg": report["rot_err_deg"],
                "sim3_scale": report["sim3"]["scale"],
                "missing_in_colmap": report["missing_in_colmap"],
            },
            indent=2,
        ),
        flush=True,
    )
    print(f"Wrote {out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
