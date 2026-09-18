# Open3D TSDF integration of FPP camera-Z maps (sidecar / depth_fusion).
# Depth must match the undistorted pinhole used at decode (newK, D cleared).
"""Fuse per-pose fpp_depth.npy into a triangle mesh + point cloud."""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from .poses import load_pose_meta, resolve_camera_c2w

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit("open3d is required: pip install open3d") from exc


def discover_pose_dirs(decode_root: Path) -> list[Path]:
    poses: list[Path] = []
    for d in sorted(Path(decode_root).iterdir()):
        if (
            d.is_dir()
            and (d / "fpp_depth.npy").is_file()
            and (d / "fpp_mask.npy").is_file()
        ):
            poses.append(d)
    return poses


def pinhole_for_depth(meta: dict, size_wh: tuple[int, int]) -> o3d.camera.PinholeCameraIntrinsic:
    """Intrinsics for decoded depth. If D is present, use getOptimalNewCameraMatrix (alpha=0)."""
    inn = meta.get("intrinsics") if isinstance(meta.get("intrinsics"), dict) else meta
    fx = float(inn["fx"])
    fy = float(inn["fy"])
    cx = float(inn["cx"])
    cy = float(inn["cy"])
    w, h = int(size_wh[0]), int(size_wh[1])
    dist = inn.get("distortion") or []
    d = np.asarray(dist, dtype=np.float64).reshape(-1)
    if d.size >= 4 and np.any(np.abs(d) > 1.0e-12):
        k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        new_k, _roi = cv2.getOptimalNewCameraMatrix(k, d, (w, h), 0.0)
        fx, fy = float(new_k[0, 0]), float(new_k[1, 1])
        cx, cy = float(new_k[0, 2]), float(new_k[1, 2])
    return o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)


def _load_color_undistorted(
    burst: Path,
    stem: str,
    shape_hw: tuple[int, int],
    meta: dict,
) -> np.ndarray:
    """White frame (stem+1), remapped to the same pinhole as depth when D is set."""
    h, w = shape_hw
    try:
        import tifffile

        white_idx = int(stem) + 1
        path = burst / f"{white_idx:05d}.tif"
        if not path.is_file():
            raise FileNotFoundError(path)
        img = tifffile.imread(str(path))
        if img.ndim == 2:
            img = np.stack([img, img, img], axis=-1)
        elif img.shape[-1] > 3:
            img = img[..., :3]
        if img.dtype != np.uint8:
            img = (
                np.clip(img, 0, 255).astype(np.uint8)
                if float(img.max()) > 1.5
                else np.clip(img * 255.0, 0, 255).astype(np.uint8)
            )
        inn = meta.get("intrinsics") if isinstance(meta.get("intrinsics"), dict) else meta
        dist = np.asarray(inn.get("distortion") or [], dtype=np.float64).reshape(-1)
        if dist.size >= 4 and np.any(np.abs(dist) > 1.0e-12):
            k = np.array(
                [
                    [float(inn["fx"]), 0.0, float(inn["cx"])],
                    [0.0, float(inn["fy"]), float(inn["cy"])],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            new_k, _ = cv2.getOptimalNewCameraMatrix(k, dist, (w, h), 0.0)
            map_x, map_y = cv2.initUndistortRectifyMap(k, dist, None, new_k, (w, h), cv2.CV_32FC1)
            img = cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR)
        if img.shape[0] != h or img.shape[1] != w:
            raise ValueError("size mismatch")
        return np.ascontiguousarray(img)
    except Exception:
        return np.full((h, w, 3), 180, dtype=np.uint8)


def fuse_tsdf(
    burst: Path,
    decode_root: Path,
    out_dir: Path,
    *,
    tool0_T_camera: np.ndarray | None,
    pose_mode: str = "auto",
    voxel_m: float = 0.002,
    sdf_trunc_m: float = 0.012,
    stride: int = 2,
    z_min_m: float = 0.35,
    z_max_m: float = 0.60,
    min_modulation: float = 0.15,
) -> dict:
    pose_dirs = discover_pose_dirs(decode_root)
    if not pose_dirs:
        raise FileNotFoundError(f"No fpp_depth.npy under {decode_root}")

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=float(voxel_m),
        sdf_trunc=float(sdf_trunc_m),
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    used: list[dict] = []
    pose_sources: dict[str, int] = {}
    for pose_dir in pose_dirs:
        stem = pose_dir.name
        try:
            meta = load_pose_meta(burst, stem)
        except FileNotFoundError:
            continue
        depth_mm = np.load(pose_dir / "fpp_depth.npy").astype(np.float32)
        mask = np.load(pose_dir / "fpp_mask.npy").astype(bool)
        if depth_mm.shape != mask.shape:
            continue

        mod_path = pose_dir / "fpp_modulation.npy"
        if mod_path.is_file() and float(min_modulation) > 0.0:
            modulation = np.load(mod_path).astype(np.float32)
            if modulation.shape == mask.shape:
                mask = mask & (modulation >= float(min_modulation))

        depth_m = np.full(depth_mm.shape, np.nan, dtype=np.float32)
        valid = mask & np.isfinite(depth_mm)
        depth_m[valid] = depth_mm[valid] * 1.0e-3
        depth_m[valid & ((depth_m < z_min_m) | (depth_m > z_max_m))] = np.nan

        color = _load_color_undistorted(burst, stem, depth_mm.shape, meta)
        meta_use = meta
        if stride > 1:
            depth_m = depth_m[::stride, ::stride].copy()
            color = color[::stride, ::stride].copy()
            inn = dict(meta["intrinsics"])
            for key in ("fx", "fy", "cx", "cy"):
                inn[key] = float(inn[key]) / float(stride)
            # Distortion already absorbed into newK path; clear so we don't double-apply.
            inn["distortion"] = []
            inn["width"] = int(depth_m.shape[1])
            inn["height"] = int(depth_m.shape[0])
            # Scale newK from full-res undistort then divide — recompute from full then scale:
            full_intr = pinhole_for_depth(meta, (depth_mm.shape[1], depth_mm.shape[0]))
            inn["fx"] = full_intr.intrinsic_matrix[0, 0] / float(stride)
            inn["fy"] = full_intr.intrinsic_matrix[1, 1] / float(stride)
            inn["cx"] = full_intr.intrinsic_matrix[0, 2] / float(stride)
            inn["cy"] = full_intr.intrinsic_matrix[1, 2] / float(stride)
            meta_use = {**meta, "intrinsics": inn}

        depth_o3d = np.where(np.isfinite(depth_m), depth_m, 0.0).astype(np.float32)
        if int(np.count_nonzero(depth_o3d)) < 1000:
            continue

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(color)),
            o3d.geometry.Image(depth_o3d),
            depth_scale=1.0,
            depth_trunc=float(z_max_m),
            convert_rgb_to_intensity=False,
        )
        c2w, src = resolve_camera_c2w(
            meta_use,
            tool0_T_camera=tool0_T_camera,
            pose_mode=pose_mode,
        )
        pose_sources[src] = pose_sources.get(src, 0) + 1
        if stride > 1:
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                int(depth_m.shape[1]),
                int(depth_m.shape[0]),
                float(meta_use["intrinsics"]["fx"]),
                float(meta_use["intrinsics"]["fy"]),
                float(meta_use["intrinsics"]["cx"]),
                float(meta_use["intrinsics"]["cy"]),
            )
        else:
            intrinsic = pinhole_for_depth(meta_use, (depth_m.shape[1], depth_m.shape[0]))
        volume.integrate(rgbd, intrinsic, np.linalg.inv(c2w))
        used.append(
            {
                "pose": stem,
                "pose_source": src,
                "valid_px": int(np.count_nonzero(depth_o3d)),
                "depth_med_m": float(np.nanmedian(depth_m)),
                "cam_t_m": c2w[:3, 3].tolist(),
            }
        )

    if not used:
        raise RuntimeError("No poses integrated (missing JSON or empty depth).")

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    pcd = volume.extract_point_cloud()

    mesh_path = out_dir / "tsdf_mesh.ply"
    pcd_path = out_dir / "tsdf_cloud.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    o3d.io.write_point_cloud(str(pcd_path), pcd)

    summary = {
        "ok": True,
        "n_poses": len(used),
        "poses": used,
        "pose_sources": pose_sources,
        "pose_mode": pose_mode,
        "voxel_m": float(voxel_m),
        "sdf_trunc_m": float(sdf_trunc_m),
        "stride": int(stride),
        "z_min_m": float(z_min_m),
        "z_max_m": float(z_max_m),
        "min_modulation": float(min_modulation),
        "mesh": str(mesh_path),
        "cloud": str(pcd_path),
        "mesh_vertices": int(np.asarray(mesh.vertices).shape[0]),
        "mesh_triangles": int(np.asarray(mesh.triangles).shape[0]),
        "cloud_points": int(np.asarray(pcd.points).shape[0]),
    }
    (out_dir / "tsdf_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary
