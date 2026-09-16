#!/usr/bin/env python3
# Offline multi-pose FPP depth fusion via Open3D TSDF (sidecar). Not wired into the GUI.
"""Fuse decoded FPP camera-Z maps (+ poses) into a TSDF mesh / point cloud.

Expects:
  --burst   Capture folder with #####.json pose sidecars
  --fpp-out Decode tree (…/fpp_out/<burst_name>/#####/fpp_depth.npy)

Depth is camera Z in mm. Poses must be **camera** optical c2w (OpenCV).

When ``scan_tcp = dlp``, older bursts labeled ``hyperfusion_tcp`` may still store the
**projector** tip. New captures write ``frame=camera_optical`` /
``pose_source=live_tf_base_tool0_x_camera_tcp`` (tool0 ⊗ ``tool_tcp_*``).

Fusion recovers camera c2w as ``base_T_flange @ tool0_T_camera`` when the JSON tip
matches ``dlp_tcp_*`` (auto), or trusts JSON when it already matches the camera TCP.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "open3d is required. Install with: pip install open3d"
    ) from exc


def _rpy_zyx_deg(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """UR-style fixed RPY → rotation (Rz @ Ry @ Rx), degrees in."""
    r, p, y = np.radians([roll, pitch, yaw])
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    cz, sz = np.cos(y), np.sin(y)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def _tool0_T_from_xyzrpy_mm(
    x_mm: float, y_mm: float, z_mm: float, roll: float, pitch: float, yaw: float
) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _rpy_zyx_deg(roll, pitch, yaw)
    T[:3, 3] = np.array([x_mm, y_mm, z_mm], dtype=np.float64) * 1.0e-3
    return T


def _parse_cfg_floats(cfg_path: Path, keys: list[str]) -> dict[str, float]:
    text = cfg_path.read_text(encoding="utf-8", errors="replace")
    out: dict[str, float] = {}
    for key in keys:
        m = re.search(
            rf"^\s*{re.escape(key)}\s*=\s*([-+0-9.eE]+)\s*$", text, flags=re.M
        )
        if m:
            out[key] = float(m.group(1))
    return out


def load_camera_tcp_from_cfg(cfg_path: Path) -> np.ndarray:
    keys = [
        "tool_tcp_x_mm",
        "tool_tcp_y_mm",
        "tool_tcp_z_mm",
        "tool_tcp_roll_deg",
        "tool_tcp_pitch_deg",
        "tool_tcp_yaw_deg",
    ]
    v = _parse_cfg_floats(cfg_path, keys)
    missing = [k for k in keys if k not in v]
    if missing:
        raise FileNotFoundError(f"Missing {missing} in {cfg_path}")
    return _tool0_T_from_xyzrpy_mm(
        v["tool_tcp_x_mm"],
        v["tool_tcp_y_mm"],
        v["tool_tcp_z_mm"],
        v["tool_tcp_roll_deg"],
        v["tool_tcp_pitch_deg"],
        v["tool_tcp_yaw_deg"],
    )


def load_dlp_tcp_from_cfg(cfg_path: Path) -> np.ndarray | None:
    keys = [
        "dlp_tcp_x_mm",
        "dlp_tcp_y_mm",
        "dlp_tcp_z_mm",
        "dlp_tcp_roll_deg",
        "dlp_tcp_pitch_deg",
        "dlp_tcp_yaw_deg",
    ]
    v = _parse_cfg_floats(cfg_path, keys)
    if len(v) != len(keys):
        return None
    return _tool0_T_from_xyzrpy_mm(
        v["dlp_tcp_x_mm"],
        v["dlp_tcp_y_mm"],
        v["dlp_tcp_z_mm"],
        v["dlp_tcp_roll_deg"],
        v["dlp_tcp_pitch_deg"],
        v["dlp_tcp_yaw_deg"],
    )


def _json_c2w(meta: dict) -> np.ndarray:
    ext = meta.get("extrinsics") or {}
    if ext.get("convention") == "camera_to_base_link_opencv" and "R" in ext and "t" in ext:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = np.asarray(ext["R"], dtype=np.float64)
        T[:3, 3] = np.asarray(ext["t"], dtype=np.float64).reshape(3)
        return T
    T_gl = np.asarray(meta["transform_matrix"], dtype=np.float64)
    flip = np.diag([1.0, -1.0, -1.0, 1.0])
    return T_gl @ flip


def _flange_c2w(meta: dict) -> np.ndarray | None:
    bt = meta.get("base_T_flange")
    if not isinstance(bt, dict) or "T" not in bt:
        return None
    T = np.asarray(bt["T"], dtype=np.float64)
    if T.shape != (4, 4):
        return None
    return T


def _se3_close(A: np.ndarray, B: np.ndarray, t_tol_m: float = 1e-3, r_tol: float = 1e-3) -> bool:
    return float(np.linalg.norm(A[:3, 3] - B[:3, 3])) < t_tol_m and float(
        np.linalg.norm(A[:3, :3] - B[:3, :3])
    ) < r_tol


def resolve_camera_c2w(
    meta: dict,
    *,
    tool0_T_camera: np.ndarray | None,
    tool0_T_dlp: np.ndarray | None,
    pose_mode: str,
) -> tuple[np.ndarray, str]:
    """Return (c2w_camera_opencv, source_tag)."""
    T_json = _json_c2w(meta)
    T_flange = _flange_c2w(meta)

    if pose_mode == "json":
        return T_json, "json_extrinsics"

    if pose_mode == "flange_camera":
        if T_flange is None or tool0_T_camera is None:
            raise RuntimeError("flange_camera mode needs base_T_flange + camera TCP")
        return T_flange @ tool0_T_camera, "flange@tool_tcp"

    # auto
    if T_flange is not None and tool0_T_camera is not None:
        T_tool = np.linalg.inv(T_flange) @ T_json
        if tool0_T_dlp is not None and _se3_close(T_tool, tool0_T_dlp):
            return T_flange @ tool0_T_camera, "auto_dlp_json→flange@tool_tcp"
        if _se3_close(T_tool, tool0_T_camera):
            return T_json, "auto_json_is_camera"
        # Prefer flange@camera when flange is present (robust for scan_tcp=dlp).
        return T_flange @ tool0_T_camera, "auto_flange@tool_tcp"

    return T_json, "json_extrinsics_fallback"


def _intrinsics(meta: dict) -> o3d.camera.PinholeCameraIntrinsic:
    inn = meta["intrinsics"]
    return o3d.camera.PinholeCameraIntrinsic(
        width=int(inn["width"]),
        height=int(inn["height"]),
        fx=float(inn["fx"]),
        fy=float(inn["fy"]),
        cx=float(inn["cx"]),
        cy=float(inn["cy"]),
    )


def _load_color(burst: Path, stem: str, shape_hw: tuple[int, int]) -> np.ndarray:
    """Prefer white frame (stem+1); else gray from depth mask."""
    try:
        import tifffile

        white_idx = int(stem) + 1
        path = burst / f"{white_idx:05d}.tif"
        if path.is_file():
            img = tifffile.imread(path)
            if img.ndim == 2:
                img = np.stack([img, img, img], axis=-1)
            elif img.shape[-1] > 3:
                img = img[..., :3]
            if img.dtype != np.uint8:
                img = (
                    np.clip(img, 0, 255).astype(np.uint8)
                    if img.max() > 1.5
                    else np.clip(img * 255.0, 0, 255).astype(np.uint8)
                )
            if img.shape[0] != shape_hw[0] or img.shape[1] != shape_hw[1]:
                raise ValueError("size mismatch")
            return np.ascontiguousarray(img)
    except Exception:
        pass
    h, w = shape_hw
    return np.full((h, w, 3), 180, dtype=np.uint8)


def discover_poses(fpp_out: Path) -> list[Path]:
    poses = []
    for d in sorted(fpp_out.iterdir()):
        if d.is_dir() and (d / "fpp_depth.npy").is_file() and (d / "fpp_mask.npy").is_file():
            poses.append(d)
    return poses


def fuse(
    burst: Path,
    fpp_out: Path,
    out_dir: Path,
    *,
    voxel_m: float,
    sdf_trunc_m: float,
    stride: int,
    z_min_m: float,
    z_max_m: float,
    tool0_T_camera: np.ndarray | None,
    tool0_T_dlp: np.ndarray | None,
    pose_mode: str,
    min_modulation: float = 0.15,
) -> dict:
    pose_dirs = discover_poses(fpp_out)
    if not pose_dirs:
        raise FileNotFoundError(f"No fpp_depth.npy under {fpp_out}")

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=float(voxel_m),
        sdf_trunc=float(sdf_trunc_m),
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    used = []
    pose_sources: dict[str, int] = {}
    for pose_dir in pose_dirs:
        stem = pose_dir.name
        meta_path = burst / f"{stem}.json"
        if not meta_path.is_file():
            continue
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        depth_mm = np.load(pose_dir / "fpp_depth.npy").astype(np.float32)
        mask = np.load(pose_dir / "fpp_mask.npy").astype(bool)
        if depth_mm.shape != mask.shape:
            continue

        # Drop low-SNR pixels before TSDF (modulation saved at decode time).
        mod_path = pose_dir / "fpp_modulation.npy"
        if mod_path.is_file() and float(min_modulation) > 0.0:
            modulation = np.load(mod_path).astype(np.float32)
            if modulation.shape == mask.shape:
                mask = mask & (modulation >= float(min_modulation))

        depth_m = np.full(depth_mm.shape, np.nan, dtype=np.float32)
        valid = mask & np.isfinite(depth_mm)
        depth_m[valid] = depth_mm[valid] * 1.0e-3
        depth_m[valid & ((depth_m < z_min_m) | (depth_m > z_max_m))] = np.nan

        if stride > 1:
            depth_m = depth_m[::stride, ::stride].copy()
            inn = dict(meta["intrinsics"])
            inn["fx"] = float(inn["fx"]) / stride
            inn["fy"] = float(inn["fy"]) / stride
            inn["cx"] = float(inn["cx"]) / stride
            inn["cy"] = float(inn["cy"]) / stride
            inn["width"] = int(depth_m.shape[1])
            inn["height"] = int(depth_m.shape[0])
            meta = {**meta, "intrinsics": inn}
            color = _load_color(burst, stem, (depth_mm.shape[0], depth_mm.shape[1]))
            color = color[::stride, ::stride].copy()
        else:
            color = _load_color(burst, stem, depth_mm.shape)

        depth_o3d = np.where(np.isfinite(depth_m), depth_m, 0.0).astype(np.float32)
        if int(np.count_nonzero(depth_o3d)) < 1000:
            continue

        depth_img = o3d.geometry.Image(depth_o3d)
        color_img = o3d.geometry.Image(np.ascontiguousarray(color))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_img,
            depth_img,
            depth_scale=1.0,
            depth_trunc=float(z_max_m),
            convert_rgb_to_intensity=False,
        )
        c2w, src = resolve_camera_c2w(
            meta,
            tool0_T_camera=tool0_T_camera,
            tool0_T_dlp=tool0_T_dlp,
            pose_mode=pose_mode,
        )
        pose_sources[src] = pose_sources.get(src, 0) + 1
        w2c = np.linalg.inv(c2w)
        intrinsic = _intrinsics(meta)
        volume.integrate(rgbd, intrinsic, w2c)
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

    out_dir.mkdir(parents=True, exist_ok=True)
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    pcd = volume.extract_point_cloud()

    mesh_path = out_dir / "fpp_tsdf_mesh.ply"
    pcd_path = out_dir / "fpp_tsdf_cloud.ply"
    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    o3d.io.write_point_cloud(str(pcd_path), pcd)

    summary = {
        "ok": True,
        "n_poses": len(used),
        "poses": used,
        "pose_sources": pose_sources,
        "pose_mode": pose_mode,
        "voxel_m": voxel_m,
        "sdf_trunc_m": sdf_trunc_m,
        "stride": stride,
        "min_modulation": float(min_modulation),
        "mesh": str(mesh_path),
        "cloud": str(pcd_path),
        "mesh_vertices": int(np.asarray(mesh.vertices).shape[0]),
        "mesh_triangles": int(np.asarray(mesh.triangles).shape[0]),
        "cloud_points": int(np.asarray(pcd.points).shape[0]),
    }
    (out_dir / "fpp_tsdf_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--burst",
        type=Path,
        required=True,
        help="Capture folder (#####.tif + #####.json)",
    )
    parser.add_argument(
        "--fpp-out",
        type=Path,
        default=None,
        help="Decode folder with per-pose subdirs (default: <burst>/fpp_out/<burst.name>)",
    )
    parser.add_argument("--out", type=Path, default=None, help="Output dir (default: <fpp-out>/tsdf)")
    parser.add_argument("--voxel-mm", type=float, default=2.0, help="TSDF voxel size (mm)")
    parser.add_argument("--sdf-trunc-mm", type=float, default=12.0, help="SDF truncation (mm)")
    parser.add_argument("--stride", type=int, default=2, help="Subsample depth for speed (1=full)")
    parser.add_argument("--z-min-mm", type=float, default=150.0)
    parser.add_argument("--z-max-mm", type=float, default=550.0)
    parser.add_argument(
        "--cfg",
        type=Path,
        default=None,
        help="hyperfusion.cfg for tool_tcp_* / dlp_tcp_* (default: app/preset/hyperfusion.cfg)",
    )
    parser.add_argument(
        "--pose-mode",
        choices=("auto", "flange_camera", "json"),
        default="auto",
        help="auto: detect DLP-labeled JSON and recover camera via flange@tool_tcp",
    )
    parser.add_argument(
        "--min-modulation",
        type=float,
        default=0.15,
        help="Drop depth where fpp_modulation.npy is below this (0=disable). Default 0.15.",
    )
    args = parser.parse_args()

    burst = Path(args.burst)
    fpp_out = (
        Path(args.fpp_out)
        if args.fpp_out is not None
        else burst / "fpp_out" / burst.name
    )
    out_dir = Path(args.out) if args.out is not None else fpp_out / "tsdf"

    repo_cfg = (
        Path(__file__).resolve().parents[2] / "preset" / "hyperfusion.cfg"
    )
    cfg_path = Path(args.cfg) if args.cfg is not None else repo_cfg

    tool0_T_camera = None
    tool0_T_dlp = None
    if args.pose_mode != "json":
        if not cfg_path.is_file():
            raise SystemExit(
                f"Need {cfg_path} for camera TCP (or pass --pose-mode json)"
            )
        tool0_T_camera = load_camera_tcp_from_cfg(cfg_path)
        tool0_T_dlp = load_dlp_tcp_from_cfg(cfg_path)

    summary = fuse(
        burst,
        fpp_out,
        out_dir,
        voxel_m=float(args.voxel_mm) * 1.0e-3,
        sdf_trunc_m=float(args.sdf_trunc_mm) * 1.0e-3,
        stride=max(1, int(args.stride)),
        z_min_m=float(args.z_min_mm) * 1.0e-3,
        z_max_m=float(args.z_max_mm) * 1.0e-3,
        tool0_T_camera=tool0_T_camera,
        tool0_T_dlp=tool0_T_dlp,
        pose_mode=str(args.pose_mode),
        min_modulation=float(args.min_modulation),
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
