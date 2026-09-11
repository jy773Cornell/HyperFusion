#!/usr/bin/env python3
"""Batch Semi ring Plan: θ×radius sweep via MoveIt sidecar; one (R,θ) per saved JSON.

Each file is apex (θ=0 home pose with Z=R) plus that latitude's base-sweep-OK pins.
Uses hyperfusion.cfg [multiview] (workspace, home, tip tolerance, φ candidates).
Saves loadable schema-4 plans into ur3e_semi_scan_routes.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

CACHE_SCHEMA = 4


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def load_hub_joints_deg(path: Path) -> tuple[list[float], str]:
    """First base-sweep-OK pin in a saved plan → joints deg + label."""
    root = json.loads(path.read_text(encoding="utf-8"))
    for pt in root.get("points") or []:
        if not isinstance(pt, dict):
            continue
        if not pt.get("base_sweep_ok"):
            continue
        joints = pt.get("joints_rad") or []
        if len(joints) != 6:
            continue
        deg = [math.degrees(float(v)) for v in joints]
        grid = pt.get("grid") if isinstance(pt.get("grid"), dict) else {}
        label = (
            f"{path.name} φ={float(grid.get('phi_deg', 0.0)):g}° "
            f"θ={float(grid.get('theta_deg', 0.0)):g}°"
        )
        return deg, label
    raise RuntimeError(f"No base_sweep_ok pin with joints in {path}")


def parse_cfg(path: Path) -> dict[str, Any]:
    """Minimal INI-ish parser for hyperfusion.cfg [multiview] keys (legacy [3d scanning]/[ur3e] accepted)."""
    section = ""
    out: dict[str, Any] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].strip().lower()
            continue
        if section not in ("multiview", "3d scanning", "ur3e") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key == "home_joints_deg":
            out[key] = [float(p.strip()) for p in value.split(",") if p.strip()]
            continue
        low = value.lower()
        if low in ("true", "false"):
            out[key] = low == "true"
            continue
        try:
            if "." in value:
                out[key] = float(value)
            else:
                out[key] = int(value)
        except ValueError:
            out[key] = value
    return out


def http_json(method: str, url: str, body: dict | None = None, timeout: float = 30.0) -> dict:
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"} if body is not None else {},
        method=method,
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.load(resp)


def wait_connected(base: str, timeout_s: float = 180.0) -> None:
    health = http_json("GET", f"{base}/health", timeout=10.0)
    if health.get("robot_connected"):
        return
    try:
        http_json("POST", f"{base}/connect/start", body={}, timeout=30.0)
    except urllib.error.HTTPError as exc:
        # Already connecting (or a stale 500) — poll status instead of aborting.
        if exc.code not in (409, 500):
            raise
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        st = http_json("GET", f"{base}/connect/status", timeout=10.0)
        if st.get("connected"):
            return
        time.sleep(2.0)
    raise RuntimeError("Timed out waiting for mock/driver connect.")


def _normalize(x: float, y: float, z: float) -> tuple[float, float, float]:
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1.0e-12:
        return 0.0, 0.0, -1.0
    return x / n, y / n, z / n


def tool_z_to_rotation_vector(
    zx: float,
    zy: float,
    zz: float,
    *,
    up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> tuple[float, float, float]:
    zx, zy, zz = _normalize(zx, zy, zz)
    ux, uy, uz = _normalize(up[0], up[1], up[2])
    up_dot_z = ux * zx + uy * zy + uz * zz
    px, py, pz = ux - up_dot_z * zx, uy - up_dot_z * zy, uz - up_dot_z * zz
    if math.sqrt(px * px + py * py + pz * pz) > 1.0e-9:
        px, py, pz = _normalize(px, py, pz)
        yx, yy, yz = -px, -py, -pz
    else:
        ref = (1.0, 0.0, 0.0) if abs(zx) < 0.9 else (0.0, 1.0, 0.0)
        yx = zy * ref[2] - zz * ref[1]
        yy = zz * ref[0] - zx * ref[2]
        yz = zx * ref[1] - zy * ref[0]
        yx, yy, yz = _normalize(yx, yy, yz)
    xx = yy * zz - yz * zy
    xy = yz * zx - yx * zz
    xz = yx * zy - yy * zx
    trace = xx + yy + zz
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle <= 1.0e-9:
        return 0.0, 0.0, 0.0
    if angle > math.pi - 1.0e-6 or abs(math.sin(angle)) < 1.0e-6:
        ax = math.sqrt(max(0.0, (xx + 1.0) * 0.5))
        ay = math.sqrt(max(0.0, (yy + 1.0) * 0.5))
        az = math.sqrt(max(0.0, (zz + 1.0) * 0.5))
        if ax >= ay and ax >= az:
            ay = math.copysign(ay, xy + yx)
            az = math.copysign(az, xz + zx)
        elif ay >= az:
            ax = math.copysign(ax, xy + yx)
            az = math.copysign(az, yz + zy)
        else:
            ax = math.copysign(ax, xz + zx)
            ay = math.copysign(ay, yz + zy)
        length = math.sqrt(ax * ax + ay * ay + az * az)
        if length < 1.0e-9:
            return math.pi, 0.0, 0.0
        return ax / length * angle, ay / length * angle, az / length * angle
    inv = 0.5 / math.sin(angle)
    return (yz - zy) * inv * angle, (zx - xz) * inv * angle, (xy - yx) * inv * angle


def apply_pin_tilt(
    zx: float,
    zy: float,
    zz: float,
    up: tuple[float, float, float],
    tilt_deg: float,
) -> tuple[float, float, float]:
    if abs(tilt_deg) <= 1.0e-9:
        return _normalize(zx, zy, zz)
    zx, zy, zz = _normalize(zx, zy, zz)
    ux, uy, uz = _normalize(up[0], up[1], up[2])
    ax = zy * uz - zz * uy
    ay = zz * ux - zx * uz
    az = zx * uy - zy * ux
    a_len = math.sqrt(ax * ax + ay * ay + az * az)
    if a_len < 1.0e-8:
        return zx, zy, zz
    ax, ay, az = ax / a_len, ay / a_len, az / a_len
    ang = math.radians(tilt_deg)
    c, s = math.cos(ang), math.sin(ang)
    dot = ax * zx + ay * zy + az * zz
    return _normalize(
        zx * c + (ay * zz - az * zy) * s + ax * dot * (1.0 - c),
        zy * c + (az * zx - ax * zz) * s + ay * dot * (1.0 - c),
        zz * c + (ax * zy - ay * zx) * s + az * dot * (1.0 - c),
    )


def rotvec_to_tool_z(rx: float, ry: float, rz: float) -> tuple[float, float, float]:
    """Tool +Z from UR rotation vector."""
    ang = math.sqrt(rx * rx + ry * ry + rz * rz)
    if ang < 1.0e-12:
        return 0.0, 0.0, 1.0
    ax, ay, az = rx / ang, ry / ang, rz / ang
    c = math.cos(ang)
    s = math.sin(ang)
    t = 1.0 - c
    # R * (0,0,1)
    return (
        ax * az * t + ay * s,
        ay * az * t - ax * s,
        az * az * t + c,
    )


def fetch_home_tcp(base: str) -> dict[str, float]:
    """Live home TCP [x,y,z,rx,ry,rz] from the sidecar (same orientation as homing)."""
    data = http_json("GET", f"{base}/pose", timeout=10.0)
    pose = data.get("pose") if isinstance(data, dict) else None
    if not isinstance(pose, list) or len(pose) != 6:
        raise RuntimeError("GET /pose did not return home TCP [x,y,z,rx,ry,rz].")
    return {
        "x": float(pose[0]),
        "y": float(pose[1]),
        "z": float(pose[2]),
        "rx": float(pose[3]),
        "ry": float(pose[4]),
        "rz": float(pose[5]),
    }


def _rpy_xyz_mat(
    x_m: float,
    y_m: float,
    z_m: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> list[list[float]]:
    """URDF / tf2: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x_m],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y_m],
        [-sp, cp * sr, cp * cr, z_m],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = (
                a[i][0] * b[0][j]
                + a[i][1] * b[1][j]
                + a[i][2] * b[2][j]
                + a[i][3] * b[3][j]
            )
    return out


def _mat_inv_rigid(t: list[list[float]]) -> list[list[float]]:
    r00, r01, r02 = t[0][0], t[0][1], t[0][2]
    r10, r11, r12 = t[1][0], t[1][1], t[1][2]
    r20, r21, r22 = t[2][0], t[2][1], t[2][2]
    px, py, pz = t[0][3], t[1][3], t[2][3]
    return [
        [r00, r10, r20, -(r00 * px + r10 * py + r20 * pz)],
        [r01, r11, r21, -(r01 * px + r11 * py + r21 * pz)],
        [r02, r12, r22, -(r02 * px + r12 * py + r22 * pz)],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotvec_xyz_mat(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> list[list[float]]:
    ang = math.sqrt(rx * rx + ry * ry + rz * rz)
    t = [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]]
    if ang < 1.0e-12:
        return t
    ax, ay, az = rx / ang, ry / ang, rz / ang
    c, s = math.cos(ang), math.sin(ang)
    k = 1.0 - c
    t[0][0] = c + ax * ax * k
    t[0][1] = ax * ay * k - az * s
    t[0][2] = ax * az * k + ay * s
    t[1][0] = ay * ax * k + az * s
    t[1][1] = c + ay * ay * k
    t[1][2] = ay * az * k - ax * s
    t[2][0] = az * ax * k - ay * s
    t[2][1] = az * ay * k + ax * s
    t[2][2] = c + az * az * k
    return t


def _mat_to_rotvec(t: list[list[float]]) -> tuple[float, float, float]:
    m00, m01, m02 = t[0][0], t[0][1], t[0][2]
    m10, m11, m12 = t[1][0], t[1][1], t[1][2]
    m20, m21, m22 = t[2][0], t[2][1], t[2][2]
    trace = m00 + m11 + m22
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle <= 1.0e-12:
        return 0.0, 0.0, 0.0
    if angle > math.pi - 1.0e-6 or abs(math.sin(angle)) < 1.0e-6:
        ax = math.sqrt(max(0.0, (m00 + 1.0) * 0.5))
        ay = math.sqrt(max(0.0, (m11 + 1.0) * 0.5))
        az = math.sqrt(max(0.0, (m22 + 1.0) * 0.5))
        if ax >= ay and ax >= az:
            ay = math.copysign(ay, m10 + m01)
            az = math.copysign(az, m20 + m02)
        elif ay >= az:
            ax = math.copysign(ax, m10 + m01)
            az = math.copysign(az, m21 + m12)
        else:
            ax = math.copysign(ax, m20 + m02)
            ay = math.copysign(ay, m21 + m12)
        length = math.sqrt(ax * ax + ay * ay + az * az)
        if length < 1.0e-12:
            return math.pi, 0.0, 0.0
        return ax / length * math.pi, ay / length * math.pi, az / length * math.pi
    inv = 0.5 / math.sin(angle)
    return (m21 - m12) * inv * angle, (m02 - m20) * inv * angle, (m10 - m01) * inv * angle


def _tcp_mm(cfg: dict[str, Any], *, camera: bool) -> dict[str, float]:
    if camera:
        return {
            "x": float(cfg.get("tool_tcp_x_mm", 0.715)),
            "y": float(cfg.get("tool_tcp_y_mm", -54.197)),
            "z": float(cfg.get("tool_tcp_z_mm", 73.755)),
            "roll": float(cfg.get("tool_tcp_roll_deg", -1.9138)),
            "pitch": float(cfg.get("tool_tcp_pitch_deg", 0.7450)),
            "yaw": float(cfg.get("tool_tcp_yaw_deg", 0.2868)),
        }
    return {
        "x": float(cfg.get("dlp_tcp_x_mm", 0.372)),
        "y": float(cfg.get("dlp_tcp_y_mm", 57.104)),
        "z": float(cfg.get("dlp_tcp_z_mm", 27.4994)),
        "roll": float(cfg.get("dlp_tcp_roll_deg", 25.0)),
        "pitch": float(cfg.get("dlp_tcp_pitch_deg", 0.0)),
        "yaw": float(cfg.get("dlp_tcp_yaw_deg", 0.0)),
    }


def _tool_mm_to_mat(tcp: dict[str, float]) -> list[list[float]]:
    return _rpy_xyz_mat(
        tcp["x"] * 0.001,
        tcp["y"] * 0.001,
        tcp["z"] * 0.001,
        math.radians(tcp["roll"]),
        math.radians(tcp["pitch"]),
        math.radians(tcp["yaw"]),
    )


def camera_world_to_dlp_world(pose: dict[str, Any], cfg: dict[str, Any]) -> dict[str, float]:
    """Camera world TCP → DLP world TCP (MoveIt tip when scan_tcp=dlp)."""
    t_cam = _tool_mm_to_mat(_tcp_mm(cfg, camera=True))
    t_dlp = _tool_mm_to_mat(_tcp_mm(cfg, camera=False))
    t_cam_dlp = _mat_mul(_mat_inv_rigid(t_cam), t_dlp)
    t_world_cam = _rotvec_xyz_mat(
        float(pose["x"]),
        float(pose["y"]),
        float(pose["z"]),
        float(pose["rx"]),
        float(pose["ry"]),
        float(pose["rz"]),
    )
    t_world_dlp = _mat_mul(t_world_cam, t_cam_dlp)
    rx, ry, rz = _mat_to_rotvec(t_world_dlp)
    return {
        "x": t_world_dlp[0][3],
        "y": t_world_dlp[1][3],
        "z": t_world_dlp[2][3],
        "rx": rx,
        "ry": ry,
        "rz": rz,
        "tool_z_x": t_world_dlp[0][2],
        "tool_z_y": t_world_dlp[1][2],
        "tool_z_z": t_world_dlp[2][2],
    }


def build_apex_pose(
    apex_radius_m: float,
    home_tcp: dict[str, float],
    cfg: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """θ=0: keep home XY + orientation; set Z to the ring radius."""
    del cfg
    rx = float(home_tcp["rx"])
    ry = float(home_tcp["ry"])
    rz = float(home_tcp["rz"])
    tzx, tzy, tzz = rotvec_to_tool_z(rx, ry, rz)
    return {
        "index": 0,
        "x": float(home_tcp["x"]),
        "y": float(home_tcp["y"]),
        "z": float(apex_radius_m),
        "rx": rx,
        "ry": ry,
        "rz": rz,
        "tool_z_x": tzx,
        "tool_z_y": tzy,
        "tool_z_z": tzz,
        "camera_up_x": 0.0,
        "camera_up_y": 0.0,
        "camera_up_z": 1.0,
        "require_perpendicular": True,
        "theta_deg": 0.0,
        "phi_deg": 0.0,
    }


def apex_pose_for_moveit(camera_pose: dict[str, Any], cfg: dict[str, Any]) -> dict[str, Any]:
    """MoveIt EE pose. scan_tcp=dlp → DLP world pose that puts the camera at *camera_pose*."""
    out = dict(camera_pose)
    out["_camera_tcp"] = {
        "x": float(camera_pose["x"]),
        "y": float(camera_pose["y"]),
        "z": float(camera_pose["z"]),
        "rx": float(camera_pose["rx"]),
        "ry": float(camera_pose["ry"]),
        "rz": float(camera_pose["rz"]),
        "tool_z_x": float(camera_pose["tool_z_x"]),
        "tool_z_y": float(camera_pose["tool_z_y"]),
        "tool_z_z": float(camera_pose["tool_z_z"]),
    }
    scan = str(cfg.get("scan_tcp", "camera")).strip().lower()
    if scan in ("dlp", "projector"):
        out.update(camera_world_to_dlp_world(camera_pose, cfg))
    return out


def _apex_camera_tcp(pose: dict[str, Any]) -> dict[str, Any]:
    saved = pose.get("_camera_tcp")
    if isinstance(saved, dict) and "x" in saved:
        return saved
    return pose


def build_ring_poses(
    radius_m: float,
    theta_deg: float,
    n_phi: int,
    tilt_deg: float,
    *,
    start_index: int = 0,
) -> list[dict[str, Any]]:
    """One latitude ring. Look-at tray XY (0,0). Camera upside-down (up = world −Z)."""
    poses: list[dict[str, Any]] = []
    theta = math.radians(theta_deg)
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    up = (0.0, 0.0, -1.0)
    for i in range(max(1, n_phi)):
        phi_deg = 360.0 * i / float(n_phi)
        phi = math.radians(phi_deg)
        x = radius_m * sin_t * math.cos(phi)
        y = radius_m * sin_t * math.sin(phi)
        z = radius_m * cos_t
        zx, zy, zz = apply_pin_tilt(-x, -y, -z, up, tilt_deg)
        rx, ry, rz = tool_z_to_rotation_vector(zx, zy, zz, up=up)
        poses.append(
            {
                "index": start_index + i,
                "x": x,
                "y": y,
                "z": z,
                "rx": rx,
                "ry": ry,
                "rz": rz,
                "tool_z_x": zx,
                "tool_z_y": zy,
                "tool_z_z": zz,
                "camera_up_x": 0.0,
                "camera_up_y": 0.0,
                "camera_up_z": -1.0,
                "require_perpendicular": False,
                "theta_deg": float(theta_deg),
                "phi_deg": float(phi_deg),
            }
        )
    return poses


def build_cylinder_poses(
    rho_m: float,
    z_m: float,
    n_phi: int,
    tilt_deg: float,
    *,
    start_index: int = 0,
) -> list[dict[str, Any]]:
    """Vertical cylinder: fixed XY radius, look-at tray origin (0,0,0). Camera up = world −Z."""
    look_deg = math.degrees(math.atan2(max(0.0, rho_m), max(1.0e-9, z_m)))
    up = (0.0, 0.0, -1.0)
    poses: list[dict[str, Any]] = []
    for i in range(max(1, n_phi)):
        phi_deg = 360.0 * i / float(n_phi)
        phi = math.radians(phi_deg)
        x = rho_m * math.cos(phi)
        y = rho_m * math.sin(phi)
        z = z_m
        zx, zy, zz = apply_pin_tilt(-x, -y, -z, up, tilt_deg)
        rx, ry, rz = tool_z_to_rotation_vector(zx, zy, zz, up=up)
        poses.append(
            {
                "index": start_index + i,
                "x": x,
                "y": y,
                "z": z,
                "rx": rx,
                "ry": ry,
                "rz": rz,
                "tool_z_x": zx,
                "tool_z_y": zy,
                "tool_z_z": zz,
                "camera_up_x": 0.0,
                "camera_up_y": 0.0,
                "camera_up_z": -1.0,
                "require_perpendicular": False,
                "theta_deg": float(look_deg),
                "phi_deg": float(phi_deg),
            }
        )
    return poses


def _is_apex_pose(pose: dict[str, Any]) -> bool:
    return bool(pose.get("require_perpendicular")) or abs(float(pose.get("theta_deg", 99.0))) < 0.75


def _active_tool_tcp(cfg: dict[str, Any]) -> dict[str, float]:
    scan = str(cfg.get("scan_tcp", "camera")).strip().lower()
    if scan in ("dlp", "projector"):
        return {
            "x": float(cfg.get("dlp_tcp_x_mm", 0.372)),
            "y": float(cfg.get("dlp_tcp_y_mm", 57.104)),
            "z": float(cfg.get("dlp_tcp_z_mm", 27.4994)),
            "roll": float(cfg.get("dlp_tcp_roll_deg", 25.0)),
            "pitch": float(cfg.get("dlp_tcp_pitch_deg", 0.0)),
            "yaw": float(cfg.get("dlp_tcp_yaw_deg", 0.0)),
        }
    return {
        "x": float(cfg.get("tool_tcp_x_mm", 0.715)),
        "y": float(cfg.get("tool_tcp_y_mm", -54.197)),
        "z": float(cfg.get("tool_tcp_z_mm", 73.755)),
        "roll": float(cfg.get("tool_tcp_roll_deg", -1.9138)),
        "pitch": float(cfg.get("tool_tcp_pitch_deg", 0.7450)),
        "yaw": float(cfg.get("tool_tcp_yaw_deg", 0.2868)),
    }


def robot_cfg_fingerprint(cfg: dict[str, Any]) -> str:
    home = cfg.get("home_joints_deg") or [90, -180, 145, -55, 90, -90]
    scan = str(cfg.get("scan_tcp", "camera")).strip().lower()
    if scan not in ("dlp", "projector"):
        scan = "camera"
    else:
        scan = "dlp"
    tcp = _active_tool_tcp(cfg)
    fp = {
        "schema": CACHE_SCHEMA,
        "ur_type": str(cfg.get("ur_type", "ur3e")),
        "scan_tcp": scan,
        "tool_tcp_x_mm": tcp["x"],
        "tool_tcp_y_mm": tcp["y"],
        "tool_tcp_z_mm": tcp["z"],
        "tool_tcp_roll_deg": tcp["roll"],
        "tool_tcp_pitch_deg": tcp["pitch"],
        "tool_tcp_yaw_deg": tcp["yaw"],
        "tool_payload_shape": str(cfg.get("tool_payload_shape", "mesh")),
        "tool_payload_mesh": str(cfg.get("tool_payload_mesh", "ur_tool_payload.stl")),
        "tool_payload_radius_mm": float(cfg.get("tool_payload_radius_mm", 80.0)),
        "ceiling_mount_height_mm": float(cfg.get("ceiling_mount_height_mm", 640.0)),
        "workspace_boundary_enabled": bool(cfg.get("workspace_boundary_enabled", True)),
        "workspace_length_mm": float(cfg.get("workspace_length_mm", 900.0)),
        "workspace_width_mm": float(cfg.get("workspace_width_mm", 600.0)),
        "workspace_height_mm": float(cfg.get("workspace_height_mm", 540.0)),
        "workspace_ceiling_clearance_mm": float(
            cfg.get("workspace_ceiling_clearance_mm", 40.0)
        ),
        "mount_roll_deg": float(cfg.get("mount_roll_deg", 180.0)),
        "mount_pitch_deg": float(cfg.get("mount_pitch_deg", 0.0)),
        "mount_yaw_deg": float(cfg.get("mount_yaw_deg", 0.0)),
        "mount_offset_x_mm": float(cfg.get("mount_offset_x_mm", 0.0)),
        "mount_offset_y_mm": float(cfg.get("mount_offset_y_mm", 0.0)),
        "scan_center_from_home_tcp": False,
        "scan_center_base_xy": True,
        "apex_over_home_tcp_xy": True,
        "apex_on_ring_sphere": True,
        "apex_look_down": False,
        "apex_keep_home_pose_z_radius": True,
        "apex_tcp": "scan",
        "camera_tcp_x_mm": float(cfg.get("tool_tcp_x_mm", 0.715)),
        "camera_tcp_y_mm": float(cfg.get("tool_tcp_y_mm", -54.197)),
        "camera_tcp_z_mm": float(cfg.get("tool_tcp_z_mm", 73.755)),
        "camera_tcp_roll_deg": float(cfg.get("tool_tcp_roll_deg", -1.9138)),
        "camera_tcp_pitch_deg": float(cfg.get("tool_tcp_pitch_deg", 0.7450)),
        "camera_tcp_yaw_deg": float(cfg.get("tool_tcp_yaw_deg", 0.2868)),
        "home_joints_deg": [float(v) for v in home],
        "scan_camera_up_world_z": bool(cfg.get("scan_camera_up_world_z", True)),
        "pin_pose_tolerance_deg": float(cfg.get("pin_pose_tolerance_deg", 0.0)),
        "pin_pose_tolerance_vertical_only": True,
        "pin_tcp_tilt_deg": float(cfg.get("pin_tcp_tilt_deg", 0.0)),
        "semi_ring_search_candidates": int(cfg.get("semi_ring_search_candidates", 360)),
    }
    return json.dumps(fp, separators=(",", ":"), ensure_ascii=False)


def plan_fingerprint(radius_m: float, theta_deg: float, n_phi: int) -> str:
    return json.dumps(
        {
            "kind": "ur3e_semi_one_ring",
            "sphere_radius_m": radius_m,
            "theta_deg": theta_deg,
            "horizontal_points": n_phi,
            "vertical_points": 2,
            "always_apex_pin": True,
            "apex_radius_m": radius_m,
            "apex_look_down": False,
            "apex_keep_home_pose_z_radius": True,
            "apex_tcp": "scan",
        },
        separators=(",", ":"),
    )


def cylinder_fingerprint(rho_m: float, z_m: float, n_phi: int) -> str:
    return json.dumps(
        {
            "kind": "ur3e_semi_cylinder_ring",
            "cyl_radius_m": rho_m,
            "z_m": z_m,
            "horizontal_points": n_phi,
        },
        separators=(",", ":"),
    )


def _geometric_apex_point(pose: dict[str, Any]) -> dict[str, Any]:
    """θ=0: home XY/orientation, Z = R. No joints yet."""
    x = float(pose["x"])
    y = float(pose["y"])
    z = float(pose["z"])
    return {
        "grid": {
            "phi_deg": 0.0,
            "theta_deg": 0.0,
            "x_m": x,
            "y_m": y,
            "z_m": z,
        },
        "tcp": {
            "x_m": x,
            "y_m": y,
            "z_m": z,
            "rx": float(pose["rx"]),
            "ry": float(pose["ry"]),
            "rz": float(pose["rz"]),
            "tool_z_x": float(pose.get("tool_z_x", 0.0)),
            "tool_z_y": float(pose.get("tool_z_y", 0.0)),
            "tool_z_z": float(pose.get("tool_z_z", 1.0)),
        },
        "reachable": False,
        "home_path_ok": False,
        "base_sweep_ok": True,
        "backup_coverage_ok": False,
        "backup_union_deg": 0.0,
        "planning_error": "apex is home pose with Z=R; joints planned at execute if missing",
        "joints_rad": [],
    }


def _point_from_result(pose: dict[str, Any], res: dict[str, Any]) -> dict[str, Any]:
    tcp_obj = res.get("tcp") if isinstance(res.get("tcp"), dict) else {}
    joints = res.get("joints") or []
    is_apex = _is_apex_pose(pose)
    point = {
        "grid": {
            "phi_deg": float(pose["phi_deg"]),
            "theta_deg": float(pose["theta_deg"]),
            "x_m": float(pose["x"]),
            "y_m": float(pose["y"]),
            "z_m": float(pose["z"]),
        },
        "tcp": {
            "x_m": float(tcp_obj.get("x", pose["x"])),
            "y_m": float(tcp_obj.get("y", pose["y"])),
            "z_m": float(tcp_obj.get("z", pose["z"])),
            "rx": float(tcp_obj.get("rx", pose["rx"])),
            "ry": float(tcp_obj.get("ry", pose["ry"])),
            "rz": float(tcp_obj.get("rz", pose["rz"])),
            "tool_z_x": float(tcp_obj.get("tool_z_x", pose["tool_z_x"])),
            "tool_z_y": float(tcp_obj.get("tool_z_y", pose["tool_z_y"])),
            "tool_z_z": float(tcp_obj.get("tool_z_z", pose["tool_z_z"])),
        },
        "reachable": True,
        "home_path_ok": True,
        "base_sweep_ok": bool(res.get("base_sweep_ok", not is_apex)),
        "backup_coverage_ok": bool(res.get("backup_coverage_ok", False)),
        "backup_union_deg": float(res.get("backup_union_deg", 0.0) or 0.0),
        "planning_error": "",
        "joints_rad": [float(v) for v in joints],
    }
    mask = res.get("pan_mask")
    if isinstance(mask, list) and mask:
        point["pan_mask"] = [bool(v) for v in mask]
    return point


def save_one_ring_plan(
    out_path: Path,
    *,
    cfg: dict[str, Any],
    radius_m: float,
    theta_deg: float,
    n_phi: int,
    poses: list[dict[str, Any]],
    results: list[dict[str, Any]],
    cached_apex: dict[str, Any] | None = None,
    home_tcp: dict[str, float] | None = None,
    plan_name: str | None = None,
    fingerprint: str | None = None,
    scan_params: dict[str, Any] | None = None,
) -> tuple[int, dict[str, Any] | None, int]:
    """Write schema-4 plan: apex (same sphere R) + base_sweep_ok ring pins.

    Always writes an apex point when the ring is saved. Planned joints if IK
    succeeded; otherwise home XY/orientation with Z = R.
    Returns (n_written, apex_point_or_none, n_ring_pins). Does not write if no ring pins.
    """
    by_index = {int(r["index"]): r for r in results if isinstance(r, dict)}
    apex_point: dict[str, Any] | None = dict(cached_apex) if cached_apex else None
    apex_pose = next((p for p in poses if _is_apex_pose(p)), None)
    ring_points: list[dict[str, Any]] = []
    for pose in poses:
        idx = int(pose["index"])
        res = by_index.get(idx)
        if res is None or not res.get("reachable"):
            continue
        if _is_apex_pose(pose):
            if res.get("home_path_ok", True):
                apex_point = _point_from_result(pose, res)
            continue
        if not res.get("home_path_ok", True):
            continue
        if not (res.get("base_sweep_ok") or res.get("backup_coverage_ok")):
            continue
        ring_points.append(_point_from_result(pose, res))

    if not ring_points:
        return 0, apex_point, 0

    if apex_point is None:
        if apex_pose is None and home_tcp and "x" in home_tcp:
            apex_pose = build_apex_pose(radius_m, home_tcp, cfg)
        if apex_pose is not None:
            apex_point = _geometric_apex_point(apex_pose)

    points: list[dict[str, Any]] = []
    if apex_point is not None:
        points.append(apex_point)
    points.extend(ring_points)

    root = {
        "schema": CACHE_SCHEMA,
        "name": plan_name or f"R{int(round(radius_m * 1000))}_T{theta_deg:g}",
        "fingerprint": fingerprint or plan_fingerprint(radius_m, theta_deg, n_phi),
        "robot_cfg_fingerprint": robot_cfg_fingerprint(cfg),
        "scan_params": scan_params
        or {
            "sphere_radius_m": radius_m,
            "horizontal_points": n_phi,
            "vertical_points": 2,
            "theta_min_deg": 0.0,
            "theta_max_deg": theta_deg,
            "apex_radius_m": radius_m,
        },
        "points": points,
        "reachable_count": sum(1 for p in points if p.get("reachable")),
        "unreachable_count": sum(1 for p in points if not p.get("reachable")),
        "home_path_ok_count": sum(1 for p in points if p.get("home_path_ok")),
        "chain_only_count": 0,
        "moveit_used": True,
        "error_message": "",
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(root, indent=2) + "\n", encoding="utf-8")
    return len(points), apex_point, len(ring_points)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cfg",
        type=Path,
        default=_repo_root() / "app" / "preset" / "hyperfusion.cfg",
    )
    parser.add_argument("--base-url", default="http://127.0.0.1:8766")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=_repo_root() / "app" / "build" / "Release" / "ur3e_semi_scan_routes",
    )
    parser.add_argument("--theta-min", type=float, default=20.0)
    parser.add_argument("--theta-max", type=float, default=40.0)
    parser.add_argument("--theta-step", type=float, default=1.0)
    parser.add_argument("--radius-min-mm", type=float, default=150.0)
    parser.add_argument("--radius-max-mm", type=float, default=300.0)
    parser.add_argument("--radius-step-mm", type=float, default=5.0)
    parser.add_argument(
        "--max-sweep-ok",
        type=int,
        default=2,
        help="Stop after this many base-sweep OK pins per ring (app default 2).",
    )
    parser.add_argument(
        "--plan-timeout-s",
        type=float,
        default=7200.0,
        help="HTTP timeout per ring plan call.",
    )
    parser.add_argument(
        "--progress-name",
        default="_batch_semi_ring_search_progress.json",
        help="Progress JSON filename inside --out-dir (unique per parallel worker).",
    )
    parser.add_argument(
        "--connect-timeout-s",
        type=float,
        default=360.0,
        help="Wait this long for sidecar /connect before aborting the worker.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip combos already recorded in progress JSON.",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Sweep largest radius and theta first.",
    )
    parser.add_argument(
        "--skip-apex",
        action="store_true",
        help="Do not plan or save the θ=0 apex pin.",
    )
    parser.add_argument(
        "--home-joints-deg",
        default=None,
        help="Override cfg home (comma-separated deg). Example: 90,-180,145,-55,90,90",
    )
    parser.add_argument(
        "--hub-plan",
        type=Path,
        default=None,
        help="Use this saved plan's first base-sweep-OK pin as the path hub "
        "(T20↔pin instead of scan-home↔pin). Skips apex.",
    )
    parser.add_argument(
        "--cylinder",
        action="store_true",
        help="Sweep a vertical cylinder (fixed XY radius, vary height). Look-at tray origin.",
    )
    parser.add_argument("--cyl-radius-mm", type=float, default=70.0)
    parser.add_argument("--height-min-mm", type=float, default=160.0)
    parser.add_argument("--height-max-mm", type=float, default=200.0)
    parser.add_argument("--height-step-mm", type=float, default=5.0)
    args = parser.parse_args()

    cfg = parse_cfg(args.cfg)
    n_phi = int(cfg.get("semi_ring_search_candidates", 360))
    n_phi = max(1, min(720, n_phi))
    tilt_deg = float(cfg.get("pin_tcp_tilt_deg", 0.0))
    tol_deg = float(cfg.get("pin_pose_tolerance_deg", 0.0))
    backup_deg = float(cfg.get("semi_backup_coverage_deg", 300.0))
    lock_up = bool(cfg.get("scan_camera_up_world_z", True))
    home = cfg.get("home_joints_deg") or [90.0, -180.0, 145.0, -55.0, 90.0, -90.0]
    if args.home_joints_deg:
        home = [float(p.strip()) for p in str(args.home_joints_deg).split(",") if p.strip()]
        if len(home) != 6:
            raise SystemExit("--home-joints-deg needs 6 values")
        cfg["home_joints_deg"] = home
    hub_label = "cfg home"
    skip_apex = bool(args.skip_apex)
    if args.hub_plan is not None:
        home, hub_label = load_hub_joints_deg(args.hub_plan)
        skip_apex = True

    workspace = {
        "enabled": bool(cfg.get("workspace_boundary_enabled", True)),
        "length_m": float(cfg.get("workspace_length_mm", 900.0)) / 1000.0,
        "width_m": float(cfg.get("workspace_width_mm", 600.0)) / 1000.0,
        "height_m": float(cfg.get("workspace_height_mm", 540.0)) / 1000.0,
        "mount_height_m": float(cfg.get("ceiling_mount_height_mm", 640.0)) / 1000.0,
        "ceiling_clearance_m": float(cfg.get("workspace_ceiling_clearance_mm", 40.0))
        / 1000.0,
    }

    cylinder = bool(args.cylinder)
    if cylinder:
        skip_apex = True
        heights_mm: list[float] = []
        h = float(args.height_min_mm)
        while h <= args.height_max_mm + 1.0e-9:
            heights_mm.append(round(h, 6))
            h += args.height_step_mm
        if args.reverse:
            heights_mm.reverse()
        rho_mm = float(args.cyl_radius_mm)
        combos = [(rho_mm, zmm) for zmm in heights_mm]
        print(
            f"Combos={len(combos)} cylinder ρ={rho_mm:g} mm "
            f"z={heights_mm[0]:g}..{heights_mm[-1]:g} mm "
            f"n_phi={n_phi} tip_tol={tol_deg} tilt={tilt_deg} backup complementary halves "
            f"hub={hub_label} skip_apex={skip_apex} out={args.out_dir}",
            flush=True,
        )
    else:
        thetas: list[float] = []
        t = float(args.theta_min)
        while t <= args.theta_max + 1.0e-9:
            thetas.append(round(t, 6))
            t += args.theta_step

        radii_mm: list[float] = []
        r = float(args.radius_min_mm)
        while r <= args.radius_max_mm + 1.0e-9:
            radii_mm.append(round(r, 6))
            r += args.radius_step_mm

        if args.reverse:
            thetas.reverse()
            radii_mm.reverse()

        combos = [(rm, th) for rm in radii_mm for th in thetas]
        print(
            f"Combos={len(combos)} theta={thetas[0]}..{thetas[-1]} R={radii_mm[0]}..{radii_mm[-1]} mm "
            f"n_phi={n_phi} tip_tol={tol_deg} tilt={tilt_deg} backup complementary halves "
            f"hub={hub_label} skip_apex={skip_apex} "
            f"apex_z=same as R out={args.out_dir}",
            flush=True,
        )

    progress_path = args.out_dir / str(args.progress_name)
    done: dict[str, Any] = {}
    if args.resume and progress_path.is_file():
        done = json.loads(progress_path.read_text(encoding="utf-8-sig"))

    wait_connected(args.base_url, timeout_s=float(args.connect_timeout_s))
    home_tcp = fetch_home_tcp(args.base_url)
    print(
        "Connected. Apex keeps home XY and orientation; only Z changes to the ring radius.",
        flush=True,
    )

    saved = 0
    failed = 0
    skipped = 0
    missing_apex = 0
    t_batch = time.time()
    # Reuse a proven apex only for the same sphere radius (apex Z = R).
    cached_apex: dict[str, Any] | None = None
    cached_apex_radius_m: float | None = None

    for i, (a_mm, b_mm) in enumerate(combos, start=1):
        if cylinder:
            rho_mm, z_mm = a_mm, b_mm
            key = f"C{int(round(rho_mm))}_Z{int(round(z_mm))}"
            radius_m = rho_mm / 1000.0
            z_m = z_mm / 1000.0
            look_deg = math.degrees(math.atan2(radius_m, max(1.0e-9, z_m)))
        else:
            radius_mm, theta_deg = a_mm, b_mm
            key = f"R{int(round(radius_mm))}_T{theta_deg:g}"
            radius_m = radius_mm / 1000.0
            look_deg = float(theta_deg)
            z_m = 0.0
        if args.resume and key in done:
            skipped += 1
            continue

        poses: list[dict[str, Any]] = []
        if not skip_apex and (
            cached_apex is None
            or cached_apex_radius_m is None
            or abs(cached_apex_radius_m - radius_m) > 1.0e-9
        ):
            poses.append(build_apex_pose(radius_m, home_tcp, cfg))
        if cylinder:
            poses.extend(
                build_cylinder_poses(
                    radius_m, z_m, n_phi, tilt_deg, start_index=len(poses)
                )
            )
        else:
            poses.extend(
                build_ring_poses(
                    radius_m, look_deg, n_phi, tilt_deg, start_index=len(poses)
                )
            )
        body = {
            "poses": poses,
            "workspace": workspace,
            "pin_pose_tolerance_deg": tol_deg,
            "scan_camera_up_world_z": lock_up,
            "semi_ring_sweep": True,
            "semi_max_sweep_ok_per_ring": max(1, int(args.max_sweep_ok)),
            "semi_ring_search_candidates": n_phi,
            "semi_backup_coverage_deg": backup_deg,
            "home_joints_deg": [float(v) for v in home],
        }

        t0 = time.time()
        try:
            resp = http_json(
                "POST",
                f"{args.base_url}/plan_hemisphere_scan",
                body=body,
                timeout=args.plan_timeout_s,
            )
        except Exception as exc:  # noqa: BLE001 — batch must continue
            failed += 1
            done[key] = {"ok": False, "error": str(exc)}
            progress_path.parent.mkdir(parents=True, exist_ok=True)
            progress_path.write_text(json.dumps(done, indent=2), encoding="utf-8")
            print(f"[{i}/{len(combos)}] {key} ERROR {exc}", flush=True)
            continue

        results = resp.get("results") if isinstance(resp, dict) else None
        if not isinstance(results, list) or not resp.get("ok", False):
            failed += 1
            err = resp.get("error") if isinstance(resp, dict) else "bad response"
            done[key] = {"ok": False, "error": err}
            progress_path.write_text(json.dumps(done, indent=2), encoding="utf-8")
            print(f"[{i}/{len(combos)}] {key} plan not ok: {err}", flush=True)
            continue

        out_file = args.out_dir / f"{key}.json"
        save_kw: dict[str, Any] = {}
        if cylinder:
            save_kw = {
                "plan_name": key,
                "fingerprint": cylinder_fingerprint(radius_m, z_m, n_phi),
                "scan_params": {
                    "cyl_radius_m": radius_m,
                    "z_m": z_m,
                    "horizontal_points": n_phi,
                    "look_from_vertical_deg": look_deg,
                },
            }
        n_ok, apex_pt, n_ring = save_one_ring_plan(
            out_file,
            cfg=cfg,
            radius_m=radius_m,
            theta_deg=look_deg,
            n_phi=n_phi,
            poses=poses,
            results=results,
            cached_apex=cached_apex,
            home_tcp=home_tcp,
            **save_kw,
        )
        if (
            apex_pt is not None
            and apex_pt.get("reachable")
            and len(apex_pt.get("joints_rad") or []) == 6
        ):
            cached_apex = apex_pt
            cached_apex_radius_m = radius_m
        dt = time.time() - t0
        if n_ok > 0:
            saved += 1
            has_apex = apex_pt is not None
            planned_apex = bool(
                apex_pt
                and apex_pt.get("reachable")
                and len(apex_pt.get("joints_rad") or []) == 6
            )
            if not planned_apex:
                missing_apex += 1
            done[key] = {
                "ok": True,
                "sweep_ok": n_ring,
                "apex_ok": planned_apex,
                "file": str(out_file),
                "seconds": round(dt, 2),
            }
            apex_tag = "apex+" if planned_apex else "apex-tcp "
            backup_n = sum(
                1
                for r in results
                if isinstance(r, dict)
                and r.get("reachable")
                and r.get("backup_coverage_ok")
            )
            kind = "BACKUP " if backup_n else apex_tag
            print(
                f"[{i}/{len(combos)}] {key} {kind}{n_ring} ({dt:.1f}s)",
                flush=True,
            )
            done[key]["backup"] = backup_n > 0
            # Only mirror top-level library writes. Subdir tests must not
            # overwrite Release/R250_T*.json with a different home fingerprint.
            if out_file.parent.name == "mvs_semi_scan_plans":
                release_copy = (
                    _repo_root()
                    / "app"
                    / "build"
                    / "Release"
                    / "mvs_semi_scan_plans"
                    / out_file.name
                )
                if (
                    release_copy.parent.is_dir()
                    and out_file.resolve() != release_copy.resolve()
                ):
                    release_copy.write_text(
                        out_file.read_text(encoding="utf-8"), encoding="utf-8"
                    )
        else:
            done[key] = {
                "ok": True,
                "sweep_ok": 0,
                "apex_ok": apex_pt is not None,
                "seconds": round(dt, 2),
            }
            print(
                f"[{i}/{len(combos)}] {key} none ({dt:.1f}s)",
                flush=True,
            )

        progress_path.parent.mkdir(parents=True, exist_ok=True)
        progress_path.write_text(json.dumps(done, indent=2), encoding="utf-8")

    elapsed = time.time() - t_batch
    print(
        f"Done in {elapsed / 60.0:.1f} min: saved={saved} empty={len(combos) - saved - failed - skipped} "
        f"failed={failed} skipped={skipped} missing_apex={missing_apex}",
        flush=True,
    )
    return 0 if failed == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
