#!/usr/bin/env python3
"""Batch Semi ring Plan: θ×radius sweep via MoveIt sidecar; one (R,θ) per saved JSON.

Each file is apex (θ=0, home_path_ok) plus that latitude's base-sweep-OK pins.
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
# Semi-fixed apex look-down height. Independent of the ring sphere radius.
APEX_RADIUS_M = 0.200


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


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
    http_json("POST", f"{base}/connect/start", body={}, timeout=30.0)
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


def build_apex_pose(apex_radius_m: float = APEX_RADIUS_M) -> dict[str, Any]:
    """θ=0 look-down pin. MoveIt snaps XY/orientation from home TCP FK; Z stays apex_radius_m."""
    zx, zy, zz = 0.0, 0.0, -1.0
    rx, ry, rz = tool_z_to_rotation_vector(zx, zy, zz, up=(1.0, 0.0, 0.0))
    return {
        "index": 0,
        "x": 0.0,
        "y": 0.0,
        "z": float(apex_radius_m),
        "rx": rx,
        "ry": ry,
        "rz": rz,
        "tool_z_x": zx,
        "tool_z_y": zy,
        "tool_z_z": zz,
        "camera_up_x": 1.0,
        "camera_up_y": 0.0,
        "camera_up_z": 0.0,
        "require_perpendicular": True,
        "theta_deg": 0.0,
        "phi_deg": 0.0,
    }


def build_ring_poses(
    radius_m: float,
    theta_deg: float,
    n_phi: int,
    tilt_deg: float,
    *,
    start_index: int = 0,
) -> list[dict[str, Any]]:
    """One latitude ring. Look-at tray XY (0,0)."""
    poses: list[dict[str, Any]] = []
    theta = math.radians(theta_deg)
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    up = (0.0, 0.0, 1.0)
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
                "camera_up_z": 1.0,
                "require_perpendicular": False,
                "theta_deg": float(theta_deg),
                "phi_deg": float(phi_deg),
            }
        )
    return poses


def _is_apex_pose(pose: dict[str, Any]) -> bool:
    return bool(pose.get("require_perpendicular")) or abs(float(pose.get("theta_deg", 99.0))) < 0.75


def robot_cfg_fingerprint(cfg: dict[str, Any]) -> str:
    home = cfg.get("home_joints_deg") or [90, -180, 145, -55, 90, -90]
    fp = {
        "schema": CACHE_SCHEMA,
        "ur_type": str(cfg.get("ur_type", "ur3e")),
        "tool_tcp_x_mm": float(cfg.get("tool_tcp_x_mm", 0.715)),
        "tool_tcp_y_mm": float(cfg.get("tool_tcp_y_mm", -54.197)),
        "tool_tcp_z_mm": float(cfg.get("tool_tcp_z_mm", 73.755)),
        "tool_tcp_roll_deg": float(cfg.get("tool_tcp_roll_deg", -1.9138)),
        "tool_tcp_pitch_deg": float(cfg.get("tool_tcp_pitch_deg", 0.7450)),
        "tool_tcp_yaw_deg": float(cfg.get("tool_tcp_yaw_deg", 0.2868)),
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
        "apex_radius_m": APEX_RADIUS_M,
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
            "apex_radius_m": APEX_RADIUS_M,
        },
        separators=(",", ":"),
    )


def _point_from_result(pose: dict[str, Any], res: dict[str, Any]) -> dict[str, Any]:
    tcp_obj = res.get("tcp") if isinstance(res.get("tcp"), dict) else {}
    joints = res.get("joints") or []
    is_apex = _is_apex_pose(pose)
    return {
        "grid": {
            "phi_deg": float(pose["phi_deg"]),
            "theta_deg": float(pose["theta_deg"]),
            "x_m": float(tcp_obj.get("x", pose["x"])) if is_apex else float(pose["x"]),
            "y_m": float(tcp_obj.get("y", pose["y"])) if is_apex else float(pose["y"]),
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
        "base_sweep_ok": not is_apex,
        "planning_error": "",
        "joints_rad": [float(v) for v in joints],
    }


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
) -> tuple[int, dict[str, Any] | None, int]:
    """Write schema-4 plan: apex (home_path_ok) + base_sweep_ok ring pins.

    Returns (n_written, apex_point_or_none, n_ring_pins). Does not write if no ring pins.
    """
    by_index = {int(r["index"]): r for r in results if isinstance(r, dict)}
    apex_point: dict[str, Any] | None = dict(cached_apex) if cached_apex else None
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
        if not res.get("base_sweep_ok") or not res.get("home_path_ok", True):
            continue
        ring_points.append(_point_from_result(pose, res))

    if not ring_points:
        return 0, apex_point, 0

    points: list[dict[str, Any]] = []
    if apex_point is not None:
        points.append(apex_point)
    points.extend(ring_points)

    root = {
        "schema": CACHE_SCHEMA,
        "name": f"R{int(round(radius_m * 1000))}_T{theta_deg:g}",
        "fingerprint": plan_fingerprint(radius_m, theta_deg, n_phi),
        "robot_cfg_fingerprint": robot_cfg_fingerprint(cfg),
        "scan_params": {
            "sphere_radius_m": radius_m,
            "horizontal_points": n_phi,
            "vertical_points": 2,
            "theta_min_deg": 0.0,
            "theta_max_deg": theta_deg,
            "apex_radius_m": APEX_RADIUS_M,
        },
        "points": points,
        "reachable_count": len(points),
        "unreachable_count": 0,
        "home_path_ok_count": len(points),
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
        default=_repo_root() / "app" / "hyperfusion.cfg",
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
        default=3,
        help="Stop after this many base-sweep OK pins per ring (app default 3).",
    )
    parser.add_argument(
        "--plan-timeout-s",
        type=float,
        default=7200.0,
        help="HTTP timeout per ring plan call.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip combos already recorded in progress JSON.",
    )
    args = parser.parse_args()

    cfg = parse_cfg(args.cfg)
    n_phi = int(cfg.get("semi_ring_search_candidates", 360))
    n_phi = max(1, min(720, n_phi))
    tilt_deg = float(cfg.get("pin_tcp_tilt_deg", 0.0))
    tol_deg = float(cfg.get("pin_pose_tolerance_deg", 0.0))
    lock_up = bool(cfg.get("scan_camera_up_world_z", True))
    home = cfg.get("home_joints_deg") or [90.0, -180.0, 145.0, -55.0, 90.0, -90.0]

    workspace = {
        "enabled": bool(cfg.get("workspace_boundary_enabled", True)),
        "length_m": float(cfg.get("workspace_length_mm", 900.0)) / 1000.0,
        "width_m": float(cfg.get("workspace_width_mm", 600.0)) / 1000.0,
        "height_m": float(cfg.get("workspace_height_mm", 540.0)) / 1000.0,
        "mount_height_m": float(cfg.get("ceiling_mount_height_mm", 640.0)) / 1000.0,
        "ceiling_clearance_m": float(cfg.get("workspace_ceiling_clearance_mm", 40.0))
        / 1000.0,
    }

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

    combos = [(rm, th) for rm in radii_mm for th in thetas]
    progress_path = args.out_dir / "_batch_semi_ring_search_progress.json"
    done: dict[str, Any] = {}
    if args.resume and progress_path.is_file():
        done = json.loads(progress_path.read_text(encoding="utf-8"))

    print(
        f"Combos={len(combos)} theta={thetas[0]}..{thetas[-1]} R={radii_mm[0]}..{radii_mm[-1]} mm "
        f"n_phi={n_phi} tip_tol={tol_deg} tilt={tilt_deg} "
        f"apex_z={APEX_RADIUS_M * 1000.0:.0f}mm out={args.out_dir}",
        flush=True,
    )

    wait_connected(args.base_url, timeout_s=360.0)
    print("Connected.", flush=True)

    saved = 0
    failed = 0
    skipped = 0
    missing_apex = 0
    t_batch = time.time()
    # Apex Z is always 200 mm; reuse one proven apex across all (R, θ).
    cached_apex: dict[str, Any] | None = None

    for i, (radius_mm, theta_deg) in enumerate(combos, start=1):
        key = f"R{int(round(radius_mm))}_T{theta_deg:g}"
        if args.resume and key in done:
            skipped += 1
            continue

        radius_m = radius_mm / 1000.0
        poses: list[dict[str, Any]] = []
        if cached_apex is None:
            poses.append(build_apex_pose(APEX_RADIUS_M))
        poses.extend(
            build_ring_poses(
                radius_m, theta_deg, n_phi, tilt_deg, start_index=len(poses)
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
        n_ok, apex_pt, n_ring = save_one_ring_plan(
            out_file,
            cfg=cfg,
            radius_m=radius_m,
            theta_deg=theta_deg,
            n_phi=n_phi,
            poses=poses,
            results=results,
            cached_apex=cached_apex,
        )
        if apex_pt is not None:
            cached_apex = apex_pt
        dt = time.time() - t0
        if n_ok > 0:
            saved += 1
            has_apex = apex_pt is not None
            if not has_apex:
                missing_apex += 1
            done[key] = {
                "ok": True,
                "sweep_ok": n_ring,
                "apex_ok": has_apex,
                "file": str(out_file),
                "seconds": round(dt, 2),
            }
            apex_tag = "apex+" if has_apex else "NO-APEX "
            print(
                f"[{i}/{len(combos)}] {key} {apex_tag}{n_ring} ({dt:.1f}s)",
                flush=True,
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
