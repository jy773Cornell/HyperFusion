#!/usr/bin/env python3
"""Find a DLP look-down pose near base_link XY=0 at the highest workspace Z.

Does not use require_perpendicular (that forces home XY + home 25° tilt).
MoveIt IK only — no motion.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

from batch_semi_ring_search import (
    fetch_home_tcp,
    http_json,
    parse_cfg,
    rotvec_to_tool_z,
    tool_z_to_rotation_vector,
    wait_connected,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def rotvec_to_tool_y(rx: float, ry: float, rz: float) -> tuple[float, float, float]:
    ang = math.sqrt(rx * rx + ry * ry + rz * rz)
    if ang < 1.0e-12:
        return 0.0, 1.0, 0.0
    ax, ay, az = rx / ang, ry / ang, rz / ang
    c = math.cos(ang)
    s = math.sin(ang)
    t = 1.0 - c
    return (
        ax * ay * t - az * s,
        ay * ay * t + c,
        az * ay * t + ax * s,
    )


def workspace_from_cfg(cfg: dict[str, Any]) -> dict[str, Any]:
    return {
        "enabled": bool(cfg.get("workspace_boundary_enabled", True)),
        "length_m": float(cfg.get("workspace_length_mm", 900.0)) / 1000.0,
        "width_m": float(cfg.get("workspace_width_mm", 600.0)) / 1000.0,
        "height_m": float(cfg.get("workspace_height_mm", 788.0)) / 1000.0,
        "mount_height_m": float(cfg.get("ceiling_mount_height_mm", 888.0)) / 1000.0,
        "ceiling_clearance_m": float(cfg.get("workspace_ceiling_clearance_mm", 40.0))
        / 1000.0,
    }


def workspace_z_limits_m(ws: dict[str, Any]) -> tuple[float, float]:
    mount = float(ws["mount_height_m"])
    height = float(ws["height_m"])
    clear = float(ws["ceiling_clearance_m"])
    floor = max(0.0, mount - height)
    top = max(floor, mount - clear)
    return floor, top


def look_down_pose(
    index: int,
    x: float,
    y: float,
    z: float,
    camera_up: tuple[float, float, float],
) -> dict[str, Any]:
    rx, ry, rz = tool_z_to_rotation_vector(0.0, 0.0, -1.0, up=camera_up)
    return {
        "index": index,
        "x": x,
        "y": y,
        "z": z,
        "rx": rx,
        "ry": ry,
        "rz": rz,
        "tool_z_x": 0.0,
        "tool_z_y": 0.0,
        "tool_z_z": -1.0,
        "camera_up_x": camera_up[0],
        "camera_up_y": camera_up[1],
        "camera_up_z": camera_up[2],
        "require_perpendicular": False,
        "theta_deg": 0.0,
        "phi_deg": math.degrees(math.atan2(y, x)) if abs(x) + abs(y) > 1.0e-9 else 0.0,
    }


def xy_samples(radii_mm: list[float], n_phi: int) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    seen: set[tuple[int, int]] = set()
    for r_mm in radii_mm:
        r = r_mm / 1000.0
        if r <= 1.0e-9:
            key = (0, 0)
            if key not in seen:
                seen.add(key)
                out.append((0.0, 0.0))
            continue
        for i in range(max(1, n_phi)):
            phi = 2.0 * math.pi * i / float(n_phi)
            x, y = r * math.cos(phi), r * math.sin(phi)
            key = (int(round(x * 10000)), int(round(y * 10000)))
            if key in seen:
                continue
            seen.add(key)
            out.append((x, y))
    return out


def plan_batch(
    base: str,
    poses: list[dict[str, Any]],
    cfg: dict[str, Any],
    workspace: dict[str, Any],
    timeout_s: float,
) -> list[dict[str, Any]]:
    home = cfg.get("home_joints_deg") or [90.0, -180.0, 145.0, -55.0, 90.0, -90.0]
    body = {
        "poses": poses,
        "workspace": workspace,
        "pin_pose_tolerance_deg": 0.0,
        "scan_camera_up_world_z": False,
        "semi_ring_sweep": False,
        "home_joints_deg": [float(v) for v in home],
    }
    resp = http_json("POST", f"{base}/plan_hemisphere_scan", body, timeout=timeout_s)
    results = resp.get("results") if isinstance(resp, dict) else None
    if not isinstance(results, list) or not resp.get("ok", False):
        raise RuntimeError(f"plan not ok: {resp}")
    return results


def pick_ok(
    poses: list[dict[str, Any]], results: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    hits: list[dict[str, Any]] = []
    for pose, res in zip(poses, results):
        if not res.get("reachable"):
            continue
        if not res.get("home_path_ok", True):
            continue
        joints = res.get("joints") or []
        if len(joints) != 6:
            continue
        tcp = res.get("tcp") if isinstance(res.get("tcp"), dict) else {}
        x = float(tcp.get("x", pose["x"]))
        y = float(tcp.get("y", pose["y"]))
        z = float(tcp.get("z", pose["z"]))
        hits.append(
            {
                "x_m": x,
                "y_m": y,
                "z_m": z,
                "xy_mm": 1000.0 * math.hypot(x, y),
                "z_mm": 1000.0 * z,
                "rx": float(tcp.get("rx", pose["rx"])),
                "ry": float(tcp.get("ry", pose["ry"])),
                "rz": float(tcp.get("rz", pose["rz"])),
                "tool_z": [
                    float(tcp.get("tool_z_x", pose["tool_z_x"])),
                    float(tcp.get("tool_z_y", pose["tool_z_y"])),
                    float(tcp.get("tool_z_z", pose["tool_z_z"])),
                ],
                "joints_rad": [float(v) for v in joints],
                "joints_deg": [math.degrees(float(v)) for v in joints],
                "error": str(res.get("error") or ""),
            }
        )
    hits.sort(key=lambda h: (h["xy_mm"], -h["z_mm"]))
    return hits


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cfg",
        type=Path,
        default=_repo_root() / "app" / "preset" / "hyperfusion.cfg",
    )
    parser.add_argument("--base-url", default="http://127.0.0.1:8771")
    parser.add_argument(
        "--out",
        type=Path,
        default=_repo_root()
        / "app"
        / "preset"
        / "mvs_scan_plans"
        / "fpp"
        / "_dlp_nadir_high.json",
    )
    parser.add_argument("--z-max-mm", type=float, default=0.0, help="0 = workspace top")
    parser.add_argument("--z-min-mm", type=float, default=350.0)
    parser.add_argument("--z-step-mm", type=float, default=20.0)
    parser.add_argument("--xy-radii-mm", default="0,40,80,120,160")
    parser.add_argument("--n-phi", type=int, default=8)
    parser.add_argument("--plan-timeout-s", type=float, default=900.0)
    args = parser.parse_args()

    cfg = parse_cfg(args.cfg)
    workspace = workspace_from_cfg(cfg)
    floor_m, top_m = workspace_z_limits_m(workspace)
    z_max_mm = float(args.z_max_mm) if args.z_max_mm > 1.0 else 1000.0 * top_m
    z_min_mm = max(1000.0 * floor_m, float(args.z_min_mm))
    z_max_mm = min(z_max_mm, 1000.0 * top_m)

    wait_connected(args.base_url, timeout_s=360.0)
    home_tcp = fetch_home_tcp(args.base_url)
    tyx, tyy, tyz = rotvec_to_tool_y(home_tcp["rx"], home_tcp["ry"], home_tcp["rz"])
    camera_up = (-tyx, -tyy, -tyz)
    n = math.sqrt(sum(v * v for v in camera_up))
    camera_up = (camera_up[0] / n, camera_up[1] / n, camera_up[2] / n)
    home_z = rotvec_to_tool_z(home_tcp["rx"], home_tcp["ry"], home_tcp["rz"])

    radii = [float(p.strip()) for p in str(args.xy_radii_mm).split(",") if p.strip()]
    samples = xy_samples(radii, max(1, int(args.n_phi)))
    zs_mm: list[float] = []
    z = z_max_mm
    while z + 1.0e-6 >= z_min_mm:
        zs_mm.append(round(z, 3))
        z -= float(args.z_step_mm)

    print(
        f"DLP nadir search Z={zs_mm[0]:g}..{zs_mm[-1]:g} mm "
        f"(workspace top={1000.0*top_m:.0f}) "
        f"XY r={radii} mm n_phi={args.n_phi} "
        f"home XY=({1000*home_tcp['x']:.1f},{1000*home_tcp['y']:.1f}) "
        f"home Z={1000*home_tcp['z']:.1f} mm "
        f"home tool_z=({home_z[0]:.3f},{home_z[1]:.3f},{home_z[2]:.3f}) "
        f"camera_up=({camera_up[0]:.3f},{camera_up[1]:.3f},{camera_up[2]:.3f})",
        flush=True,
    )

    best: dict[str, Any] | None = None
    tried = 0
    t_all = time.time()
    for z_mm in zs_mm:
        z_m = z_mm / 1000.0
        # Expand radius at this height; first hit is closest-to-center at this Z.
        offset = 0
        for r_mm in radii:
            chunk = xy_samples([r_mm], max(1, int(args.n_phi)))
            poses = [
                look_down_pose(offset + i, x, y, z_m, camera_up)
                for i, (x, y) in enumerate(chunk)
            ]
            offset += len(poses)
            t0 = time.time()
            try:
                results = plan_batch(
                    args.base_url, poses, cfg, workspace, args.plan_timeout_s
                )
            except Exception as exc:  # noqa: BLE001
                print(
                    f"Z{z_mm:g} r={r_mm:g} ERROR {exc} ({time.time()-t0:.1f}s)",
                    flush=True,
                )
                continue
            tried += len(poses)
            hits = pick_ok(poses, results)
            dt = time.time() - t0
            if hits:
                best = hits[0]
                print(
                    f"Z{z_mm:g} r={r_mm:g} HIT xy={best['xy_mm']:.1f} mm "
                    f"z={best['z_mm']:.1f} mm ({dt:.1f}s)",
                    flush=True,
                )
                break
            n_reach = sum(1 for r in results if r.get("reachable"))
            print(
                f"Z{z_mm:g} r={r_mm:g} none reach={n_reach}/{len(poses)} ({dt:.1f}s)",
                flush=True,
            )
        if best is not None:
            break

    out = {
        "ok": best is not None,
        "seconds": round(time.time() - t_all, 1),
        "tried": tried,
        "request": {
            "tool_z": [0.0, 0.0, -1.0],
            "z_mm": zs_mm,
            "xy_radii_mm": radii,
            "n_phi": int(args.n_phi),
            "workspace_top_mm": 1000.0 * top_m,
            "home_tcp_mm": {
                "x": 1000.0 * home_tcp["x"],
                "y": 1000.0 * home_tcp["y"],
                "z": 1000.0 * home_tcp["z"],
            },
        },
        "pose": best,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    if best is None:
        print(f"No look-down pose. tried={tried} -> {args.out}", flush=True)
        return 1
    print(
        f"Best DLP look-down: XY r={best['xy_mm']:.1f} mm "
        f"({1000*best['x_m']:.1f}, {1000*best['y_m']:.1f}) "
        f"Z={best['z_mm']:.1f} mm joints_deg={['%.1f' % v for v in best['joints_deg']]} "
        f"-> {args.out}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
