# Plan home-pose Z=R apex via MoveIt and insert it into saved semi ring JSONs.
# Backend helper. One IK per unique radius; does not start motion.

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from batch_semi_ring_search import (
    build_apex_pose,
    fetch_home_tcp,
    http_json,
    parse_cfg,
    plan_fingerprint,
    robot_cfg_fingerprint,
    wait_connected,
    _point_from_result,
)


def workspace_from_cfg(cfg: dict) -> dict:
    return {
        "enabled": bool(cfg.get("workspace_boundary_enabled", True)),
        "length_m": float(cfg.get("workspace_length_mm", 900.0)) / 1000.0,
        "width_m": float(cfg.get("workspace_width_mm", 600.0)) / 1000.0,
        "height_m": float(cfg.get("workspace_height_mm", 540.0)) / 1000.0,
        "mount_height_m": float(cfg.get("ceiling_mount_height_mm", 640.0)) / 1000.0,
        "ceiling_clearance_m": float(cfg.get("workspace_ceiling_clearance_mm", 40.0))
        / 1000.0,
    }


def plan_apex(base: str, cfg: dict, home_tcp: dict, radius_m: float) -> dict:
    pose = build_apex_pose(radius_m, home_tcp, cfg)
    home = cfg.get("home_joints_deg") or [90.0, -180.0, 145.0, -55.0, 90.0, -90.0]
    body = {
        "poses": [pose],
        "workspace": workspace_from_cfg(cfg),
        "pin_pose_tolerance_deg": 0.0,
        "scan_camera_up_world_z": bool(cfg.get("scan_camera_up_world_z", True)),
        "home_joints_deg": [float(v) for v in home],
    }
    resp = http_json("POST", f"{base}/plan_hemisphere_scan", body, timeout=7200.0)
    results = resp.get("results") if isinstance(resp, dict) else None
    if not isinstance(results, list) or not resp.get("ok", False):
        raise RuntimeError(f"R{int(round(radius_m * 1000))} plan not ok: {resp}")
    res = results[0] if results else {}
    if not res.get("reachable") or not res.get("home_path_ok", True):
        raise RuntimeError(
            f"R{int(round(radius_m * 1000))} apex unreachable: {res.get('error')}"
        )
    if len(res.get("joints") or []) != 6:
        raise RuntimeError(f"R{int(round(radius_m * 1000))} apex missing joints")
    point = _point_from_result(pose, res)
    # Planned TCP is world-frame (MoveIt). Keep grid in that same frame.
    tcp = point.get("tcp") if isinstance(point.get("tcp"), dict) else {}
    grid = point.get("grid") if isinstance(point.get("grid"), dict) else {}
    if tcp and grid:
        grid["x_m"] = float(tcp.get("x_m", grid.get("x_m", 0.0)))
        grid["y_m"] = float(tcp.get("y_m", grid.get("y_m", 0.0)))
        grid["z_m"] = float(tcp.get("z_m", grid.get("z_m", 0.0)))
        point["grid"] = grid
    point["base_sweep_ok"] = True
    point["no_pan"] = True
    return point


def strip_apex(points: list) -> list:
    out = []
    for pt in points:
        if not isinstance(pt, dict):
            continue
        grid = pt.get("grid") if isinstance(pt.get("grid"), dict) else {}
        if abs(float(grid.get("theta_deg", 99.0))) < 0.75:
            continue
        out.append(pt)
    return out


def radius_from_plan(path: Path, root: dict) -> float:
    params = root.get("scan_params") if isinstance(root.get("scan_params"), dict) else {}
    if "sphere_radius_m" in params:
        return float(params["sphere_radius_m"])
    name = path.stem
    if name.startswith("R") and "_T" in name:
        return float(name.split("_T", 1)[0][1:]) / 1000.0
    raise RuntimeError(f"cannot read radius from {path.name}")


def theta_from_plan(path: Path, root: dict) -> float:
    params = root.get("scan_params") if isinstance(root.get("scan_params"), dict) else {}
    if "theta_max_deg" in params:
        return float(params["theta_max_deg"])
    name = path.stem
    if "_T" in name:
        return float(name.split("_T", 1)[1])
    return 20.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cfg",
        type=Path,
        default=Path(__file__).resolve().parents[3] / "preset" / "hyperfusion.cfg",
    )
    parser.add_argument("--base-url", default="http://127.0.0.1:8767")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parents[3]
        / "preset"
        / "mvs_semi_scan_plans"
        / "fpp_scanning",
    )
    parser.add_argument("--connect-timeout-s", type=float, default=900.0)
    args = parser.parse_args()

    cfg = parse_cfg(args.cfg)
    home = cfg.get("home_joints_deg") or [90.0, -180.0, 145.0, -55.0, 90.0, -90.0]
    cfg["home_joints_deg"] = [float(v) for v in home]

    wait_connected(args.base_url, timeout_s=float(args.connect_timeout_s))
    home_tcp = fetch_home_tcp(args.base_url)
    print(
        f"home_tcp x={home_tcp['x']:.4f} y={home_tcp['y']:.4f} z={home_tcp['z']:.4f}",
        flush=True,
    )

    plans = sorted(args.out_dir.glob("R*.json"))
    if not plans:
        print(f"no R*.json in {args.out_dir}")
        return 1

    by_radius: dict[float, list[Path]] = {}
    for path in plans:
        root = json.loads(path.read_text(encoding="utf-8"))
        r = round(radius_from_plan(path, root), 6)
        by_radius.setdefault(r, []).append(path)

    n_phi = int(cfg.get("semi_ring_search_candidates", 72))
    cached: dict[float, dict] = {}
    failed = 0
    updated = 0
    for radius_m in sorted(by_radius):
        tag = f"R{int(round(radius_m * 1000))}"
        try:
            cached[radius_m] = plan_apex(args.base_url, cfg, home_tcp, radius_m)
            j = cached[radius_m]["joints_rad"]
            print(
                f"{tag} apex ok Z={cached[radius_m]['tcp']['z_m']:.3f} "
                f"xy=({cached[radius_m]['tcp']['x_m']:.4f},"
                f"{cached[radius_m]['tcp']['y_m']:.4f}) "
                f"joints_deg={[round(__import__('math').degrees(v), 1) for v in j]}",
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"{tag} apex FAILED {exc}", flush=True)
            continue

        for path in by_radius[radius_m]:
            root = json.loads(path.read_text(encoding="utf-8"))
            theta = theta_from_plan(path, root)
            rings = strip_apex(list(root.get("points") or []))
            points = [dict(cached[radius_m])] + rings
            root["points"] = points
            root["fingerprint"] = plan_fingerprint(radius_m, theta, n_phi)
            root["robot_cfg_fingerprint"] = robot_cfg_fingerprint(cfg)
            params = root.get("scan_params")
            if not isinstance(params, dict):
                params = {}
            params["sphere_radius_m"] = radius_m
            params["apex_radius_m"] = radius_m
            params["theta_min_deg"] = 0.0
            root["scan_params"] = params
            root["reachable_count"] = sum(1 for p in points if p.get("reachable"))
            root["unreachable_count"] = sum(1 for p in points if not p.get("reachable"))
            root["home_path_ok_count"] = sum(1 for p in points if p.get("home_path_ok"))
            path.write_text(json.dumps(root, indent=2) + "\n", encoding="utf-8")
            updated += 1
            rel = (
                Path(__file__).resolve().parents[3]
                / "build"
                / "Release"
                / "mvs_semi_scan_plans"
                / "fpp_scanning"
                / path.name
            )
            if rel.parent.is_dir():
                rel.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")

    print(f"Done updated={updated}/{len(plans)} failed_radii={failed}", flush=True)
    return 0 if failed == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
