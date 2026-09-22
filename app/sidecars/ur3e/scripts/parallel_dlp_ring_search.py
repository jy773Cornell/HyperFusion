#!/usr/bin/env python3
"""Search each DLP hemisphere theta with multiple MoveIt sidecars in parallel.

All workers finish their share of one theta before the search advances to the
next theta. Planning only; this script never commands robot motion.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

from batch_semi_ring_search import (
    build_cylinder_poses,
    build_ring_poses,
    cylinder_fingerprint,
    fetch_home_tcp,
    http_json,
    parse_cfg,
    ring_coverage_deg,
    save_one_ring_plan,
    wait_connected,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _workspace(cfg: dict[str, Any]) -> dict[str, Any]:
    return {
        "enabled": bool(cfg.get("workspace_boundary_enabled", True)),
        "length_m": float(cfg.get("workspace_length_mm", 900.0)) / 1000.0,
        "width_m": float(cfg.get("workspace_width_mm", 600.0)) / 1000.0,
        "height_m": float(cfg.get("workspace_height_mm", 788.0)) / 1000.0,
        "mount_height_m": float(cfg.get("ceiling_mount_height_mm", 888.0)) / 1000.0,
        "ceiling_clearance_m": float(
            cfg.get("workspace_ceiling_clearance_mm", 20.0)
        )
        / 1000.0,
    }


def _plan_subset(
    base_url: str,
    poses: list[dict[str, Any]],
    *,
    cfg: dict[str, Any],
    workspace: dict[str, Any],
    home_joints_deg: list[float],
    n_phi: int,
    timeout_s: float,
) -> list[dict[str, Any]]:
    body = {
        "poses": poses,
        "workspace": workspace,
        "pin_pose_tolerance_deg": float(cfg.get("pin_pose_tolerance_deg", 0.0)),
        "scan_camera_up_world_z": bool(cfg.get("scan_camera_up_world_z", True)),
        "semi_ring_sweep": True,
        "semi_max_sweep_ok_per_ring": 1,
        "semi_ring_search_candidates": n_phi,
        # This search only accepts a full 360-degree sweep.
        "semi_backup_coverage_deg": 0.0,
        "home_joints_deg": home_joints_deg,
    }
    response = http_json(
        "POST",
        f"{base_url}/plan_hemisphere_scan",
        body=body,
        timeout=timeout_s,
    )
    results = response.get("results") if isinstance(response, dict) else None
    if not isinstance(results, list) or not response.get("ok", False):
        raise RuntimeError(f"plan not ok from {base_url}: {response}")
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cfg",
        type=Path,
        default=_repo_root() / "app" / "preset" / "hyperfusion.cfg",
    )
    parser.add_argument("--base-urls", default="http://127.0.0.1:8771,http://127.0.0.1:8772")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=_repo_root()
        / "app"
        / "preset"
        / "mvs_scan_plans"
        / "fpp"
        / "_parallel_dlp_ring_search",
    )
    parser.add_argument("--radius-mm", type=float, default=450.0)
    parser.add_argument(
        "--fixed-z-mm",
        type=float,
        default=0.0,
        help="Use a constant TCP height and derive XY radius from each theta.",
    )
    parser.add_argument("--theta-min", type=int, default=10)
    parser.add_argument("--theta-max", type=int, default=60)
    parser.add_argument("--n-phi", type=int, default=12)
    parser.add_argument(
        "--home-joints-deg",
        default="",
        help="Path hub / effort reference as six comma-separated joint degrees.",
    )
    parser.add_argument(
        "--phi-center-deg",
        type=float,
        default=180.0,
        help="Test azimuths nearest this angle first.",
    )
    parser.add_argument("--plan-timeout-s", type=float, default=1800.0)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    cfg = parse_cfg(args.cfg)
    urls = [value.strip() for value in args.base_urls.split(",") if value.strip()]
    if len(urls) < 2:
        raise SystemExit("--base-urls needs at least two sidecars")
    for url in urls:
        wait_connected(url, timeout_s=360.0)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    progress_path = args.out_dir / "_progress.json"
    progress: dict[str, Any] = {}
    if args.resume and progress_path.is_file():
        progress = json.loads(progress_path.read_text(encoding="utf-8"))

    radius_m = args.radius_mm / 1000.0
    n_phi = max(len(urls), int(args.n_phi))
    tilt_deg = float(cfg.get("pin_tcp_tilt_deg", 0.0))
    scan_tcp = str(cfg.get("scan_tcp", "camera")).strip().lower()
    if scan_tcp not in ("dlp", "projector"):
        scan_tcp = "camera"
    # DLP yaw≈180 uses +Z-up for camera-bottom / projector-up.
    # Camera TCP yaw≈0 needs −Z-up for the same payload roll.
    camera_up = (0.0, 0.0, 1.0) if scan_tcp == "dlp" else (0.0, 0.0, -1.0)
    workspace = _workspace(cfg)
    home = [
        float(value)
        for value in (
            cfg.get("home_joints_deg")
            or [138.50, 19.42, -150.22, 40.86, 89.60, 221.30]
        )
    ]
    if args.home_joints_deg:
        home = [
            float(value.strip())
            for value in args.home_joints_deg.split(",")
            if value.strip()
        ]
        if len(home) != 6:
            raise SystemExit("--home-joints-deg needs six values")
    home_tcp = fetch_home_tcp(urls[0])

    geometry = (
        f"Z={args.fixed_z_mm:g} mm"
        if args.fixed_z_mm > 0.0
        else f"R={args.radius_mm:g} mm"
    )
    print(
        f"Parallel ring search: workers={len(urls)} {geometry} "
        f"T={args.theta_min}..{args.theta_max} n_phi={n_phi} "
        f"scan_tcp={scan_tcp} tcp_tilt={tilt_deg:g}° "
        f"camera_up_z={camera_up[2]:g} full_sweep_only=true",
        flush=True,
    )

    saved = 0
    for theta in range(args.theta_min, args.theta_max + 1):
        fixed_height = args.fixed_z_mm > 0.0
        if fixed_height:
            z_m = args.fixed_z_mm / 1000.0
            rho_m = z_m * math.tan(math.radians(float(theta)))
            key = f"Z{int(round(args.fixed_z_mm))}_T{theta}"
        else:
            rho_m = radius_m
            z_m = 0.0
            key = f"R{int(round(args.radius_mm))}_T{theta}"
        if args.resume and key in progress:
            print(f"{key} resume-skip", flush=True)
            continue

        poses = (
            build_cylinder_poses(rho_m, z_m, n_phi, tilt_deg, up=camera_up)
            if fixed_height
            else build_ring_poses(
                radius_m, float(theta), n_phi, tilt_deg, up=camera_up
            )
        )
        poses.sort(
            key=lambda pose: abs(
                ((float(pose["phi_deg"]) - args.phi_center_deg + 180.0) % 360.0)
                - 180.0
            )
        )
        subsets = [poses[index:: len(urls)] for index in range(len(urls))]
        started = time.time()
        results: list[dict[str, Any]] = []
        errors: list[str] = []
        with ThreadPoolExecutor(max_workers=len(urls)) as executor:
            futures = {
                executor.submit(
                    _plan_subset,
                    url,
                    subset,
                    cfg=cfg,
                    workspace=workspace,
                    home_joints_deg=home,
                    n_phi=n_phi,
                    timeout_s=args.plan_timeout_s,
                ): url
                for url, subset in zip(urls, subsets)
            }
            for future in as_completed(futures):
                url = futures[future]
                try:
                    results.extend(future.result())
                except Exception as exc:  # noqa: BLE001 - preserve other worker result
                    errors.append(f"{url}: {exc}")

        elapsed = time.time() - started
        if errors:
            progress[key] = {
                "ok": False,
                "errors": errors,
                "seconds": round(elapsed, 2),
            }
            print(f"{key} ERROR {'; '.join(errors)} ({elapsed:.1f}s)", flush=True)
        else:
            coverage = ring_coverage_deg(poses, results)
            if coverage == 360:
                out_path = args.out_dir / f"{key}_360.json"
                sweep_results = [
                    result
                    for result in results
                    if result.get("reachable")
                    and result.get("home_path_ok", True)
                    and result.get("base_sweep_ok")
                    and len(result.get("joints") or []) == 6
                ]

                def effort_deg(result: dict[str, Any]) -> float:
                    total = 0.0
                    for joint_rad, reference_deg in zip(result["joints"], home):
                        delta = math.degrees(float(joint_rad)) - reference_deg
                        delta = (delta + 180.0) % 360.0 - 180.0
                        total += delta * delta
                    return math.sqrt(total)

                best_result = min(sweep_results, key=effort_deg)
                best_index = int(best_result["index"])
                best_pose = next(
                    pose for pose in poses if int(pose["index"]) == best_index
                )
                fingerprint = None
                scan_params = None
                save_radius_m = radius_m
                save_home_tcp = home_tcp
                if fixed_height:
                    save_radius_m = rho_m
                    save_home_tcp = None
                    fingerprint = cylinder_fingerprint(rho_m, z_m, n_phi)
                    scan_params = {
                        "cyl_radius_m": rho_m,
                        "z_m": z_m,
                        "horizontal_points": n_phi,
                        "look_from_vertical_deg": float(theta),
                    }
                _, _, ring_count = save_one_ring_plan(
                    out_path,
                    cfg=cfg,
                    radius_m=save_radius_m,
                    theta_deg=float(theta),
                    n_phi=n_phi,
                    poses=poses,
                    results=[best_result],
                    home_tcp=save_home_tcp,
                    plan_name=f"{key}_360",
                    fingerprint=fingerprint,
                    scan_params=scan_params,
                )
                saved += 1
                progress[key] = {
                    "ok": True,
                    "sweep_ok": ring_count,
                    "best_phi_deg": float(best_pose["phi_deg"]),
                    "joint_effort_deg": round(effort_deg(best_result), 3),
                    "file": str(out_path),
                    "seconds": round(elapsed, 2),
                }
                print(
                    f"{key} HIT 360° phi={float(best_pose['phi_deg']):g}° "
                    f"effort={effort_deg(best_result):.1f}° ({elapsed:.1f}s)",
                    flush=True,
                )
            else:
                progress[key] = {
                    "ok": True,
                    "sweep_ok": 0,
                    "seconds": round(elapsed, 2),
                }
                print(f"{key} none ({elapsed:.1f}s)", flush=True)
        progress_path.write_text(json.dumps(progress, indent=2) + "\n", encoding="utf-8")

    print(f"Done: saved={saved} tested={len(progress)}", flush=True)
    return 0 if all(entry.get("ok") for entry in progress.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
