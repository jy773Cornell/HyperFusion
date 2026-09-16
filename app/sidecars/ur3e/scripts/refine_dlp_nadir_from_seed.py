#!/usr/bin/env python3
"""Refine UR3e joints near a seed so DLP TCP +Z is world look-down (stage-perpendicular).

Local FK matches Ur3eHomeScanCenter.cpp (ceiling mount + active scan TCP).
No motion. Optional MoveIt check via /plan_hemisphere_scan.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

from batch_semi_ring_search import (
    http_json,
    parse_cfg,
    rotvec_to_tool_z,
    tool_z_to_rotation_vector,
    wait_connected,
)

PI = math.pi


def mat_i() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
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


def mat_translate(x: float, y: float, z: float) -> list[list[float]]:
    t = mat_i()
    t[0][3], t[1][3], t[2][3] = x, y, z
    return t


def mat_rot_z(a: float) -> list[list[float]]:
    c, s = math.cos(a), math.sin(a)
    t = mat_i()
    t[0][0], t[0][1] = c, -s
    t[1][0], t[1][1] = s, c
    return t


def mat_rpy_xyz(roll: float, pitch: float, yaw: float, x: float, y: float, z: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    t = mat_i()
    t[0][0] = cy * cp
    t[0][1] = cy * sp * sr - sy * cr
    t[0][2] = cy * sp * cr + sy * sr
    t[0][3] = x
    t[1][0] = sy * cp
    t[1][1] = sy * sp * sr + cy * cr
    t[1][2] = sy * sp * cr - cy * sr
    t[1][3] = y
    t[2][0] = -sp
    t[2][1] = cp * sr
    t[2][2] = cp * cr
    t[2][3] = z
    return t


def dlp_world_from_joints(q: list[float], tcp: dict[str, float], mount: dict[str, float]) -> list[list[float]]:
    """World T of DLP optical TCP. q in radians."""
    t = mat_rpy_xyz(0.0, 0.0, PI, 0.0, 0.0, 0.0)
    t = mat_mul(t, mat_mul(mat_translate(0.0, 0.0, 0.15185), mat_rot_z(q[0])))
    t = mat_mul(t, mat_mul(mat_rpy_xyz(PI * 0.5, 0.0, 0.0, 0.0, 0.0, 0.0), mat_rot_z(q[1])))
    t = mat_mul(t, mat_mul(mat_translate(-0.24355, 0.0, 0.0), mat_rot_z(q[2])))
    t = mat_mul(t, mat_mul(mat_translate(-0.2132, 0.0, 0.13105), mat_rot_z(q[3])))
    t = mat_mul(t, mat_mul(mat_rpy_xyz(PI * 0.5, 0.0, 0.0, 0.0, -0.08535, 0.0), mat_rot_z(q[4])))
    t = mat_mul(t, mat_mul(mat_rpy_xyz(PI * 0.5, PI, PI, 0.0, 0.0921, 0.0), mat_rot_z(q[5])))
    t = mat_mul(t, mat_rpy_xyz(0.0, -PI * 0.5, -PI * 0.5, 0.0, 0.0, 0.0))
    t = mat_mul(t, mat_rpy_xyz(PI * 0.5, 0.0, PI * 0.5, 0.0, 0.0, 0.0))
    t = mat_mul(
        t,
        mat_rpy_xyz(
            math.radians(tcp["roll"]),
            math.radians(tcp["pitch"]),
            math.radians(tcp["yaw"]),
            tcp["x"] * 0.001,
            tcp["y"] * 0.001,
            tcp["z"] * 0.001,
        ),
    )
    world = mat_rpy_xyz(
        math.radians(mount["roll"]),
        math.radians(mount["pitch"]),
        math.radians(mount["yaw"]),
        mount["x"] * 0.001,
        mount["y"] * 0.001,
        mount["z"] * 0.001,
    )
    return mat_mul(world, t)


def pose_from_t(t: list[list[float]], *, stage_along_base_z: bool = True) -> dict[str, float]:
    """DLP pose. MoveIt / GET /pose are base_link: stage is +Z (ceiling mount)."""
    zx, zy, zz = t[0][2], t[1][2], t[2][2]
    n = math.sqrt(zx * zx + zy * zy + zz * zz) or 1.0
    zx, zy, zz = zx / n, zy / n, zz / n
    # Ceiling UR: base +Z points at the tray. World look-down is base +Z.
    align = zz if stage_along_base_z else -zz
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, align))))
    return {
        "x_mm": 1000.0 * t[0][3],
        "y_mm": 1000.0 * t[1][3],
        "z_mm": 1000.0 * t[2][3],
        "tool_z": [zx, zy, zz],
        "tilt_from_nadir_deg": tilt,
        "xy_mm": math.hypot(1000.0 * t[0][3], 1000.0 * t[1][3]),
    }


def wrap_deg(d: float) -> float:
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def cost(q: list[float], seed: list[float], tcp: dict[str, float], mount: dict[str, float]) -> float:
    p = pose_from_t(dlp_world_from_joints(q, tcp, mount))
    # Stay on this arm fold: penalize q0/q1/q2 more than wrists.
    w = (8.0, 4.0, 4.0, 1.0, 1.0, 0.4)
    jpen = sum(w[i] * (q[i] - seed[i]) ** 2 for i in range(6))
    return p["tilt_from_nadir_deg"] ** 2 + 40.0 * jpen


def _in_limits(q: list[float]) -> bool:
    # UR3e URDF: pan/lift/w1/w2 ±360°, elbow ±180°.
    lim = (
        (math.radians(-350.0), math.radians(350.0)),
        (math.radians(-350.0), math.radians(350.0)),
        (math.radians(-171.0), math.radians(171.0)),
        (math.radians(-350.0), math.radians(350.0)),
        (math.radians(-350.0), math.radians(350.0)),
        (math.radians(-350.0), math.radians(350.0)),
    )
    return all(lo <= q[i] <= hi for i, (lo, hi) in enumerate(lim))


def wrists_for_nadir(
    q: list[float],
    tcp: dict[str, float],
    mount: dict[str, float],
    *,
    jpen_w: float = 4.0,
) -> list[float]:
    """Keep pan/lift/elbow; solve wrists for optical +Z along base +Z."""
    seed = list(q)

    def c(w: list[float]) -> float:
        qq = [q[0], q[1], q[2], w[0], w[1], w[2]]
        p = pose_from_t(dlp_world_from_joints(qq, tcp, mount))
        jpen = sum((w[i] - seed[3 + i]) ** 2 for i in range(3))
        return p["tilt_from_nadir_deg"] ** 2 + jpen_w * jpen

    n = 3
    step = [math.radians(6.0)] * 3
    simp = [list(seed[3:6])]
    for i in range(n):
        v = list(seed[3:6])
        v[i] += step[i]
        simp.append(v)
    scores = [c(v) for v in simp]
    for _ in range(160):
        order = sorted(range(n + 1), key=lambda i: scores[i])
        simp = [simp[i] for i in order]
        scores = [scores[i] for i in order]
        if scores[0] < 1.0e-4:
            break
        centroid = [sum(simp[i][j] for i in range(n)) / n for j in range(n)]
        worst = simp[n]
        refl = [centroid[j] + (centroid[j] - worst[j]) for j in range(n)]
        rscore = c(refl)
        if scores[0] <= rscore < scores[n - 1]:
            simp[n], scores[n] = refl, rscore
            continue
        if rscore < scores[0]:
            expd = [centroid[j] + 2.0 * (refl[j] - centroid[j]) for j in range(n)]
            es = c(expd)
            simp[n], scores[n] = (expd, es) if es < rscore else (refl, rscore)
            continue
        contr = [centroid[j] + 0.5 * (worst[j] - centroid[j]) for j in range(n)]
        cs = c(contr)
        if cs < scores[n]:
            simp[n], scores[n] = contr, cs
            continue
        best = simp[0]
        for i in range(1, n + 1):
            simp[i] = [best[j] + 0.5 * (simp[i][j] - best[j]) for j in range(n)]
            scores[i] = c(simp[i])
    return [q[0], q[1], q[2], simp[0][0], simp[0][1], simp[0][2]]


def wrists_for_nadir_multiseed(
    q: list[float], tcp: dict[str, float], mount: dict[str, float]
) -> list[float]:
    """Try wrist flips so camera/DLP can leave a 25° local minimum."""
    w1, w2, w3 = q[3], q[4], q[5]
    guesses = [
        (w1, w2, w3),
        (w1, w2 + PI, w3),
        (w1, w2, w3 + PI),
        (w1, w2 + PI, w3 + PI),
        (w1 + math.radians(25.0), w2, w3),
        (w1 - math.radians(25.0), w2, w3),
        (w1, w2 + math.radians(90.0), w3),
        (w1, w2 - math.radians(90.0), w3),
    ]
    best = list(q)
    best_tilt = 1.0e9
    for g in guesses:
        qg = [q[0], q[1], q[2], g[0], g[1], g[2]]
        got = wrists_for_nadir(qg, tcp, mount, jpen_w=0.15)
        p = pose_from_t(dlp_world_from_joints(got, tcp, mount))
        if p["tilt_from_nadir_deg"] < best_tilt:
            best_tilt = p["tilt_from_nadir_deg"]
            best = got
    return best


def raise_on_fold(
    seed: list[float],
    tcp: dict[str, float],
    mount: dict[str, float],
    *,
    xy_keep_mm: float,
    min_base_z_mm: float,
    max_xy_mm: tuple[float, float],
) -> tuple[list[float], dict[str, float]]:
    """Same look-down fold; search lift/elbow for the smallest base_link Z (highest in the room)."""
    best_q = list(seed)
    best_p = pose_from_t(dlp_world_from_joints(best_q, tcp, mount))
    seed_xy = (best_p["x_mm"], best_p["y_mm"])
    # Ceiling mount: smaller base Z = closer to the robot base / higher off the tray.
    best_score = best_p["z_mm"]
    lift0 = math.degrees(seed[1])
    el0 = math.degrees(seed[2])
    for lift_deg in [lift0 + 2.5 * i for i in range(-14, 15)]:
        for el_deg in [el0 + 2.5 * i for i in range(-14, 15)]:
            q0 = [
                seed[0],
                math.radians(lift_deg),
                math.radians(el_deg),
                seed[3],
                seed[4],
                seed[5],
            ]
            if not _in_limits(q0):
                continue
            q = wrists_for_nadir(q0, tcp, mount)
            if not _in_limits(q):
                continue
            p = pose_from_t(dlp_world_from_joints(q, tcp, mount))
            if p["tilt_from_nadir_deg"] > 0.5:
                continue
            if p["z_mm"] < min_base_z_mm:
                continue
            if abs(p["x_mm"]) > max_xy_mm[0] or abs(p["y_mm"]) > max_xy_mm[1]:
                continue
            if math.hypot(p["x_mm"] - seed_xy[0], p["y_mm"] - seed_xy[1]) > xy_keep_mm:
                continue
            if p["z_mm"] + 1.0e-6 < best_score:
                best_score = p["z_mm"]
                best_q, best_p = q, p
    # Fine polish: allow small pan/lift/elbow + wrists, still look-down, still high.
    def raise_cost(q: list[float]) -> float:
        if not _in_limits(q):
            return 1.0e9
        p = pose_from_t(dlp_world_from_joints(q, tcp, mount))
        if p["z_mm"] < min_base_z_mm:
            return 1.0e9
        if abs(p["x_mm"]) > max_xy_mm[0] or abs(p["y_mm"]) > max_xy_mm[1]:
            return 1.0e9
        xy = math.hypot(p["x_mm"] - seed_xy[0], p["y_mm"] - seed_xy[1])
        return (
            80.0 * p["tilt_from_nadir_deg"] ** 2
            + p["z_mm"]
            + 0.05 * xy * xy
            + 400.0 * (q[0] - seed[0]) ** 2
        )

    n = 6
    step = [math.radians(v) for v in (0.2, 3.0, 3.0, 4.0, 4.0, 4.0)]
    simp = [list(best_q)]
    for i in range(n):
        v = list(best_q)
        v[i] += step[i]
        simp.append(v)
    scores = [raise_cost(v) for v in simp]
    for _ in range(220):
        order = sorted(range(n + 1), key=lambda i: scores[i])
        simp = [simp[i] for i in order]
        scores = [scores[i] for i in order]
        centroid = [sum(simp[i][j] for i in range(n)) / n for j in range(n)]
        worst = simp[n]
        refl = [centroid[j] + (centroid[j] - worst[j]) for j in range(n)]
        rscore = raise_cost(refl)
        if scores[0] <= rscore < scores[n - 1]:
            simp[n], scores[n] = refl, rscore
            continue
        if rscore < scores[0]:
            expd = [centroid[j] + 2.0 * (refl[j] - centroid[j]) for j in range(n)]
            es = raise_cost(expd)
            simp[n], scores[n] = (expd, es) if es < rscore else (refl, rscore)
            continue
        contr = [centroid[j] + 0.5 * (worst[j] - centroid[j]) for j in range(n)]
        cs = raise_cost(contr)
        if cs < scores[n]:
            simp[n], scores[n] = contr, cs
            continue
        best = simp[0]
        for i in range(1, n + 1):
            simp[i] = [best[j] + 0.5 * (simp[i][j] - best[j]) for j in range(n)]
            scores[i] = raise_cost(simp[i])
    q = simp[0]
    q[0] = seed[0]
    q = wrists_for_nadir(q, tcp, mount)
    return q, pose_from_t(dlp_world_from_joints(q, tcp, mount))


def aim_height_on_fold(
    seed: list[float],
    tcp: dict[str, float],
    mount: dict[str, float],
    *,
    target_world_z_mm: float,
    mount_height_mm: float,
    xy_keep_mm: float,
    min_base_z_mm: float,
    max_xy_mm: tuple[float, float],
    lock_y_mm: float | None = None,
) -> tuple[list[float], dict[str, float]]:
    """Same look-down fold; match DLP height above tray (world Z)."""
    target_base_z = mount_height_mm - target_world_z_mm
    best_q = list(seed)
    best_p = pose_from_t(dlp_world_from_joints(best_q, tcp, mount))
    seed_xy = (best_p["x_mm"], best_p["y_mm"])
    best_err = abs(best_p["z_mm"] - target_base_z) + 20.0 * best_p["tilt_from_nadir_deg"]
    lift0 = math.degrees(seed[1])
    el0 = math.degrees(seed[2])
    for lift_deg in [lift0 + 1.5 * i for i in range(-12, 13)]:
        for el_deg in [el0 + 1.5 * i for i in range(-12, 13)]:
            q0 = [
                seed[0],
                math.radians(lift_deg),
                math.radians(el_deg),
                seed[3],
                seed[4],
                seed[5],
            ]
            if not _in_limits(q0):
                continue
            q = wrists_for_nadir(q0, tcp, mount)
            if not _in_limits(q):
                continue
            p = pose_from_t(dlp_world_from_joints(q, tcp, mount))
            if p["tilt_from_nadir_deg"] > 0.5:
                continue
            if p["z_mm"] < min_base_z_mm:
                continue
            if abs(p["x_mm"]) > max_xy_mm[0] or abs(p["y_mm"]) > max_xy_mm[1]:
                continue
            if math.hypot(p["x_mm"] - seed_xy[0], p["y_mm"] - seed_xy[1]) > xy_keep_mm:
                continue
            err = abs(p["z_mm"] - target_base_z) + 8.0 * p["tilt_from_nadir_deg"]
            if err + 1.0e-6 < best_err:
                best_err = err
                best_q, best_p = q, p

    def aim_cost(q: list[float]) -> float:
        if not _in_limits(q):
            return 1.0e9
        p = pose_from_t(dlp_world_from_joints(q, tcp, mount))
        if p["z_mm"] < min_base_z_mm:
            return 1.0e9
        xy = math.hypot(p["x_mm"] - seed_xy[0], p["y_mm"] - seed_xy[1])
        world_z = mount_height_mm - p["z_mm"]
        ypen = 0.0
        if lock_y_mm is not None:
            ypen = 0.8 * (p["y_mm"] - lock_y_mm) ** 2
        return (
            80.0 * p["tilt_from_nadir_deg"] ** 2
            + (world_z - target_world_z_mm) ** 2
            + 0.04 * xy * xy
            + ypen
            + 400.0 * (q[0] - seed[0]) ** 2
        )

    n = 6
    step = [math.radians(v) for v in (0.2, 2.0, 2.0, 3.0, 3.0, 3.0)]
    simp = [list(best_q)]
    for i in range(n):
        v = list(best_q)
        v[i] += step[i]
        simp.append(v)
    scores = [aim_cost(v) for v in simp]
    for _ in range(200):
        order = sorted(range(n + 1), key=lambda i: scores[i])
        simp = [simp[i] for i in order]
        scores = [scores[i] for i in order]
        centroid = [sum(simp[i][j] for i in range(n)) / n for j in range(n)]
        worst = simp[n]
        refl = [centroid[j] + (centroid[j] - worst[j]) for j in range(n)]
        rscore = aim_cost(refl)
        if scores[0] <= rscore < scores[n - 1]:
            simp[n], scores[n] = refl, rscore
            continue
        if rscore < scores[0]:
            expd = [centroid[j] + 2.0 * (refl[j] - centroid[j]) for j in range(n)]
            es = aim_cost(expd)
            simp[n], scores[n] = (expd, es) if es < rscore else (refl, rscore)
            continue
        contr = [centroid[j] + 0.5 * (worst[j] - centroid[j]) for j in range(n)]
        cs = aim_cost(contr)
        if cs < scores[n]:
            simp[n], scores[n] = contr, cs
            continue
        best = simp[0]
        for i in range(1, n + 1):
            simp[i] = [best[j] + 0.5 * (simp[i][j] - best[j]) for j in range(n)]
            scores[i] = aim_cost(simp[i])
    q = simp[0]
    q[0] = seed[0]
    q = wrists_for_nadir(q, tcp, mount)
    return q, pose_from_t(dlp_world_from_joints(q, tcp, mount))


def nelder(seed: list[float], tcp: dict[str, float], mount: dict[str, float]) -> list[float]:
    n = 6
    step = [math.radians(v) for v in (2.0, 4.0, 4.0, 8.0, 8.0, 8.0)]
    simp = [list(seed)]
    for i in range(n):
        v = list(seed)
        v[i] += step[i]
        simp.append(v)
    scores = [cost(v, seed, tcp, mount) for v in simp]
    alpha, gamma, rho, sigma = 1.0, 2.0, 0.5, 0.5
    for _ in range(250):
        order = sorted(range(n + 1), key=lambda i: scores[i])
        simp = [simp[i] for i in order]
        scores = [scores[i] for i in order]
        if scores[0] < 0.01:
            break
        centroid = [sum(simp[i][j] for i in range(n)) / n for j in range(n)]
        worst = simp[n]
        refl = [centroid[j] + alpha * (centroid[j] - worst[j]) for j in range(n)]
        rscore = cost(refl, seed, tcp, mount)
        if scores[0] <= rscore < scores[n - 1]:
            simp[n], scores[n] = refl, rscore
            continue
        if rscore < scores[0]:
            expd = [centroid[j] + gamma * (refl[j] - centroid[j]) for j in range(n)]
            es = cost(expd, seed, tcp, mount)
            simp[n], scores[n] = (expd, es) if es < rscore else (refl, rscore)
            continue
        contr = [centroid[j] + rho * (worst[j] - centroid[j]) for j in range(n)]
        cs = cost(contr, seed, tcp, mount)
        if cs < scores[n]:
            simp[n], scores[n] = contr, cs
            continue
        best = simp[0]
        for i in range(1, n + 1):
            simp[i] = [best[j] + sigma * (simp[i][j] - best[j]) for j in range(n)]
            scores[i] = cost(simp[i], seed, tcp, mount)
    return simp[0]


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


def moveit_check(
    base: str,
    cfg: dict[str, Any],
    q: list[float],
    pose: dict[str, float],
) -> dict[str, Any]:
    wait_connected(base, timeout_s=60.0)
    # Planning frame is base_link: DLP +Z toward the tray = (0,0,+1).
    up = (1.0, 0.0, 0.0)
    rx, ry, rz = tool_z_to_rotation_vector(0.0, 0.0, 1.0, up=up)
    body = {
        "poses": [
            {
                "index": 0,
                "x": pose["x_mm"] / 1000.0,
                "y": pose["y_mm"] / 1000.0,
                "z": pose["z_mm"] / 1000.0,
                "rx": rx,
                "ry": ry,
                "rz": rz,
                "tool_z_x": 0.0,
                "tool_z_y": 0.0,
                "tool_z_z": 1.0,
                "camera_up_x": up[0],
                "camera_up_y": up[1],
                "camera_up_z": 0.0,
                "require_perpendicular": False,
                "theta_deg": 0.0,
                "phi_deg": 0.0,
            }
        ],
        "workspace": workspace_from_cfg(cfg),
        "pin_pose_tolerance_deg": 0.0,
        "scan_camera_up_world_z": False,
        "semi_ring_sweep": False,
        "home_joints_deg": [wrap_deg(math.degrees(v)) for v in q],
    }
    resp = http_json("POST", f"{base}/plan_hemisphere_scan", body, timeout=180.0)
    res = (resp.get("results") or [None])[0] if isinstance(resp, dict) else None
    if not isinstance(res, dict):
        return {"ok": False, "error": str(resp)}
    joints = res.get("joints") or []
    out = {
        "ok": bool(res.get("reachable") and res.get("home_path_ok", True) and len(joints) == 6),
        "reachable": bool(res.get("reachable")),
        "home_path_ok": bool(res.get("home_path_ok")),
        "error": res.get("error"),
    }
    if len(joints) == 6:
        out["joints_deg"] = [wrap_deg(math.degrees(float(v))) for v in joints]
        tcp = res.get("tcp") if isinstance(res.get("tcp"), dict) else {}
        if tcp:
            tz = rotvec_to_tool_z(float(tcp.get("rx", 0)), float(tcp.get("ry", 0)), float(tcp.get("rz", 0)))
            out["moveit_tcp_mm"] = [
                1000.0 * float(tcp.get("x", 0)),
                1000.0 * float(tcp.get("y", 0)),
                1000.0 * float(tcp.get("z", 0)),
            ]
            out["moveit_tilt_deg"] = math.degrees(math.acos(max(-1.0, min(1.0, tz[2]))))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cfg", type=Path, default=None)
    parser.add_argument("--base-url", default="http://127.0.0.1:8771")
    parser.add_argument("--joints-deg", default="90,0,-140,30,105,30")
    parser.add_argument("--raise", dest="raise_pose", action="store_true")
    parser.add_argument("--target-height-mm", type=float, default=0.0)
    parser.add_argument("--tcp", choices=("dlp", "camera"), default="dlp")
    parser.add_argument("--xy-keep-mm", type=float, default=0.0)
    parser.add_argument(
        "--lock-y-mm",
        type=float,
        default=None,
        help="Keep optical TCP Y at this base_link mm (centerline = 0).",
    )
    parser.add_argument("--skip-moveit", action="store_true")
    args = parser.parse_args()
    cfg_path = args.cfg or Path(__file__).resolve().parents[3] / "app" / "preset" / "hyperfusion.cfg"
    cfg = parse_cfg(cfg_path)
    seed_deg = [float(p.strip()) for p in str(args.joints_deg).split(",") if p.strip()]
    if len(seed_deg) != 6:
        raise SystemExit("need 6 joint degrees")
    seed = [math.radians(v) for v in seed_deg]
    if args.tcp == "camera":
        tcp = {
            "x": float(cfg.get("tool_tcp_x_mm", 0.693)),
            "y": float(cfg.get("tool_tcp_y_mm", -71.639)),
            "z": float(cfg.get("tool_tcp_z_mm", 114.199)),
            "roll": float(cfg.get("tool_tcp_roll_deg", -0.3469)),
            "pitch": float(cfg.get("tool_tcp_pitch_deg", -0.2031)),
            "yaw": float(cfg.get("tool_tcp_yaw_deg", 0.1778)),
        }
    else:
        tcp = {
            "x": float(cfg.get("dlp_tcp_x_mm", 1.4)),
            "y": float(cfg.get("dlp_tcp_y_mm", 31.743)),
            "z": float(cfg.get("dlp_tcp_z_mm", 98.538)),
            "roll": float(cfg.get("dlp_tcp_roll_deg", -25.2629)),
            "pitch": float(cfg.get("dlp_tcp_pitch_deg", 0.2395)),
            "yaw": float(cfg.get("dlp_tcp_yaw_deg", 179.9313)),
        }
    # Match GET /pose + MoveIt IK: base_link, identity mount.
    mount = {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    }
    seed_pose = pose_from_t(dlp_world_from_joints(seed, tcp, mount))
    if args.tcp == "camera":
        seed_tilt = pose_from_t(dlp_world_from_joints(seed, tcp, mount))["tilt_from_nadir_deg"]
        if seed_tilt > 5.0:
            q = wrists_for_nadir_multiseed(seed, tcp, mount)
        else:
            q = wrists_for_nadir(seed, tcp, mount)
        pose = pose_from_t(dlp_world_from_joints(q, tcp, mount))
    else:
        q = nelder(seed, tcp, mount)
        pose = pose_from_t(dlp_world_from_joints(q, tcp, mount))
    keep = float(args.xy_keep_mm) if args.xy_keep_mm > 1.0 else (200.0 if args.tcp == "camera" else 80.0)
    if args.raise_pose:
        q, pose = raise_on_fold(
            q,
            tcp,
            mount,
            xy_keep_mm=keep,
            min_base_z_mm=40.0 + float(tcp["z"]),
            max_xy_mm=(450.0, 300.0),
        )
    if args.target_height_mm > 1.0:
        q, pose = aim_height_on_fold(
            q,
            tcp,
            mount,
            target_world_z_mm=float(args.target_height_mm),
            mount_height_mm=float(cfg.get("ceiling_mount_height_mm", 888.0)),
            xy_keep_mm=keep,
            min_base_z_mm=40.0 + float(tcp["z"]),
            max_xy_mm=(450.0, 300.0),
            lock_y_mm=(
                float(args.lock_y_mm)
                if args.lock_y_mm is not None
                else (0.0 if args.tcp == "camera" else None)
            ),
        )
    joints_deg = [wrap_deg(math.degrees(v)) for v in q]
    print("seed joints_deg", [round(v, 3) for v in seed_deg])
    print(
        "seed {}  XY=({:.1f},{:.1f}) Zbase={:.1f} mm  r={:.1f}  tilt={:.2f} deg".format(
            args.tcp.upper(),
            seed_pose["x_mm"],
            seed_pose["y_mm"],
            seed_pose["z_mm"],
            seed_pose["xy_mm"],
            seed_pose["tilt_from_nadir_deg"],
        )
    )
    mount_z = float(cfg.get("ceiling_mount_height_mm", 888.0))
    world_z = mount_z - pose["z_mm"]
    print("refined joints_deg", [round(v, 3) for v in joints_deg])
    print(
        "refined {}  XY=({:.1f},{:.1f}) Zbase={:.1f} mm  height_above_tray≈{:.1f} mm  r={:.1f}  tilt={:.3f} deg".format(
            args.tcp.upper(),
            pose["x_mm"],
            pose["y_mm"],
            pose["z_mm"],
            world_z,
            pose["xy_mm"],
            pose["tilt_from_nadir_deg"],
        )
    )
    print(
        "copy: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(*joints_deg)
    )
    if not args.skip_moveit:
        chk = moveit_check(args.base_url, cfg, q, pose)
        print("moveit", json.dumps(chk, indent=2))
        if chk.get("ok") and chk.get("joints_deg"):
            print(
                "moveit copy: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(
                    *chk["joints_deg"]
                )
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
