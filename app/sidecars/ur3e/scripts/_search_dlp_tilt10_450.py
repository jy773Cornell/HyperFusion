#!/usr/bin/env python3
"""FK search: DLP TCP at ~450 mm, 10 deg inward tilt, Y=0, camera-up along +/-X.

Starts from mvs_scan_plans/fpp/450mm.json. No motion.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from refine_dlp_nadir_from_seed import (
    _in_limits,
    dlp_world_from_joints,
    parse_cfg,
    pose_from_t,
    wrap_deg,
)

REPO = Path(__file__).resolve().parents[4]
CFG = REPO / "app" / "preset" / "hyperfusion.cfg"
PLAN = REPO / "app" / "preset" / "mvs_scan_plans" / "fpp" / "450mm.json"
TARGET_TILT = 10.0
TARGET_H = 450.0


def tcp_from_cfg(cfg: dict, prefix: str) -> dict[str, float]:
    return {
        "x": float(cfg[f"{prefix}_x_mm"]),
        "y": float(cfg[f"{prefix}_y_mm"]),
        "z": float(cfg[f"{prefix}_z_mm"]),
        "roll": float(cfg[f"{prefix}_roll_deg"]),
        "pitch": float(cfg[f"{prefix}_pitch_deg"]),
        "yaw": float(cfg[f"{prefix}_yaw_deg"]),
    }


def rotvec_from_R(R: list[list[float]]) -> tuple[float, float, float]:
    tr = R[0][0] + R[1][1] + R[2][2]
    cos_a = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    ang = math.acos(cos_a)
    if ang < 1e-12:
        return 0.0, 0.0, 0.0
    if ang > math.pi - 1e-6:
        xx, yy, zz = R[0][0], R[1][1], R[2][2]
        ax = math.sqrt(max(0.0, (xx + 1) * 0.5))
        ay = math.sqrt(max(0.0, (yy + 1) * 0.5))
        az = math.sqrt(max(0.0, (zz + 1) * 0.5))
        if ax >= ay and ax >= az:
            ay = math.copysign(ay, R[0][1] + R[1][0])
            az = math.copysign(az, R[0][2] + R[2][0])
        elif ay >= az:
            ax = math.copysign(ax, R[0][1] + R[1][0])
            az = math.copysign(az, R[1][2] + R[2][1])
        else:
            ax = math.copysign(ax, R[0][2] + R[2][0])
            ay = math.copysign(ay, R[1][2] + R[2][1])
        n = math.hypot(ax, math.hypot(ay, az)) or 1.0
        return ax / n * ang, ay / n * ang, az / n * ang
    s = 2.0 * math.sin(ang)
    return (
        (R[2][1] - R[1][2]) / s * ang,
        (R[0][2] - R[2][0]) / s * ang,
        (R[1][0] - R[0][1]) / s * ang,
    )


def main() -> None:
    cfg = parse_cfg(CFG)
    dlp = tcp_from_cfg(cfg, "dlp_tcp")
    cam = {
        "x": float(cfg["tool_tcp_x_mm"]),
        "y": float(cfg["tool_tcp_y_mm"]),
        "z": float(cfg["tool_tcp_z_mm"]),
        "roll": float(cfg["tool_tcp_roll_deg"]),
        "pitch": float(cfg["tool_tcp_pitch_deg"]),
        "yaw": float(cfg["tool_tcp_yaw_deg"]),
    }
    base = dict(roll=0, pitch=0, yaw=0, x=0, y=0, z=0)
    mz = float(cfg["ceiling_mount_height_mm"])
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    seed = list(plan["rings"][0]["entry_joints_rad"])
    target_xy = TARGET_H * math.tan(math.radians(TARGET_TILT))

    def T_of(q, tcp=dlp):
        return dlp_world_from_joints(q, tcp, base)

    def cam_up(q):
        t = T_of(q, cam)
        ux, uy = -t[0][1], -t[1][1]
        n = math.hypot(ux, uy) or 1.0
        return math.degrees(math.atan2(uy / n, ux / n)), (ux / n, uy / n)

    def heading_err(yaw):
        return min(
            abs(wrap_deg(yaw)),
            abs(wrap_deg(yaw - 180.0)),
            abs(wrap_deg(yaw + 180.0)),
        )

    def signed_tilt(p, t):
        zx, zy, zz = p["tool_z"]
        x, y = t[0][3], t[1][3]
        inward_x, inward_y = -x, -y
        n = math.hypot(inward_x, inward_y)
        if n < 1e-9:
            inward_x, inward_y, n = 1.0, 0.0, 1.0
        inward_x, inward_y = inward_x / n, inward_y / n
        along = zx * inward_x + zy * inward_y
        mag = math.degrees(math.acos(max(-1.0, min(1.0, zz))))
        return mag if along >= 0.0 else -mag

    def wrap_q(q):
        out = list(q)
        for i in (0, 1, 3, 4, 5):
            out[i] = math.radians(wrap_deg(math.degrees(out[i])))
        return out

    def snap_y0(q):
        q = list(q)
        pd = pose_from_t(T_of(q, dlp))
        phi = math.atan2(pd["y_mm"], pd["x_mm"])
        q[0] += math.pi - phi
        return wrap_q(q)

    def info(q):
        t = T_of(q, dlp)
        pd = pose_from_t(t)
        pc = pose_from_t(T_of(q, cam))
        h = mz - pd["z_mm"]
        yaw, up = cam_up(q)
        st = signed_tilt(pd, t)
        look = math.degrees(math.atan2(pd["xy_mm"], max(1e-6, h)))
        deg = [wrap_deg(math.degrees(v)) for v in q]
        rx, ry, rz = rotvec_from_R([[t[i][j] for j in range(3)] for i in range(3)])
        return {
            "q": q,
            "deg": deg,
            "pd": pd,
            "pc": pc,
            "h": h,
            "st": st,
            "look": look,
            "yaw": yaw,
            "herr": heading_err(yaw),
            "up": up,
            "tcp": {
                "x_m": t[0][3],
                "y_m": t[1][3],
                "z_m": t[2][3],
                "rx": rx,
                "ry": ry,
                "rz": rz,
                "tool_z_x": pd["tool_z"][0],
                "tool_z_y": pd["tool_z"][1],
                "tool_z_z": pd["tool_z"][2],
            },
        }

    def report(label, rec):
        pd, pc = rec["pd"], rec["pc"]
        print(label)
        print("  joints %.2f, %.2f, %.2f, %.2f, %.2f, %.2f" % tuple(rec["deg"]))
        print(
            "  DLP XY=(%.1f,%.1f) r=%.1f h=%.1f signed_tilt=%.3f |tilt|=%.3f look=%.2f"
            % (pd["x_mm"], pd["y_mm"], pd["xy_mm"], rec["h"], rec["st"], pd["tilt_from_nadir_deg"], rec["look"])
        )
        print(
            "  CAM XY=(%.1f,%.1f) h=%.1f tilt=%.2f up-vs-X=%.2f herr=%.3f up=(%.3f,%.3f)"
            % (
                pc["x_mm"],
                pc["y_mm"],
                mz - pc["z_mm"],
                pc["tilt_from_nadir_deg"],
                rec["yaw"],
                rec["herr"],
                rec["up"][0],
                rec["up"][1],
            )
        )
        print("  tool_z=(%.4f,%.4f,%.4f)" % tuple(pd["tool_z"]))

    def cost(q, xy_mode: str) -> float:
        if not _in_limits(q):
            return 1e9
        t = T_of(q, dlp)
        pd = pose_from_t(t)
        h = mz - pd["z_mm"]
        st = signed_tilt(pd, t)
        yaw, _ = cam_up(q)
        xy_tgt = target_xy if xy_mode == "lookat" else 96.7
        jpen = (
            8.0 * (q[0] - seed[0]) ** 2
            + 4.0 * (q[1] - seed[1]) ** 2
            + 4.0 * (q[2] - seed[2]) ** 2
            + 0.4 * (q[3] - seed[3]) ** 2
            + 0.4 * (q[4] - seed[4]) ** 2
            + 0.2 * (q[5] - seed[5]) ** 2
        )
        return (
            90.0 * (st - TARGET_TILT) ** 2
            + 0.08 * (h - TARGET_H) ** 2
            + 0.06 * (pd["xy_mm"] - xy_tgt) ** 2
            + 0.8 * pd["y_mm"] ** 2
            + 12.0 * heading_err(yaw) ** 2
            + jpen
        )

    def nelder(q0, mode, steps, iters=200):
        n = 6
        simp = [list(q0)]
        for i in range(n):
            v = list(q0)
            v[i] += math.radians(steps[i])
            simp.append(v)
        scores = [cost(v, mode) for v in simp]
        for _ in range(iters):
            order = sorted(range(n + 1), key=lambda i: scores[i])
            simp = [simp[i] for i in order]
            scores = [scores[i] for i in order]
            cen = [sum(simp[i][j] for i in range(n)) / n for j in range(n)]
            worst = simp[n]
            refl = [cen[j] + (cen[j] - worst[j]) for j in range(n)]
            rs = cost(refl, mode)
            if scores[0] <= rs < scores[n - 1]:
                simp[n], scores[n] = refl, rs
                continue
            if rs < scores[0]:
                expd = [cen[j] + 2 * (refl[j] - cen[j]) for j in range(n)]
                es = cost(expd, mode)
                simp[n], scores[n] = (expd, es) if es < rs else (refl, rs)
                continue
            contr = [cen[j] + 0.5 * (worst[j] - cen[j]) for j in range(n)]
            cs = cost(contr, mode)
            if cs < scores[n]:
                simp[n], scores[n] = contr, cs
                continue
            b = simp[0]
            for i in range(1, n + 1):
                simp[i] = [b[j] + 0.5 * (simp[i][j] - b[j]) for j in range(n)]
                scores[i] = cost(simp[i], mode)
        return simp[0], scores[0]

    print(
        f"TARGET tilt={TARGET_TILT:g} deg height={TARGET_H:g} "
        f"xy_lookat={target_xy:.1f} mount={mz:.0f}"
    )
    report("SEED 450mm DLP look-down", info(seed))

    guesses = [list(seed)]
    for dw1 in (-25, -15, -8, 8, 15, 25):
        for dw2 in (-20, -10, 0, 10, 20):
            g = list(seed)
            g[3] += math.radians(dw1)
            g[4] += math.radians(dw2)
            guesses.append(g)
    for dl, de in ((-6, 6), (-4, 4), (-2, 2), (2, -2), (4, -4), (6, -6)):
        g = list(seed)
        g[1] += math.radians(dl)
        g[2] += math.radians(de)
        guesses.append(g)

    out = {}
    for mode in ("lookat", "keepxy"):
        best_q, best_s = None, 1e18
        for g in guesses:
            q, s = nelder(g, mode, (0.4, 1.5, 1.5, 4.0, 4.0, 4.0), iters=160)
            if s < best_s:
                best_s, best_q = s, q
        q = snap_y0(best_q)
        q, s = nelder(q, mode, (0.2, 0.8, 0.8, 2.0, 2.0, 2.0), iters=180)
        q = snap_y0(q)
        rec = info(q)
        rec["score"] = s
        print(f"MODE {mode} score={s:.3f}")
        report(f"RESULT {mode}", rec)
        out[mode] = rec

    def polish_side(q0):
        """Kill leftover tool_z_y (no left/right) while holding 10° inward tilt."""

        def pc(q):
            if not _in_limits(q):
                return 1e9
            t = T_of(q, dlp)
            pd = pose_from_t(t)
            h = mz - pd["z_mm"]
            st = signed_tilt(pd, t)
            yaw, _ = cam_up(q)
            return (
                120.0 * (st - TARGET_TILT) ** 2
                + 2500.0 * pd["tool_z"][1] ** 2
                + 0.08 * (h - TARGET_H) ** 2
                + 0.06 * (pd["xy_mm"] - target_xy) ** 2
                + 1.2 * pd["y_mm"] ** 2
                + 12.0 * heading_err(yaw) ** 2
                + 6.0 * (q[0] - q0[0]) ** 2
                + 3.0 * (q[1] - q0[1]) ** 2
                + 3.0 * (q[2] - q0[2]) ** 2
            )

        n = 6
        simp = [list(q0)]
        step = [math.radians(v) for v in (0.3, 0.8, 0.8, 6.0, 6.0, 8.0)]
        for i in range(n):
            v = list(q0)
            v[i] += step[i]
            simp.append(v)
        scores = [pc(v) for v in simp]
        for _ in range(220):
            order = sorted(range(n + 1), key=lambda i: scores[i])
            simp = [simp[i] for i in order]
            scores = [scores[i] for i in order]
            cen = [sum(simp[i][j] for i in range(n)) / n for j in range(n)]
            worst = simp[n]
            refl = [cen[j] + (cen[j] - worst[j]) for j in range(n)]
            rs = pc(refl)
            if scores[0] <= rs < scores[n - 1]:
                simp[n], scores[n] = refl, rs
                continue
            if rs < scores[0]:
                expd = [cen[j] + 2 * (refl[j] - cen[j]) for j in range(n)]
                es = pc(expd)
                simp[n], scores[n] = (expd, es) if es < rs else (refl, rs)
                continue
            contr = [cen[j] + 0.5 * (worst[j] - cen[j]) for j in range(n)]
            cs = pc(contr)
            if cs < scores[n]:
                simp[n], scores[n] = contr, cs
                continue
            b = simp[0]
            for i in range(1, n + 1):
                simp[i] = [b[j] + 0.5 * (simp[i][j] - b[j]) for j in range(n)]
                scores[i] = pc(simp[i])
        return snap_y0(simp[0])

    chosen_q = wrap_q(polish_side(wrap_q(out["lookat"]["q"])))
    chosen = info(chosen_q)
    print("POLISHED lookat (no left/right)")
    report("RESULT polished", chosen)

    payload = {
        "ok": True,
        "target": {
            "tilt_deg": TARGET_TILT,
            "height_mm": TARGET_H,
            "xy_lookat_mm": target_xy,
            "tcp": "dlp",
        },
        "lookat": {
            "joints_deg": [round(v, 2) for v in chosen["deg"]],
            "joints_rad": chosen["q"],
            "tcp": chosen["tcp"],
            "signed_tilt_deg": chosen["st"],
            "height_mm": chosen["h"],
            "xy_mm": chosen["pd"]["xy_mm"],
            "y_mm": chosen["pd"]["y_mm"],
            "look_at_origin_deg": chosen["look"],
            "camera_up_herr_deg": chosen["herr"],
        },
        "keepxy": {
            "joints_deg": [round(v, 2) for v in out["keepxy"]["deg"]],
            "signed_tilt_deg": out["keepxy"]["st"],
            "height_mm": out["keepxy"]["h"],
            "xy_mm": out["keepxy"]["pd"]["xy_mm"],
        },
    }
    dest = PLAN.parent / "_dlp_450_tilt10.json"
    dest.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    route = json.loads(PLAN.read_text(encoding="utf-8"))
    route["id"] = "450mm_tilt10"
    route["display_name"] = "450mm tilt 10"
    route["notes"]["dlp_joints_deg"] = [round(v, 2) for v in chosen["deg"]]
    route["notes"]["dlp"] = (
        "DLP TCP ~450 mm on the long-axis centerline (Y=0), optical axis "
        "tilted 10° inward toward the tray origin (look-at ≈ 10°). "
        "Camera image-up along -X. Execute spins this pin using GUI Range and Interval."
    )
    route["notes"]["pin_tcp_tilt_deg"] = -TARGET_TILT
    ring = route["rings"][0]
    ring["id"] = "dlp_450_tilt10"
    ring["display_name"] = "DLP 450 mm tilt 10°"
    ring["entry_joints_rad"] = [float(v) for v in chosen["q"]]
    ring["entry_tcp"] = chosen["tcp"]
    ring["phi_deg"] = 180.0
    ring["theta_deg"] = float(chosen["look"])
    ring["no_pan"] = False
    ring["reachable"] = True
    ring["home_path_ok"] = True
    ring["base_sweep_ok"] = True
    out_plan = PLAN.parent / "450mm_tilt10.json"
    out_plan.write_text(json.dumps(route, indent=2) + "\n", encoding="utf-8")
    print("COPY lookat: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f" % tuple(chosen["deg"]))
    print("wrote", dest)
    print("wrote", out_plan)


if __name__ == "__main__":
    main()
