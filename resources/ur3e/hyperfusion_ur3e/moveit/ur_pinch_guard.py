"""UR e-Series C403A0 pinch guard: tool flange vs forearm (lower arm).

The robot controller triggers protective stop C403A0 when a sphere around the tool
flange and a cylinder around the forearm come within ~28 mm. MoveIt SRDF does not
model this by default — we approximate it at plan/validity time.

When a tool payload is enabled, also treat a bounding sphere at tool0 (radius from
HYPERFUSION_TOOL_PAYLOAD_RADIUS_M) for forearm clearance. MoveIt uses the real CAD
mesh for FCL; this sphere is only the conservative C403A0 stand-in.

See Universal Robots forum / UR ROS description issue #112.
"""
from __future__ import annotations

import math
import os
from typing import Any, Mapping, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]

# UR-published approximation (metres).
UR_PINCH_FLANGE_SPHERE_RADIUS_M = 0.0375
UR_PINCH_FOREARM_CYLINDER_RADIUS_M = 0.0375
UR_PINCH_MIN_SURFACE_GAP_M = 0.028

FOREARM_LINK = "forearm_link"
WRIST1_LINK = "wrist_1_link"
TOOL_LINK = "tool0"
TOOL_PAYLOAD_LINK = "hyperfusion_tool_payload"
PINCH_GUARD_LINKS = (FOREARM_LINK, WRIST1_LINK, TOOL_LINK)


def tool_payload_radius_m() -> float:
    return float(os.environ.get("HYPERFUSION_TOOL_PAYLOAD_RADIUS_M", "0.077"))


def tool_payload_guard_enabled() -> bool:
    raw = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_ENABLED", "true")
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def _vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _vec_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _vec_len(v: Vec3) -> float:
    return math.sqrt(_vec_dot(v, v))


def _point_to_segment_distance(point: Vec3, seg_a: Vec3, seg_b: Vec3) -> float:
    """Shortest distance from point to line segment seg_a → seg_b."""
    ab = _vec_sub(seg_b, seg_a)
    ab_len_sq = _vec_dot(ab, ab)
    if ab_len_sq <= 1.0e-12:
        return _vec_len(_vec_sub(point, seg_a))

    t = _vec_dot(_vec_sub(point, seg_a), ab) / ab_len_sq
    t = max(0.0, min(1.0, t))
    closest = _vec_add(seg_a, _vec_scale(ab, t))
    return _vec_len(_vec_sub(point, closest))


def pinch_surface_gap_m(tool_pos: Vec3, forearm_start: Vec3, forearm_end: Vec3) -> float:
    """Approximate surface gap between flange sphere and forearm safety cylinder."""
    axis_distance_m = _point_to_segment_distance(tool_pos, forearm_start, forearm_end)
    return (
        axis_distance_m
        - UR_PINCH_FLANGE_SPHERE_RADIUS_M
        - UR_PINCH_FOREARM_CYLINDER_RADIUS_M
    )


def payload_sphere_surface_gap_m(
    tool_pos: Vec3,
    forearm_start: Vec3,
    forearm_end: Vec3,
    payload_radius_m: float,
) -> float:
    """Conservative surface gap: full payload sphere at tool0 vs forearm cylinder."""
    axis_distance_m = _point_to_segment_distance(tool_pos, forearm_start, forearm_end)
    return (
        axis_distance_m
        - payload_radius_m
        - UR_PINCH_FOREARM_CYLINDER_RADIUS_M
    )


def effective_pinch_surface_gap_m(
    link_positions: Mapping[str, Vec3],
    *,
    tool_z: Optional[Vec3] = None,
    payload_radius_m: Optional[float] = None,
) -> float:
    del tool_z  # kept for call-site compatibility
    tool = link_positions[TOOL_LINK]
    forearm = link_positions[FOREARM_LINK]
    wrist1 = link_positions[WRIST1_LINK]

    flange_gap_m = pinch_surface_gap_m(tool, forearm, wrist1)
    if (
        payload_radius_m is None
        or payload_radius_m <= 0.0
        or not tool_payload_guard_enabled()
    ):
        return flange_gap_m

    payload_gap_m = payload_sphere_surface_gap_m(
        tool,
        forearm,
        wrist1,
        payload_radius_m,
    )
    return min(flange_gap_m, payload_gap_m)


def ur_pinch_violation(
    link_positions: Mapping[str, Vec3],
    *,
    min_surface_gap_m: float = UR_PINCH_MIN_SURFACE_GAP_M,
    tool_z: Optional[Vec3] = None,
    payload_radius_m: Optional[float] = None,
) -> Optional[str]:
    """Return a reason string when the pinch guard would trigger, else None."""
    try:
        link_positions[TOOL_LINK]
        link_positions[FOREARM_LINK]
        link_positions[WRIST1_LINK]
    except KeyError as exc:
        return f"UR pinch guard: missing link {exc.args[0]}"

    if payload_radius_m is None and tool_payload_guard_enabled():
        payload_radius_m = tool_payload_radius_m()

    gap_m = effective_pinch_surface_gap_m(
        link_positions,
        tool_z=tool_z,
        payload_radius_m=payload_radius_m,
    )
    if gap_m < min_surface_gap_m:
        flange_gap_m = pinch_surface_gap_m(
            link_positions[TOOL_LINK],
            link_positions[FOREARM_LINK],
            link_positions[WRIST1_LINK],
        )
        if (
            payload_radius_m is not None
            and payload_radius_m > UR_PINCH_FLANGE_SPHERE_RADIUS_M
            and gap_m < flange_gap_m
        ):
            return (
                "tool payload within "
                f"{gap_m * 1000.0:.1f} mm surface gap of forearm "
                f"(need ≥ {min_surface_gap_m * 1000.0:.1f} mm, "
                f"pinch_radius={payload_radius_m * 1000.0:.0f} mm)"
            )
        return (
            "UR pinch guard (C403A0): tool flange within "
            f"{gap_m * 1000.0:.1f} mm surface gap of forearm "
            f"(need ≥ {min_surface_gap_m * 1000.0:.1f} mm)"
        )
    return None


def vec3_from_pose(pose: Any) -> Vec3:
    pos = pose.position
    return (float(pos.x), float(pos.y), float(pos.z))
