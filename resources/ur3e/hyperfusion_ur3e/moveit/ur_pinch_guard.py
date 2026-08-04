"""UR e-Series C403A0 pinch guard: tool flange / payload vs forearm + upper arm.

The robot controller triggers protective stop C403A0 when a sphere around the tool
flange and a cylinder around the forearm come within ~28 mm. MoveIt SRDF does not
model this by default — we approximate it at plan/validity time.

Also checks tool vs upper-arm cylinder (wrist folding into the upper arm), which is
a common hardware self-hit MoveIt's first-valid OMPL path can still miss in practice.

When a tool payload is enabled, clearance uses a hemisphere at tool0 (flat on the
flange, dome along +tool Z) with radius HYPERFUSION_TOOL_PAYLOAD_RADIUS_M — not a
full sphere, which falsely collides with the forearm at folded home poses.
Shape ``sphere`` keeps the old full-sphere pinch. MoveIt FCL still uses the URDF
collision geometry (CAD mesh).

See Universal Robots forum / UR ROS description issue #112.
"""
from __future__ import annotations

import math
import os
from typing import Any, Mapping, Optional, Tuple

Vec3 = Tuple[float, float, float]

# UR-published approximation (metres).
UR_PINCH_FLANGE_SPHERE_RADIUS_M = 0.0375
UR_PINCH_FOREARM_CYLINDER_RADIUS_M = 0.0375
UR_PINCH_UPPER_ARM_CYLINDER_RADIUS_M = 0.045
UR_PINCH_MIN_SURFACE_GAP_M = 0.028

UPPER_ARM_LINK = "upper_arm_link"
FOREARM_LINK = "forearm_link"
WRIST1_LINK = "wrist_1_link"
TOOL_LINK = "tool0"
TOOL_PAYLOAD_LINK = "hyperfusion_tool_payload"
PINCH_GUARD_LINKS = (UPPER_ARM_LINK, FOREARM_LINK, WRIST1_LINK, TOOL_LINK)


def tool_payload_radius_m() -> float:
    return float(os.environ.get("HYPERFUSION_TOOL_PAYLOAD_RADIUS_M", "0.077"))


def tool_payload_guard_enabled() -> bool:
    raw = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_ENABLED", "true")
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


def tool_payload_shape() -> str:
    raw = os.environ.get("HYPERFUSION_TOOL_PAYLOAD_SHAPE", "mesh")
    return str(raw).strip().lower() or "mesh"


def tool_payload_pinch_uses_sphere() -> bool:
    """Full sphere pinch only when URDF shape is explicitly sphere."""
    return tool_payload_shape() == "sphere"


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


def _vec_normalize(v: Vec3) -> Optional[Vec3]:
    length = _vec_len(v)
    if length <= 1.0e-12:
        return None
    return _vec_scale(v, 1.0 / length)


def _closest_point_on_segment(point: Vec3, seg_a: Vec3, seg_b: Vec3) -> Vec3:
    ab = _vec_sub(seg_b, seg_a)
    ab_len_sq = _vec_dot(ab, ab)
    if ab_len_sq <= 1.0e-12:
        return seg_a
    t = _vec_dot(_vec_sub(point, seg_a), ab) / ab_len_sq
    t = max(0.0, min(1.0, t))
    return _vec_add(seg_a, _vec_scale(ab, t))


def _point_to_segment_distance(point: Vec3, seg_a: Vec3, seg_b: Vec3) -> float:
    """Shortest distance from point to line segment seg_a → seg_b."""
    closest = _closest_point_on_segment(point, seg_a, seg_b)
    return _vec_len(_vec_sub(point, closest))


def _point_to_disk_distance_m(
    point: Vec3, center: Vec3, normal: Vec3, radius_m: float
) -> float:
    """Distance from point to a filled disk (flat flange face)."""
    n = _vec_normalize(normal)
    if n is None:
        return _vec_len(_vec_sub(point, center))
    delta = _vec_sub(point, center)
    axial = _vec_dot(delta, n)
    radial_vec = _vec_sub(delta, _vec_scale(n, axial))
    radial = _vec_len(radial_vec)
    if radial <= radius_m:
        return abs(axial)
    return math.sqrt(axial * axial + (radial - radius_m) ** 2)


def _sphere_cylinder_gap_m(
    point: Vec3,
    seg_a: Vec3,
    seg_b: Vec3,
    sphere_radius_m: float,
    cylinder_radius_m: float,
) -> float:
    return (
        _point_to_segment_distance(point, seg_a, seg_b)
        - sphere_radius_m
        - cylinder_radius_m
    )


def pinch_surface_gap_m(tool_pos: Vec3, forearm_start: Vec3, forearm_end: Vec3) -> float:
    """Approximate surface gap between flange sphere and forearm safety cylinder."""
    return _sphere_cylinder_gap_m(
        tool_pos,
        forearm_start,
        forearm_end,
        UR_PINCH_FLANGE_SPHERE_RADIUS_M,
        UR_PINCH_FOREARM_CYLINDER_RADIUS_M,
    )


def payload_sphere_surface_gap_m(
    tool_pos: Vec3,
    forearm_start: Vec3,
    forearm_end: Vec3,
    payload_radius_m: float,
    *,
    cylinder_radius_m: float = UR_PINCH_FOREARM_CYLINDER_RADIUS_M,
) -> float:
    """Conservative surface gap: full payload sphere at tool0 vs arm cylinder."""
    return _sphere_cylinder_gap_m(
        tool_pos,
        forearm_start,
        forearm_end,
        payload_radius_m,
        cylinder_radius_m,
    )


def payload_hemisphere_surface_gap_m(
    tool_pos: Vec3,
    forearm_start: Vec3,
    forearm_end: Vec3,
    payload_radius_m: float,
    tool_z: Vec3,
    *,
    cylinder_radius_m: float = UR_PINCH_FOREARM_CYLINDER_RADIUS_M,
) -> float:
    """Surface gap: flange-flat hemisphere (+tool Z dome) vs arm cylinder.

    Flat disk lies in the tool0 XY plane; curved half occupies (p-tool)·tool_z ≥ 0.
    Approaches from behind the flange use disk clearance, not a full sphere.
    """
    n = _vec_normalize(tool_z)
    if n is None:
        return payload_sphere_surface_gap_m(
            tool_pos,
            forearm_start,
            forearm_end,
            payload_radius_m,
            cylinder_radius_m=cylinder_radius_m,
        )

    closest = _closest_point_on_segment(tool_pos, forearm_start, forearm_end)
    delta = _vec_sub(closest, tool_pos)
    forward = _vec_dot(delta, n)

    if forward >= 0.0:
        return _vec_len(delta) - payload_radius_m - cylinder_radius_m

    disk_distance_m = _point_to_disk_distance_m(
        closest, tool_pos, n, payload_radius_m
    )
    return disk_distance_m - cylinder_radius_m


def _payload_gap_m(
    tool_pos: Vec3,
    seg_a: Vec3,
    seg_b: Vec3,
    payload_radius_m: float,
    tool_z: Optional[Vec3],
    *,
    cylinder_radius_m: float,
) -> float:
    if tool_payload_pinch_uses_sphere() or tool_z is None:
        return payload_sphere_surface_gap_m(
            tool_pos,
            seg_a,
            seg_b,
            payload_radius_m,
            cylinder_radius_m=cylinder_radius_m,
        )
    return payload_hemisphere_surface_gap_m(
        tool_pos,
        seg_a,
        seg_b,
        payload_radius_m,
        tool_z,
        cylinder_radius_m=cylinder_radius_m,
    )


def effective_pinch_surface_gap_m(
    link_positions: Mapping[str, Vec3],
    *,
    tool_z: Optional[Vec3] = None,
    payload_radius_m: Optional[float] = None,
) -> float:
    tool = link_positions[TOOL_LINK]
    forearm = link_positions[FOREARM_LINK]
    wrist1 = link_positions[WRIST1_LINK]

    gaps = [
        pinch_surface_gap_m(tool, forearm, wrist1),
    ]
    # Upper-arm fold: flange sphere only (large payload hemisphere falsely hits at home).
    if UPPER_ARM_LINK in link_positions:
        gaps.append(
            _sphere_cylinder_gap_m(
                tool,
                link_positions[UPPER_ARM_LINK],
                forearm,
                UR_PINCH_FLANGE_SPHERE_RADIUS_M,
                UR_PINCH_UPPER_ARM_CYLINDER_RADIUS_M,
            )
        )

    if (
        payload_radius_m is not None
        and payload_radius_m > 0.0
        and tool_payload_guard_enabled()
    ):
        gaps.append(
            _payload_gap_m(
                tool,
                forearm,
                wrist1,
                payload_radius_m,
                tool_z,
                cylinder_radius_m=UR_PINCH_FOREARM_CYLINDER_RADIUS_M,
            )
        )

    return min(gaps)


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
            shape = (
                "sphere"
                if tool_payload_pinch_uses_sphere() or tool_z is None
                else "hemisphere"
            )
            return (
                "tool payload within "
                f"{gap_m * 1000.0:.1f} mm surface gap of arm "
                f"(need ≥ {min_surface_gap_m * 1000.0:.1f} mm, "
                f"pinch_{shape}_radius={payload_radius_m * 1000.0:.0f} mm)"
            )
        return (
            "UR pinch guard (C403A0): tool flange within "
            f"{gap_m * 1000.0:.1f} mm surface gap of arm "
            f"(need ≥ {min_surface_gap_m * 1000.0:.1f} mm)"
        )
    return None


def vec3_from_pose(pose: Any) -> Vec3:
    pos = pose.position
    return (float(pos.x), float(pos.y), float(pos.z))
