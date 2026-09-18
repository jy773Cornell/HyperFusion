"""MoveIt IK + collision checking for UR3e hemisphere scan routes (WSL sidecar)."""
from __future__ import annotations

import copy
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Sequence

os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

from hyperfusion_ur3e import PKG_ROOT
from hyperfusion_ur3e.joint_angles import (
    CANONICAL_JOINT_NAMES,
    MAX_IK_BRANCH_STEPS,
    SOFT_JOINT_LIMIT_MARGIN_RAD,
    TWO_PI,
    UR3E_JOINT_LIMITS_RAD,
    coalesce_joints_for_execute,
    coalesce_joints_for_moveit,
    joint_delta_rad,
    joint_distance_continuous_rad,
    joint_distance_rad,
    joint_limit_clearance_rad,
    moveit_joint_limit_violations,
    near_soft_joint_limit,
    normalize_joint_solution_to_reference,
    pick_joint_branch,
    unwrap_joint_continuous,
    unwrap_joint_toward_preferred,
    wrap_to_pi,
    wrist3_needs_rewind,
    wrist3_on_home_branch,
    wrist3_unwind_target_rad,
)
from hyperfusion_ur3e.moveit.robot_joints import (
    branch_gap_warning,
    coalesce_goal_to_plan_start,
    execute_anchor_joints,
    read_live_joints,
)

GROUP_NAME = "ur_manipulator"
# Optical TCP (BFS sensor face centroid). Same orientation as tool0 (+Z optical axis).
EE_LINK = "hyperfusion_tcp"
PLANNING_FRAME = "world"
RVIZ_UPDATE_CUSTOM_GOAL_TOPIC = "/rviz/moveit/update_custom_goal_state"
WORKSPACE_OBJECT_ID = "hyperfusion_workspace_boundary"  # legacy id (unused)
BOUNDARY_SLAB_THICKNESS_M = 0.02
BOUNDARY_OBJECT_IDS = (
    "hyperfusion_boundary_floor",
    "hyperfusion_boundary_ceiling",
    "hyperfusion_boundary_wall_x_pos",
    "hyperfusion_boundary_wall_x_neg",
    "hyperfusion_boundary_wall_y_pos",
    "hyperfusion_boundary_wall_y_neg",
)
BOUNDARY_CEILING_OBJECT_ID = BOUNDARY_OBJECT_IDS[1]
# Ceiling-mounted base sits at/above the box top — allow these links through the ceiling slab.
BASE_LINKS_IGNORE_CEILING = (
    "base_link",
    "base_link_inertia",
    "base",
)

TOOL_PAYLOAD_LINK = "hyperfusion_tool_payload"

# Matches HyperFusion mock / default home pose (degrees → radians).
DEFAULT_HOME_JOINTS_RAD = [
    math.radians(0.0),
    math.radians(-150.0),
    math.radians(120.0),
    math.radians(0.0),
    math.radians(90.0),
    math.radians(0.0),
]

HOME_JOINT_TOLERANCE_RAD = 0.05
# Connect/verify: treat wrap-space nearness as "already home" (avoids OMPL nudges
# that soft-limit-reject when shoulder_lift/elbow sit near ±180°).
HOME_ALREADY_NEAR_RAD = math.radians(10.0)
TRAJECTORY_START_TOLERANCE_RAD = 0.05
JOINT_SETTLE_TIMEOUT_S = 3.0

# For each base IK seed, try coarse joint offsets first, then fine (one joint at a time).
IK_SEED_JOINT_OFFSETS_COARSE_RAD = tuple(
    math.radians(float(deg)) for deg in (90.0, 180.0, 270.0)
)
IK_SEED_JOINT_OFFSETS_FINE_RAD = tuple(
    math.radians(float(deg)) for deg in (45.0, 135.0, 225.0, 315.0)
)

# Plan-time: after IK succeeds, verify a short home→pin MoveIt path exists
# (same unwrap/branch rules as execute).
PLAN_PATH_CHECK_TIME_S = 5.0
PLAN_PATH_CHECK_ATTEMPTS = 4
PLAN_IK_MAX_CANDIDATES = 8
PLAN_PATH_CHECK_PLANNERS = (
    ("ompl", "RRTConnect"),
    ("pilz_industrial_motion_planner", "PTP"),
)

# MoveIt execute: try planners in order; first valid trajectory wins (no collect-all).
EXECUTE_PLANNER_ATTEMPTS = (
    ("ompl", "RRTConnect"),
    ("ompl", "RRTstar"),
    ("pilz_industrial_motion_planner", "PTP"),
)
# Home / cable-rewind: prefer short joint-space PTP before OMPL snakes.
HOME_EXECUTE_PLANNER_ATTEMPTS = (
    ("pilz_industrial_motion_planner", "PTP"),
    ("ompl", "RRTConnect"),
    ("ompl", "RRTstar"),
)
# Direct pin hops: fail fast. Via-home recovery gets a slightly larger budget.
DIRECT_ALLOWED_PLANNING_TIME_S = 5.0
DIRECT_NUM_PLANNING_ATTEMPTS = 5
RECOVERY_ALLOWED_PLANNING_TIME_S = 12.0
RECOVERY_NUM_PLANNING_ATTEMPTS = 8

# Reject IK solutions whose tool0 +Z faces away from the dome center (180° flip).
TOOL_Z_ALIGNMENT_MIN_DOT = 0.95

# Pin pose tip search (half-angle deg): vertical plane only (look-at × camera-up).
# Inside→out: mid then half; at each tip, +toward up then −toward tray.

# Trajectory scoring: sample pinch guard along planned paths before execute.
TRAJECTORY_PINCH_MIN_GAP_M = 0.028
TRAJECTORY_MAX_PINCH_SAMPLES = 96
TRAJECTORY_MAX_COLLISION_SAMPLES = 96
# Reject OMPL "snake" paths: travel may not exceed max(floor, ratio * start→goal).
TRAJECTORY_MAX_TRAVEL_FLOOR_RAD = math.radians(150.0)
TRAJECTORY_MAX_TRAVEL_RATIO = 2.5
# Hardware-only spins (cable unwind): sample this often and MoveIt-validate each pose.
HARDWARE_SPIN_MAX_STEP_RAD = math.radians(8.0)

# UR3e external-control peak joint velocity (matches teach pendant safety).
UR3E_HARDWARE_MAX_JOINT_VELOCITY_DEG_S = 190.0
UR3E_HARDWARE_MAX_JOINT_VELOCITY_RAD_S = math.radians(UR3E_HARDWARE_MAX_JOINT_VELOCITY_DEG_S)
UR3E_DEFAULT_MAX_JOINT_VELOCITY_DEG_S = 60.0
UR3E_JOINT_VELOCITY_TIME_MARGIN = 1.05
TRAJECTORY_MAX_VELOCITY_SCALEUP = 20.0
# UR External Control servos at 500 Hz; never send consecutive setpoints closer than this.
MIN_TRAJECTORY_SEGMENT_S = 0.02
HARDWARE_JOINT_TRAJECTORY_ACTION = (
    "/scaled_joint_trajectory_controller/follow_joint_trajectory"
)
MOCK_JOINT_TRAJECTORY_ACTION = (
    "/joint_trajectory_controller/follow_joint_trajectory"
)
# control_msgs/FollowJointTrajectory.Result error codes (ROS 2).
CONTROLLER_TRAJECTORY_SUCCESS = 0


def configured_max_joint_velocity_deg_s() -> float:
    """Peak joint speed cap for planned trajectories (env / hyperfusion.cfg)."""
    raw = os.environ.get("HYPERFUSION_MAX_JOINT_VELOCITY_DEG_S", "").strip()
    if raw:
        try:
            value = float(raw)
            if value > 0.0:
                return min(value, UR3E_HARDWARE_MAX_JOINT_VELOCITY_DEG_S)
        except ValueError:
            pass
    return UR3E_DEFAULT_MAX_JOINT_VELOCITY_DEG_S


def configured_max_joint_velocity_rad_s() -> float:
    return math.radians(configured_max_joint_velocity_deg_s())


@dataclass
class WorkspaceBox:
    enabled: bool = False
    length_m: float = 0.6
    width_m: float = 0.6
    """Vertical depth below the mount plane (collision box extent to floor)."""
    height_m: float = 0.65
    """World Z of the robot mount plane (tray surface remains at Z=0)."""
    mount_height_m: float = 0.65
    """Collision box top inset below the mount plane (metres)."""
    ceiling_clearance_m: float = 0.04


def workspace_from_dict(cfg: Optional[Dict[str, Any]]) -> WorkspaceBox:
    if not isinstance(cfg, dict):
        return WorkspaceBox()
    height_m = float(cfg.get("height_m", 0.65))
    mount_height_m = float(cfg.get("mount_height_m", height_m))
    ceiling_clearance_m = float(cfg.get("ceiling_clearance_m", 0.04))
    return WorkspaceBox(
        enabled=bool(cfg.get("enabled", False)),
        length_m=float(cfg.get("length_m", 0.6)),
        width_m=float(cfg.get("width_m", 0.6)),
        height_m=height_m,
        mount_height_m=mount_height_m,
        ceiling_clearance_m=max(0.0, ceiling_clearance_m),
    )


def workspace_vertical_bounds(workspace: WorkspaceBox) -> tuple[float, float]:
    """Return (z_bottom, z_top) for the collision enclosure.

    Robot mount stays at mount_height_m. Box top is inset by ceiling_clearance_m.
    Floor is mount_height_m - height_m (clamped to >= 0).
    """
    z_mount = float(workspace.mount_height_m)
    clearance = max(0.0, float(workspace.ceiling_clearance_m))
    if clearance >= float(workspace.height_m):
        clearance = max(0.0, float(workspace.height_m) - 0.01)
    z_top = z_mount - clearance
    z_bottom = max(0.0, z_mount - float(workspace.height_m))
    if z_top < z_bottom:
        z_top = z_bottom
    return z_bottom, z_top


# UR3e wrist reach ~0.50 m; TCP offset ~0.09 m. Slack so we never drop a pose
# MoveIt might still accept (known R250 T20–T30 hits sit ~0.64–0.67 m).
UR3E_MAX_BASE_TO_TCP_M = 0.85


def tcp_impossible_reason(
    x_m: float,
    y_m: float,
    z_m: float,
    workspace: WorkspaceBox,
) -> str:
    """Cheap reject before IK. Empty string = maybe possible (run MoveIt).

    Conservative: only TCP clearly outside the enclosure, or farther from the
    ceiling-mount origin than UR3e can stretch. Does not model the tool mesh.
    """
    bx = float(x_m)
    by = float(y_m)
    bz = float(z_m)
    mount_z = float(workspace.mount_height_m)
    reach = math.sqrt(bx * bx + by * by + (bz - mount_z) * (bz - mount_z))
    if reach > UR3E_MAX_BASE_TO_TCP_M + 1.0e-9:
        return (
            f"prefilter reach {reach:.3f} m > {UR3E_MAX_BASE_TO_TCP_M:.2f} m "
            f"(base→TCP)"
        )
    if not workspace.enabled:
        return ""
    z_bottom, z_top = workspace_vertical_bounds(workspace)
    half_l = float(workspace.length_m) * 0.5
    half_w = float(workspace.width_m) * 0.5
    # Outside by more than 0.5 mm — on-face TCP still goes to IK.
    slop = 0.0005
    if bz < z_bottom - slop:
        return f"prefilter TCP z={bz:.3f} m below box floor {z_bottom:.3f} m"
    if bz > z_top + slop:
        return f"prefilter TCP z={bz:.3f} m above box top {z_top:.3f} m"
    if abs(bx) > half_l + slop:
        return f"prefilter TCP |x|={abs(bx):.3f} m outside box ±{half_l:.3f} m"
    if abs(by) > half_w + slop:
        return f"prefilter TCP |y|={abs(by):.3f} m outside box ±{half_w:.3f} m"
    return ""


def pan_mask_coverage_deg(mask: Sequence[bool]) -> float:
    if not mask:
        return 0.0
    return 360.0 * float(sum(1 for bit in mask if bit)) / float(len(mask))


def pan_union_coverage_deg(mask_a: Sequence[bool], mask_b: Sequence[bool]) -> float:
    """Union of two absolute pan masks (same sample grid), in degrees."""
    n = min(len(mask_a), len(mask_b))
    if n <= 0:
        return 0.0
    covered = sum(1 for i in range(n) if mask_a[i] or mask_b[i])
    return 360.0 * float(covered) / float(n)


def pan_overlap_coverage_deg(mask_a: Sequence[bool], mask_b: Sequence[bool]) -> float:
    """Intersection of two absolute pan masks, in degrees."""
    n = min(len(mask_a), len(mask_b))
    if n <= 0:
        return 0.0
    covered = sum(1 for i in range(n) if mask_a[i] and mask_b[i])
    return 360.0 * float(covered) / float(n)


def pan_only_coverage_deg(mask_a: Sequence[bool], mask_b: Sequence[bool]) -> float:
    """Degrees in *mask_a* that are not in *mask_b*."""
    n = min(len(mask_a), len(mask_b))
    if n <= 0:
        return 0.0
    covered = sum(1 for i in range(n) if mask_a[i] and not mask_b[i])
    return 360.0 * float(covered) / float(n)


def pan_contiguous_mask_from_entry(
    mask: Sequence[bool], entry_pan_rad: float
) -> List[bool]:
    """Contiguous True run containing *entry_pan_rad* (same island Execute pans)."""
    n = len(mask)
    if n <= 0:
        return []
    bits = [bool(v) for v in mask]
    best = 0
    best_abs = 1.0e9
    for i in range(n):
        bin_pan = wrap_to_pi(TWO_PI * (float(i) / float(n)))
        delta = abs(
            math.atan2(
                math.sin(bin_pan - float(entry_pan_rad)),
                math.cos(bin_pan - float(entry_pan_rad)),
            )
        )
        if delta < best_abs:
            best_abs = delta
            best = i
    if not bits[best]:
        found: Optional[int] = None
        for step in range(1, n):
            a = (best + step) % n
            b = (best - step + n) % n
            if bits[a]:
                found = a
                break
            if bits[b]:
                found = b
                break
        if found is None:
            return [False] * n
        best = found
    start = best
    for _ in range(n - 1):
        prev = (start - 1 + n) % n
        if not bits[prev]:
            break
        start = prev
        if start == best:
            break
    out = [False] * n
    i = start
    for _ in range(n):
        if not bits[i]:
            break
        out[i] = True
        i = (i + 1) % n
        if i == start:
            break
    return out


def home_joints_rad_from_body(body: Optional[Dict[str, Any]]) -> List[float]:
    """Read scan home pose from HyperFusion cfg payload (degrees → radians)."""
    if not isinstance(body, dict):
        return list(DEFAULT_HOME_JOINTS_RAD)
    joints_deg = body.get("home_joints_deg")
    if not isinstance(joints_deg, list) or len(joints_deg) != 6:
        return list(DEFAULT_HOME_JOINTS_RAD)
    return [math.radians(float(value)) for value in joints_deg]


@dataclass
class ScanPoseTarget:
    index: int
    x_m: float
    y_m: float
    z_m: float
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0
    tool_z_x: float = 0.0
    tool_z_y: float = 0.0
    tool_z_z: float = -1.0
    # Preferred image-up / TCP upper-face direction in world (OpenCV up = −tool Y).
    camera_up_x: float = 0.0
    camera_up_y: float = 0.0
    camera_up_z: float = 1.0
    # Apex pin: exact look-down only (no tip cone); roll variants may still be tried.
    require_perpendicular: bool = False
    # Optional grid metadata (Semi ring grouping).
    theta_deg: float = 0.0
    phi_deg: float = 0.0


@dataclass
class ScanPoseResult:
    index: int
    reachable: bool
    joint_positions: List[float] = field(default_factory=list)
    error: str = ""
    # Accepted TCP when cone tolerance tilts the look-at (None = use nominal grid TCP).
    tcp_x_m: Optional[float] = None
    tcp_y_m: Optional[float] = None
    tcp_z_m: Optional[float] = None
    tcp_rx: Optional[float] = None
    tcp_ry: Optional[float] = None
    tcp_rz: Optional[float] = None
    tool_z_x: Optional[float] = None
    tool_z_y: Optional[float] = None
    tool_z_z: Optional[float] = None
    cone_tip_deg: float = 0.0
    # True when a home→pin path+unwrap exists; False = previous→pin chain only.
    home_path_ok: bool = False
    # Semi: full shoulder_pan circle at fixed other joints is collision-free.
    base_sweep_ok: bool = False
    # Semi backup: no full-spin pin; this pin is one of a complementary 2-pin pair.
    backup_coverage_ok: bool = False
    backup_union_deg: float = 0.0
    # Abs 0..360° validity bins (shared grid). Empty = not a backup pair.
    pan_mask: List[bool] = field(default_factory=list)


def _normalize(x: float, y: float, z: float) -> tuple[float, float, float]:
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1.0e-9:
        return 0.0, 0.0, -1.0
    return x / length, y / length, z / length


def _cross(
    a: Sequence[float], b: Sequence[float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _rotate_about_axis(
    v: tuple[float, float, float],
    axis: tuple[float, float, float],
    ang_rad: float,
) -> tuple[float, float, float]:
    """Rodrigues rotate unit-ish vector *v* about unit *axis* by *ang_rad*."""
    ax, ay, az = axis
    vx, vy, vz = v
    c = math.cos(ang_rad)
    s = math.sin(ang_rad)
    dot = ax * vx + ay * vy + az * vz
    cx, cy, cz = _cross(axis, v)
    return (
        vx * c + cx * s + ax * dot * (1.0 - c),
        vy * c + cy * s + ay * dot * (1.0 - c),
        vz * c + cz * s + az * dot * (1.0 - c),
    )


def iter_tool_z_cone_directions(
    zx: float,
    zy: float,
    zz: float,
    half_angle_deg: float,
    *,
    up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> List[tuple[float, tuple[float, float, float]]]:
    """Inside→out tool +Z samples in the vertical plane (look-at × camera-up).

    No left/right (azimuthal) tips — same plane as ``pin_tcp_tilt_deg``.
    Returns (signed_tip_deg, unit_tool_z): + toward *up*, − toward −up.
    Nominal (0°) is always first.
    """
    z0 = _normalize(zx, zy, zz)
    samples: List[tuple[float, tuple[float, float, float]]] = [(0.0, z0)]
    half = max(0.0, float(half_angle_deg))
    if half <= 1.0e-6:
        return samples

    ux, uy, uz = float(up[0]), float(up[1]), float(up[2])
    # Project camera-up ⊥ look-at so pitch axis is well-defined.
    dot_zu = z0[0] * ux + z0[1] * uy + z0[2] * uz
    up_perp = (ux - dot_zu * z0[0], uy - dot_zu * z0[1], uz - dot_zu * z0[2])
    up_len = math.sqrt(up_perp[0] ** 2 + up_perp[1] ** 2 + up_perp[2] ** 2)
    if up_len < 1.0e-9:
        return samples  # look-at ‖ camera-up: no vertical-plane pitch

    pitch_axis = _normalize(*_cross(z0, up_perp))
    tip_angles: List[float] = []
    mid = 0.5 * half
    if mid >= 0.5:
        tip_angles.append(mid)
    tip_angles.append(half)

    for tip_abs in tip_angles:
        tip_rad = math.radians(tip_abs)
        # +tip toward camera-up, then −tip toward tray.
        for signed in (tip_abs, -tip_abs):
            ang = tip_rad if signed > 0.0 else -tip_rad
            tilted = _rotate_about_axis(z0, pitch_axis, ang)
            samples.append((float(signed), _normalize(*tilted)))
    return samples


def scan_pose_target_with_tool_z(
    target: ScanPoseTarget,
    tool_z: Sequence[float],
    *,
    lock_camera_up: bool,
) -> ScanPoseTarget:
    """Same TCP position, new look-at (+ camera-up roll when enabled)."""
    tzx, tzy, tzz = _normalize(float(tool_z[0]), float(tool_z[1]), float(tool_z[2]))
    up = (float(target.camera_up_x), float(target.camera_up_y), float(target.camera_up_z))
    rx, ry, rz = tool_z_to_rotation_vector(
        tzx, tzy, tzz, lock_camera_up=lock_camera_up, up=up
    )
    return ScanPoseTarget(
        index=target.index,
        x_m=target.x_m,
        y_m=target.y_m,
        z_m=target.z_m,
        rx=rx,
        ry=ry,
        rz=rz,
        tool_z_x=tzx,
        tool_z_y=tzy,
        tool_z_z=tzz,
        camera_up_x=target.camera_up_x,
        camera_up_y=target.camera_up_y,
        camera_up_z=target.camera_up_z,
        require_perpendicular=target.require_perpendicular,
    )


def search_params_log(targets: Sequence[ScanPoseTarget]) -> str:
    """R / θ / ring ρ / z for plan start and complete logs (leading space, or empty)."""
    if not targets:
        return ""
    apex_z_mm: Optional[float] = None
    rings: List[str] = []
    seen: set[tuple[int, int, int]] = set()
    for t in targets:
        if bool(t.require_perpendicular) or abs(float(t.theta_deg)) < 0.75:
            if apex_z_mm is None:
                apex_z_mm = 1000.0 * float(t.z_m)
            continue
        r_mm = int(
            round(
                1000.0
                * math.sqrt(float(t.x_m) ** 2 + float(t.y_m) ** 2 + float(t.z_m) ** 2)
            )
        )
        rho_mm = 1000.0 * math.sqrt(float(t.x_m) ** 2 + float(t.y_m) ** 2)
        z_mm = 1000.0 * float(t.z_m)
        th = float(t.theta_deg)
        key = (r_mm, int(round(th)), int(round(z_mm)))
        if key in seen:
            continue
        seen.add(key)
        rings.append(f"R={r_mm}mm θ={th:g}° ρ={rho_mm:.1f}mm z={z_mm:.1f}mm")
        if len(rings) >= 6:
            break
    bits: List[str] = []
    if apex_z_mm is not None:
        bits.append(f"apex Z={apex_z_mm:.0f}mm")
    bits.extend(rings)
    if not bits:
        t0 = targets[0]
        r_mm = int(
            round(
                1000.0
                * math.sqrt(float(t0.x_m) ** 2 + float(t0.y_m) ** 2 + float(t0.z_m) ** 2)
            )
        )
        bits.append(f"R={r_mm}mm z={1000.0 * float(t0.z_m):.1f}mm")
    return " " + " | ".join(bits)


def tool_z_to_rotation_vector(
    zx: float,
    zy: float,
    zz: float,
    *,
    lock_camera_up: bool = True,
    up: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> tuple[float, float, float]:
    """UR rotation vector (axis * angle) with tool +Z aligned to (zx, zy, zz).

    When *lock_camera_up* is True, image-up ≈ *up* projected ⊥ look-at (OpenCV Y-down).
    """
    zx, zy, zz = _normalize(zx, zy, zz)
    if lock_camera_up:
        ux, uy, uz = _normalize(up[0], up[1], up[2])
        up_dot_z = ux * zx + uy * zy + uz * zz
        px, py, pz = ux - up_dot_z * zx, uy - up_dot_z * zy, uz - up_dot_z * zz
        if math.sqrt(px * px + py * py + pz * pz) > 1.0e-9:
            px, py, pz = _normalize(px, py, pz)
            yx, yy, yz = -px, -py, -pz
        else:
            ref_x, ref_y, ref_z = (1.0, 0.0, 0.0) if abs(zx) < 0.9 else (0.0, 1.0, 0.0)
            yx = zy * ref_z - zz * ref_y
            yy = zz * ref_x - zx * ref_z
            yz = zx * ref_y - zy * ref_x
            yx, yy, yz = _normalize(yx, yy, yz)
    else:
        ref_x, ref_y, ref_z = (1.0, 0.0, 0.0) if abs(zx) < 0.9 else (0.0, 1.0, 0.0)
        yx = zy * ref_z - zz * ref_y
        yy = zz * ref_x - zx * ref_z
        yz = zx * ref_y - zy * ref_x
        yx, yy, yz = _normalize(yx, yy, yz)
    xx = yy * zz - yz * zy
    xy = yz * zx - yx * zz
    xz = yx * zy - yy * zx

    # Rotation matrix columns are x, y, z tool axes.
    trace = xx + yy + zz
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    angle = math.acos(cos_angle)
    if angle <= 1.0e-9:
        return 0.0, 0.0, 0.0

    # Near 180°: (R − Rᵀ)/(2 sinθ) is unstable (apex look-down).
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
        if length <= 1.0e-9:
            return math.pi, 0.0, 0.0
        return ax / length * math.pi, ay / length * math.pi, az / length * math.pi

    ax = (yz - zy) / (2.0 * math.sin(angle))
    ay = (zx - xz) / (2.0 * math.sin(angle))
    az = (xy - yx) / (2.0 * math.sin(angle))
    return ax * angle, ay * angle, az * angle


def pose_target_to_ur_pose(target: ScanPoseTarget) -> tuple[float, float, float, float, float, float]:
    """World-frame TCP pose; prefer HyperFusion rotvec (camera-up roll) when present."""
    if abs(target.rx) + abs(target.ry) + abs(target.rz) > 1.0e-12:
        return target.x_m, target.y_m, target.z_m, target.rx, target.ry, target.rz
    up = (float(target.camera_up_x), float(target.camera_up_y), float(target.camera_up_z))
    rx, ry, rz = tool_z_to_rotation_vector(
        target.tool_z_x, target.tool_z_y, target.tool_z_z, up=up
    )
    return target.x_m, target.y_m, target.z_m, rx, ry, rz


def rotvec_to_quaternion(rx: float, ry: float, rz: float) -> tuple[float, float, float, float]:
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle <= 1.0e-9:
        return 0.0, 0.0, 0.0, 1.0
    half = angle * 0.5
    s = math.sin(half) / angle
    return rx * s, ry * s, rz * s, math.cos(half)


def quaternion_to_rotvec(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Unit quaternion → UR axis-angle rotation vector."""
    # Ensure qw >= 0 for the short-arc rotvec.
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))
    if sin_half <= 1.0e-9:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(sin_half, qw)
    axis_scale = angle / sin_half
    return qx * axis_scale, qy * axis_scale, qz * axis_scale


def quat_tool_z_axis(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """World-frame tool0 +Z axis from a unit quaternion."""
    zx = 2.0 * (qx * qz + qw * qy)
    zy = 2.0 * (qy * qz - qw * qx)
    zz = 1.0 - 2.0 * (qx * qx + qy * qy)
    return _normalize(zx, zy, zz)


class MoveItProcessManager:
    """Starts headless move_group when planning is requested."""

    def __init__(self, *, ros_distro: str, ur_type: str) -> None:
        self.ros_distro = ros_distro
        self.ur_type = ur_type
        self._process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    @staticmethod
    def _move_group_running() -> bool:
        from hyperfusion_ur3e.ros_isolation import pids_in_current_domain

        return bool(pids_in_current_domain("move_group"))

    @staticmethod
    def current_move_group_pids() -> tuple[int, ...]:
        """Sorted PIDs of move_group on this ROS_DOMAIN_ID (empty when none)."""
        from hyperfusion_ur3e.ros_isolation import pids_in_current_domain

        return tuple(sorted(pids_in_current_domain("move_group")))

    @staticmethod
    def _use_mock_hardware() -> bool:
        return os.environ.get("HYPERFUSION_USE_MOCK_HARDWARE", "false").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

    def _ros_param_bool(self, node: str, param: str) -> Optional[bool]:
        cmd = (
            f"export ROS_LOCALHOST_ONLY=1 && "
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            f"timeout 5 ros2 param get {node} {param}"
        )
        try:
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            return None
        if proc.returncode != 0:
            return None
        text = proc.stdout.strip().lower()
        if "true" in text:
            return True
        if "false" in text:
            return False
        return None

    def _move_group_controller_config_ok(self) -> bool:
        if not self._move_group_running():
            return False
        if self._use_mock_hardware():
            # Mock scan uses joint_trajectory_controller; skip slow ros2 param CLI on WSL.
            return True
        jtc_default = self._ros_param_bool(
            "/move_group",
            "moveit_simple_controller_manager.joint_trajectory_controller.default",
        )
        scaled_default = self._ros_param_bool(
            "/move_group",
            "moveit_simple_controller_manager.scaled_joint_trajectory_controller.default",
        )
        if jtc_default is None or scaled_default is None:
            return False
        if self._use_mock_hardware():
            return jtc_default and not scaled_default
        return scaled_default and not jtc_default

    @staticmethod
    def _stop_move_group() -> None:
        from hyperfusion_ur3e.ros_isolation import kill_pids, pids_in_current_domain

        kill_pids(pids_in_current_domain("moveit_ros_move_group"))
        kill_pids(pids_in_current_domain("move_group"))
        time.sleep(0.5)

    def _start_headless_stderr_forwarder(self) -> None:
        proc = self._process
        if proc is None or proc.stderr is None:
            return

        def _forward() -> None:
            try:
                for line in proc.stderr:
                    text = line.decode("utf-8", errors="replace").rstrip()
                    if text:
                        sys.stderr.write(f"UR3e MoveIt headless: {text}\n")
            except Exception:
                pass

        threading.Thread(target=_forward, daemon=True, name="ur3e-moveit-headless-log").start()

    def _move_group_ready_for_planning(self) -> bool:
        if not self._compute_ik_service_ready(timeout_s=2.0):
            return False
        if not self._move_group_running():
            return False
        if self._use_mock_hardware():
            return self._move_group_controller_config_ok()
        # Real robot: /compute_ik + move_group up is enough; driver already owns execution.
        return True

    def ensure_running(self, timeout_s: float = 120.0) -> None:
        if not self._use_mock_hardware():
            timeout_s = max(timeout_s, 180.0)
        with self._lock:
            if self._move_group_running() and self._move_group_ready_for_planning():
                return
            if self._move_group_running():
                sys.stderr.write(
                    "UR3e MoveIt: restarting move_group with HyperFusion controller mapping.\n"
                )
                self._stop_move_group()
                self._process = None

            if self._process is not None and self._process.poll() is None:
                deadline = time.time() + timeout_s
                while time.time() < deadline:
                    if self._move_group_running() and self._move_group_ready_for_planning():
                        return
                    time.sleep(0.5)
                raise RuntimeError("MoveIt move_group did not become ready in time.")

            script = PKG_ROOT / "scripts" / "launch_moveit_headless.sh"
            if not script.is_file():
                raise RuntimeError(f"Missing MoveIt launch script: {script}")

            pkg_root = PKG_ROOT.as_posix()
            mock_flag = "true" if self._use_mock_hardware() else "false"
            server_port = os.environ.get("HYPERFUSION_UR3E_SERVER_PORT", "8766")
            sys.stderr.write("UR3e MoveIt: starting headless move_group for scan planning…\n")
            from hyperfusion_ur3e.urdf.mount_config import MountConfig
            from hyperfusion_ur3e.urdf.tool_payload_config import ToolPayloadConfig

            mount_exports = MountConfig.from_env().bash_exports()
            payload_exports = ToolPayloadConfig.from_env().bash_exports()
            from hyperfusion_ur3e.urdf.tool_payload_config import ToolTcpConfig

            tcp_exports = ToolTcpConfig.from_env().bash_exports()
            runtime_urdf_export = ""
            try:
                from hyperfusion_ur3e.urdf.materialize_robot_description import (
                    materialize_runtime_robot_description,
                )

                runtime_urdf = materialize_runtime_robot_description(
                    PKG_ROOT,
                    ros_distro=self.ros_distro,
                    ur_type=self.ur_type,
                    use_mock_hardware=self._use_mock_hardware(),
                    headless_mode=self._use_mock_hardware(),
                    cfg=ToolPayloadConfig.from_env(),
                )
                runtime_urdf_export = (
                    f"export HYPERFUSION_GENERATED_URDF='{runtime_urdf.as_posix()}' && "
                )
            except Exception as exc:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — could not materialize robot_description "
                    f"before move_group start: {exc}\n"
                )
            from hyperfusion_ur3e.ros_isolation import bash_domain_exports

            cmd = (
                f"{bash_domain_exports()}"
                f"export HYPERFUSION_UR3E_REPO='{pkg_root}' && "
                f"export HYPERFUSION_UR3E_SERVER_PORT='{server_port}' && "
                f"export HYPERFUSION_USE_MOCK_HARDWARE='{mock_flag}' && "
                f"{mount_exports}"
                f"{payload_exports}"
                f"{tcp_exports}"
                f"{runtime_urdf_export}"
                f"sed 's/\\r$//' '{script.as_posix()}' | bash -s {self.ros_distro} {self.ur_type}"
            )
            self._process = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
            )
            self._start_headless_stderr_forwarder()

            deadline = time.time() + timeout_s
            last_log_s = 0.0
            while time.time() < deadline:
                if self._process.poll() is not None:
                    stderr_tail = ""
                    if self._process.stderr is not None:
                        stderr_tail = self._process.stderr.read().decode("utf-8", errors="replace")[-1500:]
                    raise RuntimeError(
                        f"MoveIt headless launch exited (code={self._process.returncode}).\n{stderr_tail}"
                    )
                if self._move_group_ready_for_planning():
                    self._verify_move_group_tool_payload()
                    sys.stderr.write("UR3e MoveIt: move_group ready for scan planning.\n")
                    sys.stderr.flush()
                    return
                now = time.time()
                if now - last_log_s >= 10.0:
                    elapsed = int(now - (deadline - timeout_s))
                    sys.stderr.write(
                        f"UR3e MoveIt: waiting for move_group /compute_ik ({elapsed}s)…\n"
                    )
                    last_log_s = now
                time.sleep(1.0)

            raise RuntimeError("Timed out waiting for MoveIt /compute_ik service.")

    @staticmethod
    def _compute_ik_service_ready(timeout_s: float) -> bool:
        """Probe /compute_ik via rclpy (ros2 service list can hang on WSL)."""
        try:
            import rclpy
            from moveit_msgs.srv import GetPositionIK
            from rclpy.node import Node
        except Exception:
            return False

        owns_init = False
        if not rclpy.ok():
            rclpy.init()
            owns_init = True
        node = Node("hyperfusion_moveit_ready_probe")
        client = node.create_client(GetPositionIK, "/compute_ik")
        try:
            return client.wait_for_service(timeout_sec=max(0.5, timeout_s))
        finally:
            node.destroy_node()
            # Keep rclpy initialized for MoveItScanPlanner._ensure_ros().
            if owns_init and not rclpy.ok():
                rclpy.shutdown()

    @staticmethod
    def _verify_move_group_tool_payload() -> None:
        """Warn when MoveIt's robot model is missing the tool-flange payload link."""
        from hyperfusion_ur3e.urdf.tool_payload_config import (
            ToolPayloadConfig,
            robot_description_has_tool_payload,
        )
        from hyperfusion_ur3e.urdf.materialize_robot_description import (
            load_runtime_robot_description_text,
        )

        cfg = ToolPayloadConfig.from_env()
        if not cfg.enabled:
            return

        urdf = load_runtime_robot_description_text(PKG_ROOT)
        if urdf and robot_description_has_tool_payload(
            urdf, expect_shape=cfg.shape, mesh_file=cfg.mesh_file
        ):
            detail = (
                cfg.mesh_file
                if cfg.shape == "mesh"
                else f"radius={cfg.radius_m * 1000.0:.0f} mm"
            )
            sys.stderr.write(
                "UR3e MoveIt: tool payload collision enabled "
                f"({cfg.shape}, {detail}, materialized URDF).\n"
            )
            return

        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.parameter_client import SyncParametersClient
        except Exception:
            return

        owns_init = False
        if not rclpy.ok():
            rclpy.init()
            owns_init = True

        node = Node("hyperfusion_moveit_payload_probe")
        try:
            client = SyncParametersClient(node, "/move_group")
            if not client.wait_for_services(timeout_sec=5.0):
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — cannot verify tool payload link "
                    "(move_group parameter service unavailable).\n"
                )
                return

            values = client.get_parameters(["robot_description"])
            if not values or values[0].type_ == 0:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — move_group robot_description unavailable; "
                    "tool payload may not be in collision checks.\n"
                )
                return

            urdf = values[0].string_value
            if TOOL_PAYLOAD_LINK not in urdf:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — robot_description has no "
                    f"'{TOOL_PAYLOAD_LINK}' link; the camera tool is NOT in MoveIt "
                    "collision checks. Restart sidecar after changing tool_payload_mesh.\n"
                )
                return

            if cfg.shape == "mesh" and cfg.mesh_file not in urdf:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — tool payload mesh "
                    f"'{cfg.mesh_file}' missing from robot_description.\n"
                )
                return

            if cfg.shape == "hemisphere" and "tool_payload_hemisphere.stl" not in urdf:
                sys.stderr.write(
                    "UR3e MoveIt: WARNING — tool payload hemisphere mesh missing from "
                    "robot_description; collision may be disabled.\n"
                )
                return

            detail = (
                cfg.mesh_file
                if cfg.shape == "mesh"
                else f"radius={cfg.radius_m * 1000.0:.0f} mm"
            )
            sys.stderr.write(
                f"UR3e MoveIt: tool payload collision enabled ({cfg.shape}, {detail}).\n"
            )
        except Exception as exc:
            sys.stderr.write(
                "UR3e MoveIt: WARNING — tool payload verification failed: "
                f"{exc}\n"
            )
        finally:
            node.destroy_node()
            if owns_init and rclpy.ok():
                rclpy.shutdown()

    @staticmethod
    def _service_ready(service_name: str, timeout_s: float, ros_distro: str) -> bool:
        if service_name == "/compute_ik":
            return MoveItProcessManager._compute_ik_service_ready(timeout_s)
        cmd = (
            f"export ROS_LOCALHOST_ONLY=1 && "
            f"source /opt/ros/{ros_distro}/setup.bash && "
            f"timeout {max(1, int(timeout_s))} ros2 service list"
        )
        try:
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=max(2.0, timeout_s + 2.0),
            )
        except subprocess.TimeoutExpired:
            return False
        return service_name in (proc.stdout or "")


class MoveItScanPlanner:
    """IK + collision validity for hemisphere scan poses."""

    def __init__(self, *, ros_distro: str = "jazzy", ur_type: str = "ur3e") -> None:
        self.ros_distro = ros_distro
        self.ur_type = ur_type
        self._process_manager = MoveItProcessManager(ros_distro=ros_distro, ur_type=ur_type)
        self._lock = threading.Lock()
        self._scene_lock = threading.RLock()
        self._node = None
        self._ik_client = None
        self._validity_client = None
        self._fk_client = None
        self._pinch_guard_warned = False
        self._move_client = None
        self._execute_trajectory_client = None
        self._joint_trajectory_client = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_stop = threading.Event()
        self._workspace_applied: Optional[tuple[Any, ...]] = None
        self._last_known_move_group_pids: tuple[int, ...] = ()
        self._plan_in_progress = False
        self._last_workspace: Optional[WorkspaceBox] = None
        self._default_workspace: Optional[WorkspaceBox] = None
        self._boundary_keepalive_thread: Optional[threading.Thread] = None
        self._boundary_keepalive_stop = threading.Event()
        self._last_plan_start_seed: List[float] = []
        self._home_joints_rad: List[float] = list(DEFAULT_HOME_JOINTS_RAD)
        self._planning_scene_client = None
        self._planning_scene_pub = None
        self._get_planning_scene_client = None
        self._move_goal_lock = threading.Lock()
        self._workspace_apply_in_progress = False
        self._workspace_apply_cv = threading.Condition(self._scene_lock)
        self._active_move_goal_handle: Any = None
        self._latest_joint_positions: List[float] = []
        self._hardware_joint_positions_cache: List[float] = []
        self._joint_state_sub = None
        self._rviz_goal_pub = None

    def apply_home_joints_from_body(self, body: Optional[Dict[str, Any]]) -> None:
        self._home_joints_rad = home_joints_rad_from_body(body)

    def home_joints_rad(self) -> List[float]:
        return list(self._home_joints_rad)

    def _ensure_ros(self, *, require_ik: bool = True) -> None:
        if self._node is not None:
            if require_ik:
                self._ensure_ik_clients()
            return

        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("hyperfusion_ur3e_scan_planner")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True, name="ur3e-moveit-spin")
        self._spin_thread.start()

        if require_ik:
            self._ensure_ik_clients()

    def _ensure_ik_clients(self) -> None:
        if self._node is None:
            raise RuntimeError("ROS node not initialized.")

        if self._ik_client is not None and self._validity_client is not None:
            return

        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionIK, GetStateValidity

        if self._ik_client is None:
            self._ik_client = self._node.create_client(GetPositionIK, "/compute_ik")
        if self._validity_client is None:
            self._validity_client = self._node.create_client(
                GetStateValidity, "/check_state_validity"
            )
        if self._rviz_goal_pub is None:
            self._rviz_goal_pub = self._node.create_publisher(
                RobotState, RVIZ_UPDATE_CUSTOM_GOAL_TOPIC, 10
            )

        if not self._ik_client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("MoveIt /compute_ik service not available.")
        if not self._validity_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("MoveIt /check_state_validity service not available.")

        if self._joint_state_sub is None:
            from sensor_msgs.msg import JointState

            def _on_joint_state(msg: JointState) -> None:
                if not msg.name:
                    return
                name_to_pos = {
                    name: float(pos)
                    for name, pos in zip(msg.name, msg.position)
                    if name in CANONICAL_JOINT_NAMES
                }
                if len(name_to_pos) == 6:
                    self._latest_joint_positions = [
                        name_to_pos[name] for name in CANONICAL_JOINT_NAMES
                    ]

            def _on_hardware_joint_state(msg: JointState) -> None:
                if not msg.name:
                    return
                name_to_pos = {
                    name: float(pos)
                    for name, pos in zip(msg.name, msg.position)
                    if name in CANONICAL_JOINT_NAMES
                }
                if len(name_to_pos) == 6:
                    self._hardware_joint_positions_cache = [
                        name_to_pos[name] for name in CANONICAL_JOINT_NAMES
                    ]

            self._joint_state_sub = self._node.create_subscription(
                JointState,
                "/joint_states_stamped",
                _on_joint_state,
                10,
            )
            self._hardware_joint_state_sub = self._node.create_subscription(
                JointState,
                "/joint_state_broadcaster/joint_states",
                _on_hardware_joint_state,
                10,
            )

    def _ensure_fk_client(self) -> None:
        if self._node is None:
            raise RuntimeError("ROS node not initialized.")
        if self._fk_client is not None:
            return

        from moveit_msgs.srv import GetPositionFK

        for service_name in ("/compute_fk", "/move_group/compute_fk"):
            client = self._node.create_client(GetPositionFK, service_name)
            if client.wait_for_service(timeout_sec=2.0):
                self._fk_client = client
                return
        self._fk_client = None

    @staticmethod
    def _use_mock_hardware() -> bool:
        return os.environ.get("HYPERFUSION_USE_MOCK_HARDWARE", "false").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )

    def _fk_link_positions(self, joints: Sequence[float]) -> Optional[dict[str, tuple[float, float, float]]]:
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        from hyperfusion_ur3e.moveit.ur_pinch_guard import PINCH_GUARD_LINKS, vec3_from_pose

        try:
            self._ensure_fk_client()
        except Exception:
            return None
        if self._fk_client is None:
            return None

        from moveit_msgs.srv import GetPositionFK

        request = GetPositionFK.Request()
        request.header.frame_id = PLANNING_FRAME
        request.fk_link_names = list(PINCH_GUARD_LINKS)
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        request.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._fk_client.call_async(request)
        response = self._wait_future(future, 5.0)
        if response is None or response.error_code.val != 1:
            return None

        positions: dict[str, tuple[float, float, float]] = {}
        for link_name, pose_stamped in zip(PINCH_GUARD_LINKS, response.pose_stamped):
            positions[link_name] = vec3_from_pose(pose_stamped.pose)
        return positions

    def _ur_pinch_guard_ok(self, joints: Sequence[float]) -> tuple[bool, str]:
        from hyperfusion_ur3e.moveit.ur_pinch_guard import ur_pinch_violation

        link_positions = self._fk_link_positions(joints)
        if link_positions is None:
            if not self._pinch_guard_warned:
                self._pinch_guard_warned = True
                sys.stderr.write(
                    "UR3e MoveIt: pinch guard skipped — /compute_fk unavailable.\n"
                )
            return True, ""

        tool_z = self._fk_tool0_z_axis(joints)
        reason = ur_pinch_violation(link_positions, tool_z=tool_z)
        if reason:
            return False, reason
        return True, ""

    def _fk_tool0_z_axis(
        self, joints: Sequence[float]
    ) -> Optional[tuple[float, float, float]]:
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        try:
            self._ensure_fk_client()
        except Exception:
            return None
        if self._fk_client is None:
            return None

        from moveit_msgs.srv import GetPositionFK

        request = GetPositionFK.Request()
        request.header.frame_id = PLANNING_FRAME
        request.fk_link_names = [EE_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        request.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._fk_client.call_async(request)
        response = self._wait_future(future, 5.0)
        if response is None or response.error_code.val != 1 or not response.pose_stamped:
            return None

        orientation = response.pose_stamped[0].pose.orientation
        return quat_tool_z_axis(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

    def _tool_z_alignment(
        self,
        joints: Sequence[float],
        desired_tool_z: Sequence[float],
    ) -> Optional[float]:
        actual = self._fk_tool0_z_axis(joints)
        if actual is None:
            return None

        desired = _normalize(
            float(desired_tool_z[0]),
            float(desired_tool_z[1]),
            float(desired_tool_z[2]),
        )
        return actual[0] * desired[0] + actual[1] * desired[1] + actual[2] * desired[2]

    def set_default_workspace(self, workspace: WorkspaceBox) -> None:
        self._default_workspace = workspace
        self._last_workspace = workspace

    def start_boundary_keepalive(self) -> None:
        with self._lock:
            if (
                self._boundary_keepalive_thread is not None
                and self._boundary_keepalive_thread.is_alive()
            ):
                return
            self._boundary_keepalive_stop.clear()
            self._boundary_keepalive_thread = threading.Thread(
                target=self._boundary_keepalive_loop,
                daemon=True,
                name="ur3e-boundary-keepalive",
            )
            self._boundary_keepalive_thread.start()

    def _boundary_keepalive_loop(self) -> None:
        first_cycle = True
        while True:
            # Poll quickly until the first boundary is applied after move_group start.
            wait_s = 0.5 if first_cycle else 3.0
            if self._boundary_keepalive_stop.wait(wait_s):
                break
            first_cycle = False

            workspace = self._default_workspace
            if workspace is None or not self._process_manager.current_move_group_pids():
                first_cycle = True
                continue

            with self._move_goal_lock:
                motion_active = self._active_move_goal_handle is not None
            if motion_active:
                continue

            workspace_key = self._workspace_key(workspace)
            if workspace_key == self._workspace_applied:
                continue

            try:
                self.ensure_workspace_boundary_visible(workspace)
            except Exception as exc:
                # Timeouts are common during MoveIt/RViz restart; next cycle retries.
                sys.stderr.write(f"UR3e MoveIt: boundary keepalive: {exc}\n")
                sys.stderr.flush()

    def ensure_workspace_boundary_visible(self, workspace: WorkspaceBox) -> bool:
        """Publish workspace collision walls into the active MoveIt planning scene."""
        if not workspace.enabled:
            return False
        if not self._process_manager.current_move_group_pids():
            return False

        workspace_key = self._workspace_key(workspace)
        if workspace_key == self._workspace_applied:
            return True

        with self._lock:
            self._ensure_ros(require_ik=False)
            self._last_workspace = workspace
        return self._apply_workspace_collision(workspace)

    def _live_joint_reading(self, *, timeout_s: float = 1.5):
        """RTDE-authoritative live joints (see moveit.robot_joints)."""
        hardware = self._ensure_hardware_joint_positions(timeout_s=timeout_s)
        stamper = self._current_joint_positions()
        return read_live_joints(hardware_rad=hardware, stamper_rad=stamper)

    def _current_joint_positions(self) -> Optional[List[float]]:
        if len(self._latest_joint_positions) != 6:
            return None
        return list(self._latest_joint_positions)

    def _hardware_joint_positions(self) -> Optional[List[float]]:
        if len(self._hardware_joint_positions_cache) != 6:
            return None
        return list(self._hardware_joint_positions_cache)

    def _ensure_hardware_joint_positions(self, timeout_s: float = 2.0) -> Optional[List[float]]:
        """Wait for raw /joint_state_broadcaster feedback (RTDE branch for execute)."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            hardware = self._hardware_joint_positions()
            if hardware is not None:
                return hardware
            # Never spin the shared SingleThreadedExecutor from this thread —
            # the dedicated spin thread owns it (concurrent spin_once can hang).
            time.sleep(0.05)
        hardware = self._hardware_joint_positions()
        if hardware is None:
            sys.stderr.write(
                "UR3e MoveIt execute: raw hardware joint_states unavailable — "
                "trajectory may use MoveIt branch (controller reject risk).\n"
            )
        return hardware

    def _moveit_start_joint_positions(self) -> Optional[List[float]]:
        """Plan-start joints: RTDE mapped into MoveIt limits (not stamper twins)."""
        reading = read_live_joints(
            hardware_rad=self._hardware_joint_positions(),
            stamper_rad=self._current_joint_positions(),
        )
        return reading.plan_start_rad

    def _wait_for_planner_joint_feedback(self, timeout_s: float = 2.0) -> None:
        """Wait until MoveIt / hardware joint caches are populated (spin thread feeds them)."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self._moveit_start_joint_positions() is not None:
                return
            time.sleep(0.05)

    def _resolve_execute_branch(
        self,
        goal_joints: Sequence[float],
        *,
        direct_only: bool,
        tcp_target: Optional[Dict[str, Any]] = None,
        refresh_ik: bool = False,
        pin_pose_tolerance_deg: float = 0.0,
        lock_camera_up: bool = True,
    ) -> tuple[List[float], Optional[List[float]]]:
        """Coalesce goal onto the RTDE-mapped plan-start branch.

        Never re-seed onto the plan-time home seed (that caused wrist branch flips).
        RTDE multi-turn hardware is applied only when unwrapping the execute trajectory.
        """
        del direct_only  # kept for call-site compatibility
        resolved_goal = [float(v) for v in goal_joints]
        self._wait_for_planner_joint_feedback(timeout_s=1.5)
        reading = self._live_joint_reading(timeout_s=1.0)
        moveit_start = reading.plan_start_rad
        warn = branch_gap_warning(reading)
        if warn:
            sys.stderr.write(f"UR3e MoveIt execute: {warn}.\n")
            sys.stderr.flush()

        if moveit_start is not None:
            branch_ref = list(moveit_start)
            resolved_goal = coalesce_goal_to_plan_start(branch_ref, resolved_goal)
        else:
            branch_ref = None

        if refresh_ik and isinstance(tcp_target, dict) and branch_ref is not None:
            refreshed = self._goal_joints_from_tcp(
                tcp_target,
                branch_ref,
                pin_pose_tolerance_deg=pin_pose_tolerance_deg,
                lock_camera_up=lock_camera_up,
            )
            if refreshed is not None:
                resolved_goal = coalesce_goal_to_plan_start(branch_ref, refreshed)

        return resolved_goal, branch_ref

    def _coalesce_goal_joints(
        self,
        reference: Sequence[float],
        goal: Sequence[float],
    ) -> List[float]:
        """Align UI / plan goal angles to the same 2π branch as *reference* (MoveIt limits)."""
        return coalesce_joints_for_moveit(reference, goal)

    def _wait_for_joint_positions_near(
        self,
        target: Sequence[float],
        *,
        tolerance_rad: float = HOME_JOINT_TOLERANCE_RAD,
        timeout_s: float = JOINT_SETTLE_TIMEOUT_S,
        stop_event: Optional[threading.Event] = None,
        reference: Optional[Sequence[float]] = None,
    ) -> Optional[List[float]]:
        """Poll /joint_states_stamped until the robot is near *target* (or timeout)."""
        ref = reference if reference is not None else target
        target_norm = self._normalize_joint_solution_to_reference(ref, target)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if stop_event is not None and stop_event.is_set():
                self.cancel_active_move()
                raise RuntimeError("Motion stopped.")
            current = self._current_joint_positions()
            if current is not None:
                current = self._normalize_joint_solution_to_reference(ref, current)
                if self._joint_distance_rad(current, target_norm) <= tolerance_rad:
                    return current
            time.sleep(0.05)
        current = self._current_joint_positions()
        if current is not None:
            return self._normalize_joint_solution_to_reference(ref, current)
        return None

    def _waypoint_after_commanded_move(
        self,
        commanded: Sequence[float],
        *,
        stop_event: Optional[threading.Event] = None,
        reference: Optional[Sequence[float]] = None,
    ) -> List[float]:
        """Use commanded joints as the motion waypoint; prefer settled joint feedback when close."""
        ref = reference if reference is not None else commanded
        commanded_norm = self._normalize_joint_solution_to_reference(ref, commanded)
        settled = self._wait_for_joint_positions_near(
            commanded_norm,
            stop_event=stop_event,
            reference=ref,
        )
        if settled is None:
            sys.stderr.write(
                "UR3e MoveIt execute: joint feedback unavailable after move — "
                "using commanded waypoint.\n"
            )
            return commanded_norm
        if self._joint_distance_rad(settled, commanded_norm) > HOME_JOINT_TOLERANCE_RAD:
            sys.stderr.write(
                "UR3e MoveIt execute: joint feedback not yet at commanded waypoint — "
                "using commanded joints for approach reference.\n"
            )
            return commanded_norm
        return settled

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        return wrap_to_pi(angle)

    @classmethod
    def _pick_joint_branch(cls, index: int, value: float, reference: float) -> float:
        return pick_joint_branch(index, value, reference)

    @classmethod
    def _unwrap_joint_to_reference(
        cls,
        joint_index: int,
        raw: float,
        reference: float,
    ) -> float:
        """Chain trajectory samples continuously (shortest wrap delta — no ±π snap)."""
        del joint_index  # all axes use continuous unwrap for execute trajectories
        return unwrap_joint_continuous(reference, raw)

    @classmethod
    def _normalize_joint_solution_to_reference(
        cls,
        reference: Sequence[float],
        joints: Sequence[float],
    ) -> List[float]:
        return normalize_joint_solution_to_reference(reference, joints)

    @staticmethod
    def _joint_delta_rad(a: float, b: float) -> float:
        return joint_delta_rad(a, b)

    @staticmethod
    def _segment_joint_delta_for_timing(reference: float, candidate: float) -> float:
        """Largest of wrapped and numeric delta — UR RTDE uses numeric joint jumps."""
        ref = float(reference)
        cand = float(candidate)
        wrapped = abs(joint_delta_rad(ref, cand))
        raw = abs(cand - ref)
        return max(wrapped, raw)

    @classmethod
    def _joint_distance_rad(cls, reference: Sequence[float], candidate: Sequence[float]) -> float:
        return joint_distance_rad(reference, candidate)

    @classmethod
    def _joints_within_ur3e_limits(cls, joints: Sequence[float]) -> bool:
        if len(joints) != 6:
            return False
        for index, value in enumerate(joints):
            limits = UR3E_JOINT_LIMITS_RAD[index]
            if limits is None:
                continue
            lo, hi = limits
            if float(value) < lo - 1e-6 or float(value) > hi + 1e-6:
                return False
        return True

    @classmethod
    def _ik_joint_angles_are_sane(cls, joints: Sequence[float]) -> bool:
        """Reject IK branches outside UR3e/MoveIt joint limits."""
        return cls._joints_within_ur3e_limits(joints)

    def _spin_loop(self) -> None:
        while not self._spin_stop.is_set():
            if self._executor is None:
                break
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                if self._spin_stop.is_set():
                    break
                raise

    def _wait_future(
        self,
        future: Any,
        timeout_s: float,
        stop_event: Optional[threading.Event] = None,
        *,
        label: str = "MoveIt call",
    ) -> Any:
        deadline = time.time() + timeout_s
        last_beat = time.time()
        while time.time() < deadline:
            if stop_event is not None and stop_event.is_set():
                self.cancel_active_move()
                raise RuntimeError("Motion stopped.")
            if future.done():
                return future.result()
            now = time.time()
            if now - last_beat >= 5.0:
                remaining = max(0.0, deadline - now)
                sys.stderr.write(
                    f"UR3e MoveIt: still waiting for {label} "
                    f"({remaining:.0f}s left)…\n"
                )
                sys.stderr.flush()
                last_beat = now
            time.sleep(0.02)
        raise TimeoutError(f"{label} timed out after {timeout_s:.0f}s.")

    def cancel_active_move(self) -> None:
        with self._move_goal_lock:
            handle = self._active_move_goal_handle
        if handle is None:
            return
        try:
            cancel_future = handle.cancel_goal_async()
            deadline = time.time() + 2.0
            while time.time() < deadline and not cancel_future.done():
                time.sleep(0.02)
        except Exception:
            pass
        with self._move_goal_lock:
            self._active_move_goal_handle = None

    def _workspace_key(self, workspace: WorkspaceBox) -> tuple[Any, ...]:
        # Include the live move_group PID set: a restarted or GUI-launched move_group
        # starts with an empty planning scene, so the boundary must be re-published.
        # Cache last non-empty PIDs — brief pgrep misses must not thrash ApplyPlanningScene.
        pids = self._process_manager.current_move_group_pids()
        if pids:
            self._last_known_move_group_pids = pids
        else:
            pids = self._last_known_move_group_pids
        return (
            pids,
            workspace.enabled,
            round(workspace.length_m, 6),
            round(workspace.width_m, 6),
            round(workspace.height_m, 6),
            round(workspace.mount_height_m, 6),
            round(workspace.ceiling_clearance_m, 6),
            "acm_base_vs_box_top_v3",
        )

    def _ensure_planning_scene_client(self) -> None:
        if self._planning_scene_client is not None:
            return

        from moveit_msgs.srv import ApplyPlanningScene

        self._planning_scene_client = self._node.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )
        if not self._planning_scene_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt /apply_planning_scene service not available.")

    def _ensure_planning_scene_pub(self) -> None:
        if self._planning_scene_pub is not None:
            return
        from moveit_msgs.msg import PlanningScene
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        # Match MoveIt PlanningSceneMonitor defaults (reliable, depth 1).
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._planning_scene_pub = self._node.create_publisher(
            PlanningScene, "/planning_scene", qos
        )

    def _publish_planning_scene_diff(self, scene: Any) -> None:
        """Async scene update via /planning_scene (no service wait)."""
        scene.is_diff = True
        scene.robot_state.is_diff = True
        self._ensure_planning_scene_pub()
        assert self._planning_scene_pub is not None
        self._planning_scene_pub.publish(scene)
        time.sleep(0.15)
        self._planning_scene_pub.publish(scene)
        time.sleep(0.35)

    def _apply_planning_scene_diff_service(
        self,
        scene: Any,
        *,
        timeout_s: float,
        label: str,
    ) -> bool:
        """Synchronous ApplyPlanningScene; returns False on timeout/error (no raise)."""
        from moveit_msgs.srv import ApplyPlanningScene

        scene.is_diff = True
        scene.robot_state.is_diff = True
        try:
            self._ensure_planning_scene_client()
            request = ApplyPlanningScene.Request()
            request.scene = scene
            future = self._planning_scene_client.call_async(request)
            self._wait_future(future, timeout_s, label=label)
            return True
        except Exception as exc:
            sys.stderr.write(f"UR3e MoveIt: {label} failed ({exc}).\n")
            sys.stderr.flush()
            return False

    def _apply_planning_scene_object(self, collision_object: Any) -> None:
        self._apply_planning_scene_objects([collision_object])

    def _apply_planning_scene_objects(self, collision_objects: Sequence[Any]) -> None:
        if not collision_objects:
            return

        from moveit_msgs.msg import PlanningScene

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects.extend(collision_objects)

        # Topic-first: bulk wall ADD via /apply_planning_scene often stalls ~90s on WSL
        # right after move_group start. Topic path is async and usually lands in <1s.
        self._publish_planning_scene_diff(scene)

    def _ensure_get_planning_scene_client(self) -> None:
        if self._get_planning_scene_client is not None:
            return
        from moveit_msgs.srv import GetPlanningScene

        self._get_planning_scene_client = self._node.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )
        if not self._get_planning_scene_client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError("MoveIt /get_planning_scene service not available.")

    @staticmethod
    def _acm_is_safe_to_replace(acm: Any) -> bool:
        """Reject incomplete ACM snapshots — replacing with them wipes SRDF disables.

        A too-early GetPlanningScene (right after move_group start) can return a sparse
        matrix. Publishing that as a diff replaces the full ACM and makes the BFS camera
        collide with wrist_3 / flange constantly.
        """
        names = set(acm.entry_names or [])
        if len(names) < 12:
            return False
        ur_links = {
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
        }
        if len(names & ur_links) < 5:
            return False
        # Tool payload must already be present with SRDF adjacent disables loaded.
        if TOOL_PAYLOAD_LINK not in names:
            return False
        if "wrist_3_link" not in names:
            return False
        return True

    @staticmethod
    def _acm_set_allowed(acm: Any, name_a: str, name_b: str, allowed: bool = True) -> None:
        """Set/allow a collision pair in an AllowedCollisionMatrix message."""
        from moveit_msgs.msg import AllowedCollisionEntry

        names = list(acm.entry_names)
        values = list(acm.entry_values)

        def ensure_name(name: str) -> int:
            if name in names:
                return names.index(name)
            index = len(names)
            names.append(name)
            # Expand every existing row by one column.
            for entry in values:
                enabled = list(entry.enabled)
                enabled.append(False)
                entry.enabled = enabled
            new_entry = AllowedCollisionEntry()
            new_entry.enabled = [False] * len(names)
            values.append(new_entry)
            return index

        ia = ensure_name(name_a)
        ib = ensure_name(name_b)
        # Rows may have been expanded; normalize lengths.
        n = len(names)
        for entry in values:
            enabled = list(entry.enabled)
            if len(enabled) < n:
                enabled.extend([False] * (n - len(enabled)))
            entry.enabled = enabled
        values[ia].enabled[ib] = bool(allowed)
        values[ib].enabled[ia] = bool(allowed)
        acm.entry_names = names
        acm.entry_values = values

    def _fetch_complete_acm(self, *, timeout_s: float = 25.0) -> Any:
        """Wait until MoveIt ACM includes UR + tool SRDF entries before mutating it."""
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene

        self._ensure_get_planning_scene_client()
        deadline = time.time() + timeout_s
        last_count = -1
        while time.time() < deadline:
            get_req = GetPlanningScene.Request()
            get_req.components.components = (
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            )
            get_future = self._get_planning_scene_client.call_async(get_req)
            try:
                get_response = self._wait_future(
                    get_future, 8.0, label="/get_planning_scene (ACM)"
                )
            except TimeoutError:
                time.sleep(0.4)
                continue
            acm = get_response.scene.allowed_collision_matrix
            count = len(acm.entry_names or [])
            if count != last_count:
                sys.stderr.write(
                    f"UR3e MoveIt: ACM snapshot has {count} entries "
                    f"(need full UR+tool matrix)…\n"
                )
                sys.stderr.flush()
                last_count = count
            if self._acm_is_safe_to_replace(acm):
                return acm
            time.sleep(0.5)
        raise RuntimeError(
            "MoveIt AllowedCollisionMatrix not ready "
            "(refusing incomplete ACM replace that would break camera SRDF disables)."
        )

    def _allow_base_ceiling_collisions(self) -> None:
        """Ignore base↔workspace-box-top collisions (ceiling mount through box top slab)."""
        from moveit_msgs.msg import PlanningScene

        acm = self._fetch_complete_acm(timeout_s=25.0)

        for link_name in BASE_LINKS_IGNORE_CEILING:
            self._acm_set_allowed(acm, link_name, BOUNDARY_CEILING_OBJECT_ID, True)

        # Re-assert SRDF tool/camera adjacent disables so a bad merge cannot re-enable them.
        # Only true fixed/adjacent mount pairs are ignored. Arm links must stay checked
        # (BFS mesh corner vs wrist_2 caused pendant C157A4 when ignored).
        for link_a, link_b in (
            ("tool0", TOOL_PAYLOAD_LINK),
            ("flange", TOOL_PAYLOAD_LINK),
            ("wrist_3_link", TOOL_PAYLOAD_LINK),
            ("tool0", "hyperfusion_tcp"),
            ("flange", "hyperfusion_tcp"),
            (TOOL_PAYLOAD_LINK, "hyperfusion_tcp"),
        ):
            self._acm_set_allowed(acm, link_a, link_b, True)

        # Force-check payload vs arm (clear any leftover allow=* from older SRDF/ACM).
        for arm_link in (
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
        ):
            self._acm_set_allowed(acm, arm_link, TOOL_PAYLOAD_LINK, False)

        sys.stderr.write(
            "UR3e MoveIt: ACM — tool payload collision CHECKING vs "
            "shoulder/upper_arm/forearm/wrist_1/wrist_2 "
            "(only tool0/flange/wrist_3 ignored as mount-adjacent).\n"
        )
        sys.stderr.flush()

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        # Apply the full merged ACM (MoveIt replaces ACM when entry_names is set on a diff).
        scene.allowed_collision_matrix = acm

        # Prefer service for ACM — topic-only early replaces wiped camera↔wrist disables.
        if not self._apply_planning_scene_diff_service(
            scene, timeout_s=20.0, label="/apply_planning_scene (ACM)"
        ):
            sys.stderr.write(
                "UR3e MoveIt: ACM service timed out — publishing validated ACM on topic.\n"
            )
            sys.stderr.flush()
            self._publish_planning_scene_diff(scene)

        sys.stderr.write(
            "UR3e MoveIt: ACM — ignore base vs workspace box top "
            f"({', '.join(BASE_LINKS_IGNORE_CEILING)} ↔ {BOUNDARY_CEILING_OBJECT_ID}).\n"
        )
        sys.stderr.flush()

    def _remove_planning_scene_object(self, object_id: str) -> None:
        from moveit_msgs.msg import CollisionObject

        remove_object = CollisionObject()
        remove_object.id = object_id
        remove_object.operation = CollisionObject.REMOVE
        self._apply_planning_scene_object(remove_object)

    def _make_box_collision_object(
        self,
        object_id: str,
        *,
        size_x: float,
        size_y: float,
        size_z: float,
        center_x: float,
        center_y: float,
        center_z: float,
    ) -> Any:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = PLANNING_FRAME

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]

        pose = Pose()
        pose.position.x = float(center_x)
        pose.position.y = float(center_y)
        pose.position.z = float(center_z)
        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def _make_workspace_boundary_objects(self, workspace: WorkspaceBox) -> List[Any]:
        """Thin collision slabs on all six faces; top inset by ceiling_clearance_m below mount."""
        thickness = BOUNDARY_SLAB_THICKNESS_M
        half_l = workspace.length_m * 0.5
        half_w = workspace.width_m * 0.5
        z_bottom, z_top = workspace_vertical_bounds(workspace)
        wall_height = max(thickness, z_top - z_bottom)
        wall_center_z = z_bottom + (wall_height * 0.5)

        return [
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[0],
                size_x=workspace.length_m,
                size_y=workspace.width_m,
                size_z=thickness,
                center_x=0.0,
                center_y=0.0,
                center_z=z_bottom - (thickness * 0.5),
            ),
            # Ceiling slab: underside at z_top closes the box; robot base mounts at/above z_top.
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[1],
                size_x=workspace.length_m,
                size_y=workspace.width_m,
                size_z=thickness,
                center_x=0.0,
                center_y=0.0,
                center_z=z_top + (thickness * 0.5),
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[2],
                size_x=thickness,
                size_y=workspace.width_m,
                size_z=wall_height,
                center_x=half_l + (thickness * 0.5),
                center_y=0.0,
                center_z=wall_center_z,
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[3],
                size_x=thickness,
                size_y=workspace.width_m,
                size_z=wall_height,
                center_x=-half_l - (thickness * 0.5),
                center_y=0.0,
                center_z=wall_center_z,
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[4],
                size_x=workspace.length_m,
                size_y=thickness,
                size_z=wall_height,
                center_x=0.0,
                center_y=half_w + (thickness * 0.5),
                center_z=wall_center_z,
            ),
            self._make_box_collision_object(
                BOUNDARY_OBJECT_IDS[5],
                size_x=workspace.length_m,
                size_y=thickness,
                size_z=wall_height,
                center_x=0.0,
                center_y=-half_w - (thickness * 0.5),
                center_z=wall_center_z,
            ),
        ]

    def _clear_workspace_boundary_objects(self) -> None:
        from moveit_msgs.msg import CollisionObject

        remove_objects = []
        for object_id in BOUNDARY_OBJECT_IDS:
            remove_object = CollisionObject()
            remove_object.id = object_id
            remove_object.operation = CollisionObject.REMOVE
            remove_objects.append(remove_object)
        try:
            self._apply_planning_scene_objects(remove_objects)
        except Exception as exc:
            sys.stderr.write(f"UR3e MoveIt: failed to clear boundary objects: {exc}\n")

    def _validity_failure_reason(self, response: Any) -> str:
        # GetStateValidity.Response.contacts is ContactInformation[] (not nested).
        contact_infos = getattr(response, "contacts", None) or []
        if contact_infos:
            pairs: List[str] = []
            for contact in contact_infos:
                body1 = str(getattr(contact, "contact_body_1", "") or "")
                body2 = str(getattr(contact, "contact_body_2", "") or "")
                bodies = [body1, body2]
                pairs.append(f"{body1} vs {body2}")
                for body in bodies:
                    if "hyperfusion_boundary_" in body:
                        return (
                            "robot arm outside workspace boundary "
                            f"({body1} vs {body2})"
                        )
                if any(TOOL_PAYLOAD_LINK in body for body in bodies):
                    other = bodies[1] if TOOL_PAYLOAD_LINK in bodies[0] else bodies[0]
                    if "forearm" in other:
                        return "tool payload near forearm (pinch / fold risk)"
                    if "wrist_2" in other:
                        return "tool payload vs wrist_2 (BFS mesh corner / fold)"
                    if "wrist_1" in other:
                        return "tool payload vs wrist_1 (fold risk)"
                    if "upper_arm" in other:
                        return "tool payload vs upper arm (fold risk)"
                    if "hyperfusion_boundary_" in other:
                        return "tool payload outside workspace boundary"
                    return f"tool payload collision ({body1} vs {body2})"
            if pairs:
                return f"collision ({pairs[0]}" + (
                    f"; +{len(pairs) - 1} more)" if len(pairs) > 1 else ")"
                )
            return f"collision ({len(contact_infos)} contact(s))"
        return "collision or joint limit violation"

    def _apply_workspace_collision(self, workspace: WorkspaceBox) -> bool:
        """Install full workspace box collision (floor, walls, ceiling) for whole-arm checks."""
        with self._workspace_apply_cv:
            workspace_key = self._workspace_key(workspace)
            if self._workspace_applied == workspace_key:
                return True

            if self._plan_in_progress:
                # Never mutate the planning scene while MoveGroup is planning —
                # concurrent ApplyPlanningScene yields instant FAILURE (99999).
                sys.stderr.write(
                    "UR3e MoveIt: skip workspace boundary update during planning.\n"
                )
                sys.stderr.flush()
                return self._workspace_applied is not None

            # Single-flight: waiters share one apply instead of stampeding after timeout.
            deadline = time.time() + 90.0
            while self._workspace_apply_in_progress and time.time() < deadline:
                self._workspace_apply_cv.wait(timeout=0.5)
                if self._workspace_applied == workspace_key:
                    return True
            if self._workspace_applied == workspace_key:
                return True
            if self._workspace_apply_in_progress:
                sys.stderr.write(
                    "UR3e MoveIt: workspace boundary apply still in progress — skipping.\n"
                )
                sys.stderr.flush()
                return self._workspace_applied is not None

            if not workspace.enabled:
                if self._workspace_applied is not None:
                    self._clear_workspace_boundary_objects()
                self._workspace_applied = workspace_key
                sys.stderr.write("UR3e MoveIt: workspace boundary disabled.\n")
                sys.stderr.flush()
                return True

            self._workspace_apply_in_progress = True

        try:
            collision_objects = self._make_workspace_boundary_objects(workspace)
            sys.stderr.write(
                "UR3e MoveIt: applying workspace boundary collision objects…\n"
            )
            sys.stderr.flush()
            self._apply_planning_scene_objects(collision_objects)
            try:
                self._allow_base_ceiling_collisions()
            except Exception as exc:
                sys.stderr.write(
                    f"UR3e MoveIt: failed to apply base↔ceiling ACM ({exc}).\n"
                    "UR3e MoveIt: workspace walls published; ACM pending retry "
                    "(incomplete ACM replace would break camera↔wrist disables).\n"
                )
                sys.stderr.flush()
                # Do not cache as applied — keepalive / next home retries ACM safely.
                return True

            with self._workspace_apply_cv:
                self._workspace_applied = workspace_key
            z_bottom, z_top = workspace_vertical_bounds(workspace)
            sys.stderr.write(
                "UR3e MoveIt: workspace boundary enabled "
                f"(enclosed box {workspace.length_m:.3f} x {workspace.width_m:.3f} x "
                f"{(z_top - z_bottom):.3f} m; mount Z={workspace.mount_height_m:.3f} m, "
                f"box top Z={z_top:.3f} m, floor Z={z_bottom:.3f} m).\n"
            )
            sys.stderr.flush()
            return True
        finally:
            with self._workspace_apply_cv:
                self._workspace_apply_in_progress = False
                self._workspace_apply_cv.notify_all()

    @staticmethod
    def _moveit_error_name(code: int) -> str:
        from moveit_msgs.msg import MoveItErrorCodes

        names = {
            MoveItErrorCodes.SUCCESS: "success",
            MoveItErrorCodes.FAILURE: "failure",
            MoveItErrorCodes.PLANNING_FAILED: "planning failed",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "no collision-free path (invalid motion plan)",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: (
                "motion plan invalidated by environment change"
            ),
            MoveItErrorCodes.CONTROL_FAILED: "controller rejected trajectory",
            MoveItErrorCodes.START_STATE_INVALID: "start state invalid (joint limits?)",
            MoveItErrorCodes.GOAL_STATE_INVALID: "goal state invalid (joint limits?)",
            MoveItErrorCodes.NO_IK_SOLUTION: "no IK solution",
            MoveItErrorCodes.TIMED_OUT: "timed out",
            MoveItErrorCodes.PREEMPTED: "preempted",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "start state in collision",
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "start state violates constraints",
            MoveItErrorCodes.GOAL_IN_COLLISION: "goal in collision",
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "goal violates constraints",
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "goal constraints violated",
            MoveItErrorCodes.INVALID_GROUP_NAME: "invalid group name",
            MoveItErrorCodes.INVALID_LINK_NAME: "invalid link name",
            MoveItErrorCodes.INVALID_OBJECT_NAME: "invalid object name",
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "frame transform failure",
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "collision checking unavailable",
            MoveItErrorCodes.ROBOT_STATE_STALE: "robot state stale",
            MoveItErrorCodes.SENSOR_INFO_STALE: "sensor info stale",
        }
        return names.get(code, f"MoveIt error {code}")

    def _make_robot_state(self, joint_positions: Sequence[float]) -> Any:
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        if len(joint_positions) != 6:
            raise ValueError("joint seed must have 6 values")

        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        state.joint_state.position = [float(v) for v in joint_positions]
        return state

    def _publish_rviz_goal_state(self, joint_positions: Sequence[float]) -> None:
        """Update MoveIt RViz Query Goal State (orange ghost) during plan/execute."""
        if self._rviz_goal_pub is None:
            return
        try:
            self._rviz_goal_pub.publish(self._make_robot_state(joint_positions))
        except Exception as exc:
            sys.stderr.write(f"UR3e MoveIt: could not publish RViz goal state: {exc}\n")

    def update_manual_target_preview(
        self,
        joint_positions: Sequence[float],
        workspace: Optional[WorkspaceBox] = None,
    ) -> Dict[str, Any]:
        """Push the UI joint target into MoveIt/RViz and refresh the collision boundary."""
        if len(joint_positions) != 6:
            raise ValueError("Manual target must have 6 joint values.")

        goal_joints = [float(v) for v in joint_positions]
        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            ws = workspace if workspace is not None else self._last_workspace
            if ws is None:
                ws = WorkspaceBox(enabled=True)
            self._apply_workspace_collision(ws)
            self._publish_rviz_goal_state(goal_joints)

        return {"ok": True}

    def _strip_trajectory_derivatives(self, trajectory: Any) -> None:
        """UR hardware controller expects position-only trajectories with time_from_start."""
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None:
            return
        for point in joint_traj.points:
            point.velocities = []
            point.accelerations = []

    def _unwrap_trajectory_waypoints(
        self,
        joint_traj: Any,
        indices: Sequence[int],
        previous: Sequence[float],
        *,
        start_point_index: int = 0,
    ) -> float:
        """Rewrite trajectory samples onto the branch chain starting at *previous*."""
        max_step_rad = 0.0
        prev = [float(v) for v in previous]
        for point_index, point in enumerate(joint_traj.points):
            if point_index < start_point_index:
                continue
            if len(point.positions) <= max(indices):
                continue
            unwrapped: List[float] = []
            for joint_index, traj_index in enumerate(indices):
                raw = float(point.positions[traj_index])
                value = self._unwrap_joint_to_reference(
                    joint_index, raw, prev[joint_index]
                )
                unwrapped.append(value)
                # Numeric step (RTDE sees this); continuous unwrap keeps it small.
                max_step_rad = max(max_step_rad, abs(value - prev[joint_index]))
                point.positions[traj_index] = value
            prev = unwrapped
        return max_step_rad

    def _unwrap_trajectory_continuous(self, trajectory: Any) -> bool:
        """Remap trajectory joints to the hardware branch and keep waypoints continuous."""
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None or not joint_traj.joint_names or not joint_traj.points:
            return False

        name_to_index = {
            name: index for index, name in enumerate(joint_traj.joint_names)
        }
        indices: List[int] = []
        for joint_name in CANONICAL_JOINT_NAMES:
            if joint_name not in name_to_index:
                return False
            indices.append(name_to_index[joint_name])

        hardware = self._ensure_hardware_joint_positions(timeout_s=1.5)
        if hardware is not None:
            previous = [float(v) for v in hardware]
        else:
            anchor = execute_anchor_joints(
                hardware_rad=None,
                fallback_rad=self._moveit_start_joint_positions(),
            )
            if anchor is not None:
                previous = list(anchor)
            else:
                first = joint_traj.points[0]
                if len(first.positions) <= max(indices):
                    return False
                previous = [
                    float(first.positions[index]) for index in indices
                ]

        max_step_rad = self._unwrap_trajectory_waypoints(
            joint_traj, indices, previous, start_point_index=0
        )

        if hardware is not None and joint_traj.points:
            first = joint_traj.points[0]
            if len(first.positions) > max(indices):
                first_positions = [
                    float(first.positions[index]) for index in indices
                ]
                start_gap = self._joint_distance_rad(hardware, first_positions)
                for joint_index, traj_index in enumerate(indices):
                    first.positions[traj_index] = float(hardware[joint_index])
                if len(joint_traj.points) > 1:
                    tail_step = self._unwrap_trajectory_waypoints(
                        joint_traj,
                        indices,
                        hardware,
                        start_point_index=1,
                    )
                    max_step_rad = max(max_step_rad, tail_step)
                if start_gap > TRAJECTORY_START_TOLERANCE_RAD:
                    sys.stderr.write(
                        "UR3e MoveIt execute: trajectory start snapped to hardware joints "
                        f"(plan start was {math.degrees(start_gap):.1f}°·joint away).\n"
                    )

        if max_step_rad > math.radians(90.0):
            sys.stderr.write(
                "UR3e MoveIt execute: trajectory unwrap — "
                f"max joint step {math.degrees(max_step_rad):.1f}° between samples.\n"
            )
        return True

    @staticmethod
    def _trajectory_joint_indices(trajectory: Any) -> Optional[List[int]]:
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None or not joint_traj.joint_names:
            return None
        name_to_index = {
            name: index for index, name in enumerate(joint_traj.joint_names)
        }
        indices: List[int] = []
        for joint_name in CANONICAL_JOINT_NAMES:
            if joint_name not in name_to_index:
                return None
            indices.append(name_to_index[joint_name])
        return indices

    @staticmethod
    def _point_time_from_start_sec(point: Any) -> float:
        stamp = point.time_from_start
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _set_point_time_from_start_sec(point: Any, seconds: float) -> None:
        seconds = max(0.0, float(seconds))
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec

    def _trajectory_max_joint_velocity_rad_s(self, trajectory: Any) -> float:
        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if indices is None or joint_traj is None or len(joint_traj.points) < 2:
            return 0.0

        max_velocity = 0.0
        max_wrapped_velocity = 0.0
        previous_positions: Optional[List[float]] = None
        previous_time = 0.0
        for point in joint_traj.points:
            if len(point.positions) <= max(indices):
                continue
            positions = [float(point.positions[index]) for index in indices]
            time_s = self._point_time_from_start_sec(point)
            if previous_positions is not None:
                dt = time_s - previous_time
                if dt > 1.0e-6:
                    for prev, curr in zip(previous_positions, positions):
                        timing_delta = self._segment_joint_delta_for_timing(prev, curr)
                        wrapped_delta = abs(self._joint_delta_rad(prev, curr))
                        velocity = timing_delta / dt
                        if velocity > max_velocity:
                            max_velocity = velocity
                        if wrapped_delta / dt > max_wrapped_velocity:
                            max_wrapped_velocity = wrapped_delta / dt
            previous_positions = positions
            previous_time = time_s
        if (
            max_velocity > max_wrapped_velocity * 1.5
            and math.degrees(max_velocity - max_wrapped_velocity) > 10.0
        ):
            sys.stderr.write(
                "UR3e MoveIt execute: trajectory has numeric branch jumps — "
                f"RTDE peak {math.degrees(max_velocity):.0f} deg/s, "
                f"wrapped {math.degrees(max_wrapped_velocity):.0f} deg/s.\n"
            )
        return max_velocity

    def _trajectory_branch_continuity_ok(
        self,
        trajectory: Any,
        *,
        quiet: bool = False,
    ) -> bool:
        """Reject trajectories whose numeric joint samples still jump by ≥90° after unwrap.

        wrist_3 may take up to ~1 full turn when we intentionally bias toward the
        home cable branch; timing stretch handles the slower spin.
        """
        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if indices is None or joint_traj is None or len(joint_traj.points) < 2:
            return True

        max_raw_deg = 0.0
        worst_joint = ""
        prev_positions: Optional[List[float]] = None
        for point in joint_traj.points:
            if len(point.positions) <= max(indices):
                continue
            positions = [float(point.positions[index]) for index in indices]
            if prev_positions is not None:
                for joint_index, (prev, curr) in enumerate(
                    zip(prev_positions, positions)
                ):
                    raw_deg = math.degrees(abs(curr - prev))
                    limit_deg = 370.0 if joint_index == 5 else 90.0
                    if raw_deg >= limit_deg and raw_deg > max_raw_deg:
                        max_raw_deg = raw_deg
                        worst_joint = CANONICAL_JOINT_NAMES[joint_index]
            prev_positions = positions

        if not worst_joint:
            return True

        if not quiet:
            detail = (
                "(cable unwrap exceeded one turn)."
                if worst_joint == "wrist_3_joint"
                else "(branch unwrap failed; UR would see >100 deg/s)."
            )
            sys.stderr.write(
                "UR3e MoveIt execute: rejecting trajectory — numeric "
                f"{worst_joint} step {max_raw_deg:.0f}° between samples "
                f"{detail}\n"
            )
        return False

    def _bias_trajectory_wrist3_toward_home(self, trajectory: Any) -> bool:
        """Optionally nudge wrist_3 toward home — never by a full ±360° spin.

        Full-turn home bias was yanking the BFS USB cable. When the only way to
        get closer to home is ±2π, leave the short unwrap and let post-pin
        cable rewind clear the wind at home instead.
        """
        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if (
            indices is None
            or joint_traj is None
            or len(joint_traj.points) < 2
            or len(indices) < 6
        ):
            return False

        hardware = self._ensure_hardware_joint_positions(timeout_s=0.5)
        if hardware is None or len(hardware) != 6:
            return False

        home = self.home_joints_rad()
        w3_traj = indices[5]
        last = joint_traj.points[-1]
        if len(last.positions) <= w3_traj:
            return False

        live_w3 = float(hardware[5])
        final_w3 = float(last.positions[w3_traj])
        preferred = unwrap_joint_toward_preferred(
            live_w3,
            final_w3,
            float(home[5]),
            max_extra_turns=1,
            max_travel_rad=math.pi,  # cable-safe: no full-turn path on pin trajs
        )
        offset = preferred - final_w3
        if abs(offset) < math.radians(1.0):
            # Would a full-turn bias have been “closer” to home? Log once for debug.
            full_turn_pref = unwrap_joint_toward_preferred(
                live_w3,
                final_w3,
                float(home[5]),
                max_extra_turns=1,
                max_travel_rad=1.5 * TWO_PI,
            )
            full_offset = full_turn_pref - final_w3
            if abs(full_offset) >= math.pi - 1.0e-6:
                sys.stderr.write(
                    "UR3e MoveIt execute: wrist_3 skip full-turn home bias "
                    f"({math.degrees(final_w3):.0f}° ↛ {math.degrees(full_turn_pref):.0f}°); "
                    f"live {math.degrees(live_w3):.0f}° vs home {math.degrees(home[5]):.0f}° — "
                    "post-pin cable rewind will clear if |Δ|≥180°.\n"
                )
            return False

        if abs(offset) >= math.pi - 1.0e-6:
            sys.stderr.write(
                "UR3e MoveIt execute: wrist_3 refuse full-turn home bias "
                f"({math.degrees(final_w3):.0f}° → {math.degrees(preferred):.0f}°).\n"
            )
            return False

        for point in joint_traj.points[1:]:
            if len(point.positions) > w3_traj:
                point.positions[w3_traj] = float(point.positions[w3_traj]) + offset

        sys.stderr.write(
            "UR3e MoveIt execute: wrist_3 home-branch unwrap "
            f"{math.degrees(final_w3):.0f}° → {math.degrees(preferred):.0f}° "
            f"(home ref {math.degrees(home[5]):.0f}°, Δ {math.degrees(offset):+.0f}°).\n"
        )
        return True

    def _prepare_trajectory_from_start(
        self,
        trajectory: Any,
        start_joints: Sequence[float],
    ) -> bool:
        """Unwrap trajectory onto *start_joints* branch (plan-time executable check)."""
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        indices = self._trajectory_joint_indices(trajectory)
        if (
            joint_traj is None
            or indices is None
            or not joint_traj.points
            or len(start_joints) != 6
        ):
            return False

        start = [float(v) for v in start_joints]
        self._unwrap_trajectory_waypoints(
            joint_traj, indices, start, start_point_index=0
        )
        first = joint_traj.points[0]
        if len(first.positions) <= max(indices):
            return False
        for joint_index, traj_index in enumerate(indices):
            first.positions[traj_index] = start[joint_index]
        if len(joint_traj.points) > 1:
            self._unwrap_trajectory_waypoints(
                joint_traj, indices, start, start_point_index=1
            )
        return self._trajectory_branch_continuity_ok(trajectory, quiet=True)

    def _stretch_trajectory_segment_times(self, trajectory: Any) -> None:
        """Ensure each segment respects joint speed and UR External Control sample period."""
        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if indices is None or joint_traj is None or len(joint_traj.points) < 2:
            return

        limit = configured_max_joint_velocity_rad_s()
        margin = UR3E_JOINT_VELOCITY_TIME_MARGIN
        prev_positions: Optional[List[float]] = None
        prev_time = 0.0
        for point_index, point in enumerate(joint_traj.points):
            if len(point.positions) <= max(indices):
                continue
            positions = [float(point.positions[index]) for index in indices]
            if point_index == 0:
                self._set_point_time_from_start_sec(point, 0.0)
                prev_positions = positions
                prev_time = 0.0
                continue

            dt_required = MIN_TRAJECTORY_SEGMENT_S
            if prev_positions is not None:
                for prev, curr in zip(prev_positions, positions):
                    delta = self._segment_joint_delta_for_timing(prev, curr)
                    if delta > 1e-9:
                        dt_required = max(dt_required, (delta / limit) * margin)

            new_time = max(self._point_time_from_start_sec(point), prev_time + dt_required)
            self._set_point_time_from_start_sec(point, new_time)
            prev_time = new_time
            prev_positions = positions

    def _ensure_trajectory_hardware_start(self, trajectory: Any) -> None:
        """Insert a t=0 hold at live RTDE joints so UR never sees a 2 ms first step."""
        from trajectory_msgs.msg import JointTrajectoryPoint

        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        hardware = self._ensure_hardware_joint_positions(timeout_s=0.5)
        if (
            indices is None
            or joint_traj is None
            or not joint_traj.points
            or hardware is None
        ):
            return

        if len(joint_traj.points) == 1:
            goal = joint_traj.points[0]
            start = JointTrajectoryPoint()
            start.positions = list(goal.positions)
            for joint_index, traj_index in enumerate(indices):
                start.positions[traj_index] = float(hardware[joint_index])
            self._set_point_time_from_start_sec(start, 0.0)
            joint_traj.points = [start, goal]
            self._unwrap_trajectory_waypoints(
                joint_traj, indices, hardware, start_point_index=1
            )
            return

        first = joint_traj.points[0]
        if len(first.positions) <= max(indices):
            return

        first_time = self._point_time_from_start_sec(first)
        first_positions = [float(first.positions[index]) for index in indices]
        start_gap = self._joint_distance_rad(hardware, first_positions)
        if first_time <= 1e-6 and start_gap <= TRAJECTORY_START_TOLERANCE_RAD:
            return

        hold = JointTrajectoryPoint()
        hold.positions = list(first.positions)
        for joint_index, traj_index in enumerate(indices):
            hold.positions[traj_index] = float(hardware[joint_index])
        self._set_point_time_from_start_sec(hold, 0.0)
        joint_traj.points.insert(0, hold)
        self._unwrap_trajectory_waypoints(
            joint_traj, indices, hardware, start_point_index=1
        )

    def _prepare_trajectory_for_robot(self, trajectory: Any) -> bool:
        """Unwrap ±π joint branches and enforce configurable peak joint velocity."""
        if not self._unwrap_trajectory_continuous(trajectory):
            return False

        self._ensure_trajectory_hardware_start(trajectory)
        self._bias_trajectory_wrist3_toward_home(trajectory)
        self._strip_trajectory_derivatives(trajectory)
        self._stretch_trajectory_segment_times(trajectory)

        if not self._trajectory_branch_continuity_ok(trajectory):
            return False

        max_velocity = self._trajectory_max_joint_velocity_rad_s(trajectory)
        limit = configured_max_joint_velocity_rad_s()
        limit_deg = configured_max_joint_velocity_deg_s()
        if max_velocity <= limit * UR3E_JOINT_VELOCITY_TIME_MARGIN:
            if max_velocity > 0.0:
                sys.stderr.write(
                    "UR3e MoveIt execute: trajectory peak joint velocity "
                    f"{math.degrees(max_velocity):.1f} deg/s (RTDE timing, "
                    f"limit {limit_deg:.0f} deg/s).\n"
                )
            return True

        scale = (max_velocity / limit) * UR3E_JOINT_VELOCITY_TIME_MARGIN
        if scale > TRAJECTORY_MAX_VELOCITY_SCALEUP:
            sys.stderr.write(
                "UR3e MoveIt execute: trajectory peak joint velocity "
                f"{math.degrees(max_velocity):.0f} deg/s exceeds "
                f"{limit_deg:.0f} deg/s even after unwrap — rejecting.\n"
            )
            return False

        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None:
            return False
        for point in joint_traj.points:
            self._set_point_time_from_start_sec(
                point,
                self._point_time_from_start_sec(point) * scale,
            )
        sys.stderr.write(
            "UR3e MoveIt execute: slowed trajectory "
            f"{scale:.2f}x to respect {limit_deg:.0f} deg/s joint limit.\n"
        )
        return True

    @classmethod
    def _joint_permutation_variants(
        cls,
        base: Sequence[float],
        offsets: Sequence[float],
    ) -> List[List[float]]:
        """Base seed plus one-joint offsets from *offsets*."""
        if len(base) != 6:
            return []
        base_list = [float(v) for v in base]
        variants: List[List[float]] = [list(base_list)]
        for joint_index in range(6):
            for offset in offsets:
                perturbed = list(base_list)
                perturbed[joint_index] += offset
                variants.append(perturbed)
        return variants

    def _generate_ik_seeds(
        self,
        home: Sequence[float],
        *,
        current_pin: Optional[Sequence[float]] = None,
    ) -> List[List[float]]:
        """Build IK seeds: home + current pin; coarse (90°) then fine (45°) permutations."""
        seen: set[tuple[float, ...]] = set()
        seeds: List[List[float]] = []

        def add(seed: Optional[Sequence[float]]) -> None:
            if seed is None or len(seed) != 6:
                return
            key = tuple(round(float(v), 4) for v in seed)
            if key in seen:
                return
            seen.add(key)
            seeds.append([float(v) for v in seed])

        base_seeds: List[Sequence[float]] = []
        if len(home) == 6:
            base_seeds.append(home)
        if current_pin is not None and len(current_pin) == 6:
            base_seeds.append(current_pin)

        # Coarse first so easy solutions hit early; fine offsets only after.
        for offsets in (IK_SEED_JOINT_OFFSETS_COARSE_RAD, IK_SEED_JOINT_OFFSETS_FINE_RAD):
            for base in base_seeds:
                for variant in self._joint_permutation_variants(base, offsets):
                    add(variant)

        return seeds

    def _start_to_goal_path_ok(
        self,
        start_joints: Sequence[float],
        goal_joints: Sequence[float],
        move_client: Any,
        *,
        label: str = "start",
    ) -> tuple[bool, str]:
        """Plan-only check: start→goal path that also survives execute unwrap rules."""
        from moveit_msgs.msg import MoveItErrorCodes

        if len(start_joints) != 6 or len(goal_joints) != 6:
            return False, f"invalid joint vectors for {label}→pin path check"

        if self._joint_distance_rad(start_joints, goal_joints) <= HOME_JOINT_TOLERANCE_RAD:
            return True, ""

        start = [float(v) for v in start_joints]
        goal = coalesce_joints_for_moveit(start, goal_joints)
        last_error = f"no collision-free path from {label}"
        planners = self._planners_without_ptp_if_elbow_flip(
            start, goal, PLAN_PATH_CHECK_PLANNERS
        )

        for pipeline_id, planner_id in planners:
            try:
                error_code, planned = self._plan_move_group_joint_goal(
                    goal,
                    move_client,
                    pipeline_id=pipeline_id,
                    planner_id=planner_id,
                    motion_scale=0.15,
                    start_joints=start,
                    allowed_planning_time=PLAN_PATH_CHECK_TIME_S,
                    num_planning_attempts=PLAN_PATH_CHECK_ATTEMPTS,
                )
            except RuntimeError as exc:
                last_error = str(exc)
                continue

            if error_code != MoveItErrorCodes.SUCCESS or planned is None:
                last_error = (
                    f"no collision-free path from {label} "
                    f"({self._moveit_error_name(error_code)}, code={error_code})"
                )
                continue

            prepared = copy.deepcopy(planned)
            if not self._prepare_trajectory_from_start(prepared, start):
                last_error = (
                    f"no collision-free path from {label} (unwrap/branch failed)"
                )
                continue

            waypoints = self._trajectory_joint_waypoints(prepared)
            if len(waypoints) < 2:
                last_error = f"no collision-free path from {label} (empty trajectory)"
                continue
            soft_ok, soft_reason = self._trajectory_soft_joint_limits_ok(waypoints)
            if not soft_ok:
                last_error = (
                    f"no collision-free path from {label} ({soft_reason})"
                )
                continue
            mesh_ok, mesh_reason = self._trajectory_mesh_collision_ok(waypoints)
            if not mesh_ok:
                last_error = f"no collision-free path from {label} ({mesh_reason})"
                continue
            pinch_ok, _gap_m, pinch_reason = self._trajectory_pinch_ok(waypoints)
            if not pinch_ok:
                last_error = f"no collision-free path from {label} ({pinch_reason})"
                continue
            return True, ""

        return False, last_error

    def _home_to_pin_path_ok(
        self,
        home_joints: Sequence[float],
        pin_joints: Sequence[float],
        move_client: Any,
    ) -> tuple[bool, str]:
        """Backward-compatible alias for home→pin path+unwrap check."""
        return self._start_to_goal_path_ok(
            home_joints, pin_joints, move_client, label="home"
        )
    def _solve_ik(
        self,
        pose: Sequence[float],
        seed_joints: Sequence[float],
        *,
        frame_id: str = PLANNING_FRAME,
        avoid_collisions: bool = True,
    ) -> tuple[Optional[List[float]], str]:
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import MoveItErrorCodes

        if len(seed_joints) != 6:
            return None, "no joint seed (robot state unavailable)"

        x_m, y_m, z_m, rx, ry, rz = pose
        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP_NAME
        request.ik_request.ik_link_name = EE_LINK
        request.ik_request.avoid_collisions = bool(avoid_collisions)
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = frame_id
        request.ik_request.pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = float(x_m)
        request.ik_request.pose_stamped.pose.position.y = float(y_m)
        request.ik_request.pose_stamped.pose.position.z = float(z_m)
        qx, qy, qz, qw = rotvec_to_quaternion(rx, ry, rz)
        request.ik_request.pose_stamped.pose.orientation.x = qx
        request.ik_request.pose_stamped.pose.orientation.y = qy
        request.ik_request.pose_stamped.pose.orientation.z = qz
        request.ik_request.pose_stamped.pose.orientation.w = qw
        request.ik_request.robot_state = self._make_robot_state(seed_joints)

        future = self._ik_client.call_async(request)
        response = self._wait_future(future, 10.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, self._moveit_error_name(response.error_code.val)

        name_to_pos = {
            name: pos
            for name, pos in zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        }
        joints = [float(name_to_pos.get(name, 0.0)) for name in CANONICAL_JOINT_NAMES]
        joints = self._normalize_joint_solution_to_reference(seed_joints, joints)
        return joints, ""

    def _collect_ik_solutions(
        self,
        pose: Sequence[float],
        reference_joints: Sequence[float],
        ik_seeds: Sequence[Sequence[float]],
        *,
        desired_tool_z: Optional[Sequence[float]] = None,
        max_solutions: int = PLAN_IK_MAX_CANDIDATES,
        prefer_collision_free_ik: bool = True,
        relax_apex_self_collision: bool = False,
    ) -> tuple[List[List[float]], str]:
        """Collect up to *max_solutions* unique valid IK joint sets (unsorted)."""
        last_error = "no IK solution"
        seen_solutions: set[tuple[float, ...]] = set()
        solutions: List[List[float]] = []
        check_tool_z = (
            desired_tool_z is not None
            and len(desired_tool_z) == 3
            and not self._use_mock_hardware()
        )
        # Apex / near-home vertical: try collision-aware IK first, then allow
        # collision-blind IK + explicit validity (avoids false NO_IK near walls).
        avoid_modes = (True,) if prefer_collision_free_ik else (True, False)

        def _apex_self_collision(reason: str) -> bool:
            low = (reason or "").lower()
            return "forearm" in low or "pinch" in low or "fold" in low

        for seed_joints in ik_seeds:
            if len(solutions) >= max_solutions:
                break
            seed_for_ik = self._normalize_joint_solution_to_reference(
                reference_joints, seed_joints
            )
            if not self._ik_joint_angles_are_sane(seed_for_ik):
                continue
            seed_valid, seed_reason = self._state_is_valid(seed_for_ik)
            if not seed_valid:
                if seed_reason:
                    last_error = seed_reason
                # Home itself is mesh-vs-forearm in this payload. Keep every seed
                # for Z-only apex so IK can start from the live home family.
                if not relax_apex_self_collision:
                    continue

            joints = None
            ik_error = "no IK solution"
            for avoid in avoid_modes:
                joints, ik_error = self._solve_ik(
                    pose, seed_for_ik, avoid_collisions=avoid
                )
                if joints is not None:
                    break
            if joints is None:
                if ik_error:
                    last_error = ik_error
                continue

            joints = self._normalize_joint_solution_to_reference(reference_joints, joints)
            if not self._ik_joint_angles_are_sane(joints):
                last_error = "invalid IK joint angles"
                continue

            solution_key = tuple(round(v, 4) for v in joints)
            if solution_key in seen_solutions:
                continue
            seen_solutions.add(solution_key)

            valid, reason = self._state_is_valid(joints)
            if not valid:
                if relax_apex_self_collision and _apex_self_collision(reason or ""):
                    pass
                else:
                    last_error = reason or "invalid state"
                    continue

            if check_tool_z:
                alignment = self._tool_z_alignment(joints, desired_tool_z)
                if alignment is not None and alignment < TOOL_Z_ALIGNMENT_MIN_DOT:
                    if alignment < 0.0:
                        last_error = "IK tool Z faces away from dome center"
                    else:
                        last_error = "IK tool Z misaligned with dome center"
                    continue

            solutions.append(list(joints))

        return solutions, last_error

    def _solve_ik_multi_seed(
        self,
        pose: Sequence[float],
        reference_joints: Sequence[float],
        ik_seeds: Sequence[Sequence[float]],
        *,
        desired_tool_z: Optional[Sequence[float]] = None,
        prefer_collision_free_ik: bool = True,
    ) -> tuple[Optional[List[float]], str, bool]:
        """Try IK seeds in order; return the first collision-free solution."""
        solutions, last_error = self._collect_ik_solutions(
            pose,
            reference_joints,
            ik_seeds,
            desired_tool_z=desired_tool_z,
            max_solutions=1,
            prefer_collision_free_ik=prefer_collision_free_ik,
        )
        if not solutions:
            return None, last_error, False
        # With max_solutions=1, "recovered" means we had to walk past seed 0 — unknown here.
        return solutions[0], last_error, False

    def _pick_ik_closest_to_home_with_path(
        self,
        pose: Sequence[float],
        home_joints: Sequence[float],
        ik_seeds: Sequence[Sequence[float]],
        move_client: Any,
        *,
        last_pin_joints: Optional[Sequence[float]] = None,
        desired_tool_z: Optional[Sequence[float]] = None,
        prefer_collision_free_ik: bool = True,
        allow_previous_pin_path: bool = True,
        require_base_sweep: bool = False,
        base_sweep_samples: int = 36,
        relax_apex_self_collision: bool = False,
    ) -> tuple[Optional[List[float]], str, bool, bool, Optional[List[bool]]]:
        """Collect IK candidates; prefer any home→pin path before previous→pin.

        Returns (joints, error, recovered, home_path_ok, partial_abs_pan_mask).

        Two-pass pick (critical for hemisphere rings):
        1) Scan *all* IK candidates for a home→pin path+unwrap.
        2) Only if none work, accept previous→pin (plan-order chain).

        When ``require_base_sweep`` (semi rings): 360° pan at frozen joints
        runs before any path plan. Candidates that fail pan never pay OMPL.
        Best partial-pan IK is returned with ``home_path_ok=False`` and an
        absolute pan mask for backup pair search.

        Prefers the home elbow sign so mirror azimuths keep the same arm family
        instead of an elbow-flip branch. Soft joint-limit clearance still ranks
        within each family.
        """
        slide_err = ""
        if relax_apex_self_collision:
            slid, slide_err = self._solve_apex_z_slide(pose, home_joints)
            if slid is not None:
                return slid, "", False, True, None
        solutions, collect_error = self._collect_ik_solutions(
            pose,
            home_joints,
            ik_seeds,
            desired_tool_z=desired_tool_z,
            max_solutions=PLAN_IK_MAX_CANDIDATES,
            prefer_collision_free_ik=prefer_collision_free_ik,
            relax_apex_self_collision=relax_apex_self_collision,
        )
        if not solutions:
            return None, slide_err or collect_error or "no IK solution", False, False, None
        if relax_apex_self_collision:
            closest = min(
                solutions,
                key=lambda j: self._joint_distance_rad(home_joints, j),
            )
            normalized = self._normalize_joint_solution_to_reference(home_joints, closest)
            if normalized is not None and self._ik_joint_angles_are_sane(normalized):
                return normalized, "", False, True, None

        home_elbow = float(home_joints[2])

        def rank_key(joints: Sequence[float]) -> tuple[float, float, float]:
            elbow_flip = 1.0 if home_elbow * float(joints[2]) < 0.0 else 0.0
            clearance = joint_limit_clearance_rad(joints)
            return (
                elbow_flip,
                -clearance,
                self._joint_distance_rad(home_joints, joints),
            )

        ranked = sorted(solutions, key=rank_key)
        soft_ok = [
            j for j in ranked if not near_soft_joint_limit(j, margin_rad=SOFT_JOINT_LIMIT_MARGIN_RAD)
        ]
        ordered: List[Sequence[float]] = list(soft_ok) + [
            j for j in ranked if j not in soft_ok
        ]
        if soft_ok and len(soft_ok) < len(ranked):
            sys.stderr.write(
                f"UR3e MoveIt: preferring {len(soft_ok)}/{len(ranked)} IK candidate(s) "
                f"≥{math.degrees(SOFT_JOINT_LIMIT_MARGIN_RAD):.0f}° from joint limits.\n"
            )

        def normalize_candidate(
            joints: Sequence[float],
        ) -> Optional[List[float]]:
            normalized = self._normalize_joint_solution_to_reference(home_joints, joints)
            if not self._ik_joint_angles_are_sane(normalized):
                return None
            return normalized

        last_path_error = "no collision-free path from home or previous pin"
        last_sweep_error = ""
        any_sweep_ok = False
        best_partial_joints: Optional[List[float]] = None
        best_partial_mask: Optional[List[bool]] = None
        best_partial_bits = -1

        # Pass 1: optional 360° pan, then home→pin (do not early-accept previous→pin).
        for rank_index, joints in enumerate(ordered):
            normalized = normalize_candidate(joints)
            if normalized is None:
                last_path_error = "invalid IK joint angles"
                continue

            if require_base_sweep:
                sweep_ok, sweep_err = self._base_sweep_ok(
                    normalized, samples=base_sweep_samples
                )
                if not sweep_ok:
                    last_sweep_error = sweep_err or "base_sweep failed"
                    abs_mask = self._base_sweep_abs_mask(
                        normalized, samples=base_sweep_samples
                    )
                    bits = sum(1 for bit in abs_mask if bit)
                    if bits > best_partial_bits:
                        best_partial_bits = bits
                        best_partial_joints = list(normalized)
                        best_partial_mask = abs_mask
                    continue
                any_sweep_ok = True

            home_ok, home_error = self._start_to_goal_path_ok(
                home_joints, normalized, move_client, label="home"
            )
            if not home_ok:
                last_path_error = home_error or last_path_error
                continue

            if near_soft_joint_limit(normalized):
                sys.stderr.write(
                    "UR3e MoveIt: accepted pin IK within soft joint-limit margin "
                    f"(clearance {math.degrees(joint_limit_clearance_rad(normalized)):.1f}°) "
                    "— no ≥"
                    f"{math.degrees(SOFT_JOINT_LIMIT_MARGIN_RAD):.0f}° alternative "
                    "with a clear home→pin path.\n"
                )
            return normalized, "", rank_index > 0, True, None

        if require_base_sweep and last_sweep_error and not any_sweep_ok:
            if best_partial_joints is not None and best_partial_mask:
                return (
                    best_partial_joints,
                    last_sweep_error,
                    False,
                    False,
                    best_partial_mask,
                )
            return None, last_sweep_error, False, False, None

        if (
            require_base_sweep
            or not allow_previous_pin_path
            or last_pin_joints is None
            or len(last_pin_joints) != 6
        ):
            if require_base_sweep and best_partial_joints is not None and best_partial_mask:
                return (
                    best_partial_joints,
                    last_path_error or last_sweep_error or "base_sweep failed",
                    False,
                    False,
                    best_partial_mask,
                )
            return None, last_path_error, False, False, None

        # Pass 2: previous→pin only after no home→pin candidate existed.
        prev_start = self._normalize_joint_solution_to_reference(
            last_pin_joints, last_pin_joints
        )
        for rank_index, joints in enumerate(ordered):
            normalized = normalize_candidate(joints)
            if normalized is None:
                last_path_error = "invalid IK joint angles"
                continue

            goal_from_prev = coalesce_joints_for_moveit(prev_start, normalized)
            prev_ok, prev_error = self._start_to_goal_path_ok(
                prev_start, goal_from_prev, move_client, label="previous pin"
            )
            if not prev_ok:
                last_path_error = prev_error or last_path_error
                continue

            sys.stderr.write(
                "UR3e MoveIt: pin IK has previous→pin path only "
                "(no home→pin among candidates) — execute may need live IK "
                "recovery if the ring sweep approaches from home.\n"
            )
            if near_soft_joint_limit(normalized):
                sys.stderr.write(
                    "UR3e MoveIt: accepted pin IK within soft joint-limit margin "
                    f"(clearance {math.degrees(joint_limit_clearance_rad(normalized)):.1f}°) "
                    "via previous-pin path — no ≥"
                    f"{math.degrees(SOFT_JOINT_LIMIT_MARGIN_RAD):.0f}° alternative.\n"
                )
            return normalized, "", True, False, None

        return None, last_path_error, False, False, None

    def _state_is_valid(
        self,
        joints: Sequence[float],
        *,
        check_pinch: bool = True,
    ) -> tuple[bool, str]:
        from moveit_msgs.srv import GetStateValidity
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        request = GetStateValidity.Request()
        request.group_name = GROUP_NAME
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        request.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._validity_client.call_async(request)
        response = self._wait_future(future, 10.0)
        if not response.valid:
            reason = self._validity_failure_reason(response)
            return False, reason

        if check_pinch:
            pinch_ok, pinch_reason = self._ur_pinch_guard_ok(joints)
            if not pinch_ok:
                return False, pinch_reason
        return True, ""

    def _base_sweep_ok(
        self,
        joints: Sequence[float],
        *,
        samples: int = 36,
    ) -> tuple[bool, str]:
        """Plan-time check: shoulder_pan full turn at fixed other joints is valid."""
        if joints is None or len(joints) != 6:
            return False, "base_sweep: invalid joints"
        n = max(8, int(samples))
        base = [float(v) for v in joints]
        pan0 = base[0]
        for i in range(n):
            sample = list(base)
            sample[0] = pan0 + (2.0 * math.pi) * (float(i) / float(n))
            ok, reason = self._state_is_valid(sample, check_pinch=True)
            if not ok:
                return False, reason or f"base_sweep collision at sample {i}/{n}"
        return True, ""

    def _base_sweep_abs_mask(
        self,
        joints: Sequence[float],
        *,
        samples: int = 36,
    ) -> List[bool]:
        """Validity on a shared 0..360° pan grid (other joints frozen)."""
        if joints is None or len(joints) != 6:
            return []
        n = max(8, int(samples))
        base = [float(v) for v in joints]
        mask: List[bool] = []
        for i in range(n):
            sample = list(base)
            sample[0] = wrap_to_pi((2.0 * math.pi) * (float(i) / float(n)))
            ok, _reason = self._state_is_valid(sample, check_pinch=True)
            mask.append(bool(ok))
        return mask

    def _accept_max_union_backup_pairs(
        self,
        results: List[ScanPoseResult],
        partial_by_index: Dict[int, Dict[str, Any]],
        sweep_ok_by_ring: Dict[int, int],
        home_joints: Sequence[float],
        move_client: Any,
        *,
        min_union_deg: float,
    ) -> int:
        """If a ring has no full-spin pin, keep a complementary 2-pin pan pair.

        Rank by the executed contiguous island from each pin's entry (what
        Execute actually pans), not the holey raw mask. Reject same-half pairs
        (high overlap / little unique coverage). ``backup_union_deg`` is that
        executed union. ``min_union_deg`` boosts pairs that reach that coverage.
        """
        min_exec_union_deg = 180.0
        min_unique_each_deg = 60.0
        max_overlap_frac = 0.50

        by_ring: Dict[int, List[Dict[str, Any]]] = {}
        for target_index, item in partial_by_index.items():
            ring_key = int(item["ring_key"])
            if sweep_ok_by_ring.get(ring_key, 0) > 0:
                continue
            row = dict(item)
            row["target_index"] = int(target_index)
            by_ring.setdefault(ring_key, []).append(row)

        result_by_index = {int(item.index): item for item in results}
        accepted = 0
        for ring_key, cands in by_ring.items():
            if len(cands) < 2:
                continue
            ranked: List[
                tuple[
                    float,
                    float,
                    float,
                    float,
                    Dict[str, Any],
                    Dict[str, Any],
                    List[bool],
                    List[bool],
                ]
            ] = []
            skipped_same = 0
            for i in range(len(cands)):
                for j in range(i + 1, len(cands)):
                    left = cands[i]
                    right = cands[j]
                    joints_l = left.get("joints") or []
                    joints_r = right.get("joints") or []
                    if len(joints_l) != 6 or len(joints_r) != 6:
                        continue
                    exec_l = pan_contiguous_mask_from_entry(
                        left.get("mask") or [], float(joints_l[0])
                    )
                    exec_r = pan_contiguous_mask_from_entry(
                        right.get("mask") or [], float(joints_r[0])
                    )
                    cov_l = pan_mask_coverage_deg(exec_l)
                    cov_r = pan_mask_coverage_deg(exec_r)
                    only_l = pan_only_coverage_deg(exec_l, exec_r)
                    only_r = pan_only_coverage_deg(exec_r, exec_l)
                    overlap = pan_overlap_coverage_deg(exec_l, exec_r)
                    union_deg = pan_union_coverage_deg(exec_l, exec_r)
                    smaller = min(cov_l, cov_r)
                    same_half = smaller > 1.0e-9 and overlap >= max_overlap_frac * smaller
                    if (
                        same_half
                        or only_l < min_unique_each_deg
                        or only_r < min_unique_each_deg
                        or union_deg < min_exec_union_deg
                    ):
                        skipped_same += 1
                        continue
                    pref = 1.0 if union_deg + 1.0e-9 >= float(min_union_deg) else 0.0
                    ranked.append(
                        (
                            pref,
                            only_l + only_r,
                            union_deg,
                            -overlap,
                            left,
                            right,
                            exec_l,
                            exec_r,
                        )
                    )
            ranked.sort(
                key=lambda row: (row[0], row[1], row[2], row[3]), reverse=True
            )
            if not ranked:
                if skipped_same > 0:
                    sys.stderr.write(
                        "UR3e MoveIt: ring backup skipped — "
                        f"{skipped_same} pair(s) were same-half / low unique pan.\n"
                    )
                continue
            for _pref, unique_deg, union_deg, _neg_ov, left, right, exec_l, exec_r in ranked:
                pair_ok = True
                for cand in (left, right):
                    joints = [float(v) for v in cand["joints"]]
                    home_ok, _err = self._start_to_goal_path_ok(
                        home_joints, joints, move_client, label="home"
                    )
                    if not home_ok:
                        pair_ok = False
                        break
                    back_ok, _err = self._start_to_goal_path_ok(
                        joints, home_joints, move_client, label="pin→home"
                    )
                    if not back_ok:
                        pair_ok = False
                        break
                if not pair_ok:
                    continue
                exec_masks = (exec_l, exec_r)
                for cand, exec_mask in zip((left, right), exec_masks):
                    result = result_by_index.get(int(cand["target_index"]))
                    sample = cand["sample"]
                    if result is None:
                        continue
                    result.reachable = True
                    result.home_path_ok = True
                    result.base_sweep_ok = False
                    result.backup_coverage_ok = True
                    result.backup_union_deg = float(union_deg)
                    result.pan_mask = [bool(v) for v in exec_mask]
                    result.joint_positions = [float(v) for v in cand["joints"]]
                    result.cone_tip_deg = float(cand["tip_deg"])
                    result.error = ""
                    result.tcp_x_m = float(sample.x_m)
                    result.tcp_y_m = float(sample.y_m)
                    result.tcp_z_m = float(sample.z_m)
                    result.tcp_rx = float(sample.rx)
                    result.tcp_ry = float(sample.ry)
                    result.tcp_rz = float(sample.rz)
                    result.tool_z_x = float(sample.tool_z_x)
                    result.tool_z_y = float(sample.tool_z_y)
                    result.tool_z_z = float(sample.tool_z_z)
                    accepted += 1
                sys.stderr.write(
                    "UR3e MoveIt: ring backup pair "
                    f"φ={float(left['sample'].phi_deg):g}°+"
                    f"{float(right['sample'].phi_deg):g}° "
                    f"exec_union={union_deg:.1f}° unique={unique_deg:.1f}° "
                    "(complementary halves).\n"
                )
                break
        return accepted

    def plan_poses(
        self,
        targets: Sequence[ScanPoseTarget],
        workspace: WorkspaceBox,
        *,
        initial_seed: Optional[Sequence[float]] = None,
        pin_pose_tolerance_deg: float = 0.0,
        lock_camera_up: bool = True,
        semi_ring_sweep: bool = False,
        semi_max_sweep_ok_per_ring: int = 2,
        semi_ring_search_candidates: int = 360,
        semi_backup_coverage_deg: float = 300.0,
    ) -> List[ScanPoseResult]:
        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            self._last_workspace = workspace
            self._apply_workspace_collision(workspace)

            if initial_seed is None or len(initial_seed) != 6:
                raise RuntimeError(
                    "Cannot plan scan — current joint state unavailable (connect the robot first)."
                )

            results: List[ScanPoseResult] = []
            # Home-centric IK seeds; prefer home→pin, fall back to last-reachable→pin.
            plan_start_seed = [float(v) for v in self.home_joints_rad()]
            self._last_plan_start_seed = list(plan_start_seed)
            initial_robot = [float(v) for v in initial_seed]
            last_reachable_pin: Optional[List[float]] = None
            multi_seed_recoveries = 0
            path_rejected = 0
            cone_recoveries = 0
            chain_only_count = 0
            prefilter_rejected = 0
            total = len(targets)
            move_client = self._ensure_move_client()
            tolerance_deg = max(0.0, float(pin_pose_tolerance_deg))
            max_per_ring = max(1, int(semi_max_sweep_ok_per_ring))
            sweep_ok_by_ring: Dict[int, int] = {}
            partial_by_index: Dict[int, Dict[str, Any]] = {}
            n_phi = max(1, min(720, int(semi_ring_search_candidates)))
            backup_min_deg = max(0.0, float(semi_backup_coverage_deg))
            # Full-turn base-sweep samples (~every 10° of pan).
            base_sweep_samples = max(8, min(72, max(36, n_phi // 7)))
            sys.stderr.write(
                "UR3e MoveIt: planning "
                f"{total} scan pose(s){search_params_log(targets)}"
                " (multi-IK; home→pin preferred, "
                "previous→pin fallback"
                + (
                    f"; pin tip ±{tolerance_deg:.1f}° (vertical plane)"
                    if tolerance_deg > 1.0e-6
                    else ""
                )
                + f"; cam-up={str(bool(lock_camera_up)).lower()}"
                + (
                    f"; semi base-sweep first {max_per_ring}/ring"
                    f", {n_phi} φ candidates/ring"
                    "; pan before paths; box/reach prefilter"
                    "; require home↔pin"
                    + (
                        f"; backup 2-pin union>{float(semi_backup_coverage_deg):g}°"
                        if float(semi_backup_coverage_deg) > 1.0e-6
                        else ""
                    )
                    if semi_ring_sweep
                    else ""
                )
                + ")…\n"
            )

            for pose_index, target in enumerate(targets):
                if pose_index > 0 and pose_index % 10 == 0:
                    sys.stderr.write(
                        f"UR3e MoveIt: planned {pose_index}/{total} pose(s)…\n"
                    )
                result = ScanPoseResult(index=target.index, reachable=False)
                ring_key = int(round(float(target.theta_deg) * 2.0))
                if (
                    semi_ring_sweep
                    and not target.require_perpendicular
                    and sweep_ok_by_ring.get(ring_key, 0) >= max_per_ring
                ):
                    result.error = (
                        f"semi ring θ≈{target.theta_deg:.1f}° already has "
                        f"{max_per_ring} base-sweep OK pin(s)"
                    )
                    results.append(result)
                    continue

                if not target.require_perpendicular:
                    pre_err = tcp_impossible_reason(
                        target.x_m, target.y_m, target.z_m, workspace
                    )
                    if pre_err:
                        prefilter_rejected += 1
                        result.error = pre_err
                        results.append(result)
                        continue

                last_pick_error = "IK failed"
                try:
                    # Apex: home XY/ori in the planning frame; only Z is the ring radius.
                    pin_target = target
                    if target.require_perpendicular:
                        pin_target = self._apex_target_from_home(
                            target, plan_start_seed
                        )
                        sys.stderr.write(
                            "UR3e MoveIt: apex pin — home pose with Z="
                            f"{pin_target.z_m:.3f} m (no extra roll, no cone, no tilt).\n"
                        )
                    current_pin_seed = (
                        last_reachable_pin
                        if last_reachable_pin is not None
                        else initial_robot
                    )
                    ik_seeds = self._generate_ik_seeds(
                        plan_start_seed,
                        current_pin=current_pin_seed,
                    )
                    # Prefer home seed first for vertical apex approach.
                    if pin_target.require_perpendicular:
                        ik_seeds = [list(plan_start_seed)] + [
                            s for s in ik_seeds if s != list(plan_start_seed)
                        ]
                    pin_tol_deg = (
                        0.0 if pin_target.require_perpendicular else float(tolerance_deg)
                    )

                    # Apex: keep the requested camera-up (home facing). No 180° re-roll.
                    roll_targets: List[ScanPoseTarget] = [pin_target]

                    picked = False
                    for roll_i, roll_target in enumerate(roll_targets):
                        if picked:
                            break
                        for tip_deg, tool_z in iter_tool_z_cone_directions(
                            roll_target.tool_z_x,
                            roll_target.tool_z_y,
                            roll_target.tool_z_z,
                            pin_tol_deg,
                            up=(
                                roll_target.camera_up_x,
                                roll_target.camera_up_y,
                                roll_target.camera_up_z,
                            ),
                        ):
                            if abs(tip_deg) <= 1.0e-9:
                                sample_target = roll_target
                            else:
                                sample_target = scan_pose_target_with_tool_z(
                                    roll_target,
                                    tool_z,
                                    lock_camera_up=lock_camera_up,
                                )
                            pose = pose_target_to_ur_pose(sample_target)
                            joints, pick_error, recovered, home_path_ok, pan_mask = (
                                self._pick_ik_closest_to_home_with_path(
                                    pose,
                                    plan_start_seed,
                                    ik_seeds,
                                    move_client,
                                    last_pin_joints=last_reachable_pin,
                                    desired_tool_z=(
                                        sample_target.tool_z_x,
                                        sample_target.tool_z_y,
                                        sample_target.tool_z_z,
                                    ),
                                    prefer_collision_free_ik=not pin_target.require_perpendicular,
                                    allow_previous_pin_path=not semi_ring_sweep,
                                    require_base_sweep=(
                                        semi_ring_sweep
                                        and not pin_target.require_perpendicular
                                    ),
                                    base_sweep_samples=base_sweep_samples,
                                    relax_apex_self_collision=pin_target.require_perpendicular,
                                )
                            )
                            if joints is None:
                                last_pick_error = pick_error or last_pick_error
                                if pick_error and (
                                    "path from home" in pick_error
                                    or "path from previous" in pick_error
                                    or "previous pin" in pick_error
                                ):
                                    if abs(tip_deg) <= 1.0e-9 and roll_i == 0:
                                        path_rejected += 1
                                continue

                            if (
                                semi_ring_sweep
                                and not pin_target.require_perpendicular
                                and not home_path_ok
                                and pan_mask
                                and backup_min_deg > 1.0e-6
                            ):
                                bits = sum(1 for bit in pan_mask if bit)
                                prev = partial_by_index.get(int(target.index))
                                if prev is None or bits > int(prev.get("bits", -1)):
                                    partial_by_index[int(target.index)] = {
                                        "ring_key": ring_key,
                                        "joints": list(joints),
                                        "mask": list(pan_mask),
                                        "bits": bits,
                                        "sample": sample_target,
                                        "tip_deg": float(tip_deg),
                                    }
                                last_pick_error = pick_error or last_pick_error
                                continue

                            if recovered:
                                multi_seed_recoveries += 1
                            if abs(tip_deg) > 1.0e-9:
                                cone_recoveries += 1
                            if not home_path_ok:
                                chain_only_count += 1

                            if semi_ring_sweep:
                                if not home_path_ok:
                                    last_pick_error = (
                                        "semi requires home→pin path (chain-only rejected)"
                                    )
                                    if abs(tip_deg) <= 1.0e-9 and roll_i == 0:
                                        path_rejected += 1
                                    continue
                                return_ok, return_err = self._start_to_goal_path_ok(
                                    joints,
                                    plan_start_seed,
                                    move_client,
                                    label="pin→home",
                                )
                                if not return_ok:
                                    last_pick_error = (
                                        return_err or "semi pin→home path failed"
                                    )
                                    if abs(tip_deg) <= 1.0e-9 and roll_i == 0:
                                        path_rejected += 1
                                    continue

                            # Semi: 360° pan already required inside the IK pick.
                            base_sweep_ok = True

                            if pin_target.require_perpendicular and roll_i > 0:
                                sys.stderr.write(
                                    "UR3e MoveIt: apex used alternate camera-up "
                                    f"({sample_target.camera_up_x:.0f},"
                                    f"{sample_target.camera_up_y:.0f},"
                                    f"{sample_target.camera_up_z:.0f}) "
                                    "(look-at still perpendicular).\n"
                                )

                            result.reachable = True
                            result.home_path_ok = bool(home_path_ok)
                            result.base_sweep_ok = bool(base_sweep_ok) or bool(
                                pin_target.require_perpendicular
                            )
                            result.joint_positions = joints
                            result.cone_tip_deg = float(tip_deg)
                            result.tcp_x_m = float(sample_target.x_m)
                            result.tcp_y_m = float(sample_target.y_m)
                            result.tcp_z_m = float(sample_target.z_m)
                            result.tcp_rx = float(sample_target.rx)
                            result.tcp_ry = float(sample_target.ry)
                            result.tcp_rz = float(sample_target.rz)
                            result.tool_z_x = float(sample_target.tool_z_x)
                            result.tool_z_y = float(sample_target.tool_z_y)
                            result.tool_z_z = float(sample_target.tool_z_z)
                            last_reachable_pin = list(joints)
                            if (
                                semi_ring_sweep
                                and result.base_sweep_ok
                                and not pin_target.require_perpendicular
                            ):
                                sweep_ok_by_ring[ring_key] = (
                                    sweep_ok_by_ring.get(ring_key, 0) + 1
                                )
                            picked = True
                            break
                    if not picked:
                        result.error = last_pick_error or "IK failed"
                except Exception as exc:
                    result.error = str(exc)
                results.append(result)

            if (
                semi_ring_sweep
                and backup_min_deg > 1.0e-6
                and partial_by_index
            ):
                backup_kept = self._accept_max_union_backup_pairs(
                    results,
                    partial_by_index,
                    sweep_ok_by_ring,
                    plan_start_seed,
                    move_client,
                    min_union_deg=backup_min_deg,
                )
                if backup_kept > 0:
                    sys.stderr.write(
                        "UR3e MoveIt: backup 2-pin complementary pan accepted "
                        f"{backup_kept} pin(s) (no full-spin on those rings, "
                        f"prefer exec union>{backup_min_deg:g}°).\n"
                    )

            if multi_seed_recoveries > 0:
                sys.stderr.write(
                    "UR3e MoveIt: multi-IK / path pick recovered "
                    f"{multi_seed_recoveries} additional reachable pose(s).\n"
                )
            if cone_recoveries > 0:
                sys.stderr.write(
                    "UR3e MoveIt: pin-pose vertical tip recovered "
                    f"{cone_recoveries} pose(s) (tolerance ±{tolerance_deg:.1f}°).\n"
                )
            if prefilter_rejected > 0:
                sys.stderr.write(
                    "UR3e MoveIt: box/reach prefilter skipped "
                    f"{prefilter_rejected} pose(s) before IK.\n"
                )
            if path_rejected > 0:
                sys.stderr.write(
                    "UR3e MoveIt: path+unwrap check rejected "
                    f"{path_rejected} pose(s) (had IK, no home→pin or previous→pin path).\n"
                )
            if chain_only_count > 0:
                sys.stderr.write(
                    "UR3e MoveIt: "
                    f"{chain_only_count} reachable pose(s) are previous→pin only "
                    "(no home→pin) — preview marks them chain-only.\n"
                )
            reachable = sum(1 for item in results if item.reachable)
            home_ok_count = sum(
                1 for item in results if item.reachable and item.home_path_ok
            )
            sweep_ok_count = sum(
                1 for item in results if item.reachable and item.base_sweep_ok
            )
            backup_ok_count = sum(
                1 for item in results if item.reachable and item.backup_coverage_ok
            )
            sys.stderr.write(
                f"UR3e MoveIt: plan complete{search_params_log(targets)} — "
                f"{reachable}/{total} reachable "
                f"({home_ok_count} home→pin, {chain_only_count} chain-only"
                + (f", {sweep_ok_count} base-sweep OK" if semi_ring_sweep else "")
                + (f", {backup_ok_count} backup-union" if backup_ok_count else "")
                + ").\n"
            )

            return results

    def _ensure_move_client(self) -> Any:
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient

        if self._move_client is None:
            self._move_client = ActionClient(self._node, MoveGroup, "/move_action")
        if not self._move_client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("MoveIt /move_action server not available.")
        return self._move_client

    def _ensure_execute_trajectory_client(self) -> Optional[Any]:
        from moveit_msgs.action import ExecuteTrajectory
        from rclpy.action import ActionClient

        if self._execute_trajectory_client is not None:
            return self._execute_trajectory_client

        for action_name in ("/execute_trajectory", "/move_group/execute_trajectory"):
            client = ActionClient(self._node, ExecuteTrajectory, action_name)
            if client.wait_for_server(timeout_sec=2.0):
                self._execute_trajectory_client = client
                return client
        return None

    def _joint_trajectory_action_name(self) -> str:
        if self._use_mock_hardware():
            return MOCK_JOINT_TRAJECTORY_ACTION
        return HARDWARE_JOINT_TRAJECTORY_ACTION

    def _ensure_joint_trajectory_client(self) -> Optional[Any]:
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        if self._joint_trajectory_client is not None:
            return self._joint_trajectory_client

        action_name = self._joint_trajectory_action_name()
        client = ActionClient(self._node, FollowJointTrajectory, action_name)
        if client.wait_for_server(timeout_sec=60.0):
            self._joint_trajectory_client = client
            return client
        return None

    def _log_trajectory_start_deltas(self, trajectory: Any) -> None:
        """Log per-joint gap between trajectory start and live MoveIt / RTDE feedback."""
        indices = self._trajectory_joint_indices(trajectory)
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if indices is None or joint_traj is None or not joint_traj.points:
            return

        first = joint_traj.points[0]
        if len(first.positions) <= max(indices):
            return

        traj_start = [float(first.positions[index]) for index in indices]
        moveit = self._moveit_start_joint_positions()
        hardware = self._hardware_joint_positions()
        parts: List[str] = []
        for joint_index, joint_name in enumerate(CANONICAL_JOINT_NAMES):
            traj_deg = math.degrees(traj_start[joint_index])
            detail = f"{joint_name}={traj_deg:.1f}°"
            if moveit is not None:
                delta = joint_delta_rad(moveit[joint_index], traj_start[joint_index])
                detail += f" moveitΔ={math.degrees(delta):+.1f}°"
            if hardware is not None:
                delta = joint_delta_rad(hardware[joint_index], traj_start[joint_index])
                detail += f" rtdeΔ={math.degrees(delta):+.1f}°"
            parts.append(detail)
        sys.stderr.write(
            "UR3e MoveIt execute: trajectory start — "
            + "; ".join(parts)
            + ".\n"
        )

    def _execute_trajectory_via_controller(
        self,
        trajectory: Any,
        *,
        stop_event: Optional[threading.Event] = None,
    ) -> int:
        """Send a hardware-branch trajectory directly to ros2_control (bypass MoveIt execute)."""
        from moveit_msgs.msg import MoveItErrorCodes

        client = self._ensure_joint_trajectory_client()
        if client is None:
            sys.stderr.write(
                "UR3e MoveIt execute: joint trajectory action server unavailable "
                f"({self._joint_trajectory_action_name()}).\n"
            )
            return MoveItErrorCodes.CONTROL_FAILED

        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None or not joint_traj.points:
            return MoveItErrorCodes.FAILURE

        from control_msgs.action import FollowJointTrajectory

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        sys.stderr.write(
            "UR3e MoveIt execute: sending RTDE-branch trajectory to "
            f"{self._joint_trajectory_action_name()} "
            f"({len(joint_traj.points)} waypoints).\n"
        )

        send_future = client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, 20.0, stop_event)
        if goal_handle is None or not goal_handle.accepted:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("stopped")
            sys.stderr.write(
                "UR3e MoveIt execute: joint trajectory goal rejected by controller.\n"
            )
            return MoveItErrorCodes.CONTROL_FAILED

        with self._move_goal_lock:
            self._active_move_goal_handle = goal_handle

        try:
            result_future = goal_handle.get_result_async()
            result = self._wait_future(result_future, 120.0, stop_event)
            error_code = int(result.result.error_code)
            if error_code == CONTROLLER_TRAJECTORY_SUCCESS:
                return MoveItErrorCodes.SUCCESS

            error_string = str(getattr(result.result, "error_string", "") or "").strip()
            detail = f"code={error_code}"
            if error_string:
                detail += f", {error_string}"
            sys.stderr.write(
                "UR3e MoveIt execute: controller rejected trajectory "
                f"({detail}).\n"
            )
            return MoveItErrorCodes.CONTROL_FAILED
        finally:
            with self._move_goal_lock:
                self._active_move_goal_handle = None

    def _build_move_group_joint_goal(
        self,
        goal_joints: Sequence[float],
        *,
        pipeline_id: str,
        planner_id: str,
        motion_scale: float,
        plan_only: bool,
        start_joints: Optional[Sequence[float]] = None,
        allowed_planning_time: float = DIRECT_ALLOWED_PLANNING_TIME_S,
        num_planning_attempts: int = DIRECT_NUM_PLANNING_ATTEMPTS,
    ) -> Any:
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint, PlanningOptions

        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.pipeline_id = pipeline_id
        goal.request.planner_id = planner_id
        goal.request.num_planning_attempts = max(1, int(num_planning_attempts))
        goal.request.allowed_planning_time = max(0.5, float(allowed_planning_time))
        goal.request.max_velocity_scaling_factor = motion_scale
        goal.request.max_acceleration_scaling_factor = motion_scale

        if start_joints is not None and len(start_joints) == 6:
            goal.request.start_state = self._make_robot_state(start_joints)

        constraints = Constraints()
        for joint_name, value in zip(CANONICAL_JOINT_NAMES, goal_joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(value)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only
        return goal

    @staticmethod
    def _trajectory_joint_waypoints(trajectory: Any) -> List[List[float]]:
        if trajectory is None:
            return []
        joint_traj = getattr(trajectory, "joint_trajectory", None)
        if joint_traj is None or not joint_traj.joint_names or not joint_traj.points:
            return []

        name_to_index = {
            name: index for index, name in enumerate(joint_traj.joint_names)
        }
        indices: List[int] = []
        for joint_name in CANONICAL_JOINT_NAMES:
            if joint_name not in name_to_index:
                return []
            indices.append(name_to_index[joint_name])

        waypoints: List[List[float]] = []
        for point in joint_traj.points:
            if len(point.positions) <= max(indices):
                continue
            waypoints.append([float(point.positions[index]) for index in indices])
        return waypoints

    def _trajectory_joint_travel_rad(self, waypoints: Sequence[Sequence[float]]) -> float:
        if len(waypoints) < 2:
            return 0.0
        total = 0.0
        for start, end in zip(waypoints[:-1], waypoints[1:]):
            total += self._joint_distance_rad(start, end)
        return total

    def _trajectory_soft_joint_limits_ok(
        self,
        waypoints: Sequence[Sequence[float]],
    ) -> tuple[bool, str]:
        """Reject trajectories that enter the soft keep-out near MoveIt hard limits.

        Start sample is skipped so a near-limit live pose can still retreat; goal and
        interior samples must stay ≥ soft margin from MoveIt hard limits.
        """
        if len(waypoints) < 2:
            return True, ""
        margin = SOFT_JOINT_LIMIT_MARGIN_RAD
        for index, joints in enumerate(waypoints):
            if index == 0 or len(joints) != 6:
                continue
            clearance = joint_limit_clearance_rad(joints)
            if clearance < margin:
                return (
                    False,
                    "trajectory within "
                    f"{math.degrees(margin):.0f}° of joint limit "
                    f"(min clearance {math.degrees(clearance):.1f}° at waypoint {index})",
                )
        return True, ""

    @staticmethod
    def _elbow_family_differs(
        start_joints: Sequence[float],
        goal_joints: Sequence[float],
    ) -> bool:
        """True when elbow signs disagree (joint-space PTP folds the payload through the arm)."""
        if len(start_joints) < 3 or len(goal_joints) < 3:
            return False
        start_elbow = float(start_joints[2])
        goal_elbow = float(goal_joints[2])
        near_zero = math.radians(5.0)
        if abs(start_elbow) < near_zero or abs(goal_elbow) < near_zero:
            return False
        return start_elbow * goal_elbow < 0.0

    def _planners_without_ptp_if_elbow_flip(
        self,
        start_joints: Sequence[float],
        goal_joints: Sequence[float],
        attempts: Sequence[tuple[str, str]],
    ) -> List[tuple[str, str]]:
        """Drop Pilz PTP on elbow-family flips — linear interpolate hits the payload."""
        planners = list(attempts)
        if not self._elbow_family_differs(start_joints, goal_joints):
            return planners
        filtered = [
            (pipeline, planner)
            for pipeline, planner in planners
            if pipeline != "pilz_industrial_motion_planner"
        ]
        if filtered:
            sys.stderr.write(
                "UR3e MoveIt: elbow-family flip — skipping PTP "
                "(joint interpolate folds payload through the arm).\n"
            )
            sys.stderr.flush()
            return filtered
        return planners

    def _densify_trajectory_waypoints(
        self,
        waypoints: Sequence[Sequence[float]],
        *,
        max_step_rad: float = HARDWARE_SPIN_MAX_STEP_RAD,
    ) -> List[List[float]]:
        """Fill gaps between MoveIt waypoints so mid-path folds are sampled."""
        if not waypoints:
            return []
        dense: List[List[float]] = [list(map(float, waypoints[0]))]
        for nxt in waypoints[1:]:
            segment = self._interpolate_joint_waypoints(
                dense[-1], nxt, max_step_rad=max_step_rad
            )
            if len(segment) >= 2:
                dense.extend(segment[1:])
        return dense

    def _trajectory_pinch_ok(
        self,
        waypoints: Sequence[Sequence[float]],
    ) -> tuple[bool, float, str]:
        from hyperfusion_ur3e.moveit.ur_pinch_guard import (
            effective_pinch_surface_gap_m,
            tool_payload_radius_m,
        )

        if not waypoints:
            return True, float("inf"), ""

        samples = self._densify_trajectory_waypoints(waypoints)
        count = len(samples)
        if count <= TRAJECTORY_MAX_PINCH_SAMPLES:
            sample_indices = list(range(count))
        else:
            step = max(1, count // TRAJECTORY_MAX_PINCH_SAMPLES)
            sample_indices = list(range(0, count, step))
            if sample_indices[-1] != count - 1:
                sample_indices.append(count - 1)

        payload_radius_m = tool_payload_radius_m()
        min_gap_m = float("inf")
        for index in sample_indices:
            joints = samples[index]
            link_positions = self._fk_link_positions(joints)
            if link_positions is None:
                return True, float("inf"), ""
            tool_z = self._fk_tool0_z_axis(joints)
            gap_m = effective_pinch_surface_gap_m(
                link_positions,
                tool_z=tool_z,
                payload_radius_m=payload_radius_m,
            )
            min_gap_m = min(min_gap_m, gap_m)
            if gap_m < TRAJECTORY_PINCH_MIN_GAP_M:
                return (
                    False,
                    gap_m,
                    f"trajectory pinch gap {gap_m * 1000.0:.1f} mm at sample {index}",
                )
        return True, min_gap_m, ""

    def _trajectory_mesh_collision_ok(
        self,
        waypoints: Sequence[Sequence[float]],
    ) -> tuple[bool, str]:
        """Re-check MoveIt collisions on densified waypoints (mock and hardware)."""
        if len(waypoints) < 2:
            return True, ""

        samples = self._densify_trajectory_waypoints(waypoints)
        count = len(samples)
        if count <= TRAJECTORY_MAX_COLLISION_SAMPLES:
            sample_indices = list(range(count))
        else:
            step = max(1, count // TRAJECTORY_MAX_COLLISION_SAMPLES)
            sample_indices = list(range(0, count, step))
            if sample_indices[-1] != count - 1:
                sample_indices.append(count - 1)

        for index in sample_indices:
            # Skip index 0: live start may already be slightly in collision after a stop.
            if index == 0:
                continue
            ok, reason = self._state_is_valid(samples[index], check_pinch=False)
            if not ok:
                return False, f"trajectory mesh collision at sample {index}: {reason}"
        return True, ""

    def _trajectory_travel_ok(
        self,
        waypoints: Sequence[Sequence[float]],
        *,
        start_joints: Optional[Sequence[float]],
        goal_joints: Sequence[float],
    ) -> tuple[bool, float, str]:
        """Reject long OMPL snakes relative to the straight start→goal joint distance."""
        travel_rad = self._trajectory_joint_travel_rad(waypoints)
        if start_joints is not None and len(start_joints) == 6:
            straight_rad = self._joint_distance_rad(start_joints, goal_joints)
        elif len(waypoints) >= 2:
            straight_rad = self._joint_distance_rad(waypoints[0], waypoints[-1])
        else:
            straight_rad = travel_rad
        max_travel_rad = max(
            TRAJECTORY_MAX_TRAVEL_FLOOR_RAD,
            TRAJECTORY_MAX_TRAVEL_RATIO * straight_rad,
        )
        if travel_rad > max_travel_rad + 1.0e-6:
            return (
                False,
                travel_rad,
                "trajectory too long "
                f"({math.degrees(travel_rad):.1f}°·joint > "
                f"{math.degrees(max_travel_rad):.1f}° limit; "
                f"straight={math.degrees(straight_rad):.1f}°)",
            )
        return True, travel_rad, ""

    def _log_moveit_joint_issues(self, joints: Sequence[float], *, label: str) -> None:
        issues = moveit_joint_limit_violations(joints)
        if issues:
            sys.stderr.write(
                f"UR3e MoveIt: {label} outside URDF joint limits — "
                + "; ".join(issues)
                + "\n"
            )

    @staticmethod
    def _joints_deg_csv(joints: Sequence[float]) -> str:
        return ", ".join(f"{math.degrees(float(v)):.1f}" for v in joints)

    def _format_state_invalid_reason(
        self,
        joints: Sequence[float],
        validity_reason: str,
    ) -> str:
        limit_issues = moveit_joint_limit_violations(joints)
        if limit_issues:
            return "; ".join(limit_issues)
        if validity_reason:
            return validity_reason
        return "collision or joint limit violation"

    def _plan_move_group_joint_goal(
        self,
        goal_joints: Sequence[float],
        move_client: Any,
        *,
        pipeline_id: str,
        planner_id: str,
        motion_scale: float,
        start_joints: Optional[Sequence[float]] = None,
        stop_event: Optional[threading.Event] = None,
        allowed_planning_time: float = DIRECT_ALLOWED_PLANNING_TIME_S,
        num_planning_attempts: int = DIRECT_NUM_PLANNING_ATTEMPTS,
    ) -> tuple[int, Optional[Any]]:
        goal = self._build_move_group_joint_goal(
            goal_joints,
            pipeline_id=pipeline_id,
            planner_id=planner_id,
            motion_scale=motion_scale,
            plan_only=True,
            start_joints=start_joints,
            allowed_planning_time=allowed_planning_time,
            num_planning_attempts=num_planning_attempts,
        )
        send_future = move_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, 20.0, stop_event)
        if goal_handle is None or not goal_handle.accepted:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("stopped")
            raise RuntimeError("MoveIt rejected motion goal.")

        result_future = goal_handle.get_result_async()
        # Planner wall clock ≈ allowed_planning_time; leave headroom for ROS overhead.
        result_wait_s = max(30.0, float(allowed_planning_time) + 15.0)
        result = self._wait_future(result_future, result_wait_s, stop_event)
        error_code = int(result.result.error_code.val)
        planned = getattr(result.result, "planned_trajectory", None)
        return error_code, planned

    def _execute_planned_trajectory(
        self,
        trajectory: Any,
        *,
        already_prepared: bool = False,
        stop_event: Optional[threading.Event] = None,
    ) -> int:
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import MoveItErrorCodes

        if not already_prepared and not self._prepare_trajectory_for_robot(trajectory):
            sys.stderr.write(
                "UR3e MoveIt execute: trajectory failed unwrap/velocity check.\n"
            )
            return MoveItErrorCodes.FAILURE

        self._log_trajectory_start_deltas(trajectory)

        if not self._use_mock_hardware():
            return self._execute_trajectory_via_controller(
                trajectory, stop_event=stop_event
            )

        execute_client = self._ensure_execute_trajectory_client()
        if execute_client is None:
            return MoveItErrorCodes.FAILURE

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = execute_client.send_goal_async(goal)
        goal_handle = self._wait_future(send_future, 20.0, stop_event)
        if goal_handle is None or not goal_handle.accepted:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("stopped")
            return MoveItErrorCodes.FAILURE

        with self._move_goal_lock:
            self._active_move_goal_handle = goal_handle

        try:
            result_future = goal_handle.get_result_async()
            result = self._wait_future(result_future, 120.0, stop_event)
            return int(result.result.error_code.val)
        finally:
            with self._move_goal_lock:
                self._active_move_goal_handle = None

    def _goal_joints_from_tcp(
        self,
        tcp_target: Dict[str, Any],
        current_joints: Sequence[float],
        *,
        pin_pose_tolerance_deg: float = 0.0,
        lock_camera_up: bool = True,
    ) -> Optional[List[float]]:
        """Re-run IK from the live robot pose (optional look-at cone, inside→out)."""
        base = ScanPoseTarget(
            index=0,
            x_m=float(tcp_target.get("x_m", tcp_target.get("x", 0.0))),
            y_m=float(tcp_target.get("y_m", tcp_target.get("y", 0.0))),
            z_m=float(tcp_target.get("z_m", tcp_target.get("z", 0.0))),
            rx=float(tcp_target.get("rx", 0.0)),
            ry=float(tcp_target.get("ry", 0.0)),
            rz=float(tcp_target.get("rz", 0.0)),
            tool_z_x=float(tcp_target.get("tool_z_x", 0.0)),
            tool_z_y=float(tcp_target.get("tool_z_y", 0.0)),
            tool_z_z=float(tcp_target.get("tool_z_z", -1.0)),
            camera_up_x=float(tcp_target.get("camera_up_x", 0.0)),
            camera_up_y=float(tcp_target.get("camera_up_y", 0.0)),
            camera_up_z=float(tcp_target.get("camera_up_z", 1.0)),
            require_perpendicular=bool(
                tcp_target.get("require_perpendicular", False)
            ),
        )
        reference = list(current_joints)
        plan_start = (
            self._last_plan_start_seed
            if len(self._last_plan_start_seed) == 6
            else self.home_joints_rad()
        )
        ik_seeds = self._generate_ik_seeds(
            plan_start,
            current_pin=reference,
        )
        tolerance_deg = max(0.0, float(pin_pose_tolerance_deg))
        if base.require_perpendicular:
            tolerance_deg = 0.0  # apex: exact look-down only
        last_error = "no IK solution"
        for tip_deg, tool_z in iter_tool_z_cone_directions(
            base.tool_z_x,
            base.tool_z_y,
            base.tool_z_z,
            tolerance_deg,
            up=(base.camera_up_x, base.camera_up_y, base.camera_up_z),
        ):
            if abs(tip_deg) <= 1.0e-9:
                sample = base
            else:
                sample = scan_pose_target_with_tool_z(
                    base, tool_z, lock_camera_up=lock_camera_up
                )
            pose = pose_target_to_ur_pose(sample)
            joints, ik_error, _recovered = self._solve_ik_multi_seed(
                pose,
                reference,
                ik_seeds,
                desired_tool_z=(
                    sample.tool_z_x,
                    sample.tool_z_y,
                    sample.tool_z_z,
                ),
            )
            if joints is None:
                last_error = ik_error or last_error
                continue

            joints = self._normalize_joint_solution_to_reference(plan_start, joints)
            joints = self._normalize_joint_solution_to_reference(reference, joints)
            if not self._ik_joint_angles_are_sane(joints):
                last_error = "invalid IK joint angles"
                continue

            valid, reason = self._state_is_valid(joints)
            if not valid:
                last_error = reason or "collision"
                continue

            if abs(tip_deg) > 1.0e-9:
                sys.stderr.write(
                    "UR3e MoveIt execute: live IK refresh used vertical tip "
                    f"tip={tip_deg:.1f}° (tolerance ±{tolerance_deg:.1f}°).\n"
                )
            else:
                sys.stderr.write(
                    "UR3e MoveIt execute: refreshed IK goal from current robot pose.\n"
                )
            return joints

        sys.stderr.write(
            f"UR3e MoveIt execute: live IK refresh failed ({last_error}).\n"
        )
        return None

    def _motion_scale_for(
        self,
        a: Optional[Sequence[float]],
        b: Optional[Sequence[float]],
    ) -> float:
        """MoveIt velocity/accel scale from joint travel (short hops run faster)."""
        travel = 0.0
        if a is not None and b is not None:
            travel = self._joint_distance_rad(a, b)
        if travel > 5.0:
            return 0.08
        if travel > 3.0:
            return 0.12
        if travel > 1.5:
            return 0.18
        return 0.25

    def _plan_and_execute_move(
        self,
        goal_joints: Sequence[float],
        move_client: Any,
        *,
        motion_scale: float,
        stop_event: Optional[threading.Event] = None,
        recovery: bool = False,
        planner_attempts: Optional[Sequence[tuple[str, str]]] = None,
    ) -> tuple[bool, int, str]:
        """Try planners in order; execute the first valid trajectory (no collect-all)."""
        from moveit_msgs.msg import MoveItErrorCodes

        # Single source of truth: RTDE → plan start (robot_joints.read_live_joints).
        reading = self._live_joint_reading(timeout_s=1.5)
        hardware_start = reading.hardware_rad
        moveit_start = reading.plan_start_rad
        warn = branch_gap_warning(reading)
        if warn:
            sys.stderr.write(f"UR3e MoveIt execute: {warn}.\n")
            sys.stderr.flush()
        plan_goal = coalesce_goal_to_plan_start(moveit_start, goal_joints)
        if hardware_start is not None:
            sys.stderr.write(
                "UR3e MoveIt execute: plan from RTDE-mapped start; "
                "execute unwrap anchors to live hardware.\n"
            )

        self._publish_rviz_goal_state(plan_goal)
        if moveit_start is not None:
            self._log_moveit_joint_issues(moveit_start, label="current")
        self._log_moveit_joint_issues(plan_goal, label="goal")
        if moveit_start is not None:
            sys.stderr.write(
                "UR3e MoveIt execute: "
                f"start=[{self._joints_deg_csv(moveit_start)}] "
                f"goal=[{self._joints_deg_csv(plan_goal)}] "
                f"Δ={math.degrees(self._joint_distance_rad(moveit_start, plan_goal)):.1f}°·joint\n"
            )
        else:
            sys.stderr.write(
                "UR3e MoveIt execute: "
                f"goal=[{self._joints_deg_csv(plan_goal)}] (no MoveIt start feedback)\n"
            )

        # FPP apex = scan home: start==goal. Pilz/OMPL reject empty PTP (code 99999).
        if moveit_start is not None:
            already_rad = self._joint_distance_rad(moveit_start, plan_goal)
            if already_rad <= HOME_JOINT_TOLERANCE_RAD:
                start_ok, _start_reason = self._state_is_valid(moveit_start)
                if start_ok:
                    sys.stderr.write(
                        "UR3e MoveIt execute: already at goal "
                        f"(Δ={math.degrees(already_rad):.1f}°·joint) — skipping empty plan.\n"
                    )
                    sys.stderr.flush()
                    return True, MoveItErrorCodes.SUCCESS, "success"

        if recovery:
            allowed_planning_time = RECOVERY_ALLOWED_PLANNING_TIME_S
            num_planning_attempts = RECOVERY_NUM_PLANNING_ATTEMPTS
        else:
            allowed_planning_time = DIRECT_ALLOWED_PLANNING_TIME_S
            num_planning_attempts = DIRECT_NUM_PLANNING_ATTEMPTS

        attempts = (
            list(planner_attempts)
            if planner_attempts is not None
            else list(EXECUTE_PLANNER_ATTEMPTS)
        )
        if moveit_start is not None:
            attempts = self._planners_without_ptp_if_elbow_flip(
                moveit_start, plan_goal, attempts
            )

        last_code = MoveItErrorCodes.FAILURE
        last_name = "failure"

        with self._scene_lock:
            self._plan_in_progress = True
        try:
            if moveit_start is not None:
                start_ok, start_reason = self._state_is_valid(moveit_start)
                if not start_ok:
                    detail = self._format_state_invalid_reason(moveit_start, start_reason)
                    sys.stderr.write(
                        f"UR3e MoveIt execute: start state invalid before plan — {detail}\n"
                    )
                    sys.stderr.flush()
                    return False, MoveItErrorCodes.START_STATE_INVALID, detail
            goal_ok, goal_reason = self._state_is_valid(plan_goal)
            if not goal_ok:
                detail = self._format_state_invalid_reason(plan_goal, goal_reason)
                sys.stderr.write(
                    f"UR3e MoveIt execute: goal state invalid before plan — {detail}\n"
                )
                sys.stderr.flush()
                return False, MoveItErrorCodes.GOAL_STATE_INVALID, detail

            for pipeline_id, planner_id in attempts:
                try:
                    error_code, planned = self._plan_move_group_joint_goal(
                        plan_goal,
                        move_client,
                        pipeline_id=pipeline_id,
                        planner_id=planner_id,
                        motion_scale=motion_scale,
                        start_joints=moveit_start,
                        stop_event=stop_event,
                        allowed_planning_time=allowed_planning_time,
                        num_planning_attempts=num_planning_attempts,
                    )
                except RuntimeError:
                    raise

                if error_code != MoveItErrorCodes.SUCCESS or planned is None:
                    last_code = error_code
                    last_name = self._moveit_error_name(error_code)
                    sys.stderr.write(
                        "UR3e MoveIt execute: "
                        f"{pipeline_id}/{planner_id} plan failed "
                        f"({last_name}, code={error_code}).\n"
                    )
                    continue

                prepared = copy.deepcopy(planned)
                if not self._prepare_trajectory_for_robot(prepared):
                    sys.stderr.write(
                        "UR3e MoveIt execute: "
                        f"{pipeline_id}/{planner_id} rejected (unwrap/velocity limit).\n"
                    )
                    continue

                waypoints = self._trajectory_joint_waypoints(prepared)
                if len(waypoints) < 2:
                    sys.stderr.write(
                        "UR3e MoveIt execute: "
                        f"{pipeline_id}/{planner_id} returned empty trajectory.\n"
                    )
                    continue

                travel_ok, travel_rad, travel_reason = self._trajectory_travel_ok(
                    waypoints,
                    start_joints=moveit_start,
                    goal_joints=plan_goal,
                )
                if not travel_ok:
                    sys.stderr.write(
                        "UR3e MoveIt execute: "
                        f"{pipeline_id}/{planner_id} rejected ({travel_reason}).\n"
                    )
                    continue

                soft_ok, soft_reason = self._trajectory_soft_joint_limits_ok(waypoints)
                if not soft_ok:
                    sys.stderr.write(
                        "UR3e MoveIt execute: "
                        f"{pipeline_id}/{planner_id} rejected ({soft_reason}).\n"
                    )
                    continue

                mesh_ok, mesh_reason = self._trajectory_mesh_collision_ok(waypoints)
                if not mesh_ok:
                    sys.stderr.write(
                        "UR3e MoveIt execute: "
                        f"{pipeline_id}/{planner_id} rejected ({mesh_reason}).\n"
                    )
                    continue

                pinch_ok, min_gap_m, pinch_reason = self._trajectory_pinch_ok(waypoints)
                if not pinch_ok:
                    sys.stderr.write(
                        "UR3e MoveIt execute: "
                        f"{pipeline_id}/{planner_id} rejected ({pinch_reason}).\n"
                    )
                    continue

                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} candidate "
                    f"travel={math.degrees(travel_rad):.1f}°·joint "
                    f"min pinch gap={min_gap_m * 1000.0:.1f} mm "
                    "(first valid — executing).\n"
                )
                with self._scene_lock:
                    self._plan_in_progress = False
                execute_code = self._execute_planned_trajectory(
                    prepared, already_prepared=True, stop_event=stop_event
                )
                if execute_code == MoveItErrorCodes.SUCCESS:
                    return True, MoveItErrorCodes.SUCCESS, "success"
                last_code = execute_code
                last_name = self._moveit_error_name(execute_code)
                sys.stderr.write(
                    "UR3e MoveIt execute: "
                    f"{pipeline_id}/{planner_id} execute failed "
                    f"({last_name}, code={execute_code}) — trying next planner.\n"
                )
                with self._scene_lock:
                    self._plan_in_progress = True
        finally:
            with self._scene_lock:
                self._plan_in_progress = False

        return False, last_code, last_name

    def _resolve_goal_joints(
        self,
        goal_joints: Sequence[float],
        at_joints: Sequence[float],
        tcp_target: Optional[Dict[str, Any]] = None,
        *,
        refresh_ik: bool = True,
        pin_pose_tolerance_deg: float = 0.0,
        lock_camera_up: bool = True,
    ) -> tuple[Optional[List[float]], str]:
        """Joint-space goal at *at_joints*, optionally refreshed with live IK for scan pins."""
        resolved = self._normalize_joint_solution_to_reference(at_joints, goal_joints)
        if refresh_ik and isinstance(tcp_target, dict):
            refreshed = self._goal_joints_from_tcp(
                tcp_target,
                at_joints,
                pin_pose_tolerance_deg=pin_pose_tolerance_deg,
                lock_camera_up=lock_camera_up,
            )
            if refreshed is not None:
                resolved = refreshed

        if not self._ik_joint_angles_are_sane(resolved):
            return None, "goal outside UR3e limits"

        goal_valid, goal_reason = self._state_is_valid(resolved)
        if not goal_valid:
            return None, goal_reason or "collision or limits"

        return resolved, ""

    def _resolve_pin_approach_goal(
        self,
        goal_joints: Sequence[float],
        at_joints: Sequence[float],
        tcp_target: Optional[Dict[str, Any]] = None,
        *,
        pin_pose_tolerance_deg: float = 0.0,
        lock_camera_up: bool = True,
    ) -> tuple[Optional[List[float]], str]:
        """Resolve scan-pin goal joints; prefer stored plan joints over live IK refresh."""
        planned, planned_error = self._resolve_goal_joints(
            goal_joints,
            at_joints,
            tcp_target=None,
            refresh_ik=False,
            pin_pose_tolerance_deg=pin_pose_tolerance_deg,
            lock_camera_up=lock_camera_up,
        )
        if planned is not None:
            return planned, ""

        if isinstance(tcp_target, dict):
            refreshed, refresh_error = self._resolve_goal_joints(
                goal_joints,
                at_joints,
                tcp_target,
                refresh_ik=True,
                pin_pose_tolerance_deg=pin_pose_tolerance_deg,
                lock_camera_up=lock_camera_up,
            )
            if refreshed is not None:
                return refreshed, ""
            return None, refresh_error or planned_error or "goal unreachable"

        return None, planned_error or "goal unreachable"

    def _execute_via_home(
        self,
        goal_joints: Sequence[float],
        current_joints: Optional[Sequence[float]],
        move_client: Any,
        *,
        tcp_target: Optional[Dict[str, Any]] = None,
        motion_scale: Optional[float] = None,
        stop_event: Optional[threading.Event] = None,
        log_prefix: str = "no direct collision-free path",
        pin_pose_tolerance_deg: float = 0.0,
        lock_camera_up: bool = True,
    ) -> Dict[str, Any]:
        """Retreat to fixed home, then approach the pin from the home branch."""
        sys.stderr.write(
            f"UR3e MoveIt execute: {log_prefix} — "
            "retreating to home pose, then approaching pin.\n"
        )
        hardware = self._ensure_hardware_joint_positions(timeout_s=1.5)
        self._wait_for_planner_joint_feedback(timeout_s=1.5)
        moveit_start = self._moveit_start_joint_positions()
        if moveit_start is None and hardware is not None:
            moveit_start = coalesce_joints_for_moveit(hardware, hardware)

        home_ref = (
            moveit_start
            if moveit_start is not None
            else (hardware if hardware is not None else self.home_joints_rad())
        )
        # Plan/validity must stay on the MoveIt branch; RTDE unwrap happens at execute.
        home_joints = coalesce_joints_for_moveit(home_ref, self.home_joints_rad())
        home_valid, home_reason = self._state_is_valid(home_joints)
        if not home_valid:
            return {
                "ok": False,
                "skipped": True,
                "error": f"no direct path; home pose invalid ({home_reason or 'collision'})",
            }

        live_before_retreat = self._current_joint_positions()
        if live_before_retreat is not None:
            live_before_retreat = self._normalize_joint_solution_to_reference(
                live_before_retreat, live_before_retreat
            )
        retreat_from = (
            moveit_start
            if moveit_start is not None
            else (live_before_retreat if live_before_retreat is not None else current_joints)
        )

        retreat_scale = motion_scale
        if retreat_scale is None:
            retreat_scale = self._motion_scale_for(retreat_from, home_joints)
        ok_retreat, _rcode, rname = self._plan_and_execute_move(
            home_joints,
            move_client,
            motion_scale=retreat_scale,
            stop_event=stop_event,
            recovery=True,
        )
        if not ok_retreat:
            return {
                "ok": False,
                "skipped": True,
                "error": f"no direct path and could not retreat to home ({rname})",
            }

        current2 = self._waypoint_after_commanded_move(
            home_joints,
            stop_event=stop_event,
            reference=home_ref,
        )

        goal2, goal_error = self._resolve_pin_approach_goal(
            goal_joints,
            current2,
            tcp_target,
            pin_pose_tolerance_deg=pin_pose_tolerance_deg,
            lock_camera_up=lock_camera_up,
        )
        if goal2 is None:
            # Planned joints invalid from home — try live cone IK before giving up.
            recovered = self._try_home_cone_ik_execute(
                current2,
                move_client,
                tcp_target=tcp_target,
                motion_scale=motion_scale,
                stop_event=stop_event,
                pin_pose_tolerance_deg=pin_pose_tolerance_deg,
                lock_camera_up=lock_camera_up,
                skip_joints=None,
            )
            if recovered.get("ok"):
                return recovered
            return {
                "ok": False,
                "skipped": True,
                "error": goal_error or "pin unreachable from home",
            }

        approach_scale = motion_scale
        if approach_scale is None:
            approach_scale = self._motion_scale_for(current2, goal2)
        ok2, _c2, n2 = self._plan_and_execute_move(
            goal2,
            move_client,
            motion_scale=approach_scale,
            stop_event=stop_event,
            recovery=True,
        )
        if ok2:
            sys.stderr.write("UR3e MoveIt execute: reached pin via home pose.\n")
            return {"ok": True, "executed": 1, "via_home": True}

        # Cached / resolved joints failed from home — re-IK with cone and retry.
        recovered = self._try_home_cone_ik_execute(
            current2,
            move_client,
            tcp_target=tcp_target,
            motion_scale=motion_scale,
            stop_event=stop_event,
            pin_pose_tolerance_deg=pin_pose_tolerance_deg,
            lock_camera_up=lock_camera_up,
            skip_joints=goal2,
        )
        if recovered.get("ok"):
            return recovered

        return {
            "ok": False,
            "skipped": True,
            "error": f"no collision-free path even via home ({n2})",
        }

    def _try_home_cone_ik_execute(
        self,
        at_home_joints: Sequence[float],
        move_client: Any,
        *,
        tcp_target: Optional[Dict[str, Any]],
        motion_scale: Optional[float],
        stop_event: Optional[threading.Event],
        pin_pose_tolerance_deg: float,
        lock_camera_up: bool,
        skip_joints: Optional[Sequence[float]],
    ) -> Dict[str, Any]:
        """After via-home planned joints fail: cone IK from home, home→pin only."""
        if not isinstance(tcp_target, dict):
            return {"ok": False}

        sys.stderr.write(
            "UR3e MoveIt execute: via-home planned joints failed — "
            "trying live cone IK from home.\n"
        )
        home = [float(v) for v in at_home_joints]
        plan_start = (
            self._last_plan_start_seed
            if len(self._last_plan_start_seed) == 6
            else self.home_joints_rad()
        )
        ik_seeds = self._generate_ik_seeds(plan_start, current_pin=home)
        base = ScanPoseTarget(
            index=0,
            x_m=float(tcp_target.get("x_m", tcp_target.get("x", 0.0))),
            y_m=float(tcp_target.get("y_m", tcp_target.get("y", 0.0))),
            z_m=float(tcp_target.get("z_m", tcp_target.get("z", 0.0))),
            rx=float(tcp_target.get("rx", 0.0)),
            ry=float(tcp_target.get("ry", 0.0)),
            rz=float(tcp_target.get("rz", 0.0)),
            tool_z_x=float(tcp_target.get("tool_z_x", 0.0)),
            tool_z_y=float(tcp_target.get("tool_z_y", 0.0)),
            tool_z_z=float(tcp_target.get("tool_z_z", -1.0)),
            camera_up_x=float(tcp_target.get("camera_up_x", 0.0)),
            camera_up_y=float(tcp_target.get("camera_up_y", 0.0)),
            camera_up_z=float(tcp_target.get("camera_up_z", 1.0)),
            require_perpendicular=bool(
                tcp_target.get("require_perpendicular", False)
            ),
        )
        tolerance_deg = max(0.0, float(pin_pose_tolerance_deg))
        if base.require_perpendicular:
            tolerance_deg = 0.0  # apex: exact look-down only

        skip_key = None
        if skip_joints is not None and len(skip_joints) == 6:
            skip_key = tuple(round(float(v), 4) for v in skip_joints)

        tried = 0
        for tip_deg, tool_z in iter_tool_z_cone_directions(
            base.tool_z_x,
            base.tool_z_y,
            base.tool_z_z,
            tolerance_deg,
            up=(base.camera_up_x, base.camera_up_y, base.camera_up_z),
        ):
            if abs(tip_deg) <= 1.0e-9:
                sample = base
            else:
                sample = scan_pose_target_with_tool_z(
                    base, tool_z, lock_camera_up=lock_camera_up
                )
            pose = pose_target_to_ur_pose(sample)
            joints, _err, _rec, home_ok, _mask = self._pick_ik_closest_to_home_with_path(
                pose,
                home,
                ik_seeds,
                move_client,
                last_pin_joints=None,
                desired_tool_z=(sample.tool_z_x, sample.tool_z_y, sample.tool_z_z),
                prefer_collision_free_ik=not base.require_perpendicular,
                allow_previous_pin_path=False,
                relax_apex_self_collision=base.require_perpendicular,
            )
            if joints is None or not home_ok:
                continue
            key = tuple(round(float(v), 4) for v in joints)
            if skip_key is not None and key == skip_key:
                continue

            tried += 1
            scale = motion_scale
            if scale is None:
                scale = self._motion_scale_for(home, joints)
            ok, _code, name = self._plan_and_execute_move(
                joints,
                move_client,
                motion_scale=scale,
                stop_event=stop_event,
                recovery=True,
            )
            if ok:
                tip_note = (
                    f" (vertical tip={tip_deg:.1f}°)"
                    if abs(tip_deg) > 1.0e-9
                    else ""
                )
                sys.stderr.write(
                    "UR3e MoveIt execute: reached pin via home + live cone IK"
                    f"{tip_note}.\n"
                )
                return {
                    "ok": True,
                    "executed": 1,
                    "via_home": True,
                    "live_ik_recovery": True,
                }
            sys.stderr.write(
                f"UR3e MoveIt execute: live cone IK candidate failed ({name}).\n"
            )

        if tried == 0:
            sys.stderr.write(
                "UR3e MoveIt execute: no alternate home→pin IK found in cone.\n"
            )
        return {"ok": False}

    def _fk_ee_pose(
        self, joints: Sequence[float]
    ) -> Optional[tuple[float, float, float, float, float, float]]:
        """Optical TCP pose (x,y,z,rx,ry,rz) from MoveIt FK — same frame as scan IK."""
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        try:
            self._ensure_fk_client()
        except Exception:
            return None
        if self._fk_client is None:
            return None

        from moveit_msgs.srv import GetPositionFK

        request = GetPositionFK.Request()
        request.header.frame_id = PLANNING_FRAME
        request.fk_link_names = [EE_LINK]
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(CANONICAL_JOINT_NAMES)
        request.robot_state.joint_state.position = [float(v) for v in joints]

        future = self._fk_client.call_async(request)
        response = self._wait_future(future, 5.0)
        if response is None or response.error_code.val != 1 or not response.pose_stamped:
            if EE_LINK != "tool0":
                request.fk_link_names = ["tool0"]
                future = self._fk_client.call_async(request)
                response = self._wait_future(future, 5.0)
                if (
                    response is None
                    or response.error_code.val != 1
                    or not response.pose_stamped
                ):
                    return None
            else:
                return None

        pose = response.pose_stamped[0].pose
        rx, ry, rz = quaternion_to_rotvec(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            rx,
            ry,
            rz,
        )

    def _fk_ee_position(
        self, joints: Sequence[float]
    ) -> Optional[tuple[float, float, float]]:
        pose = self._fk_ee_pose(joints)
        if pose is None:
            return None
        return pose[0], pose[1], pose[2]

    def _apex_target_from_home(
        self,
        target: ScanPoseTarget,
        home_joints: Sequence[float],
    ) -> ScanPoseTarget:
        """Apex = home TCP XY/orientation in the planning frame, Z = ring radius.

        GET /pose is base_link; MoveIt IK is world. Use FK so apex matches rings.
        """
        home_pose = self._fk_ee_pose(home_joints)
        if home_pose is None:
            return target
        hx, hy, hz, hrx, hry, hrz = home_pose
        qx, qy, qz, qw = rotvec_to_quaternion(hrx, hry, hrz)
        tzx, tzy, tzz = quat_tool_z_axis(qx, qy, qz, qw)
        sys.stderr.write(
            "UR3e MoveIt: apex from home FK "
            f"xy=({hx:.4f},{hy:.4f}) z {hz:.3f}→{float(target.z_m):.3f} m.\n"
        )
        return ScanPoseTarget(
            index=target.index,
            x_m=float(hx),
            y_m=float(hy),
            z_m=float(target.z_m),
            rx=float(hrx),
            ry=float(hry),
            rz=float(hrz),
            tool_z_x=float(tzx),
            tool_z_y=float(tzy),
            tool_z_z=float(tzz),
            camera_up_x=target.camera_up_x,
            camera_up_y=target.camera_up_y,
            camera_up_z=target.camera_up_z,
            require_perpendicular=True,
            theta_deg=target.theta_deg,
            phi_deg=target.phi_deg,
        )

    def _solve_apex_z_slide(
        self,
        pose: Sequence[float],
        home_joints: Sequence[float],
    ) -> tuple[Optional[List[float]], str]:
        """IK from home by walking Z only (collision-blind, 10 mm steps)."""
        home_pose = self._fk_ee_pose(home_joints)
        if home_pose is None:
            return None, "apex Z-slide: no home FK"
        hx, hy, hz, hrx, hry, hrz = home_pose
        target_z = float(pose[2])
        span = abs(target_z - hz)
        n_steps = max(1, int(round(span / 0.010)))
        current = [float(v) for v in home_joints]
        last_error = "apex Z-slide: no IK"
        seed_pose = (hx, hy, hz, hrx, hry, hrz)
        seed_joints, seed_err = self._solve_ik(
            seed_pose, current, avoid_collisions=False
        )
        if seed_joints is not None:
            current = self._normalize_joint_solution_to_reference(
                home_joints, seed_joints
            )
        for i in range(1, n_steps + 1):
            z = hz + (target_z - hz) * (i / float(n_steps))
            step_pose = (hx, hy, z, hrx, hry, hrz)
            joints, err = self._solve_ik(
                step_pose, current, avoid_collisions=False
            )
            if joints is None:
                last_error = err or last_error
                return None, f"apex Z-slide at z={z:.3f}: {last_error}"
            current = self._normalize_joint_solution_to_reference(home_joints, joints)
            if not self._ik_joint_angles_are_sane(current):
                return None, f"apex Z-slide at z={z:.3f}: invalid joint angles"
        return current, ""

    def _tcp_target_for_pin_pose_cone(
        self,
        goal_joints: Sequence[float],
        tcp_target: Optional[Dict[str, Any]],
    ) -> Optional[Dict[str, Any]]:
        """Build / relax a TCP dict so pin_pose_tolerance_deg cone can apply.

        Semi-fixed ring entries: never require_perpendicular. If TCP is missing,
        FK the goal joints and aim tool +Z toward the tray scan-center projection.
        """
        import math

        out: Dict[str, Any]
        if isinstance(tcp_target, dict):
            out = dict(tcp_target)
        else:
            fk = self._fk_ee_pose(goal_joints)
            if fk is None:
                return None
            x_m, y_m, z_m, rx, ry, rz = fk
            out = {
                "x_m": float(x_m),
                "y_m": float(y_m),
                "z_m": float(z_m),
                "rx": float(rx),
                "ry": float(ry),
                "rz": float(rz),
            }

        out["require_perpendicular"] = False
        out.setdefault("camera_up_x", 0.0)
        out.setdefault("camera_up_y", 0.0)
        out.setdefault("camera_up_z", 1.0)

        # Prefer explicit tool_z; else look toward tray origin under the pin XY.
        tx = float(out.get("tool_z_x", 0.0) or 0.0)
        ty = float(out.get("tool_z_y", 0.0) or 0.0)
        tz = float(out.get("tool_z_z", 0.0) or 0.0)
        if (tx * tx + ty * ty + tz * tz) < 1.0e-12:
            x_m = float(out.get("x_m", out.get("x", 0.0)))
            y_m = float(out.get("y_m", out.get("y", 0.0)))
            z_m = float(out.get("z_m", out.get("z", 0.0)))
            # Look at tray under the pin (x,y,0) — cone tilts around this axis.
            dx = 0.0
            dy = 0.0
            dz = -max(abs(z_m), 0.05)
            norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            out["tool_z_x"] = dx / norm
            out["tool_z_y"] = dy / norm
            out["tool_z_z"] = dz / norm
            _ = (x_m, y_m)  # pin XY kept in out for IK pose

        sys.stderr.write(
            "UR3e MoveIt execute: pin-pose cone enabled for semi-fixed ring entry.\n"
        )
        sys.stderr.flush()
        return out

    def execute_single_waypoint(
        self,
        joints: Sequence[float],
        workspace: Optional[WorkspaceBox] = None,
        *,
        tcp_target: Optional[Dict[str, Any]] = None,
        stop_event: Optional[threading.Event] = None,
        require_home_first: bool = False,
        direct_only: bool = False,
        pin_pose_tolerance_deg: float = 0.0,
        lock_camera_up: bool = True,
        allow_pin_pose_cone: bool = False,
    ) -> Dict[str, Any]:
        """Plan and execute one collision-aware joint-space motion via MoveIt.

        Order: direct current→pin; then retreat home→pin.
        Skips the pin if both fail.

        When *direct_only* is True, only attempt a path from the current robot state;
        never retreat via home. Failures return MoveIt error names (collision, IK,
        planning failed, etc.).

        Manual joint **Move** sets *direct_only*: same MoveIt plan+execute as scan pins,
        but without home retreat on failure.

        *allow_pin_pose_cone*: semi-fixed ring entries — force non-apex cone relaxation
        (``pin_pose_tolerance_deg``) and FK-fill TCP when missing.
        """
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "error": "stopped"}

        if len(joints) != 6:
            raise ValueError("Waypoint must have 6 joint values.")

        goal_joints = [float(v) for v in joints]
        tolerance_deg = max(0.0, float(pin_pose_tolerance_deg))

        if allow_pin_pose_cone:
            tcp_target = self._tcp_target_for_pin_pose_cone(goal_joints, tcp_target)

        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            ws = workspace if workspace is not None else self._last_workspace
            if ws is None:
                ws = WorkspaceBox(enabled=True)
            self._last_workspace = ws

        try:
            self._apply_workspace_collision(ws)
        except Exception as exc:
            return {
                "ok": False,
                "skipped": not direct_only,
                "error": f"could not apply workspace boundary ({exc})",
            }

        with self._lock:
            move_client = self._ensure_move_client()

        self._wait_for_planner_joint_feedback(timeout_s=2.0)
        planned_joints = list(goal_joints)
        goal_joints, branch_ref = self._resolve_execute_branch(
            planned_joints,
            direct_only=direct_only,
            tcp_target=tcp_target,
            refresh_ik=bool(allow_pin_pose_cone and isinstance(tcp_target, dict)),
            pin_pose_tolerance_deg=tolerance_deg,
            lock_camera_up=lock_camera_up,
        )
        if branch_ref is not None and not self._ik_joint_angles_are_sane(goal_joints):
            return {
                "ok": False,
                "skipped": not direct_only,
                "error": "goal joints outside UR3e limits",
            }
        if branch_ref is None:
            sys.stderr.write(
                "UR3e MoveIt execute: joint feedback unavailable — "
                "using goal without live branch coalesce.\n"
            )

        try:
            if branch_ref is not None:
                start_check = branch_ref
                start_valid, start_reason = self._state_is_valid(start_check)
                if not start_valid:
                    detail = self._format_state_invalid_reason(start_check, start_reason)
                    return {
                        "ok": False,
                        "skipped": not direct_only,
                        "error": f"start state invalid: {detail}",
                    }

            goal_valid, goal_reason = self._state_is_valid(goal_joints)
            if not goal_valid and isinstance(tcp_target, dict) and branch_ref is not None:
                if not allow_pin_pose_cone:
                    detail = self._format_state_invalid_reason(goal_joints, goal_reason)
                    return {
                        "ok": False,
                        "skipped": not direct_only,
                        "error": f"planned joints invalid: {detail}",
                    }
                sys.stderr.write(
                    "UR3e MoveIt execute: planned joints invalid "
                    f"({goal_reason or 'collision'}) — trying live IK fallback"
                    + (
                        f" (cone ±{tolerance_deg:.1f}°)"
                        if tolerance_deg > 1.0e-6
                        else ""
                    )
                    + ".\n"
                )
                goal_joints, branch_ref = self._resolve_execute_branch(
                    planned_joints,
                    direct_only=direct_only,
                    tcp_target=tcp_target,
                    refresh_ik=True,
                    pin_pose_tolerance_deg=tolerance_deg,
                    lock_camera_up=lock_camera_up,
                )
                goal_valid, goal_reason = self._state_is_valid(goal_joints)
            if not goal_valid:
                detail = self._format_state_invalid_reason(goal_joints, goal_reason)
                return {
                    "ok": False,
                    "skipped": not direct_only,
                    "error": f"goal state invalid: {detail}",
                }

            # 1) Direct path from current pose (skipped when home-first is required).
            if not require_home_first:
                motion_scale = self._motion_scale_for(branch_ref, goal_joints)
                ok, error_code, error_name = self._plan_and_execute_move(
                    goal_joints, move_client, motion_scale=motion_scale, stop_event=stop_event
                )
                if ok:
                    return {"ok": True, "executed": 1}
                if direct_only:
                    return {
                        "ok": False,
                        "error": error_name,
                        "moveit_error_code": int(error_code),
                    }

            if direct_only:
                return {
                    "ok": False,
                    "error": "home-first motion is not allowed for direct-only moves",
                }

            # 2) One via-home attempt, then skip.
            home_prefix = (
                "ring boundary — home-first required"
                if require_home_first
                else "no direct collision-free path"
            )
            return self._execute_via_home(
                goal_joints,
                branch_ref,
                move_client,
                tcp_target=tcp_target,
                stop_event=stop_event,
                log_prefix=home_prefix,
                pin_pose_tolerance_deg=tolerance_deg,
                lock_camera_up=lock_camera_up,
            )
        except RuntimeError as exc:
            if stop_event is not None and stop_event.is_set():
                return {"ok": False, "stopped": True, "error": str(exc)}
            raise

    def execute_move_home(
        self,
        workspace: Optional[WorkspaceBox] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> Dict[str, Any]:
        """Collision-aware MoveIt motion to the configured scan home pose."""
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "error": "stopped"}

        with self._lock:
            self._process_manager.ensure_running()
            self._ensure_ros()
            ws = workspace if workspace is not None else self._last_workspace
            if ws is None:
                ws = WorkspaceBox(enabled=True)
            self._last_workspace = ws

        # Keep ApplyPlanningScene / action wait outside the planner lock so UI
        # preview and joint polls cannot deadlock against this home request.
        sys.stderr.write("UR3e MoveIt execute: preparing scan home motion…\n")
        sys.stderr.flush()
        try:
            # Topic-based boundary apply (~1s); formerly blocked ~90s on WSL service hang.
            self._apply_workspace_collision(ws)
        except Exception as exc:
            return {
                "ok": False,
                "skipped": True,
                "error": f"could not apply workspace boundary ({exc})",
            }

        with self._lock:
            move_client = self._ensure_move_client()

        reading = self._live_joint_reading(timeout_s=1.5)
        self._wait_for_planner_joint_feedback(timeout_s=0.5)
        # Prefer RTDE-mapped joints for "already home" and goal coalescing so a
        # stamper +2π twin does not make home look far away (or plan wrong).
        current_joints = reading.plan_start_rad
        warn = branch_gap_warning(reading)
        if warn:
            sys.stderr.write(f"UR3e MoveIt execute: {warn}.\n")
            sys.stderr.flush()

        home_ref = (
            current_joints if current_joints is not None else self.home_joints_rad()
        )
        home_joints = coalesce_goal_to_plan_start(home_ref, self.home_joints_rad())

        if current_joints is not None:
            wrap_dist = self._joint_distance_rad(current_joints, home_joints)
            # Wrap-space nearness matches what the UI shows. Continuous wrist_3 cable
            # turns are unwound during scans — do not block connect home verify.
            if wrap_dist <= HOME_ALREADY_NEAR_RAD:
                current_valid, current_reason = self._state_is_valid(current_joints)
                if current_valid:
                    sys.stderr.write(
                        "UR3e MoveIt execute: already at scan home pose "
                        f"(Δ={math.degrees(wrap_dist):.1f}°·joint wrap).\n"
                    )
                    sys.stderr.flush()
                    return {"ok": True, "already_at_home": True}
                return {
                    "ok": False,
                    "skipped": True,
                    "error": (
                        "at configured home joints but pose is invalid "
                        f"({current_reason or 'collision or limits'})"
                    ),
                }

        home_valid, home_reason = self._state_is_valid(home_joints)
        if not home_valid:
            return {
                "ok": False,
                "skipped": True,
                "error": f"home pose invalid ({home_reason or 'collision or limits'})",
            }

        # Always use MoveIt (Pilz-first) — never RTDE-only nudge (that bypassed mesh checks).
        motion_scale = self._motion_scale_for(current_joints, home_joints)
        ok, _code, error_name = self._plan_and_execute_move(
            home_joints,
            move_client,
            motion_scale=motion_scale,
            stop_event=stop_event,
            recovery=True,
            planner_attempts=HOME_EXECUTE_PLANNER_ATTEMPTS,
        )
        if ok:
            sys.stderr.write("UR3e MoveIt execute: reached scan home pose.\n")
            sys.stderr.flush()
            return {"ok": True}

        return {
            "ok": False,
            "skipped": True,
            "error": f"could not move to home ({error_name})",
        }

    def _interpolate_joint_waypoints(
        self,
        start_joints: Sequence[float],
        goal_joints: Sequence[float],
        *,
        max_step_rad: float = HARDWARE_SPIN_MAX_STEP_RAD,
    ) -> List[List[float]]:
        """Dense linear interpolation in continuous joint space (for validity sampling)."""
        start = [float(v) for v in start_joints]
        goal = [float(v) for v in goal_joints]
        deltas = [goal[i] - start[i] for i in range(6)]
        max_abs = max(abs(d) for d in deltas)
        if max_abs <= 1.0e-9:
            return [start, goal]
        steps = max(2, int(math.ceil(max_abs / max(1.0e-6, float(max_step_rad)))) + 1)
        waypoints: List[List[float]] = []
        for index in range(steps):
            t = index / float(steps - 1)
            waypoints.append([start[i] + deltas[i] * t for i in range(6)])
        return waypoints

    def _execute_hardware_joint_spin(
        self,
        start_joints: Sequence[float],
        goal_joints: Sequence[float],
        *,
        stop_event: Optional[threading.Event] = None,
        label: str = "hardware joint move",
        skip_collision_check: bool = False,
    ) -> tuple[bool, str]:
        """Send a multi-point RTDE trajectory after MoveIt-validating every sample.

        Used for continuous wrist_3 cable unwind (MoveIt ±π cannot plan multi-turn)
        and for semi-fixed shoulder_pan ring spins (skip_collision_check=True — operator
        guarantees the ring stays clear of boundaries; no MoveIt planning).
        """
        from builtin_interfaces.msg import Duration
        from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        if len(start_joints) != 6 or len(goal_joints) != 6:
            return False, "expected 6 joints"

        start = [float(v) for v in start_joints]
        goal = [float(v) for v in goal_joints]
        # Continuous wrist_3: wrapped distance treats −450°≈−90° and skips cable unwind.
        if joint_distance_continuous_rad(start, goal) <= HOME_JOINT_TOLERANCE_RAD:
            return True, ""

        samples = self._interpolate_joint_waypoints(start, goal)
        if not skip_collision_check:
            for index, sample in enumerate(samples):
                # Validity uses principalized joints for FCL; geometry matches continuous pose
                # for limited joints and wrist_3 within a few turns.
                check_joints = coalesce_joints_for_moveit(sample, sample)
                ok, reason = self._state_is_valid(check_joints, check_pinch=True)
                if not ok:
                    detail = reason or "collision or limits"
                    sys.stderr.write(
                        f"UR3e MoveIt: {label} blocked — MoveIt collision at sample "
                        f"{index}/{len(samples) - 1}: {detail}\n"
                    )
                    sys.stderr.flush()
                    return False, f"collision along spin ({detail})"

        trajectory = RobotTrajectory()
        joint_traj = JointTrajectory()
        joint_traj.joint_names = list(CANONICAL_JOINT_NAMES)

        points: List[Any] = []
        for sample in samples:
            point = JointTrajectoryPoint()
            point.positions = list(sample)
            point.time_from_start = Duration(sec=0, nanosec=0)
            points.append(point)
        joint_traj.points = points
        trajectory.joint_trajectory = joint_traj

        self._strip_trajectory_derivatives(trajectory)
        self._stretch_trajectory_segment_times(trajectory)

        sys.stderr.write(
            f"UR3e MoveIt: {label} "
            f"[{self._joints_deg_csv(start)}] → [{self._joints_deg_csv(goal)}] "
            f"({len(samples)} MoveIt-validated samples).\n"
        )
        sys.stderr.flush()

        try:
            code = self._execute_trajectory_via_controller(
                trajectory, stop_event=stop_event
            )
        except RuntimeError as exc:
            if stop_event is not None and stop_event.is_set():
                return False, "stopped"
            return False, str(exc)

        if int(code) == int(MoveItErrorCodes.SUCCESS):
            return True, ""
        return False, self._moveit_error_name(int(code))

    def execute_hardware_joint_move(
        self,
        goal_joints: Sequence[float],
        *,
        workspace: Optional[WorkspaceBox] = None,
        stop_event: Optional[threading.Event] = None,
        skip_collision_check: bool = False,
        label: str = "hardware joint move",
    ) -> Dict[str, Any]:
        """Direct hardware joint trajectory (no MoveIt planning).

        Semi-fixed ring spins use this with skip_collision_check=True.
        """
        _ = workspace  # reserved for future soft workspace gates
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "error": "stopped"}

        if len(goal_joints) != 6:
            return {"ok": False, "error": "expected 6 joints"}

        hardware = self._ensure_hardware_joint_positions(timeout_s=1.5)
        if hardware is None or len(hardware) != 6:
            return {"ok": False, "error": "no hardware joint feedback"}

        start = [float(v) for v in hardware]
        goal = [float(v) for v in goal_joints]
        ok, reason = self._execute_hardware_joint_spin(
            start,
            goal,
            stop_event=stop_event,
            label=label,
            skip_collision_check=bool(skip_collision_check),
        )
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "error": "stopped"}
        if not ok:
            err = reason or "hardware joint move failed"
            if err == "stopped":
                return {"ok": False, "stopped": True, "error": "stopped"}
            return {"ok": False, "error": err}
        return {"ok": True}

    def maybe_rewind_wrist3_cable(
        self,
        workspace: Optional[WorkspaceBox] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> Dict[str, Any]:
        """If wrist_3 is ≥½ turn from home, retreat home then unwind.

        MoveIt pin trajectories are left unchanged except home-branch unwrap on
        wrist_3. Detection uses live RTDE wrist_3 vs configured home (half-turn
        threshold). Unwind is an RTDE wrist_3 spin with MoveIt collision samples
        every ≤8° (MoveIt ±π cannot plan multi-turn cable recovery).
        """
        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "rewound": False, "error": "stopped"}

        hardware = self._ensure_hardware_joint_positions(timeout_s=1.5)
        if hardware is None or len(hardware) != 6:
            return {
                "ok": True,
                "rewound": False,
                "skipped": True,
                "reason": "no hardware joint feedback",
            }

        home = self.home_joints_rad()
        live_w3 = float(hardware[5])
        ref_w3 = float(home[5])
        # Continuous |Δ|≥180° is the cable-risk gate (not only integer turn peel).
        if not wrist3_needs_rewind(live_w3, ref_w3):
            return {
                "ok": True,
                "rewound": False,
                "turns": 0,
                "live_wrist3_deg": math.degrees(live_w3),
                "ref_wrist3_deg": math.degrees(ref_w3),
            }

        _target, turns = wrist3_unwind_target_rad(live_w3, ref_w3)
        detected_turns = int(turns) if abs(turns) >= 1 else (
            1 if (live_w3 - ref_w3) > 0.0 else -1
        )
        sys.stderr.write(
            "UR3e MoveIt: wrist_3 cable rewind — "
            f"{detected_turns:+d} turn(s) "
            f"(live {math.degrees(live_w3):.0f}°, "
            f"home ref {math.degrees(ref_w3):.0f}°, "
            f"|Δ|={math.degrees(abs(live_w3 - ref_w3)):.0f}°, threshold ±180°). "
            "Retreating home, then hardware wrist_3 unwind.\n"
        )
        sys.stderr.flush()

        home_result = self.execute_move_home(workspace=workspace, stop_event=stop_event)
        if home_result.get("stopped"):
            return {"ok": False, "stopped": True, "rewound": False, "error": "stopped"}
        if not home_result.get("ok"):
            return {
                "ok": False,
                "rewound": False,
                "turns": detected_turns,
                "error": (
                    "could not retreat home before wrist_3 unwind "
                    f"({home_result.get('error') or 'home failed'})"
                ),
            }

        if stop_event is not None and stop_event.is_set():
            return {"ok": False, "stopped": True, "rewound": False, "error": "stopped"}

        # Re-measure: home motion may have already removed some turns via unwrap.
        hardware = self._ensure_hardware_joint_positions(timeout_s=1.5)
        if hardware is None or len(hardware) != 6:
            return {
                "ok": False,
                "rewound": False,
                "turns": detected_turns,
                "error": "lost hardware joints after home retreat",
            }

        # At home pose, drive wrist_3 exactly to the configured home angle
        # (continuous multi-turn target — not the nearest wrapped equivalent).
        live_after = float(hardware[5])
        target_w3 = float(ref_w3)
        if wrist3_on_home_branch(
            live_after, target_w3, tolerance_rad=HOME_JOINT_TOLERANCE_RAD
        ):
            sys.stderr.write(
                "UR3e MoveIt: wrist_3 cable already near home branch after retreat — "
                "no extra unwind needed.\n"
            )
            sys.stderr.flush()
            return {
                "ok": True,
                "rewound": True,
                "turns": detected_turns,
                "live_wrist3_deg": math.degrees(live_w3),
                "ref_wrist3_deg": math.degrees(ref_w3),
            }

        unwind_goal = list(hardware)
        unwind_goal[5] = target_w3
        ok_spin, spin_err = self._execute_hardware_joint_spin(
            hardware,
            unwind_goal,
            stop_event=stop_event,
            label=(
                f"wrist_3 cable unwind to home "
                f"({math.degrees(live_after):.0f}° → {math.degrees(target_w3):.0f}°)"
            ),
        )
        if not ok_spin:
            if spin_err == "stopped" or (
                stop_event is not None and stop_event.is_set()
            ):
                return {"ok": False, "stopped": True, "rewound": False, "error": "stopped"}
            return {
                "ok": False,
                "rewound": False,
                "turns": detected_turns,
                "error": f"wrist_3 unwind failed ({spin_err})",
            }

        # Confirm RTDE actually left the wound branch (spin must not no-op on wrap).
        hardware = self._ensure_hardware_joint_positions(timeout_s=1.5)
        if hardware is None or len(hardware) != 6:
            return {
                "ok": False,
                "rewound": False,
                "turns": detected_turns,
                "error": "lost hardware joints after wrist_3 unwind",
            }
        live_final = float(hardware[5])
        if not wrist3_on_home_branch(
            live_final, target_w3, tolerance_rad=HOME_JOINT_TOLERANCE_RAD
        ):
            return {
                "ok": False,
                "rewound": False,
                "turns": detected_turns,
                "error": (
                    "wrist_3 unwind did not reach home branch "
                    f"(live {math.degrees(live_final):.0f}°, "
                    f"target {math.degrees(target_w3):.0f}°)"
                ),
            }

        sys.stderr.write(
            "UR3e MoveIt: wrist_3 cable rewind complete "
            f"({detected_turns:+d} turn(s) cleared; "
            f"live {math.degrees(live_final):.0f}°).\n"
        )
        sys.stderr.flush()
        return {
            "ok": True,
            "rewound": True,
            "turns": detected_turns,
            "live_wrist3_deg": math.degrees(live_w3),
            "final_wrist3_deg": math.degrees(live_final),
            "target_wrist3_deg": math.degrees(target_w3),
            "ref_wrist3_deg": math.degrees(ref_w3),
        }

    def execute_waypoints(
        self,
        waypoints: Sequence[Sequence[float]],
        *,
        workspace: Optional[WorkspaceBox] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> Dict[str, Any]:
        """Plan collision-aware joint motions and execute each waypoint in order."""
        ws = workspace if workspace is not None else self._last_workspace
        executed = 0
        for index, joints in enumerate(waypoints):
            if stop_event is not None and stop_event.is_set():
                return {"ok": True, "stopped": True, "executed": executed}

            try:
                result = self.execute_single_waypoint(
                    joints,
                    workspace=ws,
                    stop_event=stop_event,
                )
            except (RuntimeError, ValueError, TimeoutError) as exc:
                if stop_event is not None and stop_event.is_set():
                    return {"ok": True, "stopped": True, "executed": executed}
                raise RuntimeError(f"MoveIt failed at waypoint {index}: {exc}") from exc

            if not result.get("ok"):
                if result.get("stopped"):
                    return {"ok": True, "stopped": True, "executed": executed}
                raise RuntimeError(result.get("error") or "MoveIt execution failed.")
            executed += 1

            # Between pins: unwind wrist_3 at home if a full turn has accumulated.
            if index + 1 < len(waypoints):
                rewind = self.maybe_rewind_wrist3_cable(
                    workspace=ws, stop_event=stop_event
                )
                if rewind.get("stopped"):
                    return {"ok": True, "stopped": True, "executed": executed}
                if not rewind.get("ok"):
                    sys.stderr.write(
                        "UR3e MoveIt: wrist_3 rewind warning — "
                        f"{rewind.get('error') or 'failed'}; continuing scan.\n"
                    )
                    sys.stderr.flush()

        return {"ok": True, "executed": executed}


_planner: Optional[MoveItScanPlanner] = None
_planner_lock = threading.Lock()


def get_scan_planner(*, ros_distro: str = "jazzy", ur_type: str = "ur3e") -> MoveItScanPlanner:
    global _planner
    with _planner_lock:
        if _planner is None:
            _planner = MoveItScanPlanner(ros_distro=ros_distro, ur_type=ur_type)
        return _planner
