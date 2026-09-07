"""UR3e joint angle wrapping and 2π branch selection (MoveIt + joint_states)."""
from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Sequence

TWO_PI = 2.0 * math.pi

CANONICAL_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Matches ur_description/config/ur3e/joint_limits.yaml (planning / MoveIt).
MOVEIT_JOINT_LIMITS_RAD: tuple[Optional[tuple[float, float]], ...] = (
    (-TWO_PI, TWO_PI),  # shoulder_pan ±360°
    (-TWO_PI, TWO_PI),  # shoulder_lift ±360°
    (-math.pi, math.pi),  # elbow ±180° (artificial cap in ur_description)
    (-TWO_PI, TWO_PI),  # wrist_1 ±360°
    (-TWO_PI, TWO_PI),  # wrist_2 ±360°
    None,  # wrist_3: continuous (no position limits in URDF)
)

# Wider branch search for execute trajectories on hardware (multi-turn wrists / pan).
UR3E_JOINT_LIMITS_RAD: tuple[Optional[tuple[float, float]], ...] = (
    (-4.0 * TWO_PI, 4.0 * TWO_PI),  # shoulder_pan
    (-TWO_PI, TWO_PI),  # shoulder_lift
    (-math.pi, math.pi),  # elbow
    (-4.0 * TWO_PI, 4.0 * TWO_PI),  # wrist_1
    (-4.0 * TWO_PI, 4.0 * TWO_PI),  # wrist_2
    None,  # wrist_3 continuous
)

MAX_IK_BRANCH_STEPS = 8
JOINT_STREAM_DEADBAND_RAD = 0.0015  # ~0.09° — ignore RTDE noise in live display / MoveIt feed


def wrap_to_pi(angle: float) -> float:
    """Wrap to (-π, π] using atan2 (consistent at ±π)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def joint_delta_rad(reference: float, candidate: float) -> float:
    """Shortest signed delta from *reference* to *candidate* in (-π, π]."""
    return math.atan2(
        math.sin(candidate - reference),
        math.cos(candidate - reference),
    )


def pick_joint_branch(
    index: int,
    value: float,
    reference: float,
    *,
    limits_table: tuple[Optional[tuple[float, float]], ...] = UR3E_JOINT_LIMITS_RAD,
) -> float:
    """Pick an equivalent joint angle within limits, nearest *reference*."""
    limits = limits_table[index]
    candidates: List[float] = []
    raw = float(value)
    for step in range(-MAX_IK_BRANCH_STEPS, MAX_IK_BRANCH_STEPS + 1):
        candidate = raw + (step * TWO_PI)
        if limits is not None:
            if candidate < limits[0] - 1e-9 or candidate > limits[1] + 1e-9:
                continue
        # Continuous joints (limits is None): keep multi-turn candidates; do not wrap_to_pi.
        if not any(abs(existing - candidate) < 1e-9 for existing in candidates):
            candidates.append(candidate)

    if not candidates:
        if limits is None:
            return float(reference) + joint_delta_rad(reference, raw)
        lo, hi = limits
        return max(lo, min(hi, float(reference) + joint_delta_rad(reference, raw)))

    def sort_key(candidate: float) -> tuple[float, float, float, float]:
        # Equivalent 2π branches can differ by ~1e-9 in wrapped delta; bucket so
        # tie-break prefers the branch nearest *reference* (home / live stream).
        delta = abs(joint_delta_rad(reference, candidate))
        return (
            round(delta, 6),
            abs(candidate - reference),
            abs(wrap_to_pi(candidate)),
            abs(candidate),
        )

    candidates.sort(key=sort_key)
    best = candidates[0]
    if abs(joint_delta_rad(reference, best)) < 1e-4:
        if limits is None or (limits[0] - 1e-6 <= float(reference) <= limits[1] + 1e-6):
            return float(reference)
    return best


def unwrap_joint_continuous(reference: float, raw: float) -> float:
    """Place *raw* on the continuous branch of *reference* (shortest wrapped delta)."""
    return float(reference) + joint_delta_rad(reference, raw)


def unwrap_joint_toward_preferred(
    live: float,
    raw: float,
    preferred: float,
    *,
    max_extra_turns: int = 1,
    max_travel_rad: float = math.pi,
) -> float:
    """Place *raw* near *live*, preferring the 2π branch closer to *preferred*.

    Cable-safe: never pick a branch that requires more than *max_travel_rad*
    (default ±180°) of wrist travel from *live*. Full-turn “home bias” spins
    yank the tool/USB cable even when the numeric distance to home shrinks.
    """
    nearest = unwrap_joint_continuous(live, raw)
    best = nearest
    best_pref = abs(best - float(preferred))
    max_k = max(0, int(max_extra_turns))
    max_travel = max(0.0, float(max_travel_rad))
    for k in range(-max_k, max_k + 1):
        candidate = nearest + float(k) * TWO_PI
        travel = abs(candidate - float(live))
        if travel > max_travel + 1.0e-6:
            continue
        pref_dist = abs(candidate - float(preferred))
        if pref_dist + 1.0e-9 < best_pref:
            best = candidate
            best_pref = pref_dist
        elif abs(pref_dist - best_pref) <= 1.0e-9 and travel + 1.0e-9 < abs(
            best - float(live)
        ):
            best = candidate
            best_pref = pref_dist
    return best


def wrist3_needs_rewind(live_rad: float, ref_rad: float) -> bool:
    """True when continuous |live−home| is ≥ half a turn (cable-risk threshold)."""
    return abs(float(live_rad) - float(ref_rad)) >= math.pi - 1.0e-9


def wrist3_on_home_branch(
    live_rad: float,
    ref_rad: float,
    *,
    tolerance_rad: float = 0.05,
) -> bool:
    """True when continuous wrist_3 is within *tolerance* of the home reference."""
    return abs(float(live_rad) - float(ref_rad)) <= float(tolerance_rad)


def joint_delta_for_distance(index: int, reference: float, candidate: float) -> float:
    """Signed joint delta for distance checks.

    Continuous joints (URDF limits None, e.g. wrist_3) use absolute multi-turn
    difference so one full cable turn is not treated as already-at-goal.
    Limited joints use the shortest wrapped delta.
    """
    if 0 <= index < len(UR3E_JOINT_LIMITS_RAD) and UR3E_JOINT_LIMITS_RAD[index] is None:
        return float(candidate) - float(reference)
    return joint_delta_rad(reference, candidate)


def joint_distance_continuous_rad(
    reference: Sequence[float], candidate: Sequence[float]
) -> float:
    """L2 joint distance with continuous (non-wrapped) deltas for unlimited joints."""
    if len(reference) != 6 or len(candidate) != 6:
        return float("inf")
    total_sq = 0.0
    for index, (ref, cand) in enumerate(zip(reference, candidate)):
        delta = joint_delta_for_distance(index, ref, cand)
        total_sq += delta * delta
    return math.sqrt(total_sq)


def wrist3_completed_turns(live_rad: float, ref_rad: float) -> int:
    """Signed turns to peel so remainder (live−ref) lies in (−π, π).

    Half-turn policy: rewind when |live−home| ≥ 180° (not only after a full turn).
    """
    rem = float(live_rad) - float(ref_rad)
    turns = 0
    while rem >= math.pi - 1.0e-9:
        rem -= TWO_PI
        turns += 1
    while rem < -math.pi - 1.0e-9:
        rem += TWO_PI
        turns -= 1
    return turns


def wrist3_unwind_target_rad(live_rad: float, ref_rad: float) -> tuple[float, int]:
    """Return (target_wrist_3, turns_to_remove). Target is live after peeling turns."""
    turns = wrist3_completed_turns(live_rad, ref_rad)
    return float(live_rad) - float(turns) * TWO_PI, turns


def stabilize_joint_reading(last: float, new: float) -> float:
    """Hold the previous branch when hardware noise is sub-degree.

    Do not hold across a ±2π principalize snap (wrap-equal but far in numeric
    space) — that left MoveIt on a +360° twin after hardware unwrap.
    """
    if abs(joint_delta_rad(last, new)) < JOINT_STREAM_DEADBAND_RAD:
        if abs(last - new) < (math.pi * 0.5):
            return last
    return new


def home_joints_deg_from_env() -> Optional[List[float]]:
    raw = os.environ.get("HYPERFUSION_HOME_JOINTS_DEG", "").strip()
    if not raw:
        return None
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) != 6:
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None


def home_joints_rad_by_name(home_deg: Sequence[float]) -> Dict[str, float]:
    if len(home_deg) != 6:
        return {}
    return {
        name: math.radians(float(deg))
        for name, deg in zip(CANONICAL_JOINT_NAMES, home_deg)
    }


def normalize_joint_solution_to_reference(
    reference: Sequence[float],
    joints: Sequence[float],
    *,
    limits_table: tuple[Optional[tuple[float, float]], ...] = UR3E_JOINT_LIMITS_RAD,
) -> List[float]:
    """Pick nearest valid 2π branch per joint within the given limits."""
    if len(reference) != 6 or len(joints) != 6:
        return [float(v) for v in joints]
    return [
        pick_joint_branch(index, float(joint), float(ref), limits_table=limits_table)
        for index, (ref, joint) in enumerate(zip(reference, joints))
    ]


def _snap_continuous_joints_to_reference(
    reference: Sequence[float],
    joints: Sequence[float],
) -> List[float]:
    """Keep multi-turn joints on the reference branch when the pose is equivalent."""
    if len(reference) != 6 or len(joints) != 6:
        return [float(v) for v in joints]
    coalesced = [float(v) for v in joints]
    # pan, wrist_1, wrist_2, wrist_3 — joints that commonly accumulate turns on hardware.
    for index in (0, 3, 4, 5):
        if abs(joint_delta_rad(reference[index], coalesced[index])) < 1e-4:
            coalesced[index] = float(reference[index])
    return coalesced


def coalesce_joints_for_moveit(
    reference: Sequence[float],
    joints: Sequence[float],
) -> List[float]:
    """Map *joints* to the 2π branch nearest *reference* within MoveIt URDF limits."""
    if len(reference) != 6 or len(joints) != 6:
        return normalize_joint_solution_to_reference(
            reference, joints, limits_table=MOVEIT_JOINT_LIMITS_RAD
        )
    coalesced = normalize_joint_solution_to_reference(
        reference, joints, limits_table=MOVEIT_JOINT_LIMITS_RAD
    )
    return _snap_continuous_joints_to_reference(reference, coalesced)


def coalesce_joints_for_execute(
    reference: Sequence[float],
    joints: Sequence[float],
) -> List[float]:
    """Map *joints* to the RTDE / controller branch nearest live hardware."""
    if len(reference) != 6 or len(joints) != 6:
        return normalize_joint_solution_to_reference(
            reference, joints, limits_table=UR3E_JOINT_LIMITS_RAD
        )
    coalesced = normalize_joint_solution_to_reference(
        reference, joints, limits_table=UR3E_JOINT_LIMITS_RAD
    )
    return _snap_continuous_joints_to_reference(reference, coalesced)


# Soft keep-out near MoveIt hard limits (pendant "close to joint limit" protective stop).
# margin_rad = how far (radians) each limited joint must stay inside the hard URDF/MoveIt
# limit. Example: hard limit ±180° and margin 5° ⇒ usable band about ±175°.
# Was 10° (too strict for dense dome + wrist sweep); 5° is the loosened default.
SOFT_JOINT_LIMIT_MARGIN_RAD = math.radians(5.0)


def joint_limit_clearance_rad(
    joints: Sequence[float],
    *,
    limits_table: tuple[Optional[tuple[float, float]], ...] = MOVEIT_JOINT_LIMITS_RAD,
) -> float:
    """Minimum distance (rad) from any limited joint to its nearest MoveIt hard limit.

    Continuous joints (limits is None) are ignored. Returns +inf if no limited joints.
    """
    if len(joints) != 6:
        return 0.0
    clearance = float("inf")
    for index, value in enumerate(joints):
        limits = limits_table[index]
        if limits is None:
            continue
        lo, hi = limits
        v = float(value)
        clearance = min(clearance, v - lo, hi - v)
    if clearance == float("inf"):
        return float("inf")
    return float(clearance)


def near_soft_joint_limit(
    joints: Sequence[float],
    *,
    margin_rad: float = SOFT_JOINT_LIMIT_MARGIN_RAD,
    limits_table: tuple[Optional[tuple[float, float]], ...] = MOVEIT_JOINT_LIMITS_RAD,
) -> bool:
    """True when any limited joint is within *margin_rad* of a MoveIt hard limit."""
    return joint_limit_clearance_rad(joints, limits_table=limits_table) < float(margin_rad)


def moveit_joint_limit_violations(joints: Sequence[float]) -> List[str]:
    """Human-readable MoveIt limit violations (empty when all joints are in range)."""
    if len(joints) != 6:
        return ["expected 6 joint values"]
    issues: List[str] = []
    for index, value in enumerate(joints):
        limits = MOVEIT_JOINT_LIMITS_RAD[index]
        if limits is None:
            continue
        lo, hi = limits
        if float(value) < lo - 1e-6 or float(value) > hi + 1e-6:
            issues.append(
                f"{CANONICAL_JOINT_NAMES[index]}={math.degrees(float(value)):.1f}° "
                f"(MoveIt allows {math.degrees(lo):.0f}°…{math.degrees(hi):.0f}°)"
            )
    return issues


def joint_distance_rad(reference: Sequence[float], candidate: Sequence[float]) -> float:
    if len(reference) != 6 or len(candidate) != 6:
        return float("inf")
    total_sq = 0.0
    for ref, cand in zip(reference, candidate):
        delta = joint_delta_rad(ref, cand)
        total_sq += delta * delta
    return math.sqrt(total_sq)


def wrap_joint_for_stream(name: str, value: float, reference: float) -> float:
    """Branch-continuous wrap for /joint_states (MoveIt / ur_description limits).

    Continuous joints are published in (-π, π] so MoveIt's monitored state stays
    in the principal range (multi-turn RTDE values remain on the hardware topic).
    """
    try:
        index = CANONICAL_JOINT_NAMES.index(name)
    except ValueError:
        return wrap_to_pi(value)
    limits = MOVEIT_JOINT_LIMITS_RAD[index]
    if limits is None:
        return wrap_to_pi(value)
    return pick_joint_branch(
        index,
        float(value),
        float(reference),
        limits_table=MOVEIT_JOINT_LIMITS_RAD,
    )
