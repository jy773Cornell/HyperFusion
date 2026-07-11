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

# Wider branch search for execute trajectories on hardware.
UR3E_JOINT_LIMITS_RAD: tuple[Optional[tuple[float, float]], ...] = (
    (-TWO_PI, TWO_PI),
    (-TWO_PI, TWO_PI),
    (-math.pi, math.pi),
    (-TWO_PI, TWO_PI),
    (-TWO_PI, TWO_PI),
    None,
)

MAX_IK_BRANCH_STEPS = 4
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
        else:
            candidate = wrap_to_pi(candidate)
        if not any(abs(existing - candidate) < 1e-9 for existing in candidates):
            candidates.append(candidate)

    if not candidates:
        if limits is None:
            return wrap_to_pi(raw)
        lo, hi = limits
        return max(lo, min(hi, wrap_to_pi(raw)))

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


def stabilize_joint_reading(last: float, new: float) -> float:
    """Hold the previous branch when hardware noise is sub-degree."""
    if abs(joint_delta_rad(last, new)) < JOINT_STREAM_DEADBAND_RAD:
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
    """Keep wrist_3 on the reference branch when the pose is equivalent."""
    if len(reference) != 6 or len(joints) != 6:
        return [float(v) for v in joints]
    coalesced = [float(v) for v in joints]
    if abs(joint_delta_rad(reference[5], coalesced[5])) < 1e-4:
        coalesced[5] = float(reference[5])
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
    """Branch-continuous wrap for /joint_states (MoveIt / ur_description limits)."""
    try:
        index = CANONICAL_JOINT_NAMES.index(name)
    except ValueError:
        return wrap_to_pi(value)
    return pick_joint_branch(
        index,
        float(value),
        float(reference),
        limits_table=MOVEIT_JOINT_LIMITS_RAD,
    )
