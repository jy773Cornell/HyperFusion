# MoveIt layer: single source of truth for live UR3e joint readings.
# Policy: RTDE / joint_state_broadcaster is authoritative for plan start and
# execute unwrap. Stamper /joint_states is display-only and may lag on a +2π twin.
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from hyperfusion_ur3e.joint_angles import (
    coalesce_joints_for_execute,
    coalesce_joints_for_moveit,
    joint_distance_rad,
)


def _numeric_joint_distance_rad(
    reference: Sequence[float], candidate: Sequence[float]
) -> float:
    """L2 distance on raw joint numbers (detects +2π twins; no wrap)."""
    if len(reference) != 6 or len(candidate) != 6:
        return float("inf")
    total_sq = 0.0
    for ref, cand in zip(reference, candidate):
        delta = float(cand) - float(ref)
        total_sq += delta * delta
    return math.sqrt(total_sq)

# Log when stamper and RTDE disagree by more than this (typical +1 pan turn).
BRANCH_GAP_WARN_RAD = math.radians(5.0)


@dataclass(frozen=True)
class LiveJointReading:
    """Normalized live joints for planning vs hardware execute.

    *hardware_rad* — raw RTDE branch (trajectory unwrap anchor).
    *plan_start_rad* — same pose inside MoveIt joint limits (MoveGroup start_state).
    *stamper_rad* — optional /joint_states (may be a wrap twin; not used for planning).
    *branch_gap_rad* — joint-space distance stamper vs plan_start (0 if no stamper).
    """

    hardware_rad: Optional[List[float]]
    plan_start_rad: Optional[List[float]]
    stamper_rad: Optional[List[float]] = None
    branch_gap_rad: float = 0.0

    @property
    def ok(self) -> bool:
        return self.plan_start_rad is not None and len(self.plan_start_rad) == 6

    @property
    def stamper_diverged(self) -> bool:
        return self.branch_gap_rad > BRANCH_GAP_WARN_RAD


def _as_six(joints: Optional[Sequence[float]]) -> Optional[List[float]]:
    if joints is None or len(joints) != 6:
        return None
    return [float(v) for v in joints]


def read_live_joints(
    *,
    hardware_rad: Optional[Sequence[float]],
    stamper_rad: Optional[Sequence[float]] = None,
) -> LiveJointReading:
    """Resolve live joints with RTDE as the source of truth.

    Plan start is always hardware mapped into MoveIt limits when hardware is
    available. Stamper is recorded only to detect / log branch drift.
    """
    hardware = _as_six(hardware_rad)
    stamper = _as_six(stamper_rad)

    if hardware is not None:
        plan_start = coalesce_joints_for_moveit(hardware, hardware)
    elif stamper is not None:
        plan_start = coalesce_joints_for_moveit(stamper, stamper)
    else:
        plan_start = None

    gap = 0.0
    if stamper is not None and plan_start is not None:
        # Numeric (not wrap) distance — wrap distance is ~0 for +2π twins.
        gap = float(_numeric_joint_distance_rad(stamper, plan_start))

    return LiveJointReading(
        hardware_rad=hardware,
        plan_start_rad=plan_start,
        stamper_rad=stamper,
        branch_gap_rad=gap,
    )


def plan_start_joints(
    *,
    hardware_rad: Optional[Sequence[float]],
    stamper_rad: Optional[Sequence[float]] = None,
) -> Optional[List[float]]:
    """Joints to put in MoveGroup start_state (RTDE-mapped when possible)."""
    return read_live_joints(hardware_rad=hardware_rad, stamper_rad=stamper_rad).plan_start_rad


def execute_anchor_joints(
    *,
    hardware_rad: Optional[Sequence[float]],
    fallback_rad: Optional[Sequence[float]] = None,
) -> Optional[List[float]]:
    """RTDE branch for trajectory unwrap; optional fallback (e.g. plan start)."""
    hardware = _as_six(hardware_rad)
    if hardware is not None:
        return list(hardware)
    return _as_six(fallback_rad)


def coalesce_goal_to_plan_start(
    plan_start: Optional[Sequence[float]],
    goal_joints: Sequence[float],
) -> List[float]:
    """Map a goal onto the plan-start 2π branch inside MoveIt limits."""
    goal = [float(v) for v in goal_joints]
    if plan_start is not None and len(plan_start) == 6:
        return coalesce_joints_for_moveit(plan_start, goal)
    return coalesce_joints_for_moveit(goal, goal)


def coalesce_goal_to_hardware(
    hardware: Optional[Sequence[float]],
    goal_joints: Sequence[float],
) -> List[float]:
    """Map a goal onto the live RTDE execute branch."""
    goal = [float(v) for v in goal_joints]
    if hardware is not None and len(hardware) == 6:
        return coalesce_joints_for_execute(hardware, goal)
    return coalesce_joints_for_execute(goal, goal)


def branch_gap_warning(reading: LiveJointReading) -> Optional[str]:
    """Human-readable warning when stamper disagrees with RTDE plan start."""
    if not reading.stamper_diverged:
        return None
    return (
        "plan start snapped to RTDE branch "
        f"(stamper was {math.degrees(reading.branch_gap_rad):.1f}°·joint away)"
    )


def reading_start_goal_csv(
    plan_start: Sequence[float],
    plan_goal: Sequence[float],
    *,
    joints_deg_csv,
) -> Tuple[str, str, float]:
    """Helper for log lines: start csv, goal csv, Δ rad."""
    return (
        joints_deg_csv(plan_start),
        joints_deg_csv(plan_goal),
        float(joint_distance_rad(plan_start, plan_goal)),
    )
