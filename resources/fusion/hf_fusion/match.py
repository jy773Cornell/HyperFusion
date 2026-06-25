# Object matching and median-shift estimation for dual-camera fusion alignment.
from __future__ import annotations

import statistics
from dataclasses import dataclass


@dataclass(frozen=True)
class CentroidRecord:
    roi: int
    label: str
    cx_px: float
    cy_px: float
    cx_mm: float
    cy_mm: float


@dataclass(frozen=True)
class MatchedPair:
    pair_index: int
    fx_roi: int
    sw_roi: int
    fx_mm: tuple[float, float]
    sw_mm: tuple[float, float]
    delta_mm: tuple[float, float]


@dataclass(frozen=True)
class ShiftEstimate:
    dx_mm: float
    dy_mm: float
    dx_px: float
    dy_px: float
    matched_count: int
    residual_std_mm: tuple[float, float]
    pairs: list[MatchedPair]
    low_confidence: bool
    review_required: bool


def _sort_left_to_right(records: list[CentroidRecord]) -> list[CentroidRecord]:
    return sorted(records, key=lambda r: r.cx_mm)


def records_from_table(rows: list[dict]) -> list[CentroidRecord]:
    return [
        CentroidRecord(
            roi=int(row["roi"]),
            label=str(row.get("label", "")),
            cx_px=float(row["cx_px"]),
            cy_px=float(row["cy_px"]),
            cx_mm=float(row["cx_mm"]),
            cy_mm=float(row["cy_mm"]),
        )
        for row in rows
    ]


def match_by_sorted_order(
    fx_records: list[CentroidRecord],
    sw_records: list[CentroidRecord],
    max_pair_distance_mm: float = 50.0,
) -> list[MatchedPair]:
    fx_sorted = _sort_left_to_right(fx_records)
    sw_sorted = _sort_left_to_right(sw_records)
    if len(fx_sorted) != len(sw_sorted):
        raise ValueError(
            f"Object count mismatch: fx10e={len(fx_sorted)} swir3={len(sw_sorted)} "
            "(Hungarian matching not implemented yet; need equal counts)"
        )
    if len(fx_sorted) < 2:
        raise ValueError(f"Need at least 2 matched objects for median shift, got {len(fx_sorted)}")

    pairs: list[MatchedPair] = []
    for index, (fx, sw) in enumerate(zip(fx_sorted, sw_sorted)):
        distance = ((fx.cx_mm - sw.cx_mm) ** 2 + (fx.cy_mm - sw.cy_mm) ** 2) ** 0.5
        if distance > max_pair_distance_mm:
            raise ValueError(
                f"Pair {index + 1} distance {distance:.1f} mm exceeds gate {max_pair_distance_mm} mm "
                f"(fx roi{fx.roi} vs sw roi{sw.roi})"
            )
        pairs.append(
            MatchedPair(
                pair_index=index + 1,
                fx_roi=fx.roi,
                sw_roi=sw.roi,
                fx_mm=(fx.cx_mm, fx.cy_mm),
                sw_mm=(sw.cx_mm, sw.cy_mm),
                delta_mm=(fx.cx_mm - sw.cx_mm, fx.cy_mm - sw.cy_mm),
            )
        )
    return pairs


def estimate_median_shift(
    pairs: list[MatchedPair],
    fx_mm_per_pixel: float,
    residual_std_review_mm: float = 5.0,
) -> ShiftEstimate:
    dx_values = [pair.delta_mm[0] for pair in pairs]
    dy_values = [pair.delta_mm[1] for pair in pairs]
    dx_mm = statistics.median(dx_values)
    dy_mm = statistics.median(dy_values)

    if len(pairs) >= 2:
        residual_dx = statistics.stdev([value - dx_mm for value in dx_values])
        residual_dy = statistics.stdev([value - dy_mm for value in dy_values])
    else:
        residual_dx = 0.0
        residual_dy = 0.0

    return ShiftEstimate(
        dx_mm=dx_mm,
        dy_mm=dy_mm,
        dx_px=dx_mm / fx_mm_per_pixel,
        dy_px=dy_mm / fx_mm_per_pixel,
        matched_count=len(pairs),
        residual_std_mm=(residual_dx, residual_dy),
        pairs=pairs,
        low_confidence=len(pairs) < 2,
        review_required=residual_dx > residual_std_review_mm or residual_dy > residual_std_review_mm,
    )
