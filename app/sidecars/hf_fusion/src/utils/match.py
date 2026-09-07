# Object matching for dual-camera fusion alignment.
from __future__ import annotations

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
class PerRoiShift:
    pair: MatchedPair
    dx_mm: float
    dy_mm: float
    dx_px: float
    dy_px: float


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
    if len(fx_records) != len(sw_records):
        raise ValueError(
            f"Object count mismatch: fx10e={len(fx_records)} swir3={len(sw_records)} "
            "(Hungarian matching not implemented yet; need equal counts)"
        )
    if len(fx_records) < 1:
        raise ValueError(f"Need at least 1 matched object, got {len(fx_records)}")

    fx_by_roi = {record.roi: record for record in fx_records}
    sw_by_roi = {record.roi: record for record in sw_records}
    # GSAM assigns row-major ROI ids (top-to-bottom, right-to-left); prefer those when both cameras share the same set.
    if set(fx_by_roi) == set(sw_by_roi):
        fx_sorted = [fx_by_roi[roi] for roi in sorted(fx_by_roi)]
        sw_sorted = [sw_by_roi[roi] for roi in sorted(sw_by_roi)]
    else:
        fx_sorted = _sort_left_to_right(fx_records)
        sw_sorted = _sort_left_to_right(sw_records)

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


def per_roi_shifts(pairs: list[MatchedPair], fx_mm_per_pixel: float) -> list[PerRoiShift]:
    return [
        PerRoiShift(
            pair=pair,
            dx_mm=pair.delta_mm[0],
            dy_mm=pair.delta_mm[1],
            dx_px=pair.delta_mm[0] / fx_mm_per_pixel,
            dy_px=pair.delta_mm[1] / fx_mm_per_pixel,
        )
        for pair in pairs
    ]


def shift_spread_mm(pairs: list[MatchedPair]) -> dict[str, float]:
    dx_values = [pair.delta_mm[0] for pair in pairs]
    dy_values = [pair.delta_mm[1] for pair in pairs]
    return {
        "dx_min": min(dx_values),
        "dx_max": max(dx_values),
        "dy_min": min(dy_values),
        "dy_max": max(dy_values),
        "dx_spread": max(dx_values) - min(dx_values),
        "dy_spread": max(dy_values) - min(dy_values),
    }
