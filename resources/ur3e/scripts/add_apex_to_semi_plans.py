# Add default Semi apex (θ=0) point to every ring plan JSON that lacks one.
# Matches app defaultSemiFixedTopPose() so preview/load see a real θ≈0 pin.

from __future__ import annotations

import json
from pathlib import Path

APEX = {
    "grid": {
        "phi_deg": 0.0,
        "theta_deg": 0.0,
        "x_m": -0.07501500004324846,
        "y_m": -0.016443060582171126,
        "z_m": 0.2,
    },
    "tcp": {
        "x_m": -0.07501500004324846,
        "y_m": -0.016443060582171126,
        "z_m": 0.2,
        "rx": -2.221441296101876,
        "ry": 2.2214412956462506,
        "rz": 3.852205944410706e-07,
        "tool_z_x": 0.0,
        "tool_z_y": 0.0,
        "tool_z_z": -1.0,
    },
    "reachable": True,
    "home_path_ok": True,
    # Top has no base-sweep requirement; keep true so loaders treat it as OK.
    "base_sweep_ok": True,
    "planning_error": "",
    "joints_rad": [
        1.320915979459176,
        -3.345937397893044,
        2.4709152402327033,
        -0.6957748605175326,
        1.5707963267948966,
        -1.3209159800161308,
    ],
}

DIRS = [
    Path(r"d:\Pototypy\HyperFusion\app\build\Release\ur3e_semi_scan_routes"),
    Path(r"d:\Pototypy\HyperFusion\resources\ur3e\ur3e_semi_scan_routes"),
]


def has_apex(points: list) -> bool:
    for pt in points:
        th = (pt.get("grid") or {}).get("theta_deg", 999.0)
        if abs(float(th)) < 0.75:
            return True
    return False


def main() -> None:
    total = updated = skipped = errors = 0
    for d in DIRS:
        if not d.is_dir():
            print(f"missing {d}")
            continue
        for path in sorted(d.glob("R*.json")):
            total += 1
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                points = data.get("points") or []
                if has_apex(points):
                    skipped += 1
                    continue
                data["points"] = [dict(APEX)] + list(points)
                n = len(data["points"])
                reach = sum(1 for p in data["points"] if p.get("reachable"))
                home = sum(1 for p in data["points"] if p.get("home_path_ok"))
                data["reachable_count"] = reach
                data["unreachable_count"] = max(0, n - reach)
                data["home_path_ok_count"] = home
                path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
                updated += 1
            except Exception as exc:  # noqa: BLE001
                errors += 1
                print(f"ERR {path}: {exc}")

    print(f"total={total} updated={updated} already_had_apex={skipped} errors={errors}")
    sample = DIRS[0] / "R160_T20_30_38.json"
    if sample.exists():
        data = json.loads(sample.read_text(encoding="utf-8"))
        thetas = sorted(
            {round(float(pt["grid"]["theta_deg"]), 1) for pt in data["points"]}
        )
        print(f"sample {sample.name}: points={len(data['points'])} thetas={thetas}")


if __name__ == "__main__":
    main()
