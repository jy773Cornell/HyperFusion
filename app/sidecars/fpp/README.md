# HyperFusion FPP decoder (offline)

Decode one **26-frame HDMI PSP burst** (u then v, 1 / 8 / 80 period sines, 4 shifts) to projector **(u, v)** maps. Legacy 14-frame (u only) folders still decode. The projected patch is **smaller than the BFS FOV** — pixels outside the lit region are masked (NaN).

- **Plane calibration:** `app/calibration/multiview/scripts/calibrate_fpp.py` (not `calibrate_bfs.py`)
- **Sample decode:** this sidecar (`fpp_cli.py`)
- **Captures:** `app/calibration/multiview/FPP/`

## Burst (same order as the app)

| # | `fpp_pattern` | Role |
|---|----------------|------|
| 0 | Black | Ambient |
| 1 | `PSP white` | White / albedo |
| 2–5 | `PSP sine 1` 0/90/180/270 | 1-period sine (unique column) |
| 6–9 | `PSP sine 8` 0/90/180/270 | 8-period sine (mid unwrap) |
| 10–13 | `PSP sine 80` 0/90/180/270 | 80-period sine **u** (fine column) |
| 14–17 | `PSP sine v 1` 0/90/180/270 | 1-period sine **v** (unique row) |
| 18–21 | `PSP sine v 8` 0/90/180/270 | 8-period sine **v** |
| 22–25 | `PSP sine v 80` 0/90/180/270 | 80-period sine **v** (fine row) |

HDMI only. Projected PNGs live in `app/calibration/multiview/patterns/psp/` (1280×720). JSON names must stay `PSP*` so decode uses the sine TPU path.

JSON from Execute (`fpp_pattern`, `fpp_step_index`, camera K / extrinsics) is used when present. Filename order is the fallback.

USB TPG / splash-hybrid folders are not decoded. Recapture with FPP HDMI.

## Setup

```powershell
cd app\sidecars\fpp
.\setup_venv.ps1
.\.venv\Scripts\Activate.ps1
```

Or use the multiview calib venv (`app/calibration/multiview/.venv`) after `pip install -r scripts/requirements.txt`.

## Calibrate (flat surface, each pose)

```powershell
cd app\calibration\multiview
.\.venv\Scripts\python scripts\calibrate_fpp.py --input FPP --out FPP
```

One `{stem}_fpp_calib.npz` per burst plus `fpp_calib_index.json`. `P` is fitted on `Z = ceiling_mount_height` in `base_link` unless you pass `--plane-z-m`. Apex JSON may remap camera `t` into the MVS sample-static frame; the decoder subtracts `stage_output.output_translation_m` so `P` stays in the room.

## Decode a sample burst

```powershell
cd app\sidecars\fpp
python fpp_cli.py --input D:\path\to\multiview_burst --calib ..\..\calibration\multiview\FPP --out D:\path\to\out
```

`--calib` as a folder picks `{stem}_fpp_calib.npz`, or the nearest camera `t` in the index (25 mm).

`--calib` also writes `Δu` vs the plane (projector pixels). Full lit patch is kept (`--bg-min-disparity-px 0`). Metric mm needs a non-degenerate `P` (tilted boards) — not this sidecar path yet.

Each burst writes `fpp_mask.npy`, `fpp_projector_u.npy`, `fpp_phase.npy`, `fpp_depth.npy` (`u` in px), plus PNG previews. Unlit FOV stays NaN.
