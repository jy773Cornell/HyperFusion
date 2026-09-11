# HyperFusion FPP decoder (offline)

Decode one **26-frame HDMI PSP burst** (u then v, 1 / 8 / 80 period sines, 4 shifts) to projector **(u, v)** maps. Legacy 14-frame (u only) folders still decode. The projected patch is **smaller than the BFS FOV** — pixels outside the lit region are masked (NaN).

- **Metric geometry (tilted checkerboard):** `fpp_cal/calibrate_fpp_geometry.py`
- **White-frame QA:** `fpp_cal/check_fpp_board.py`
- **Sample decode:** this sidecar (`fpp_cli.py`) — default uses stereo YAML for **camera Z in mm**
- **Geometry captures:** `fpp_cal/checkerboard/` (`00000`–`00015`)

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

HDMI only. Projected PNGs live in `app/calibration/multiview/fpp_cal/patterns/psp/` (1280×720). JSON names must stay `PSP*` so decode uses the sine TPU path.

JSON from Execute (`fpp_pattern`, `fpp_step_index`, camera K / extrinsics) is used when present. Filename order is the fallback.

USB TPG / splash-hybrid folders are not decoded. Recapture with FPP HDMI.

## Setup

```powershell
cd app\sidecars\fpp
.\setup_venv.ps1
.\.venv\Scripts\Activate.ps1
```

Or use the multiview calib venv (`app/calibration/multiview/.venv`) after `pip install -r scripts/requirements.txt`.

## Calibrate (tilted checkerboard → millimetres)

Do **not** use an empty tray. Board pitch is **18 mm** (`bfs_cal/board.yaml`). Captures live in `fpp_cal/checkerboard/` as `00000`–`00015` (13 fit + 3 hold-out).

```powershell
cd app\calibration\multiview
.\.venv\Scripts\python fpp_cal\check_fpp_board.py --input fpp_cal\checkerboard
.\.venv\Scripts\python fpp_cal\calibrate_fpp_geometry.py --holdout 3
```

Writes `fpp_cal/results/camera_projector_stereo.yaml`. That stereo is reused at every later robot pose.

## Decode a sample burst (metric depth)

```powershell
cd app\sidecars\fpp
..\..\calibration\multiview\.venv\Scripts\python fpp_cli.py `
  --input D:\path\to\multiview_object_burst `
  --out D:\path\to\out
```

Requires `fpp_cal/results/camera_projector_stereo.yaml` (or `--stereo path`). Outputs:

- `fpp_depth.npy` / `fpp_depth.png` — camera Z in **millimetres**
- projector maps + mask

Override stereo path: `--stereo path\to.yaml` · projector-u only: `--no-stereo`.

Not wired into the GUI yet — Capture still only writes the burst; run this CLI offline.
