# DLP / FPP camera–projector stereo (`dlp_cal`)

Folder-local Multiview FPP geometry calibration. Writes `results/camera_projector_stereo.yaml`; **`dlp_tcp_*` in `hyperfusion.cfg`** is composed separately as camera TCP ∘ stereo.

Board definition is shared: `../bfs_cal/board.yaml` (17.5 mm, 10×7 inner corners).

## Layout

| Path | Role |
|------|------|
| `checkerboard/` | GOOD FPP bursts used for the active stereo fit (26 TIFF + JSON each) |
| `results/` | `camera_projector_stereo.yaml` + hold-out validation |
| `patterns/psp/` | HDMI PSP stills for the projector (black/white + 12u + 12v) |
| `calibrate_fpp_geometry.py` | Fit projector `K` + `camera_T_projector` (smart / last hold-out) |
| `monitor_fpp_board.py` | Live GOOD / WEAK / REJECT collector (all 70 corners in lit patch) |
| `check_fpp_board.py` | One-shot board / patch check |
| `generate_psp.py` | Regenerate `patterns/psp/` |

Burst folders are large (~0.9 GB each) and typically gitignored.

## Active stereo set (2026-09-23)

- **Data:** 20 GOOD bursts in `checkerboard/` (from `Camera_Cal/dlp_cal/good`)
- **Board:** 17.5 mm squares
- **Fit / hold-out:** 16 / 4 (smart diversity hold-out)
- **Hold-out:** RMS **2.00 mm**, median **1.11 mm**, reproj **1.56 px**
- **Baseline:** **93.4 mm** (camera ↔ projector optical)
- **Cfg:** optical `dlp_tcp_*` composed into `hyperfusion.cfg` (2026-09-23)

## Rerun

```powershell
cd D:\Pototypy\HyperFusion\app\calibration\multiview
.\.venv\Scripts\python.exe dlp_cal\calibrate_fpp_geometry.py `
  --input dlp_cal\checkerboard `
  --board bfs_cal\board.yaml `
  --out dlp_cal\results `
  --holdout 4 `
  --holdout-mode smart
```

Then recompose `dlp_tcp_*` = current `tool_tcp_*` ∘ **inv(stereo)**.
Stereo YAML stores `X_proj = R @ X_cam + t`, so `cam_T_proj = inv(R,t)` (not `R,t` directly).
Disconnect/Connect the robot after updating cfg.
