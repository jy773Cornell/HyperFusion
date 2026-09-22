# BFS eye-in-hand calibration (`bfs_cal`)

Folder-local Multiview BFS + UR3e checkerboard calibration. **Does not write `hyperfusion.cfg`** unless you copy values by hand.

## Layout

| Path | Role |
|------|------|
| `board.yaml` | Physical board: **11×8 squares**, **10×7** inner corners, **17.5 mm** pitch |
| `checkerboard/` | Stills used for the active classical fit (TIFF + pose JSON) |
| `results/` | Classical `K`/`D`, hand–eye, board pose, hold-out check (source of truth for cfg) |
| `calibrate_bfs.py` | Classical OpenCV intrinsics + hand–eye |
| `monitor_checkerboard.py` | Live GOOD/WEAK/REJECT collector |

TIFF/JSON under `checkerboard/` are gitignored (large). YAML/JSON results are kept.

## Active classical set (2026-09-22)

- **Data:** 70 GOOD stills in `checkerboard/` (from `Camera_Cal/bfs_cal/good`)
- **Board:** 17.5 mm squares
- **Fit / hold-out:** 62 / 8 (smart diversity hold-out)
- **Hold-out:** median **3.52 mm**, **0.33°**, **0.50 px**
- **Intrinsics RMS:** 0.666 px
- **Hand–eye:** andreff → `tool_tcp_*` in `hyperfusion.cfg`
- **Cfg written:** 2026-09-22 (BFS `K`/`D` + camera TCP + `ceiling_mount_height_mm`)

## Rerun

```powershell
cd D:\Pototypy\HyperFusion\app\calibration\multiview
.\.venv\Scripts\python.exe bfs_cal\calibrate_bfs.py `
  --board bfs_cal\board.yaml `
  --images bfs_cal\checkerboard `
  --out bfs_cal\results `
  --holdout 8 `
  --holdout-mode smart
```
