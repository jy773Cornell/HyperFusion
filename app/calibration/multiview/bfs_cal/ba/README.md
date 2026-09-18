# Bundle adjustment (folder-local)

Joint BA for Multiview lab checkerboards under `app/calibration/multiview`.
**Does not write `hyperfusion.cfg`.**

## Scripts

| Script | Model |
|--------|--------|
| `run_bundle_adjustment.py` | Fixed shared board (paper-like). **Fails** on this dataset — board poses scatter tens of mm. |
| `run_bundle_adjustment_v2.py` | Per-view board + soft shared-board prior + shared `K`/`HE`. **Use this.** |

## Latest v2 result (`ba/results/`)

| Metric | Value |
|--------|--------|
| Train reprojection RMS | **~0.61 px** |
| Holdout PnP RMS | **~0.65 px** |
| Board spread (median) | **~68 mm** (not collapsed) |
| HE suggestion xyz mm | **≈ [-9.9, -67.3, 161.0]** |

Interpretation: intrinsics BA is healthy; a **single fixed board + one HE** is not supported by all 106 stills (board moved and/or flange/tip inconsistency). Soft prior cannot invent rigidity that is not in the data.

## Run

```powershell
cd D:\Pototypy\HyperFusion\app\calibration\multiview
.\.venv\Scripts\python.exe ba\run_bundle_adjustment_v2.py
```

## Outputs

- `ba/results/camera_intrinsics.yaml`
- `ba/results/flange_T_camera.yaml` (suggestion only)
- `ba/results/base_T_board.yaml` (average of per-view boards)
- `ba/results/ba_report.json`
