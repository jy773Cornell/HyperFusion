# HyperFusion FPP decoder + MVS depth fusion (offline)

New captures are a **26-frame HDMI PSP burst**. Color stills come from the plan
**RGB ring** (not a full-white frame inside the FPP sequence). Older captures
that begin with a visual-color TIFF remain supported for decode.

## Pipeline

```
{datafolder}/multiview/          raw #####.tif + .json  (unchanged)
  processed/
    metadata.json                stage timings + paths
    decode/<stem>/fpp_*.npy      per-pin decode
    fusion/
      dense_point_cloud.ply      ← only geometry output
      summary.json
```

Stages: decode → tray crop → filter → confidence → pose refine → consistency → densify → ROI/SOR/ROR.

FPP MVS is **sweep/ring only** (no apex). Stem `00000` is the first ring pin, not nadir.

## Setup

```powershell
cd app\sidecars\fpp
.\setup_venv.ps1
# classical fusion deps (open3d) also:
cd depth_fusion
.\setup_venv.ps1
```

## Run (recommended)

```powershell
cd app\sidecars\fpp
.\.venv\Scripts\python.exe fpp_mvs_cli.py `
  --input D:\Data_JY\...\Nagara_DM_Cluster_1_T
```

Accepts the cluster root or its `multiview/` folder. Skips automatically if pose JSON has no `fpp_pattern`.

App auto-runs this after an FPP MVS capture when `fpp_mvs_auto_process = true` in `hyperfusion.cfg`.

## Other CLIs

| Script | Role |
|--------|------|
| `fpp_mvs_cli.py` | **Full** decode + dense cloud → `processed/` |
| `fpp_cli.py` | Decode only (`--flat` for `processed/decode` layout) |
| `depth_fusion/fpp_mesh_refine/run_fpp_mesh_refine.py` | Fusion only (given an existing decode tree) |
| `depth_fusion/fuse_tsdf.py` | Simple TSDF CLI |

## Calibrate (tilted checkerboard → millimetres)

```powershell
cd app\calibration\multiview
.\.venv\Scripts\python dlp_cal\check_fpp_board.py --input dlp_cal\checkerboard
.\.venv\Scripts\python dlp_cal\calibrate_fpp_geometry.py --holdout 3
```

Writes `dlp_cal/results/camera_projector_stereo.yaml`.
