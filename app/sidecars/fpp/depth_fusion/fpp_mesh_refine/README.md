# Classical FPP dense-cloud refine

Lives under `app/sidecars/fpp/depth_fusion/fpp_mesh_refine`.

```
per-view decode depth (fpp_depth.npy)
→ edge-aware depth filtering
→ depth confidence (C_FPP: modulation + phase + smoothness + edges + mask
  interior + depth jumps)
→ small constrained pose refinement
→ full-resolution multi-view consistency / outlier rejection
→ require two-view support in 2 mm cells
→ confidence-weighted densify (back-project refined depth)
→ optional score-gated TSDF (``--enable-tsdf``; secondary mesh/cloud)
→ robot-pose/tray-derived 200 mm workspace → components → SOR → ROR
→ multi-scale normal estimation / surface-consistency filtering
→ tight 1.5 mm components; remove fragments below 500 points
→ optional light smooth
→ dense_point_cloud.ply
→ orthographic top_view_depth_mm.npy + top_view_depth.png
→ apex-only apex_top_view_depth_mm.npy + apex_top_view_depth.png
```

Tray plane crop and dense cleanup are on by default. The physical workspace is
centered where the robot camera optical axes meet the fitted tray plane; it keeps
a 200 × 200 mm stage area and 200 mm above the stage toward the robot.

## Run (Cluster_1_T sweep)

```powershell
cd app\sidecars\fpp\depth_fusion
..\.venv\Scripts\python.exe fpp_mesh_refine\run_fpp_mesh_refine.py `
  --burst D:\Data_JY\2026_Grape_Data_Collection\2026_CLEREL_Niagara\clusters\Nagara_DM_Cluster_1_T\multiview `
  --hand-eye D:\Data_JY\2026_Grape_Data_Collection\2026_CLEREL_Niagara\clusters\camera_cal\bfs_cal\results\flange_T_camera.yaml `
  --pose-mode flange_camera --mode sweep
```

Or: `.\.venv\Scripts\python.exe ...` after `.\setup_venv.ps1` in this folder.
Unified-pipeline output: `<burst>/processed/fusion/`

| File | Meaning |
|------|---------|
| `dense_point_cloud.ply` | Cleaned dense cloud |
| `top_view_depth_mm.npy` | 0.5 mm/pixel topmost height above the stage; invalid is NaN |
| `top_view_depth.png` | Color preview of the top-view height map |
| `apex_top_view_depth_mm.npy` | Same map geometry using only the apex FPP decode |
| `apex_top_view_depth.png` | Color preview of the apex-only depth map |
| `summary.json` | Stage stats |

Dense cleanup (default on): `--sor-k 25 --sor-std 1.75`,
`--workspace-width-mm 200 --workspace-depth-mm 200`, and multi-scale normal
consistency. Fusion requires two-view support in 2 mm cells. The final component
pass uses a 1.5 mm radius and removes detached fragments below 500 points.
Use `--no-surface-filter` to disable normal filtering,
`--light-smooth` for mild surface smoothing, or `--no-cleanup` to skip cleanup.
