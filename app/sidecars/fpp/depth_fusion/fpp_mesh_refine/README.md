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
```

Tray plane crop is **off by default** (ambient object-only FPP usually has no
tray). ROI then uses an object-centered ``base_link`` +Z box from camera
look-at. Pass ``--remove-tray`` only when the tray is lit and you want the old
RANSAC stage crop (near-horizontal planes only).

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
| `summary.json` | Stage stats |

Dense cleanup (default on): `--sor-k 25 --sor-std 1.75`,
`--workspace-width-mm 300 --workspace-depth-mm 300` (300×300×300 mm box; depth =
height along ``base_link`` +Z). Also ``--workspace-height-mm`` as an alias for
depth. Multi-scale normal consistency. Fusion requires two-view support in 2 mm
cells when ``--fusion-mode intersection``. The final component pass uses a 1.5 mm
radius and removes detached fragments below 500 points.
Use `--no-surface-filter` to disable normal filtering,
`--light-smooth` for mild surface smoothing, or `--no-cleanup` to skip cleanup.
