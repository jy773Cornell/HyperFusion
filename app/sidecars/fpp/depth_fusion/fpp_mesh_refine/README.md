# Classical FPP dense-cloud refine

Lives under `app/sidecars/fpp/depth_fusion/fpp_mesh_refine`.

```
per-view decode depth (fpp_depth.npy)
→ edge-aware depth filtering
→ depth confidence estimation
→ small constrained pose refinement
→ multi-view consistency / outlier rejection
→ densify (back-project refined depth)
→ ROI/component → SOR → ROR → optional light smooth
→ dense_point_cloud.ply
```

Tray plane crop and dense cleanup are on by default.

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
| `dense_point_cloud.ply` | Only geometry output — cleaned dense cloud |
| `summary.json` | Stage stats |

Dense cleanup (default on): `--sor-k 25 --sor-std 1.75`; `--light-smooth` for mild surface smooth; `--no-cleanup` to skip.
