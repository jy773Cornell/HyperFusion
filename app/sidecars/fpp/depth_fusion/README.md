# FPP depth fusion (classical) — under `app/sidecars/fpp/depth_fusion`

Prefer the unified CLI at the parent folder:

```powershell
cd app\sidecars\fpp
.\.venv\Scripts\python.exe fpp_mvs_cli.py --input D:\path\to\Cluster_X
```

That writes `{Cluster_X}/multiview/processed/{decode,fusion}` + `metadata.json`.

## This folder

| Path | Role |
|------|------|
| `hyperfusion_depth_fusion/` | Shared poses + TSDF helpers |
| `fpp_mesh_refine/` | Classical densify + cleanup pipeline |
| `fuse_tsdf.py` | Optional simple TSDF CLI |

```powershell
cd app\sidecars\fpp\depth_fusion
.\setup_venv.ps1   # or reuse ..\.venv
```
