# HyperFusion dual-camera fusion (offline)

FX10e + SWIR3 **spatial registration and spectral fusion** in three stages:

1. **Coarse alignment** — `src/coarse_alignment.py`
2. **Phase correction** — `src/phase_correction.py`
3. **Spectral fusion** — `src/spectral_fusion.py`

Integrated by `src/hf_fusion_pipeline.py`. The app invokes `fusion_cli.py` as a one-shot subprocess.

## Layout

```
resources/hf_fusion/
  fusion_cli.py                  # CLI entry (used by HyperFusion app)
  requirements.txt
  src/
    hf_fusion_pipeline.py        # pipeline integrator (run_fusion_pipeline)
    coarse_alignment.py
    phase_correction.py
    spectral_fusion.py
    utils/                       # types, masks, config, envi, resample, dataset, viz, ...
```



## CLI

```powershell
cd resources\hf_fusion
.\.venv\Scripts\Activate.ps1
python fusion_cli.py --session E:\chiptest --mode reflectance
```

On success, prints one JSON line to stdout:

```json
{"ok": true, "alignment_json": "...", "roi_count": 3, "pipeline_complete": true}
```



### Prerequisites

Per camera under `{session}/{mode}/{camera}/preprocessed/`:

- `*_rgb.png` (from capture post-process)
- `*_ffc.hdr` / `.raw` (from capture post-process)
- `segmentation/segmentation_results.json` + `segmentation/masks/` (from GSAM)



## Programmatic use

```python
from pathlib import Path

from src.hf_fusion_pipeline import FusionPipelineParams, run_fusion_pipeline

result = run_fusion_pipeline(
    FusionPipelineParams(
        session=Path(r"E:\chiptest"),
        mode="reflectance",
    )
)

print(result.alignment_json)
```



## Outputs

```
{session}/{mode}/fusion/
  roi_spectra_fx10e_swir_{mode}.csv    # all ROIs combined (GSAM-compatible columns)
  roi_spectra_fx10e_swir_{mode}.png    # all ROIs mean ± 1σ plot (shaded bands per ROI)
  metadata/
    alignment.json
    fx10e_centroids.png
    swir3_centroids.png
  roi_{NNN}_fx10e_swir3/
    roi_{NNN}_fx10e_swir3.raw / .hdr
    roi_{NNN}_rgb_overlay.png
    roi_{NNN}_maskoverlay_outline.png
    roi_{NNN}_mask.npy / .png
    roi_spectra.csv                    # mean spectrum (same columns as GSAM roi_spectra.csv)
    roi_spectra_plot.png               # single-ROI mean ± 1σ vs wavelength
```



## Environment

One venv beside the deployed app (not in git — too large):

```powershell
cd app\build\Release\hf_fusion
.\setup_venv.ps1
```



## App integration (`hyperfusion.cfg`)

```ini
[fusion]
fusion_margin_mm = 5.0
fusion_timeout_ms = 3600000
```

After a Release build, scripts land in `{app}/hf_fusion/`. Run `setup_venv.ps1` there once (`build_app.ps1` does this automatically when `.venv` is missing).