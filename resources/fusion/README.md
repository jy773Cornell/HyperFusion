# HyperFusion dual-camera fusion (offline)

Step-by-step RGB alignment pipeline before HSI cube fusion.

## Dataset layout

Expects a HyperFusion capture session, e.g. `E:\chiptest`:

```
{session}/{mode}/{fx10e|swir3}/preprocessed/
  *_rgb.png
  *_ffc.hdr / *_ffc.raw
  segmentation/segmentation_results.json
  segmentation/masks/mask_*.npy
```

Outputs are written under `{session}/{mode}/fusion/stepXX/`.

## Steps

| Step | Script | Description |
|------|--------|-------------|
| 1 | `step01_load.py` | Load RGB + masks, centroids in px/mm |
| 2 | `step02_upsample.py` | Upsample SWIR RGB (bicubic) + masks (nearest) to FX10e ground grid |
| 3 | `step03_match.py` | Match in mm, median centroid shift |
| 4 | `step04_shift.py` | Apply final shift to upsampled SWIR |
| 5 | `step05_crop.py` | Crop RGB to FOV intersection (step 2 only) |
| 6 | `step06_hsi.py` | Same coarse transforms on FFC HSI cubes → phase 1 complete |

## Environment

Create and use the local venv (once):

```powershell
cd resources/fusion
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Dependencies: `numpy`, `Pillow`, `opencv-python` (see `requirements.txt`).

## Run (reflectance on chiptest)

```powershell
cd resources/fusion
.\.venv\Scripts\Activate.ps1
python step01_load.py --session E:\chiptest --mode reflectance
python step02_upsample.py --session E:\chiptest --mode reflectance
python step03_match.py --session E:\chiptest --mode reflectance
python step05_crop.py --session E:\chiptest --mode reflectance
python step06_hsi.py --session E:\chiptest --mode reflectance
```

Steps 3–4 (match + shift) are used by step 6 for HSI when `step03_report.json` exists. Step 5 crops RGB to the FX/SWIR FOV overlap only.
