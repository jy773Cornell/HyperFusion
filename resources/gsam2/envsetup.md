# GSAM2 environment (WSL) for HyperFusion

HyperFusion runs GSAM2 as a **WSL sidecar**: the Windows app starts `gsam2_server.py` in Ubuntu via `wsl.exe`. Models stay loaded in WSL; scan outputs on `/mnt/d/...` are read/written from both sides.

---

## Prerequisites

| Item | Notes |
|------|--------|
| **WSL 2 + Ubuntu** | PowerShell (Admin): `wsl --install`, reboot, pick **Ubuntu** |
| **NVIDIA GPU driver (Windows)** | WSL2 uses the host driver; run `nvidia-smi` inside WSL to confirm |
| **Python ≥ 3.10** | Ubuntu 22.04+ ships `python3` (3.10+); this repo was tested with 3.12 |
| **Checkpoint** | `checkpoints/sam2.1_hiera_large.pt` must exist under this folder |

Optional: CUDA toolkit (`nvcc`) only if you build SAM2 CUDA extensions. HyperFusion uses the vendored SAM2 Python code with `PYTHONPATH`; the optional CUDA extension is **not** required for segmentation.

---

## One-time setup

From **WSL**, in this directory (`resources/gsam2`):

```bash
cd /mnt/d/Pototypy/HyperFusion/resources/gsam2   # adjust drive/path if needed
chmod +x install_venv.sh
./install_venv.sh
```

This creates **`./venv`** and installs:

- PyTorch + torchvision (CUDA wheels, default index `cu128`)
- GroundingDINO deps (`transformers`, …)
- SAM2 runtime deps (`hydra-core`, `opencv-python`, `supervision`, …)

**Note:** Installing on `/mnt/d/...` can look frozen for several minutes while pip writes large wheels (PyTorch ~3 GB total). That is normal. If you interrupted it, resume with:

```bash
source ./venv/bin/activate
pip install -r requirements.txt
```

For faster installs, put the venv on the WSL filesystem instead (see below).

Activate manually (for debugging):

```bash
source ./venv/bin/activate
```

---

## HyperFusion config

In `app/hyperfusion.cfg` → `[segmentation]`:

```ini
wsl_distro = Ubuntu
wsl_bash_command = source ./venv/bin/activate
sam2_repo_linux =                          ; empty = auto (/mnt/d/.../resources/gsam2)
server_port = 8765
```

The app runs: `cd <sam2_repo_linux> && source ./venv/bin/activate && python gsam2_server.py ...`

**Start the server** from the Capture tab → Preprocessing → **Start GSAM server**, or test manually:

```bash
cd /mnt/d/Pototypy/HyperFusion/resources/gsam2
source ./venv/bin/activate
export PYTHONPATH="/mnt/d/Pototypy/HyperFusion/resources:$PYTHONPATH"
python gsam2_server.py --host 0.0.0.0 --port 8765 --warmup
```

Health check from Windows PowerShell:

```powershell
curl http://127.0.0.1:8765/health
```

---

## Paths: Windows vs WSL

| Windows | WSL |
|---------|-----|
| `D:\Pototypy\HyperFusion\resources\gsam2` | `/mnt/d/Pototypy/HyperFusion/resources/gsam2` |
| Scan output `D:\data\capture\...` | `/mnt/d/data/capture/...` |

- **Code + venv** can live on `/mnt/d/...` (same git tree as the app).
- **Large temp writes** are often faster on the WSL filesystem (`~/data/...`); HyperFusion scan folders on `D:` are fine via `/mnt/d/`.

---

## CPU-only / different CUDA

Edit `install_venv.sh` or install PyTorch yourself before `requirements.txt`:

```bash
# CPU only
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Other CUDA index (match your driver): cu124, cu126, cu128, …
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
```

Then set in `hyperfusion.cfg`:

```ini
detector_device = cpu
sam2_device = cpu
```

(GPU strongly recommended for production.)

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `wsl.exe` not found | Install WSL 2; distro name must match `wsl_distro` |
| Server never becomes ready | Check Log tab; run `gsam2_server.py` manually in WSL for traceback |
| `Could not read image` | Paths must be WSL paths under `/mnt/...` (app converts automatically) |
| GroundingDINO download fails | First run needs Hugging Face access; `huggingface-cli login` if gated |
| Out of GPU memory | Lower `max samples` in UI or use a smaller SAM2 checkpoint in cfg |
| Wrong Python | Recreate venv: `rm -rf venv && ./install_venv.sh` |
| Install appears stuck on `/mnt/d/` | Wait 5–15 min; or resume with `pip install -r requirements.txt` |
| Interrupted install | PyTorch may be done; run `pip install -r requirements.txt` to finish |

---

## What the sidecar produces

After a stage scan with **Run GSAM segmentation** enabled, each stream gets:

`preprocessed/segmentation/`

- `mask_*.png`, `mask_*.npy`, `overlay.png`, `stack_mask.png`
- `segmented_rgb/roi_*.png` — per-sample masked RGB
- `roi_spectra.csv`, `roi_spectra_plot.png`
- `segmentation_results.json`
