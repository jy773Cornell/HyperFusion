#!/usr/bin/env bash
# Create app/sidecars/gsam2/venv for the HyperFusion GSAM2 WSL sidecar.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON="${PYTHON:-python3}"
VENV_DIR="$SCRIPT_DIR/venv"
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"

echo "==> GSAM2 venv: $VENV_DIR"
echo "==> Python: $($PYTHON --version)"

if [[ ! -d "$VENV_DIR" ]]; then
    "$PYTHON" -m venv "$VENV_DIR"
fi

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip wheel setuptools

echo "==> Installing PyTorch (CUDA wheel index: $TORCH_INDEX)..."
python -m pip install torch torchvision --index-url "$TORCH_INDEX"

echo "==> Installing GSAM2 Python dependencies..."
python -m pip install -r requirements.txt

CKPT_DIR="$SCRIPT_DIR/checkpoints"
CKPT="$CKPT_DIR/sam2.1_hiera_large.pt"
CKPT_URL="${SAM2_CHECKPOINT_URL:-https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt}"
if [[ ! -f "$CKPT" ]]; then
    mkdir -p "$CKPT_DIR"
    echo "==> Downloading SAM2.1 large checkpoint (required; ~900 MB)..."
    if command -v curl >/dev/null 2>&1; then
        curl -L --fail --retry 3 -o "$CKPT" "$CKPT_URL"
    elif command -v wget >/dev/null 2>&1; then
        wget -O "$CKPT" "$CKPT_URL"
    else
        echo "WARNING: curl/wget missing; place sam2.1_hiera_large.pt in $CKPT_DIR" >&2
    fi
fi

echo "==> Verifying imports..."
python - <<'PY'
from _sam2_bootstrap import ensure_sam2_package

ensure_sam2_package()
import torch
import cv2
import transformers
import supervision as sv
import sam2
print("torch", torch.__version__, "cuda", torch.cuda.is_available())
if torch.cuda.is_available():
    print("gpu", torch.cuda.get_device_name(0))
print("sam2 package OK")
PY

echo ""
echo "Done. Activate with:"
echo "  source $VENV_DIR/bin/activate"
echo ""
echo "HyperFusion hyperfusion.cfg should use:"
echo "  wsl_bash_command = source ./venv/bin/activate"
