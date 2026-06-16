#!/usr/bin/env bash
# Create resources/sam2/venv for the HyperFusion GSAM2 WSL sidecar.
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

echo "==> Verifying imports..."
PYTHONPATH="$(dirname "$SCRIPT_DIR")" python - <<'PY'
import sys
from pathlib import Path

sys.path.insert(0, str(Path.cwd().parent))
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
