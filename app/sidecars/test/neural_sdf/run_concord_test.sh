#!/usr/bin/env bash
set -euo pipefail
ROOT=/mnt/d/Pototypy/HyperFusion
PY="$ROOT/app/sidecars/gsam2/venv/bin/python"
INPUT=/mnt/d/Data_JY/2026_Grape_Data_Collection/2026_CLEREL_Concord/clusters/Concord_BMD_Cluster_1_B/multiview/processed/fusion/dense_point_cloud.ply
OUT=/mnt/d/Data_JY/2026_Grape_Data_Collection/2026_CLEREL_Concord/clusters/Concord_BMD_Cluster_1_B/multiview/processed/fusion/neural_sdf_test
exec "$PY" "$ROOT/app/sidecars/test/neural_sdf/train_neural_sdf.py" --input "$INPUT" --out "$OUT" --device cuda --steps 5000