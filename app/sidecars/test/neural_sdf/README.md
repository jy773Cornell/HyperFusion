# Neural SDF test

This is an isolated CUDA/WSL experiment for fitting an implicit signed-distance field to a fused HyperFusion PLY cloud containing points and normals.

It reuses the CUDA PyTorch runtime already installed at `app/sidecars/gsam2/venv`; it does not alter the FPP pipeline.

Run from WSL:

```bash
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/test/neural_sdf
./run_concord_test.sh
```

Outputs go to the selected cluster's `multiview/processed/fusion/neural_sdf_test/` folder:

- `model.pt` — checkpoint
- `normalization.json` — center/scale used by the field
- `train_metrics.json` — final loss data

The test trains on surface zero-level constraints, normal-direction offsets, normal-gradient alignment, and Eikonal regularization. It is experimental: it will smooth small gaps but cannot correct a misaligned input cloud or reliably infer unseen surfaces.