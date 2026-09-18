# FPP dense-cloud refine (HyperFusion / fpp / depth_fusion)
# Classical pipeline. No robot I/O. Results under dataset processed/fusion/.
"""
per-view decode depth
→ tray crop → filter → confidence → pose refine → consistency
→ densify → ROI/SOR/ROR → dense_point_cloud.ply
"""

__all__ = ["__version__"]
__version__ = "0.1.0"
