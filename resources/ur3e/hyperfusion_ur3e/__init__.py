"""HyperFusion UR3e ROS 2 sidecar package (WSL)."""
from pathlib import Path

# resources/ur3e — package root (config/, launch/, scripts/).
PKG_ROOT = Path(__file__).resolve().parent.parent

__version__ = "0.1.0"
