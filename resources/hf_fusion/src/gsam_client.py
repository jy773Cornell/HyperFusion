# HTTP client for the GSAM2 sidecar (backend/offline). No hardware I/O.
from __future__ import annotations

import json
import urllib.request
from pathlib import Path

DEFAULT_GSAM_URL = "http://127.0.0.1:8765"


def win_to_wsl(path: Path) -> str:
    text = str(path.resolve()).replace("\\", "/")
    if len(text) >= 2 and text[1] == ":":
        return f"/mnt/{text[0].lower()}/{text[2:].lstrip('/')}"
    return text


def post_json(url: str, path: str, body: dict, timeout: int = 900) -> dict:
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url.rstrip("/") + path, data=data, headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def gsam_health(url: str, timeout: int = 5) -> dict:
    with urllib.request.urlopen(url.rstrip("/") + "/health", timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def require_gsam_ready(url: str) -> dict:
    health = gsam_health(url)
    if not health.get("model_loaded"):
        raise RuntimeError(f"GSAM server is up but models are not loaded: {health}")
    return health
