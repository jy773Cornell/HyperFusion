#!/usr/bin/bash
# Clean restart of UR3e WSL sidecar + ROS driver (HyperFusion).
set -euo pipefail

PORT="${UR3E_SIDECAR_PORT:-8766}"
BASE="http://127.0.0.1:${PORT}"

echo "=== UR3e clean start (port ${PORT}) ==="

if curl -s --max-time 2 "${BASE}/health" >/dev/null 2>&1; then
  echo "Disconnecting sidecar…"
  curl -s --max-time 5 -X POST "${BASE}/disconnect" -H "Content-Type: application/json" -d "{}" || true
  echo "Shutting down sidecar…"
  curl -s --max-time 5 -X POST "${BASE}/shutdown" -H "Content-Type: application/json" -d "{}" || true
  sleep 2
fi

echo "Stopping stale UR ROS processes…"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "${SCRIPT_DIR}/kill_stale_ur_ros.sh"

if ss -tln 2>/dev/null | grep -q ":${PORT} "; then
  echo "Port ${PORT} still in use — killing holder…"
  fuser -k "${PORT}/tcp" 2>/dev/null || true
  sleep 1
fi

echo "Port ${PORT} status:"
ss -tln 2>/dev/null | grep ":${PORT} " || echo "  free"

echo "Done. Start sidecar from HyperFusion app or:"
echo "  cd app/sidecars/ur3e && ./venv/bin/ur3e_server --port ${PORT} ..."
