#!/usr/bin/env bash
# HyperFusion UR3e connect + small pan move diagnostic (run inside WSL).
set -euo pipefail
PORT="${1:-8766}"
BASE="http://127.0.0.1:${PORT}"
ROBOT_IP="${ROBOT_IP:-192.168.1.10}"

echo "=== health ==="
curl -s "${BASE}/health" | python3 -m json.tool || true

echo "=== connect/start (press Play on External Control when prompted) ==="
curl -s -X POST "${BASE}/connect/start" \
  -H "Content-Type: application/json" \
  -d "{\"ip\":\"${ROBOT_IP}\"}" | python3 -m json.tool || true

for i in $(seq 1 60); do
  STATUS=$(curl -s "${BASE}/connect/status")
  PHASE=$(echo "$STATUS" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('phase',''))" 2>/dev/null || echo "?")
  MSG=$(echo "$STATUS" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('message',''))" 2>/dev/null || echo "")
  echo "[${i}] phase=${PHASE} ${MSG}"
  OK=$(echo "$STATUS" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('ok',False))" 2>/dev/null || echo "False")
  if [ "$OK" = "True" ] && [ "$PHASE" = "done" ]; then
    break
  fi
  if [ "$PHASE" = "failed" ]; then
    echo "$STATUS" | python3 -m json.tool
    exit 1
  fi
  sleep 2
done

echo "=== joints (MoveIt branch) ==="
curl -s "${BASE}/joints" | python3 -m json.tool

read -r -p "Pan delta degrees (default 5): " DELTA
DELTA="${DELTA:-5}"

python3 <<PY
import json, math, urllib.request
base = "${BASE}"
joints = json.load(urllib.request.urlopen(base + "/joints"))["positions"]
pan = joints[0] + math.radians(float("${DELTA}"))
target = [pan] + joints[1:]
body = {
    "joints": target,
    "direct_only": True,
    "home_joints_deg": [160, 0, -90, 0, 90, 180],
    "workspace": {"enabled": True, "length_m": 0.8, "width_m": 0.6, "height_m": 0.6},
}
req = urllib.request.Request(
    base + "/execute_scan_waypoint",
    data=json.dumps(body).encode(),
    headers={"Content-Type": "application/json"},
    method="POST",
)
with urllib.request.urlopen(req, timeout=180) as resp:
    result = json.load(resp)
print(json.dumps(result, indent=2))
PY

echo "=== joints after move ==="
curl -s "${BASE}/joints" | python3 -m json.tool
