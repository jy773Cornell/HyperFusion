#!/usr/bin/env bash
BASE="http://127.0.0.1:8766"
for i in $(seq 1 40); do
  echo "--- poll $i ---"
  curl -s "${BASE}/connect/status"
  echo
  PHASE=$(curl -s "${BASE}/connect/status" | python3 -c "import sys,json; print(json.load(sys.stdin).get('phase',''))")
  if [ "$PHASE" = "done" ] || [ "$PHASE" = "failed" ]; then
    exit 0
  fi
  sleep 3
done
