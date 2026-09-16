#!/usr/bin/env bash
set -eo pipefail
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e
sed -i 's/\r$//' ./scripts/_start_mock_isolated.sh 2>/dev/null || true
pkill -9 -f 'hyperfusion_ur3e.sidecar.server .*--port 8771' 2>/dev/null || true
pkill -9 -f 'runtime_p8771' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.46' 2>/dev/null || true
sleep 1
LOGDIR=/mnt/d/Pototypy/HyperFusion/app/preset/mvs_scan_plans/fpp
mkdir -p "$LOGDIR"
: > "$LOGDIR/_mock_8771.log"
nohup bash ./scripts/_start_mock_isolated.sh 8771 46 50041 '90,-180,145,-55,90,-90' \
  >"$LOGDIR/_mock_8771.log" 2>&1 &
echo "start11=$!"
sleep 4
ss -ltn | grep 8771 || echo 'not-listening-yet'
