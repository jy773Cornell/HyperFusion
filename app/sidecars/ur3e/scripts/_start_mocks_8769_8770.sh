#!/usr/bin/env bash
# Start isolated mocks on 8769/8770 only (leave 8766/8767/8768 alone).
set -eo pipefail
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e

pkill -9 -f 'hyperfusion_ur3e.sidecar.server .*--port 8769' 2>/dev/null || true
pkill -9 -f 'hyperfusion_ur3e.sidecar.server .*--port 8770' 2>/dev/null || true
pkill -9 -f 'runtime_p8769' 2>/dev/null || true
pkill -9 -f 'runtime_p8770' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.44' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.45' 2>/dev/null || true
sleep 2

LOGDIR=/mnt/d/Pototypy/HyperFusion/app/preset/mvs_scan_plans/fpp
mkdir -p "$LOGDIR"
: > "$LOGDIR/_mock_8769.log"
: > "$LOGDIR/_mock_8770.log"

nohup bash ./scripts/_start_mock_isolated.sh 8769 44 50021 '90,-180,145,-55,90,-90' \
  >"$LOGDIR/_mock_8769.log" 2>&1 &
echo "start9=$!"
sleep 2
nohup bash ./scripts/_start_mock_isolated.sh 8770 45 50031 '90,-180,145,-55,90,-90' \
  >"$LOGDIR/_mock_8770.log" 2>&1 &
echo "start10=$!"
sleep 3
ss -ltn | grep -E '8769|8770' || echo 'not-listening-yet'
