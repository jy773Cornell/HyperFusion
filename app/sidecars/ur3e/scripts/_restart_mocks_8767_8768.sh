#!/usr/bin/env bash
# Restart isolated mocks on 8767/8768 only (never 8766).
set -eo pipefail
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e

# Kill prior sidecars + runtime trees for these ports only.
pkill -9 -f 'hyperfusion_ur3e.sidecar.server .*--port 8767' 2>/dev/null || true
pkill -9 -f 'hyperfusion_ur3e.sidecar.server .*--port 8768' 2>/dev/null || true
pkill -9 -f 'runtime_p8767' 2>/dev/null || true
pkill -9 -f 'runtime_p8768' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.42' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.43' 2>/dev/null || true
sleep 2

LOGDIR=/mnt/d/Pototypy/HyperFusion/app/preset/mvs_scan_plans/fpp
mkdir -p "$LOGDIR"
: > "$LOGDIR/_mock_8767.log"
: > "$LOGDIR/_mock_8768.log"

nohup bash ./scripts/_start_mock_isolated.sh 8767 42 50001 '90,-180,145,-55,90,-90' \
  >"$LOGDIR/_mock_8767.log" 2>&1 &
echo "start7=$!"
sleep 2
nohup bash ./scripts/_start_mock_isolated.sh 8768 43 50011 '90,-180,145,-55,90,-90' \
  >"$LOGDIR/_mock_8768.log" 2>&1 &
echo "start8=$!"
sleep 3
ss -ltn | grep -E '8767|8768' || echo 'not-listening-yet'
