#!/usr/bin/env bash
# Restart isolated mocks 8767-8770 (never 8766). Uses _start_mock_isolated.sh cfg.
set -eo pipefail
cd /mnt/d/Pototypy/HyperFusion/app/sidecars/ur3e
sed -i 's/\r$//' ./scripts/_start_mock_isolated.sh 2>/dev/null || true

for port in 8767 8768 8769 8770; do
  pkill -9 -f "hyperfusion_ur3e.sidecar.server .*--port ${port}" 2>/dev/null || true
  pkill -9 -f "runtime_p${port}" 2>/dev/null || true
done
pkill -9 -f 'ROS_DOMAIN_ID=.42' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.43' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.44' 2>/dev/null || true
pkill -9 -f 'ROS_DOMAIN_ID=.45' 2>/dev/null || true
sleep 2

LOGDIR=/mnt/d/Pototypy/HyperFusion/app/preset/mvs_scan_plans/fpp
mkdir -p "$LOGDIR"
HOME_DEG='90,-180,145,-55,90,-90'

nohup bash ./scripts/_start_mock_isolated.sh 8767 42 50001 "$HOME_DEG" >"$LOGDIR/_mock_8767.log" 2>&1 &
echo "start7=$!"
sleep 1
nohup bash ./scripts/_start_mock_isolated.sh 8768 43 50011 "$HOME_DEG" >"$LOGDIR/_mock_8768.log" 2>&1 &
echo "start8=$!"
sleep 1
nohup bash ./scripts/_start_mock_isolated.sh 8769 44 50021 "$HOME_DEG" >"$LOGDIR/_mock_8769.log" 2>&1 &
echo "start9=$!"
sleep 1
nohup bash ./scripts/_start_mock_isolated.sh 8770 45 50031 "$HOME_DEG" >"$LOGDIR/_mock_8770.log" 2>&1 &
echo "start10=$!"
sleep 4
ss -ltn | grep -E '8767|8768|8769|8770' || echo 'not-listening-yet'
