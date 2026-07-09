#!/usr/bin/env bash
# Ping the UR controller from WSL to verify LAN reachability before connecting.
set -euo pipefail

ROBOT_IP="${1:-192.168.0.10}"
COUNT="${2:-3}"

echo "==> Checking reachability of UR controller at ${ROBOT_IP} (${COUNT} pings)"
if ping -c "${COUNT}" -W 2 "${ROBOT_IP}"; then
    echo "OK: controller responded to ping."
    exit 0
fi

echo "FAIL: could not reach ${ROBOT_IP} from WSL." >&2
echo "Hints:" >&2
echo "  - Confirm the robot is powered and on the same subnet as Windows." >&2
echo "  - Try mirrored networking in %USERPROFILE%\\.wslconfig (Windows 11):" >&2
echo "      [wsl2]" >&2
echo "      networkingMode=mirrored" >&2
echo "  - Reboot WSL: wsl --shutdown" >&2
exit 1
