#!/usr/bin/env bash
# Repair mismatched ROS 2 UR / ros2_control packages in WSL (common after partial apt upgrades).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

detect_ros_distro() {
    if [[ -n "${ROS_DISTRO:-}" ]]; then
        echo "$ROS_DISTRO"
        return
    fi
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")"
        case "$codename" in
            jammy) echo "humble" ;;
            noble) echo "jazzy" ;;
            *) echo "jazzy" ;;
        esac
        return
    fi
    echo "jazzy"
}

ROS_DISTRO="$(detect_ros_distro)"

echo "==> Stopping stale UR ROS processes..."
pkill -f ur_control.launch.py 2>/dev/null || true
pkill -f joint_states_stamper 2>/dev/null || true
sleep 1

echo "==> Upgrading ROS ${ROS_DISTRO} UR + ros2_control packages (sudo)..."
sudo apt-get update
sudo apt-get install -y --only-upgrade \
    "ros-${ROS_DISTRO}-ur" \
    "ros-${ROS_DISTRO}-ur-robot-driver" \
    "ros-${ROS_DISTRO}-ur-controllers" \
    "ros-${ROS_DISTRO}-ur-dashboard-msgs" \
    "ros-${ROS_DISTRO}-ur-msgs" \
    "ros-${ROS_DISTRO}-ur-description" \
    "ros-${ROS_DISTRO}-controller-manager" \
    "ros-${ROS_DISTRO}-controller-manager-msgs" \
    "ros-${ROS_DISTRO}-diagnostic-updater" \
    "ros-${ROS_DISTRO}-force-torque-sensor-broadcaster" \
    "ros-${ROS_DISTRO}-hardware-interface" \
    "ros-${ROS_DISTRO}-joint-state-broadcaster" \
    "ros-${ROS_DISTRO}-joint-trajectory-controller" \
    "ros-${ROS_DISTRO}-pose-broadcaster" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-ros2-controllers"

echo "==> Verifying ros2_control can list controllers..."
# ROS setup.bash trips 'set -u'; relax nounset only while sourcing.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
echo "    (No driver running yet — 'waiting for service' here is expected.)"
timeout 8 ros2 control list_controllers || true

echo ""
echo "==> Repair complete. Restart HyperFusion and click Connect again."
echo "    Package root: $PKG_ROOT"
