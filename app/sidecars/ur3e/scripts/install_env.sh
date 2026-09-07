#!/usr/bin/env bash
# One-time UR3e WSL environment setup for HyperFusion (ROS 2 Humble + UR driver + Python venv).
# Run inside WSL Ubuntu 22.04. Idempotent — safe to re-run.
set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Live in scripts/; package root is the parent (setup.py, requirements.txt, venv).
if [[ -f "$THIS_DIR/../setup.py" ]]; then
    ROOT="$(cd "$THIS_DIR/.." && pwd)"
else
    ROOT="$THIS_DIR"
fi
cd "$ROOT"

PYTHON="${PYTHON:-python3}"
VENV_DIR="$ROOT/venv"

# Pick the ROS 2 distro that matches the Ubuntu release (override with ROS_DISTRO=...).
detect_ros_distro() {
    if [[ -n "${ROS_DISTRO:-}" ]]; then
        echo "$ROS_DISTRO"
        return
    fi
    local codename=""
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-}")"
    fi
    case "$codename" in
        jammy)  echo "humble" ;;   # Ubuntu 22.04
        noble)  echo "jazzy" ;;    # Ubuntu 24.04
        *)      echo "humble" ;;
    esac
}

ROS_DISTRO="$(detect_ros_distro)"

SKIP_ROS=false
SKIP_VENV=false

usage() {
    echo "Usage: $0 [--skip-ros] [--skip-venv]"
    echo ""
    echo "  --skip-ros   Skip ROS 2 / UR driver install (sidecar will not run)"
    echo "  --skip-venv  ROS 2 + UR packages only (no ./venv)"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-ros) SKIP_ROS=true ;;
        --skip-venv) SKIP_VENV=true ;;
        -h|--help) usage ;;
        *)
            echo "Unknown option: $1" >&2
            usage
            ;;
    esac
    shift
done

chmod +x scripts/*.sh

# WSL/bash fails on CRLF line endings (set: pipefail\r invalid option).
for script in install_env.sh scripts/*.sh; do
    if [[ -f "$script" ]]; then
        sed -i 's/\r$//' "$script" 2>/dev/null || true
    fi
done

install_ros() {
    if [[ -r /etc/os-release ]]; then
        # shellcheck disable=SC1091
        source /etc/os-release
        case "${UBUNTU_CODENAME:-}" in
            jammy|noble) ;;  # supported: 22.04 -> humble, 24.04 -> jazzy
            *)
                echo "WARNING: ${PRETTY_NAME:-unknown} is untested; using ROS 2 ${ROS_DISTRO}." >&2
                ;;
        esac
    fi

    echo "==> Installing ROS 2 ${ROS_DISTRO} + UR driver (requires sudo)..."

    if ! command -v rosdep >/dev/null 2>&1; then
        sudo apt-get update
        sudo apt-get install -y curl gnupg lsb-release software-properties-common
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
        sudo apt-get update
        sudo apt-get install -y \
            ros-"${ROS_DISTRO}"-ros-base \
            python3-colcon-common-extensions \
            python3-rosdep \
            python3-vcstool
        if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
            sudo rosdep init
        fi
        rosdep update
    fi

    sudo apt-get install -y "ros-${ROS_DISTRO}-ur"

    # Extra controller packages (names differ by distro).
    local -a extra_pkgs=(
        "ros-${ROS_DISTRO}-controller-manager"
        "ros-${ROS_DISTRO}-controller-manager-msgs"
        "ros-${ROS_DISTRO}-diagnostic-updater"
        "ros-${ROS_DISTRO}-force-torque-sensor-broadcaster"
        "ros-${ROS_DISTRO}-hardware-interface"
        "ros-${ROS_DISTRO}-joint-state-broadcaster"
        "ros-${ROS_DISTRO}-joint-trajectory-controller"
        "ros-${ROS_DISTRO}-ros2-control"
        "ros-${ROS_DISTRO}-ros2-controllers"
    )
    if [[ "$ROS_DISTRO" == "humble" ]]; then
        extra_pkgs+=("ros-humble-scaled-joint-trajectory-controller")
        extra_pkgs+=("ros-humble-moveit-ros")
    elif [[ "$ROS_DISTRO" == "jazzy" ]]; then
        extra_pkgs+=("ros-jazzy-ur-controllers")
        extra_pkgs+=("ros-jazzy-moveit-ros")
    fi

    for pkg in "${extra_pkgs[@]}"; do
        if apt-cache show "$pkg" &>/dev/null; then
            sudo apt-get install -y "$pkg"
        else
            echo "NOTE: optional package not in apt, skipping: $pkg" >&2
        fi
    done

    echo "==> ROS 2 ${ROS_DISTRO} + ros-${ROS_DISTRO}-ur installed."
    echo "    Source before driver launch: source /opt/ros/${ROS_DISTRO}/setup.bash"
}

install_venv() {
    echo "==> UR3e sidecar venv: $VENV_DIR"
    echo "==> Python: $($PYTHON --version)"

    if [[ ! -d "$VENV_DIR" ]]; then
        "$PYTHON" -m venv "$VENV_DIR"
    fi

    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
    python -m pip install --upgrade pip wheel setuptools
    python -m pip install -r requirements.txt
    python -m pip install -e .

    echo "==> Verifying hyperfusion_ur3e package..."
    python - <<'PY'
from hyperfusion_ur3e.bridge import Ur3eRosBridge
from hyperfusion_ur3e.sidecar.server import build_arg_parser
print("hyperfusion_ur3e OK")
PY
}

if [[ "$SKIP_ROS" == false ]]; then
    install_ros
fi

if [[ "$SKIP_VENV" == false ]]; then
    install_venv
fi

echo ""
echo "Real robot on LAN (one-time, from Windows PowerShell as Administrator):"
echo "  cd app/sidecars/ur3e && .\\install_env.ps1 -SetupRobotNetwork -ShutdownWsl"
echo "  # or: .\\scripts\\setup_wsl_robot_network.ps1 -ShutdownWsl"
echo ""
echo "==> UR3e environment ready."
echo "Simulation server:"
echo "  cd $ROOT && source /opt/ros/\$ROS_DISTRO/setup.bash && ./venv/bin/ur3e_server --use-mock-hardware --port 8766"
echo "  # or: ./venv/bin/python -m hyperfusion_ur3e.sidecar.server --use-mock-hardware --port 8766"
echo ""
echo "ROS launch (after colcon build --symlink-install in this directory):"
echo "  source install/setup.bash && ros2 launch hyperfusion_ur3e sidecar.launch.py"
echo "  source install/setup.bash && ros2 launch hyperfusion_ur3e moveit.launch.py"
echo ""
echo "Health check from Windows PowerShell:"
echo "  wsl curl -s http://127.0.0.1:8766/health"
