# Shared WSLg / X11 display setup for RViz (sourced by launch_*.sh).
# Non-interactive `wsl.exe bash -lc` often has empty DISPLAY/WAYLAND — without this,
# rviz2 can init OpenGL but never show a Windows window.

# Prefer X11 via WSLg (more reliable than Wayland for Qt/RViz from HyperFusion).
if [[ -d /mnt/wslg ]] || [[ -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY="${DISPLAY:-:0}"
  # Private runtime dir: Qt rejects WSLg's 0777 /mnt/wslg/runtime-dir.
  _hf_xdg="/tmp/hyperfusion-xdg-runtime-${UID:-$(id -u)}"
  mkdir -p "${_hf_xdg}"
  chmod 700 "${_hf_xdg}" 2>/dev/null || true
  export XDG_RUNTIME_DIR="${_hf_xdg}"
  # Force XCB so we do not depend on wayland-0 in the private runtime dir.
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  unset WAYLAND_DISPLAY
  echo "UR3e display: DISPLAY=${DISPLAY} QT_QPA_PLATFORM=${QT_QPA_PLATFORM} XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" >&2
elif [[ -z "${DISPLAY:-}" ]]; then
  echo "UR3e display: WARNING — no WSLg/X11 detected and DISPLAY is unset; RViz window may not appear." >&2
fi
