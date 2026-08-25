# UR3e WSL sidecar (ROS 2) for HyperFusion

HyperFusion controls a **Universal Robots UR3e** via a **WSL sidecar**: the Windows app starts `ur3e_server` in Ubuntu through `wsl.exe`, mirroring the GSAM2 pattern. The sidecar always uses **ROS 2 / ur_robot_driver**. Set `use_mock_hardware = true` for simulation (no physical robot) or `false` for the real arm.

UI lives under **Multiview** (UR3e robot + BFS camera). Hemisphere scan planning/execute uses **MoveIt** with a CAD tool mesh and an optical TCP at the BFS sensor face.

---

## Prerequisites

| Item | Notes |
|------|--------|
| **WSL 2 + Ubuntu 22.04 or 24.04** | `wsl --install`, reboot, pick **Ubuntu** |
| **Robot on LAN** | Default controller IP `192.168.0.10` (adjust in `hyperfusion.cfg`) |
| **ROS 2** | Humble (Jammy) or Jazzy (Noble) — installed by `install_env.sh` |
| **Python ≥ 3.10** | Ubuntu ships `python3` |

Optional: if WSL cannot reach the robot, or External Control cannot reach the PC, run (Administrator PowerShell recommended):

```powershell
cd resources\ur3e
.\scripts\setup_wsl_robot_network.ps1 -ShutdownWsl
```

This writes `%USERPROFILE%\.wslconfig` (`networkingMode=mirrored`), adds inbound firewall rules for TCP **50001–50004**, and **removes** stale `netsh portproxy` rules to `127.0.0.1` (those break mirrored WSL — the robot must reach `reverse_ip:50002` on the shared LAN IP). Or use `.\install_env.ps1 -SetupRobotNetwork -ShutdownWsl`.

---

## One-time setup

`install_env.sh` is a **bash** script — it must run inside WSL, not in PowerShell.

### From Windows PowerShell (recommended)

```powershell
cd D:\Pototypy\HyperFusion\resources\ur3e   # adjust drive/path
.\install_env.ps1
```

`install_env.ps1` delegates to WSL Ubuntu and runs `./install_env.sh`.

### From WSL

```bash
cd /mnt/d/Pototypy/HyperFusion/resources/ur3e   # adjust drive/path
chmod +x install_env.sh scripts/*.sh
./install_env.sh
```

Optional flags:

| Flag | Purpose |
|------|---------|
| `--skip-ros` | Python venv only (no ROS 2 / UR driver apt install) |
| `--skip-venv` | ROS 2 + UR packages only (no `./venv`) |

This installs:

1. **ROS 2** + `ros-*-ur` (Universal Robots driver stack) and MoveIt packages
2. **`./venv`** with an editable install of **`hyperfusion_ur3e`** and console script **`./venv/bin/ur3e_server`**

Re-run after pulling package changes: `rm -rf venv && ./install_env.sh` (or `.\install_env.ps1`).

---

## Manual server test (simulation)

```bash
cd /mnt/d/Pototypy/HyperFusion/resources/ur3e
source /opt/ros/jazzy/setup.bash   # or humble on 22.04
./venv/bin/ur3e_server --use-mock-hardware --port 8766
```

Alternative entry point:

```bash
./venv/bin/python -m hyperfusion_ur3e.sidecar.server --use-mock-hardware --port 8766
```

Health check from **Windows PowerShell** (same pattern as GSAM2 — curl via WSL):

```powershell
wsl curl -s http://127.0.0.1:8766/health
wsl curl -s http://127.0.0.1:8766/joints
wsl curl -s -X POST http://127.0.0.1:8766/connect -H "Content-Type: application/json" -d "{}"
wsl curl -s http://127.0.0.1:8766/pose
```

Stop the server:

```powershell
wsl curl -s -X POST http://127.0.0.1:8766/shutdown
```

---

## MoveIt (optional)

MoveIt + RViz runs in a **separate WSL window** — not inside the HyperFusion sidecar. Use it for interactive planning; the app uses the HTTP sidecar for scripted moves and scan execute.

**Requirements:** robot connected in HyperFusion (or sidecar running with mock hardware) and live **`/joint_states`**.

From WSL:

```bash
cd /mnt/d/Pototypy/HyperFusion/resources/ur3e
./scripts/launch_moveit.sh jazzy ur3e    # ros_distro, ur_type
```

Or from the HyperFusion **Multiview → UR3e** tab: **Start MoveIt** / **Stop MoveIt**. Closing the RViz window stops MoveIt.

Preflight (WSL):

```bash
./scripts/wait_for_joint_states.sh jazzy 120
ros2 topic echo /joint_states --once
```

After a colcon build in this directory you can also use:

```bash
source install/setup.bash
ros2 launch hyperfusion_ur3e moveit.launch.py
```

---

## Tool payload + optical TCP

Scan IK / MoveIt tip the **BFS sensor face**, not the flange center.

| Frame / link | Role |
|--------------|------|
| `tool0` | UR flange frame (zero TCP) |
| `hyperfusion_tool_payload` | CAD collision/visual mesh bolted to `tool0` |
| `hyperfusion_tcp` | Optical TCP (orange sphere in RViz) — MoveIt scan tip |

**Mesh**

- File: `urdf/meshes/ur_tool_payload.stl` (metres, flange origin)
- Shape: `tool_payload_shape = mesh`
- Visual/collision origins apply a **180° pan about Z** so CAD XY matches `tool0` after export
- `tool_payload_radius_mm` is the **C403A0 pinch-guard sphere only**, not mesh size

**TCP (cfg → tool0)**

Measure the sensor-face centroid in Fusion (flange origin). With the mesh pan-180 applied, convert CAD → tool0 as **`(−x, −y, z)`**:

| | X | Y | Z |
|--|---|---|---|
| Fusion CAD (example) | 0 | +56.035 mm | 20 mm |
| `hyperfusion.cfg` (`tool_tcp_*_mm`) | 0 | **−56.035** | **20** |

Restart the UR3e sidecar after changing mesh/TCP so `config/runtime_robot_description.urdf` rematerializes.

More scan design notes: [`hemisphere-scan.md`](hemisphere-scan.md).

---

## Robot network check

Before connecting to hardware (`use_mock_hardware = false`):

```bash
./scripts/check_robot_network.sh 192.168.0.10
```

---

## HyperFusion config

In `app/hyperfusion.cfg` → `[multiview]` (aliases `[3d scanning]` and `[ur3e]` still accepted):

```ini
use_multiview = true
wsl_distro = Ubuntu
ur3e_repo_linux =                          ; empty = auto (/mnt/d/.../resources/ur3e)
server_port = 8766
robot_ip = 192.168.1.10
reverse_ip = 192.168.1.20                  ; External Control remote PC
dashboard_port = 29999
rtde_port = 30004
use_mock_hardware = true
prestart_driver = false
ros_distro = jazzy
ur_type = ur3e
max_linear_speed_m_per_s = 0.05
max_linear_accel_m_per_s2 = 0.3
max_joint_velocity_deg_s = 60

tool_payload_shape = mesh
tool_payload_mesh = ur_tool_payload.stl
tool_payload_radius_mm = 50                ; pinch sphere only
tool_tcp_x_mm = 0                          ; optical TCP in tool0 (mm)
tool_tcp_y_mm = -56.035
tool_tcp_z_mm = 20

ceiling_mount_height_mm = 600
workspace_boundary_enabled = true
workspace_length_mm = 900
workspace_width_mm = 600
workspace_height_mm = 600
mount_roll_deg = 180
mount_pitch_deg = 0
mount_yaw_deg = 0
mount_offset_x_mm = 0
mount_offset_y_mm = 0
home_joints_deg = 160, 0, -90, 0, 90, 180
```

| Key | Notes |
|-----|--------|
| `use_mock_hardware` | `true` = UR driver fake hardware; `false` = real robot + External Control |
| `prestart_driver` | `true` warms ROS driver at app launch (~2 min) |
| `tool_payload_*` | CAD mesh for MoveIt collision; radius = pinch guard only |
| `tool_tcp_*_mm` | Optical TCP offset in **tool0** (after mesh pan-180) |
| `home_joints_deg` | Scan home / retreat / mock startup pose (deg) |
| `mount_*` / `ceiling_mount_height_mm` | World → base_link; roll 180 = ceiling |

The Windows app launches:

```bash
cd <ur3e_repo_linux> && source /opt/ros/<ros_distro>/setup.bash && ./venv/bin/ur3e_server ...
```

The WSL sidecar starts automatically when HyperFusion launches (if `use_multiview = true`). Sidecar status is written to the **Multiview** log channel.

Set `use_mock_hardware = false` only when the robot is powered, networked, and ready.

---

## HTTP API (v1)

| Method | Path | Body | Purpose |
|--------|------|------|---------|
| `GET` | `/health` | — | Sidecar status; includes `joint_states_ok` |
| `POST` | `/connect` | `{"ip":"192.168.0.10"}` optional | Connect / start driver |
| `POST` | `/disconnect` | — | Disconnect |
| `GET` | `/pose` | — | TCP pose `[x,y,z,rx,ry,rz]` (flange/`tool0` TF today) |
| `GET` | `/joints` | — | Joint names + positions (rad) |
| `POST` | `/move_l` | `{"pose":[...],"speed":0.05,"wait":true}` | Linear TCP move |
| `POST` | `/move_j` | `{"joints":[...],"speed":1.0,"wait":true}` | Joint-space move |
| `POST` | `/stop` | — | Halt motion |
| `POST` | `/shutdown` | — | Stop sidecar |

Connect waits for live `/joint_states` before reporting success (required for MoveIt and joint moves). Hemisphere scan plan/execute uses additional MoveIt HTTP endpoints from the Windows app.

---

## Layout

```
resources/ur3e/
  package.xml / setup.py
  hyperfusion_ur3e/
    bridge/                        # ROS ↔ HTTP adapter
    driver/                        # ur_control subprocess manager
    moveit/                        # scan planner, pinch guard
    sidecar/                       # HTTP server (ur3e_server)
    urdf/                          # materialize + tool/TCP config helpers
  urdf/
    hyperfusion_ur3e.urdf.xacro    # ceiling mount + payload + hyperfusion_tcp
    hyperfusion_tool_payload.xacro
    meshes/ur_tool_payload.stl
  srdf/hyperfusion_ur.srdf.xacro
  config/
    runtime_robot_description.urdf # rematerialized on sidecar/MoveIt start
  launch/
    hyperfusion_ur_rsp.launch.py
    hyperfusion_moveit.launch.py
    …
  scripts/
    check_robot_network.sh
    wait_for_joint_states.sh
    launch_moveit.sh
  hemisphere-scan.md
  install_env.sh / install_env.ps1
```

Legacy root shims (`ur3e_server.py`, …) may still exist but are **not** used — the app calls `./venv/bin/ur3e_server` from the installed package.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `./install_env.sh` does nothing in PowerShell | Use `.\install_env.ps1` or run the script inside WSL |
| `wsl.exe` not found | Install WSL 2; distro name must match `wsl_distro` |
| `/health` unreachable from Windows | Use `wsl curl` (not bare `curl`); confirm server `--host 0.0.0.0` |
| Robot ping fails from WSL | Mirrored networking or fix lab subnet routing |
| `ros-*-ur` install fails | Use Ubuntu 22.04 (Humble) or 24.04 (Jazzy); re-run `./install_env.sh` |
| Wrong Python / broken venv | `rm -rf venv && ./install_env.sh` |
| Connect slow (~90s–2 min) | Expected on first connect — UR driver launch |
| `/joint_states` missing after connect | In WSL: `ros2 control list_controllers` and `ros2 topic echo /joint_states --once` |
| MoveIt execute fails: joint state time 0 | Restart sidecar (reconnect robot). Sidecar runs `joint_states_stamper`. |
| Tool mesh / TCP wrong in RViz | Restart sidecar after cfg/mesh changes; confirm orange `hyperfusion_tcp` marker |
| Mesh appears rotated 180° in XY | Mesh origins use pan-180; flip `tool_tcp_y_mm` sign if TCP is on the wrong side |
| Scan pin skips after controller reject | Check External Control still Playing; stranded folded poses need pendant recovery / home |
| TF unavailable in simulation | Pose fallback is used; motion may still work via joint trajectory |

---

## Safety

- **No motion on sidecar start** — connect is explicit via `/connect` or the Multiview / UR3e tab.
- Keep **E-stop** on the teach pendant accessible.
- Use conservative `max_linear_speed_m_per_s` / `max_joint_velocity_deg_s` until the rig is validated.
- UR **C403A0** pinch stop (flange vs forearm) cannot be disabled; HyperFusion enforces a matching guard using `tool_payload_radius_mm`.
