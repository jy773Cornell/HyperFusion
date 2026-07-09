# UR3e WSL sidecar (ROS 2) for HyperFusion

HyperFusion controls a **Universal Robots UR3e** via a **WSL sidecar**: the Windows app starts `ur3e_server` in Ubuntu through `wsl.exe`, mirroring the GSAM2 pattern. The sidecar always uses **ROS 2 / ur_robot_driver**. Set `use_mock_hardware = true` for simulation (no physical robot) or `false` for the real arm.

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

This writes `%USERPROFILE%\.wslconfig` (`networkingMode=mirrored`) and adds inbound firewall rules for TCP **50001–50004**. Or use `.\install_env.ps1 -SetupRobotNetwork -ShutdownWsl`.

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

MoveIt + RViz runs in a **separate WSL window** — not inside the HyperFusion sidecar. Use it for interactive planning; the app uses the HTTP sidecar for scripted moves.

**Requirements:** robot connected in HyperFusion (or sidecar running with mock hardware) and live **`/joint_states`**.

From WSL:

```bash
cd /mnt/d/Pototypy/HyperFusion/resources/ur3e
./scripts/launch_moveit.sh jazzy ur3e    # ros_distro, ur_type
```

Or from the HyperFusion **UR3e** tab: **Start MoveIt** / **Stop MoveIt**. Closing the RViz window stops MoveIt.

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

## Robot network check

Before connecting to hardware (`use_mock_hardware = false`):

```bash
./scripts/check_robot_network.sh 192.168.0.10
```

---

## HyperFusion config

In `app/hyperfusion.cfg` → `[ur3e]`:

```ini
wsl_distro = Ubuntu
ur3e_repo_linux =                          ; empty = auto (/mnt/d/.../resources/ur3e)
server_port = 8766
robot_ip = 192.168.0.10
dashboard_port = 29999
rtde_port = 30004
use_mock_hardware = true
prestart_driver = false
ros_distro = jazzy
ur_type = ur3e
max_linear_speed_m_per_s = 0.05
max_linear_accel_m_per_s2 = 0.3
motion_type = move_j                       ; move_j | move_l (UR3e tab Move button)
```

| Key | Notes |
|-----|--------|
| `use_mock_hardware` | `true` = UR driver fake hardware; `false` = real robot |
| `prestart_driver` | `true` warms ROS driver at app launch (~2 min); keep `false` on real hardware until validated |
| `motion_type` | `move_j` = joint targets from UR3e tab sliders; `move_l` = linear TCP move using current `/pose` |

The Windows app launches:

```bash
cd <ur3e_repo_linux> && source /opt/ros/<ros_distro>/setup.bash && ./venv/bin/ur3e_server ...
```

The WSL sidecar starts automatically when HyperFusion launches. Sidecar status is written to the **UR3e** log tab only.

Set `use_mock_hardware = false` only when the robot is powered, networked, and ready.

---

## HTTP API (v1)

| Method | Path | Body | Purpose |
|--------|------|------|---------|
| `GET` | `/health` | — | Sidecar status; includes `joint_states_ok` |
| `POST` | `/connect` | `{"ip":"192.168.0.10"}` optional | Connect / start driver |
| `POST` | `/disconnect` | — | Disconnect |
| `GET` | `/pose` | — | TCP pose `[x,y,z,rx,ry,rz]` |
| `GET` | `/joints` | — | Joint names + positions (rad) |
| `POST` | `/move_l` | `{"pose":[...],"speed":0.05,"wait":true}` | Linear TCP move |
| `POST` | `/move_j` | `{"joints":[...],"speed":1.0,"wait":true}` | Joint-space move |
| `POST` | `/stop` | — | Halt motion |
| `POST` | `/shutdown` | — | Stop sidecar |

Connect waits for live `/joint_states` before reporting success (required for MoveIt and joint moves).

---

## Layout

```
resources/ur3e/                    # ROS 2 ament_python package
  package.xml
  setup.py
  hyperfusion_ur3e/
    bridge/                        # ROS ↔ HTTP adapter (Ur3eRosBridge)
    driver/                        # ur_control subprocess manager
    sidecar/                       # HTTP server (ur3e_server entry point)
    nodes/                         # future rclpy nodes
  launch/
    sidecar.launch.py
    moveit.launch.py
    ur3e_bringup.launch.py
  config/ur3e_sidecar.yaml
  scripts/
    check_robot_network.sh
    wait_for_joint_states.sh
    launch_moveit.sh
  install_env.sh                   # WSL bash installer
  install_env.ps1                  # PowerShell → WSL wrapper
```

Legacy root shims (`ur3e_server.py`, `ur3e_ros_bridge.py`, `ur3e_ros_driver.py`) may still exist but are **not** used — the app calls `./venv/bin/ur3e_server` from the installed package.

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
| Connect slow (~90s) | Expected on first connect — UR driver launch |
| `/joint_states` missing after connect | In WSL: `ros2 control list_controllers` and `ros2 topic echo /joint_states --once` |
| MoveIt execute fails: joint state time 0 | Restart sidecar (reconnect robot). Sidecar runs `joint_states_stamper` which republishes `/joint_states` with valid timestamps. Confirm: `ros2 topic echo /joint_states --once` shows non-zero stamp. |
| TF unavailable in simulation | Pose fallback is used; motion may still work via joint trajectory |

---

## Safety

- **No motion on sidecar start** — connect is explicit via `/connect` or the UR3e tab.
- Keep **E-stop** on the teach pendant accessible.
- Use conservative `max_linear_speed_m_per_s` until the rig is validated.
