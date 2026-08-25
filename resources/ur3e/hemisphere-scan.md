#  Hemisphere Scan — Technical Report

**HyperFusion** | Ceiling-mounted UR3e | MoveIt-backed dome scan over sample tray

---

## 1. Purpose

The hemisphere scan moves the UR3e **optical TCP** (BFS sensor face, `hyperfusion_tcp`) to a grid of poses on a dome above the sample tray. Each pose aims the tool **+Z** inward (toward the sphere center) for multi-angle imaging.

Two execute modes share the same dome geometry and Plan sidecar:

| Mode | Plan | Execute |
| ---- | ---- | ------- |
| **Auto** | Dense pin grid; prefer **home→pin** MoveIt path (`home_path_ok`) | Visit pins top→bottom; MoveIt direct, retreat via home on fail |
| **Semi** | One entry per θ ring (+ apex); **home↔pin** + **base-sweep** gate | Top still → ring entries → hardware `shoulder_pan` spin → MoveIt ring↔ring / home |

The system must:

- Plan which poses are **reachable** (IK + static collision + path gates per mode)
- Execute **collision-aware** MoveIt hops where required
- **Retreat to home** when a direct path fails (Auto pins; Semi ring hops / end)
- **Skip** unreachable pins and continue (Auto); Semi aborts harder on entry/home failures
- Match behavior on **mock hardware** and the **real robot**

---



## 2. System architecture

```
┌─────────────────────────────────────────────────────────────┐
│  HyperFusion (Windows, Qt/C++)                              │
│  • Ur3eHemisphereScanSettingsWidget  — Auto/Semi params     │
│  • Ur3eHemisphereScanPreviewWidget   — 3D dome + pin preview │
│  • Ur3eScanRoutePlanWidget           — saved Auto/Semi routes│
│  • Ur3ePanelController               — orchestration, logging  │
│  • Ur3eSemiFixedScan / Execute       — Semi route + ring spin│
│  • Ur3eClient                        — HTTP via wsl.exe curl │
└──────────────────────────┬──────────────────────────────────┘
                           │ HTTP JSON
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  WSL sidecar (ur3e_server.py)                               │
│  • ros_bridge.py — /plan_hemisphere_scan                    │
│                  /execute_scan_waypoint                       │
│                  /execute_move_home                           │
│                  /execute_hardware_joint_move (pan / wrists)  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  scan_planner.py (MoveIt)                                   │
│  • Plan: IK + home→pin (+ Semi pin→home, base_sweep)        │
│  • Execute: direct → home → approach                        │
│  • /compute_ik, /check_state_validity, /move_action         │
│  • Headless move_group (launch_moveit_headless.sh)          │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  ur_robot_driver + ros2_control (mock or real UR3e)         │
└─────────────────────────────────────────────────────────────┘
```

**Design choice:** Windows talks to ROS/MoveIt through a **WSL sidecar over HTTP**, not direct DDS. Same pattern as GSAM2.

**Saved routes** (beside `app.exe`):

| Directory | Contents |
| --------- | -------- |
| `ur3e_scan_routes/` | Auto Plan JSON |
| `ur3e_semi_scan_routes/` | Semi Plan JSON (re-Plan after planner changes) |
| `ur3e_last_scan_plan.json` | Last Auto plan cache (`remember_last_scan_plan`) |

---



## 3. Geometry and coordinates



### 3.1 Dome grid

Pins lie on a **spherical cap** above the sample tray:

| Parameter         | Meaning                                               |
| ----------------- | ----------------------------------------------------- |
| **θ (theta)**     | Polar angle from dome apex (0°) toward tray rim (90°) |
| **φ (phi)**       | Azimuth around the dome (0°–360°)                     |
| **Sphere radius R** | Distance from sphere center to each pin (UI, mm → m) |

Grid generation (`generateHemisphereScanPoints`):

- θ: linear samples from `thetaMinDeg` → `thetaMaxDeg` (`verticalPoints` rings)
- φ: evenly spaced per ring (`horizontalPoints` per ring)
- Cartesian position on the sphere (relative to sphere center):
  - `x = R sin(θ) cos(φ)`
  - `y = R sin(θ) sin(φ)`
  - `z = R cos(θ)`
- World position: add tray height (see §3.4), then apply mount yaw/pitch/offset for scene alignment

### 3.2 Sample tray vs scan hemisphere (flat rim)

The UR3e scan model uses a **fixed tray** in the MoveIt / preview world frame. This is **not** the same as `[sample_stage_position]` in `hyperfusion.cfg` (FX10e/SWIR **linear rail** positions in mm).

| Reference | World Z | Notes |
| --------- | ------- | ----- |
| Workspace floor / tray **bottom** | **0 mm** | Workspace collision box floor slab |
| Tray **top** + **sphere center** | **20 mm** | Constant `kSampleTrayHeightM` (0.02 m); dome axis XY = tray/base **(0,0)** |
| Flat rim of scan hemisphere (θ = 90°) | **20 mm** | Equator of the scan sphere |
| Dome apex (θ = 0°, over home TCP XY) | **20 mm + R** | Highest pin ring |
| Lowest pin ring (θ = θ max) | **20 mm + R cos(θ max)** | Above rim if θ max < 90° |

**Pin height in world frame** (before mount transform):

```text
tip Z = 20 mm + R × cos(θ)
```

At **θ = 90°** (the flat opening of the hemisphere): `tip Z = 20 mm` — the rim lies in the **same horizontal plane as the tray top** (0 mm vertical gap by design). There is no separate physical “contact” mesh; the rim is a geometric cut plane, like a bowl sitting with its opening on the tray surface.

**Sample height:** The model does not add sample thickness. A tall sample on the tray extends **above** Z = 20 mm into the dome; only the tray plane is modeled.

**Tray footprint (preview / clamp):** 540 × 490 mm centered at the origin (`kSampleTrayLengthM` × `kSampleTrayWidthM`).

### 3.3 Tool payload vs scan sphere vs optical TCP

Three different “tool” concepts:

| Object | Source | Role |
| ------ | ------ | ---- |
| **Scan sphere** | UI sphere radius + grid | Virtual dome; pin poses for planning |
| **Tool payload mesh** | `tool_payload_mesh` STL on `tool0` | MoveIt collision/visual (BFS mount CAD) |
| **Pinch sphere** | `tool_payload_radius_mm` | C403A0 flange↔forearm guard only (not mesh size) |
| **Optical TCP** | `tool_tcp_*_mm` → link `hyperfusion_tcp` | MoveIt scan IK tip (sensor-face centroid) |

Scan pins target **`hyperfusion_tcp`**, not the flange (`tool0`). The CAD mesh may use a **180° pan about Z** so Fusion XY matches `tool0`; TCP offsets in cfg are in **tool0** after that pan (Fusion CAD `(x,y,z)` → tool0 `(−x,−y,z)`).

Example (`hyperfusion.cfg` `[multiview]`):

```ini
tool_payload_shape = mesh
tool_payload_mesh = ur_tool_payload.stl
tool_payload_radius_mm = 50          # pinch sphere only
tool_tcp_x_mm = 0                    # optical TCP in tool0 (mm)
tool_tcp_y_mm = -56.035              # Fusion +56.035 after pan-180
tool_tcp_z_mm = 20
```

Restart the UR3e sidecar after changing mesh/TCP so `runtime_robot_description.urdf` rematerializes. In RViz, `hyperfusion_tcp` is a small orange sphere on the sensor face.

### 3.4 TCP orientation

For each grid point, `tcpPoseForHemispherePoint` sets:

- **Position:** pin on the scan dome, offset by tray height (20 mm), then mount transform (yaw/pitch/offset)
- **Tool +Z:** unit vector **inward** toward the dome center `(0, 0, tray_height)` after mount transform
- **`pin_tcp_tilt_deg`:** tip angle (deg) of tool +Z relative to the **nominal point-at-scan-center** TCP pose — baked into TCP `rx,ry,rz`, **not** a wrist joint. Positive tips toward camera-up; negative toward the tray. **Apex (θ=0)** sits over **home TCP XY** (exact look-down; no tip search, no tilt). **Ring pins** use the base_link-centered dome / look-at at tray XY `(0,0)`. **`pin_pose_tolerance_deg`** tips only in the vertical plane (look-at × camera-up) around that (tilted) axis — no left/right.
- **Rotation:** UR rotation vector (axis-angle) from that frame
- **IK tip link:** `hyperfusion_tcp` (same orientation as `tool0`, translated by `tool_tcp_*_mm`)
- **IK filter:** MoveIt solutions whose tip +Z dot product with the desired inward axis is below 0.95 are rejected (prevents 180° “facing outward” wrist flips)

### 3.5 Workspace boundary and robot mount

Robot mount height and the MoveIt workspace box are **separate** settings in `hyperfusion.cfg` `[multiview]` (alias `[ur3e]`):

```ini
# World Z of robot base / ceiling mount. Tray + hemisphere stay at Z=0 / 20 mm.
ceiling_mount_height_mm = 600

# Collision box: X/Y centered on tray; Z depth extends downward from mount plane.
workspace_boundary_enabled = true
workspace_length_mm = 900
workspace_width_mm = 600
workspace_height_mm = 600
```

| Key | Meaning |
| --- | ------- |
| **`ceiling_mount_height_mm`** | World Z of `base_link` (URDF, RViz, sidecar). Does **not** move the tray or scan pins. |
| **`workspace_height_mm`** | Vertical **depth** of the collision box **below** the mount plane (relative to robot), not mount height. |
| **`workspace_ceiling_clearance_mm`** | Gap between mount plane and collision-box **top** (default 40 mm). |
| **`workspace_length_mm` / `workspace_width_mm`** | Horizontal extent centered on the tray origin. |

**Workspace box in world Z:**

- Mount plane (robot base): `Z = ceiling_mount_height_mm`
- Top face (collision): `Z = ceiling_mount_height_mm − workspace_ceiling_clearance_mm`
- Bottom face: `Z = max(0, ceiling_mount_height_mm − workspace_height_mm)`
- Example: mount 600 mm, clearance 40 mm, depth 560 mm → box from Z = 40 mm to Z = 560 mm

MoveIt adds thin collision slabs on **all six faces** (floor, ceiling, four walls) during plan/execute. The ceiling face is at the mount plane (`Z = ceiling_mount_height_mm`); the robot base bolts at/above that plane.

**Max scan radius clamp:** `maxHemisphereRadiusM` uses horizontal workspace/tray limits and vertical reach `ceiling_mount_height_mm − 20 mm` (tray top to mount), not `workspace_height_mm`.

**Not affected by these keys:** `[sample_stage_position]` (spectrometer rail), hemisphere grid params (UI), or tray/sphere geometry at Z = 0 / 20 mm.

### 3.6 Mount orientation (align RViz with real robot)

The robot base is placed in the URDF via a **world → base_link** transform at `Z = ceiling_mount_height_mm`. Defaults match a ceiling mount (roll = 180°). If RViz/simulation looks rotated or mirrored vs your real install, tune these keys in `hyperfusion.cfg` `[multiview]`:

```ini
mount_roll_deg = 180
mount_pitch_deg = 0
mount_yaw_deg = 0        # try 180 if left/right or forward/back is flipped
mount_offset_x_mm = 0
mount_offset_y_mm = 0    # shift base laterally if bolt pattern is off-center
```

| Symptom | Try |
| -------- | ----- |
| Robot appears rotated 180° around vertical vs real | `mount_yaw_deg = 180` (or add/subtract 180 from current yaw) |
| Left/right (Y) reversed | `mount_yaw_deg = 180` or negate `mount_offset_y_mm` |
| Sim arm reaches opposite corner of tray | adjust `mount_yaw_deg` in 90° steps first |

**Also applied to:** URDF robot base (roll/pitch/yaw + offset at ceiling height), hemisphere **pin preview** (tray, dome, workspace box, pins), and MoveIt scan TCP targets (yaw/pitch/offset — roll is ceiling robot flip only).

Restart the UR3e sidecar and reopen MoveIt/RViz after changing mount height or orientation (URDF is built at launch). Restart the app to reload `hyperfusion.cfg` for the scan-route preview.

### 3.7 Scan home pose

From `hyperfusion.cfg` `[multiview]`:

```ini
home_joints_deg = 90, -180, 145, -55, 90, -90
```

Six joint angles (degrees): `pan, lift, elbow, wrist_1, wrist_2, wrist_3`.

Used for:

- Primary IK seed at plan and execute
- Execute retreat target
- Move to home **before** first pin/ring and **after** scan completes
- Mock-hardware startup pose (`initial_positions.yaml`)

**Important:** Home must pass MoveIt **static collision** checks inside the workspace box, not just match joint numbers. Joint tolerance alone is not enough — see §6.6 and §11.

### 3.7.1 Per-pin wrist photo grid

Configured in the **Scanning** GUI (persisted in app settings), not `hyperfusion.cfg`:

- Enable / step / steps each way  
- Which of `wrist_1` / `wrist_2` / `wrist_3` to permute  
- Live image-count estimate: **center + non-zero wrist offsets** → `(2N)^k + 1` poses per pin (same for Auto and Semi)

At each imaging pose: capture/dwell at the nominal pin, then permute enabled wrists by ±N×step (non-zero offsets only). Wrist deltas and **return to nominal** use **hardware joint moves** (not MoveIt) — MoveIt `direct_only` often fails after multi-turn Semi pan. Failed wrist offsets are **skipped**; failure to return to nominal **aborts** that pin’s sweep.

Progress bars count imaging poses: `basePins × imagesPerPin()` (center + sweeps).

`scan_capture_stabilize_ms` and `scan_camera_up_world_z` remain in `hyperfusion.cfg`.

### 3.8 IK seeds (multi-IK closest-to-home)

IK seeds are generated automatically — there are **no cfg-defined midpoints**. For each pin, the seed set is built from two base poses:

1. **`home_joints_deg`**
2. **Current pin position** — robot joints at plan start; after the first reachable pin, the last reachable pin's solution

For each base pose, every joint is offset individually, **coarse first then fine**:

1. **90°, 180°, 270°**
2. **45°, 135°, 225°, 315°**

Up to **8** unique valid IK solutions are collected, ranked by **joint distance to home**. A candidate is kept if **either**:

1. **home → pin** path + unwrap succeeds (`home_path_ok = true`), or  
2. **previous reachable pin → pin** path + unwrap succeeds (chain-only; `home_path_ok = false`)

Path check tries RRTConnect, then Pilz (~5 s).

**Semi** additionally requires **pin → home** (see §5.6) before accepting an entry; Auto leaves chain-only pins reachable for execute-via-home recovery.

---



## 4. Configuration summary


| Setting             | Source            | Role                                      |
| ------------------- | ----------------- | ----------------------------------------- |
| Mode Auto / Semi    | UI                | Dense pin visit vs ring-entry + pan spin  |
| Grid size (H × V)   | UI                | Pin count (Auto); Semi uses V rings + interval |
| Sphere radius       | UI                | Scan dome size (pin placement)            |
| θ range             | UI                | Which part of hemisphere                  |
| Semi pan interval / dir | UI            | `shoulder_pan` step around each ring      |
| `ceiling_mount_height_mm` | `hyperfusion.cfg` | Robot mount Z in world (URDF)       |
| Workspace box       | `hyperfusion.cfg` | MoveIt collision limits (depth below mount) |
| `tool_payload_shape` / `tool_payload_mesh` | `hyperfusion.cfg` | CAD collision mesh on `tool0` |
| `tool_payload_radius_mm` | `hyperfusion.cfg` | Pinch-guard sphere only (C403A0) |
| `tool_tcp_*_mm`     | `hyperfusion.cfg` | Optical TCP offset in tool0 → `hyperfusion_tcp` |
| Mount RPY / offset  | `hyperfusion.cfg` | URDF base pose (RViz vs real alignment)   |
| Home joints         | `hyperfusion.cfg` | Primary IK seed + retreat pose            |
| `pin_tcp_tilt_deg`  | `hyperfusion.cfg` | Tip angle of tool +Z from look-at-center TCP pose (+up/−down); not a wrist joint |
| `pin_pose_tolerance_deg` | `hyperfusion.cfg` | Vertical-plane tip half-angle around (tilted) look-at (+/− camera-up; no left/right) |
| IK seeds            | auto              | home + current pose, 45° joint permutations |
| `use_mock_hardware` | `hyperfusion.cfg` | Mock vs real robot                        |
| `max_joint_velocity_deg_s` | `hyperfusion.cfg` | Execute trajectory peak joint speed cap |

**Separate from UR3e scan:** `[sample_stage_position]` — linear spectrometer stage (mm along rail), not tray Z or dome geometry.


---



## 5. Plan phase

**Trigger:** User presses **Plan** in the Scanning panel (Auto or Semi).

### 5.1 App (C++)

1. Read scan params from `Ur3eHemisphereScanSettingsWidget`
2. Build grid locally: `generateHemisphereScanPoints`
3. Compute TCP pose per pin: `tcpPoseForHemispherePoint`
4. POST all poses to sidecar: `/plan_hemisphere_scan`
  - Includes `workspace` (`length_m`, `width_m`, `height_m`, `mount_height_m`) and `home_joints_deg`
  - Semi sets `semi_ring_sweep: true` (and max sweep-OK pins per ring)
5. Requires robot **connected** (`initial_seed` = current joints — used as the first pin’s “current pin” seed until a reachable pin is found)
6. Semi: build route via `semiFixedRouteFromHemispherePlan` → save under `ur3e_semi_scan_routes/`



### 5.2 Sidecar — home-centric IK (`plan_poses`)

For **each pin independently** (no route chaining for motion, but IK seeds can chain):

```
For each pin:
  1. Build IK seeds: (home + current pin) × 45° per-joint permutations
  2. For each seed → validate seed, then /compute_ik (collision-aware)
  3. Normalize solution to home branch
  4. /check_state_validity (static collision + limits + pinch guard)
  5. Prefer IK with home→pin MoveIt path; else previous→pin (chain-only)
  6. Store joints, home_path_ok, error reason
  7. If reachable, use this pin’s joints as “current pin” seed for the next pin
```

**Auto:** a green pin with `home_path_ok` has a proven **home→pin** path. Chain-only greens may still execute via retreat/approach.

**Semi:** see §5.6 — entries must prove **both** directions and (rings) base-sweep.

### 5.3 Multi-seed IK (auto-generated per pin)


| #   | Seed source |
| --- | ----------- |
| Base 1 | Home (`home_joints_deg`) |
| Base 2 | **Current pin position** — robot joints at plan start for early pins; after the first reachable pin, the **last reachable pin’s joint solution** |
| Permutations | Each base pose + one joint offset by 45°/90°/135°/180°/225°/270°/315° (one joint at a time) |


Seeds that fail static collision (`/check_state_validity`) are **skipped** before calling `/compute_ik`. The best valid solution **closest to home** (plan) or **current arm pose** (execute) is kept.

**Note:** cfg-defined IK midpoints (`ik_seed_midpoints_deg`) were removed — seeds are now only home + current pose with 45° per-joint permutations.

### 5.4 Joint branch normalization

UR joints have multiple valid representations (e.g. 209° vs −151°). The planner:

- Wraps angles to UR3e limits
- Picks the branch **nearest** the reference (home at plan time)
- Avoids branch drift that broke execute when pins were chained



### 5.5 Plan output


| Result          | Preview color | Meaning                                           |
| --------------- | ------------- | ------------------------------------------------- |
| **Reachable + home_path_ok** | Green | Valid IK + static OK + home→pin path (Auto); Semi = **home↔pin** |
| **Reachable, chain-only** | Orange | IK OK but only previous→pin (Auto); Semi **rejects** these |
| **Unreachable** | Blue        | No IK, collision, invalid joints, or Semi gate fail |
| **Plan joints** | —             | 6 stored joint angles (rad) per reachable pin     |

**Remember last plan** (`remember_last_scan_plan` in `hyperfusion.cfg`, default on): after a successful **Auto** Plan, results are written to `ur3e_last_scan_plan.json` beside the app. On the next startup the plan is reloaded automatically only if the current scan UI params and robot geometry keys in `hyperfusion.cfg` `[multiview]` match the fingerprint stored with the cache (TCP, payload, mount, workspace, home, `ur_type`, mock flag). Changing grid/radius/θ or those cfg keys invalidates the cache until you Plan again.

### 5.6 Semi Plan gates (`semi_ring_sweep`)

When Semi Plan runs, `scan_planner.plan_poses(..., semi_ring_sweep=True)`:

1. Same IK pick as Auto (prefer **home→pin**).
2. **Reject** chain-only candidates (no home→pin).
3. Require **pin→home** MoveIt path (`_start_to_goal_path_ok`, label `pin→home`). Fail → log `semi pin→home path failed`, pin not accepted.
4. Ring pins (not apex): require `_base_sweep_ok` — full `shoulder_pan` turn at fixed other joints passes static validity / pinch samples.
5. Apex (`require_perpendicular`): over **home TCP XY**, exact look-down; **no** tip cone; **no** base-sweep; still needs **home↔pin**. Rings stay on base XY `(0,0)`.
6. Cap how many base-sweep-OK pins are kept per θ ring (`semi_max_sweep_ok_per_ring`).
7. **`semi_ring_search_candidates`** (cfg, default **360**): number of φ candidates per θ ring, spaced evenly over 360° (e.g. `260` → ~1.4° steps). Legacy alias: `semi_ring_search_buffer_deg`.

`home_path_ok = true` on Semi results means **both** home→pin and pin→home succeeded.

**Route build** (`semiFixedRouteFromHemispherePlan`):

- One ring entry per θ: requires `base_sweep_ok` **and** `homePathOk`; prefer nearest to home.
- Top/apex: only if `homePathOk`; if none, route has **no** invented top (`hasTopPose = false`).
- Legacy plans without `base_sweep_ok` fall back to reachable + `homePathOk` only.

After changing Semi planner Python: **restart sidecar**, then **re-Plan Semi** and save/select the new route. Old JSON may still load but is not guaranteed return-home-checked.

---



## 6. Execute phase

**Trigger:** User presses **Execute** (after successful Plan).

Sections §6.1–§6.6 describe **Auto**. Semi is §6.7.

### 6.1 Visit order (top ring first, home-nearest entry)

Rings are visited **top → bottom** (θ from apex toward tray rim):

1. **Top reachable ring** — start at the pin on that ring **closest to home** (joint-space distance).
2. **Sweep that ring** in φ order (continue around the same θ layer).
3. **Next ring downward** — enter at the pin **closest to where the arm just finished**, then sweep φ.
4. Repeat until all reachable rings are done.

Only **reachable** pins are visited. Built by `buildHemisphereScanExecutionOrder`.

### 6.2 Execute sequence (full)

```
Execute pressed
    │
    ▼
① Verify / move to HOME (MoveIt, collision-aware)
    │  fail → homing dialog (Retry / Start scan anyway / Abort)
    ▼
② For each pin (top-ring-first order):
    │
    ├─ Live IK refresh (seeds: home + current pose, 45° permutations)
    │     success → use refreshed joints
    │     fail    → fall back to Plan joints
    │
    ├─ Validate start state
    │
    ├─ Validate full goal state
    │
    ├─ Try DIRECT: current → pin (MoveIt)
    │     plan with OMPL + Pilz (plan-only), reject pinch-unsafe paths
    │     pick **shortest joint-space travel**, then execute that trajectory
    │     success → dwell 2 s → next pin
    │     fail    → retreat via home
    │
    ├─ RETREAT: current → HOME (MoveIt)
    │     fail → skip pin, continue
    │
    ├─ Settle at home, then resolve goal (Plan joints first, live IK fallback)
    │
    └─ APPROACH: home → pin (MoveIt)
          success → dwell 2 s → next pin
          fail    → skip pin, continue
    │
    ▼
③ Return to HOME (MoveIt)
    │  fail → log warning, still finish
    ▼
Scan complete (summary: executed / skipped)
```

### 6.3 Bookend homing


| When                 | Behavior                                                         |
| -------------------- | ---------------------------------------------------------------- |
| **Before first pin** | MoveIt verify/move to home; dialog on failure                   |
| **After last pin**   | Always return to home (unless fatal mid-scan error)              |
| **Mid-scan retreat** | Only when direct path fails                                     |
| **After connect**    | MoveIt verify/move to home; dialog on failure                    |
| **Before disconnect**| MoveIt verify/move to home; dialog on failure                    |
| **Before app close** | MoveIt verify/move to home if robot connected; Cancel keeps app open |


### 6.4 Per-pin timing

- **Settle** at each pose from `scan_capture_stabilize_ms` in `[multiview]` (default **500 ms**), for both motion-only Execute and BFS still capture
- **Wrist sweep** (Scanning GUI): after the nominal pin, permute selected wrists by ±steps×step → **(2N)^k+1** poses per pin; wrist hops are **hardware** joint moves; failed offsets skipped; return-to-nominal abort on failure
- **UR3e Execute + BFS connected:** folder dialog → `multiview_yyyyMMdd_HHmmss/` → settle + BFS TIFF + `transforms.json` (same still pipeline as Capture Record Multiview); works motion-only when no capture folder
- Joint poll ~**100 ms** during motion (async, non-blocking UI)
- MoveIt motion timeout up to **120 s** per leg

### 6.4.1 Capture tab — Multiview RGB record

When **Multiview RGB** is enabled on the Capture tab (BFS + UR3e connected + plan ready + stage connected):

| Button | Behavior |
| ------ | -------- |
| **Preview** | Same as UR3e **Execute** (motion only; no photos; wrist sweep still runs) |
| **Record** | If HSI modes/cameras selected → existing HSI Record first; then stage → `sample_multiview_position_mm` (default **1600**), hemisphere scan with cfg settle + wrist grid + BFS TIFF per pose |

Stills land in `{dataset}/multiview/00000.tif` + matching `00000.json` (per-image optical TCP pose) plus aggregate `transforms.json` (OpenGL `camera_to_world`). Multiview-only Record (no HSI) creates the dataset folder and writes only `multiview/`.



### 6.5 Skip behavior

If a pin cannot be reached (no path even via home):

- Log: `skipped — <reason>`
- **Continue** to next pin
- Summary at end: `N pin(s) skipped`

Scan is **best-effort**, not all-or-nothing.

### 6.6 Homing verification and dialog

After connect, before scan execute, before disconnect, and before app close, the app calls `/execute_move_home` which:

1. Applies workspace collision geometry
2. If joints match `home_joints_deg` within tolerance → runs MoveIt **`/check_state_validity`** on the current pose
3. If not at home → **direct** MoveIt path to home
4. If direct fails → homing dialog (invalid home, no path, collision)

If homing fails after the direct attempt:

- A **warning dialog** appears: **UR3e Homing Failed**
- Shows the MoveIt reason and configured home joints
- Joint controls are enabled while the dialog is open (jog manually, then **Retry**)

Context-specific buttons:

| Context         | Retry | Secondary              | Cancel        |
| --------------- | ----- | ---------------------- | ------------- |
| After connect   | ✓     | Continue without homing | Disconnect    |
| Before scan     | ✓     | Start scan anyway       | Abort scan    |
| Before disconnect | ✓   | Disconnect anyway       | Cancel        |
| Before close    | ✓     | Close anyway            | Cancel (stay open) |


### 6.7 Semi execute (`Ur3eSemiFixedScanExecute`)

Semi does **not** visit every Auto pin. It uses one **entry** joint set per θ ring (+ optional top), then spins `shoulder_pan` in hardware.

```
Execute pressed
    │
    ▼
① Verify / move to HOME (same homing dialog as Auto)
    │
    ▼
② MoveIt → top (θ=0) still + optional wrist sweep (hardware)
    │
    ▼
③ For each ring (top → bottom):
    │
    ├─ Always: unwind to previous entry (if any) → MoveIt home
    │     then MoveIt → **plan** ring entry joints (no live IK / cone)
    │
    ├─ Principalize shoulder_pan onto (−π, π] branch; capture live entryBranch
    │
    ├─ Hardware pan steps (±360° / interval) with still + wrist sweep per sample
    │     wrist offsets + return-to-nominal = hardware (not MoveIt)
    │
    ├─ Hardware return to entryBranch
    │
    └─ Principalize before next ring (then home again before next entry)
    │
    ▼
④ retreatToScanHome:
    │  • Hardware unwind to last entryBranch — REQUIRED (abort home if fail)
    │  • Verify live joints ≈ entry (wrap-aware, ~0.08 rad)
    │  • MoveIt /execute_move_home from that validated entry
    │  • On MoveIt fail → joint-waypoint MoveIt retry (still no hardware interpolate-to-home)
    ▼
Scan complete
```

| Aspect | Behavior |
| ------ | -------- |
| **Ring→ring** | Always **via home**, then plan entry joints (no direct hop) |
| **Entry goal** | Plan `entryJointsRad` only — no live IK / pin-pose cone |
| **Pan / wrists** | Hardware joint moves; init sweep picks entry±2π so edge/samples stay in shoulder_pan **±360°** |
| **Home** | Always **MoveIt** from plan-validated **entry** joints, not wound post-pan state |
| **wrist_3 unwind** | Not used between rings; entry unwind + principalize handle branch |

---



## 7. Live IK vs Plan joints


|                  | **Plan joints**        | **Live IK (execute)**                                      |
| ---------------- | ---------------------- | ---------------------------------------------------------- |
| **When**         | Plan time              | Each pin before motion                                     |
| **Seeds**        | Home + current pin (45° perms) | Home + **current robot joints** (45° perms)        |
| **Pick nearest** | Home branch            | **Current arm** pose                                       |
| **On failure**   | Pin marked unreachable | **Fall back to Plan joints**                               |
| **Motion?**      | No                     | No — only sets goal; MoveIt moves after                    |


**After retreat to home:** live IK runs again from home; if that fails, Plan joints are used for the home→pin leg.

### 7.1 When does the robot go home?


| Situation            | Goes home?                            |
| -------------------- | ------------------------------------- |
| Plan                 | No — home is reference only           |
| Execute start        | Yes — before first pin / Semi top     |
| Execute end          | Yes — after last pin / Semi last ring |
| Direct move succeeds | No — stays at pin (settle / still)    |
| Direct move fails    | Yes — retreat, then approach (Auto); Semi via-home then retry entry |
| Skipped pin (Auto)   | No — stays at current pose, continues |
| Semi home abort      | If entry unwind/verify fails — no MoveIt home from wound pose |


---



## 8. Motion planning (MoveIt execute)



### 8.1 Planner order (per leg)

1. OMPL **RRTConnect**
2. OMPL **RRTstar**
3. Pilz **PTP**

**First valid trajectory wins** (no collect-all / “easiest path” pick). Direct hops use a short planning budget (~5 s × 5 attempts); via-home recovery uses a larger budget (~12 s). Live multi-seed IK is skipped when planned joints are still valid; one via-home attempt then skip.

MoveIt only — no direct `joint_trajectory_controller` fallback.

### 8.2 Speed scaling

MoveIt `max_velocity_scaling_factor` from joint travel (before planning):

| Travel (rad sum) | Scale |
| ---------------- | ----- |
| > 5.0            | 0.10  |
| > 3.0            | 0.18  |
| > 1.5            | 0.28  |
| else             | 0.40  |

After planning, each candidate trajectory is checked against the **UR3e external-control joint velocity limit of 190 deg/s** (peak across all joints). If a path exceeds that, HyperFusion **slows the trajectory timing** before execute. ±π wrist unwrap runs first so velocity is measured on continuous joint motion.

### 8.3 Common MoveIt error codes


| Code    | Name                | Typical cause                   |
| ------- | ------------------- | ------------------------------- |
| **-2**  | INVALID_MOTION_PLAN | No collision-free path          |
| **-4**  | CONTROL_FAILED      | Controller rejected trajectory  |
| **-26** | START_STATE_INVALID | Start joints invalid for MoveIt |


### 8.4 Manual joint Move (UR3e panel)

The **Move** button calls `/execute_scan_waypoint` with **`direct_only: true`**:

- **One leg:** current pose → slider target (MoveIt, seeded from live joints)
- Same trajectory prep as scan execute (multi-planner, ±π unwrap, joint velocity cap)
- **No** home fallback — MoveIt error is shown in the log


---



## 9. UI and visualization



### 9.1 Settings panel (Multiview)

- Mode: **Auto** vs **Semi**
- Sphere radius, grid (H × V), θ range; Semi: pan interval / direction
- Wrist sweep controls; live pose-count estimate
- **Plan** / **Execute**; saved route list (`ur3e_scan_routes` / `ur3e_semi_scan_routes`)



### 9.2 3D preview (`Ur3eHemisphereScanPreviewWidget`)

- Dome, tray, workspace boundary, pin normals
- **Plan view:** green = reachable + `home_path_ok`, orange = chain-only, blue = unreachable
- **Execute view:**

| Color   | Meaning                                      |
| ------- | -------------------------------------------- |
| Green   | Completed                                    |
| Yellow  | Pending (reachable, not yet visited)         |
| Red     | Execute failed / skipped (was reachable at plan) |
| Blue    | Unreachable at plan                          |
| Purple  | Current active pin (pulse)                   |

- Orbit camera (mouse drag / wheel)



### 9.3 Logging

Execute logs include:

- Step N/M, point index, TCP, target joints
- Arrived pose and joints
- Skip reasons
- Home before/after messages



### 9.4 UI performance

Joint polling during execute uses **async** `wsl.exe curl` on a background thread so the 3D view stays responsive.

---



## 10. Key design decisions


| Decision                         | Rationale                                                             |
| -------------------------------- | --------------------------------------------------------------------- |
| **Optical TCP at sensor**        | Scan poses aim BFS face (`hyperfusion_tcp`), not flange               |
| **Home-centric plan**            | Every pin judged on same branch as retreat; fixes chained-branch bugs |
| **Plan-time home→pin (Auto)**    | Prefer pins with proven approach from home; fewer execute surprises   |
| **Plan-time home↔pin (Semi)**    | Ring entries must return home via MoveIt after multi-turn pan         |
| **Semi base-sweep gate**         | Entry must allow full `shoulder_pan` circle without static collision  |
| **No route chaining at plan**    | Avoids −304° pan drift from pin-to-pin seeds                          |
| **Direct first, home fallback**  | Fast direct path; retreat via home if direct fails                   |
| **Semi home from entry only**    | Unwind+verify before MoveIt home — not from wound post-pan joints     |
| **Hardware pan / wrist deltas**  | MoveIt often fails after continuous multi-turn RTDE branch            |
| **Easiest trajectory wins**      | Plan-only all pipelines; pick shortest joint travel + pinch-safe path |
| **Skip on failure (Auto)**       | Partial scans better than aborting entire route                       |
| **Static plan + path gates**     | Plan validates hold pose and selected MoveIt legs                     |
| **Configurable home in cfg**     | Match real robot home; same for mock and field                        |
| **Auto IK seeds (home + current, 45° perms)** | Simple, dense seed set; no cfg midpoints to maintain    |
| **MoveIt home validity**         | Joint match alone is insufficient — home must pass collision check    |
| **Homing dialog**                | Manual jog + retry when home invalid or unreachable                   |
| **Home-nearest ring sweep**      | Short, simple trajectories; fewer C403A0 pinch stops                  |


---



## 11. What Plan does *not* guarantee

### Auto

A **green** pin (`home_path_ok`) means:

> There exists a collision-free **static pose** and a MoveIt **home→pin** path at plan time.

It does **not** guarantee:

> There is a collision-free **path from every mid-scan pose** the arm may occupy (e.g. after a skipped pin or live IK change).

Orange (chain-only) pins may need **retreat via home** at execute. Some greens may still **skip** if the live start state differs from plan.

### Semi

An accepted ring/top entry means:

> **home↔pin** MoveIt paths and (rings) **base-sweep** static samples succeeded at plan time.

It does **not** guarantee:

> Direct **ring→ring** MoveIt always succeeds (via-home fallback remains), or that hardware pan never hits path-tolerance / protective stops mid-spin.

Execute still **must** unwind to the validated entry before MoveIt home — plan does not check post-pan continuous joints.

### 11.1 Home pose must be MoveIt-valid

A common failure mode: `home_joints_deg` matches the mock/hardware joint readout, but the arm **collides** with the workspace box or itself in MoveIt. Symptoms:

- Connect / pre-scan reports “verified at home” (old behavior) or **Homing Failed** dialog (current)
- Plan succeeds (goal poses valid from home **seed**)
- Execute skips every pin with **`start state invalid: collision or joint limit violation`**

**Fix:** Choose a home pose that passes collision checks inside the workspace boundary, or adjust the workspace box.

### 11.2 UR protective stop C403A0 (flange vs lower arm)

On **e-Series** UR robots (including UR3e), the controller raises **C403A0** when the **tool flange** comes within about **28 mm** of the **lower arm** (forearm). This is a **firmware safety feature** for finger pinch protection — it **cannot be disabled**.

MoveIt’s default collision model does not include this keep-out zone, so a pose can look “green” in Plan and still trip a protective stop during Execute when the arm folds tightly.

**Prevention (in order):**

1. **Re-plan after sidecar restart** — HyperFusion rejects static poses that violate the UR pinch model (sphere + cylinder approximation) during `/check_state_validity`. Bad pins show as **blue / unreachable** instead of failing on hardware.
2. **Tighten the scan dome** — reduce `theta_max`, `sphere_radius`, or grid size so pins stay away from elbow-fold configurations (common on ceiling mounts).
3. **Adjust `home_joints_deg`** — use a less folded home pose (open the wrist/elbow away from the forearm).
4. **On the pendant** — after C403A0, tap **Enable Robot** (usually no recovery mode; arm was paused, not faulted).

If stops still occur **mid-motion** (not at the goal pose), the path is crossing the pinch zone between waypoints — shrink the dome further or pick a home pose that lets OMPL route around the elbow.

### 11.3 External Control joint velocity limit (±π wrap)

If the pendant shows **External Control speed limit** on **joint 5** (wrist_3) or another joint, consecutive trajectory samples were too close in time (often **0.002 s** at the 500 Hz External Control rate) or had a branch jump. HyperFusion **unwraps** trajectories on the hardware joint branch, snaps the first waypoint to live feedback, strips velocity/acceleration fields, enforces a minimum **20 ms** between samples, and caps peak joint speed via `max_joint_velocity_deg_s` in `hyperfusion.cfg` (default in tree often **40–60 deg/s**; UR hardware allows up to **190**). Lower the cap if warnings persist; restart the sidecar after changing config.

---



## 12. Operational checklist

1. Start sidecar / connect UR3e (Multiview tab)
2. Set `ceiling_mount_height_mm`, workspace box, `tool_tcp_*_mm`, and payload mesh in `hyperfusion.cfg`
3. Set `home_joints_deg` to match robot (6 values; must be collision-free and pinch-safe in workspace)
4. **Connect** — confirm homing succeeds (or fix pose via dialog)
5. Choose **Auto** or **Semi** — **Plan** — review green / orange / blue
6. **Semi:** save/select route under `ur3e_semi_scan_routes`; after planner changes, **re-Plan** (do not rely on old JSON)
7. **Execute** — home → pins/rings → home
8. After **Python** changes → restart sidecar
9. After **C++** changes → rebuild app
10. Avoid running two `move_group` instances without restart
11. Avoid RViz **Plan & Execute** while HyperFusion scan plan is running (shared `move_group`)

---



## 13. Glossary


| Term             | Definition                                                     |
| ---------------- | -------------------------------------------------------------- |
| **Pin**          | One dome grid point + TCP pose + (if reachable) joint solution |
| **Tray**         | Sample surface plane at Z = 20 mm; bottom at Z = 0             |
| **Flat rim**     | Scan hemisphere equator (θ = 90°); coplanar with tray top      |
| **Scan sphere**  | Virtual dome for pin placement (UI radius R)                   |
| **Payload mesh** | CAD collision on `tool0` (`tool_payload_mesh`)                 |
| **Pinch sphere** | C403A0 guard radius (`tool_payload_radius_mm`)                 |
| **Optical TCP**  | Sensor-face tip `hyperfusion_tcp` (`tool_tcp_*_mm`)            |
| **Ring**         | All pins at the same θ (elevation / height layer)              |
| **Auto**         | Dense pin visit; MoveIt between pins                           |
| **Semi**         | One entry per ring + hardware `shoulder_pan` spin              |
| **Entry branch** | Live continuous joints after MoveIt arrives at a Semi ring entry |
| **home_path_ok** | Auto: home→pin OK; Semi: home↔pin OK                           |
| **base_sweep_ok**| Semi: full pan circle at entry joints is statically valid      |
| **Plan joints**  | Joint angles stored at Plan for a pin                          |
| **Home**         | Configured safe reference pose; retreat and bookends               |
| **IK seed**      | home or current pose (+ 45° per-joint permutations) used for IK    |
| **Current pin**  | Robot joints at plan start, or last reachable pin’s plan solution  |
| **Live IK**      | Re-solve IK at execute from current (or home) pose             |
| **Direct leg**   | MoveIt path: current → pin / ring entry                        |
| **Retreat leg**  | MoveIt path: current → home (Semi: after required entry unwind) |
| **Approach leg** | MoveIt path: home → pin                                        |
| **Skip**         | Pin unreachable; scan continues (Auto)                         |


---



## 14. File reference


| Layer          | Main files                                                                          |
| -------------- | ----------------------------------------------------------------------------------- |
| Config         | `app/hyperfusion.cfg` `[multiview]`, `HyperFusionConfig.cpp`                      |
| Grid / TCP     | `Ur3eHemisphereScan.cpp`, `Ur3eHemisphereScanReachability.cpp`                      |
| Semi route     | `Ur3eSemiFixedScan.cpp`, `Ur3eSemiFixedScan.hpp`                                    |
| Semi execute   | `Ur3eSemiFixedScanExecute.cpp`                                                      |
| UI             | `Ur3eHemisphereScanSettingsWidget.cpp`, `Ur3eHemisphereScanPreviewWidget.cpp`, `Ur3eScanRoutePlanWidget.cpp` |
| Orchestration  | `Ur3ePanelController.cpp` (Auto execute + Semi host)                        |
| HTTP client    | `Ur3eClient.cpp`                                                                    |
| Sidecar        | `hyperfusion_ur3e/bridge/ros_bridge.py`, `hyperfusion_ur3e/sidecar/http_handler.py` |
| MoveIt planner | `hyperfusion_ur3e/moveit/scan_planner.py`                                   |
| URDF / TCP     | `urdf/hyperfusion_ur3e.urdf.xacro`, `urdf/hyperfusion_tool_payload.xacro`           |
| Tool/TCP env   | `hyperfusion_ur3e/urdf/tool_payload_config.py`, `materialize_robot_description.py`  |
| This report    | `resources/ur3e/hemisphere-scan.md`                                                 |


---



## 15. HTTP API (sidecar)


| Endpoint                      | Purpose                                |
| ----------------------------- | -------------------------------------- |
| `POST /plan_hemisphere_scan`  | IK + validity + path gates for all poses |
| `POST /execute_scan_waypoint` | MoveIt plan+execute one pin            |
| `POST /execute_move_home`     | MoveIt plan+execute to configured home |
| `POST /execute_hardware_joint_move` | Exact joint move (Semi pan / wrists; Auto wrists) |


Request bodies include `workspace` (`length_m`, `width_m`, `height_m`, `mount_height_m`) and `home_joints_deg` from `hyperfusion.cfg`.

`/plan_hemisphere_scan` Semi flags:

| Field | Meaning |
| ----- | ------- |
| `semi_ring_sweep` | Enable home↔pin + base-sweep Semi gates |
| `semi_max_sweep_ok_per_ring` | Cap accepted base-sweep-OK pins per θ |

`/execute_scan_waypoint` flags:

| Field | Meaning |
| ----- | ------- |
| `direct_only` | Manual Move: no home retreat |
| `require_home_first` | Skip direct; home then approach (reserved) |
| `tcp` | Pin TCP target (for live IK refresh) |
| `joints` | 6 plan (or refreshed) joint angles (rad) |
