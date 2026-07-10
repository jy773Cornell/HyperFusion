#  Hemisphere Scan — Technical Report

**HyperFusion** | Ceiling-mounted UR3e | MoveIt-backed dome scan over sample tray

---

## 1. Purpose

The hemisphere scan moves the UR3e TCP to a grid of poses on a dome above the sample tray. Each pose aims the tool inward (toward the sphere center) for multi-angle imaging.

The system must:

- Plan which poses are **reachable** (IK + static collision)
- Execute **collision-aware** motion between pins
- **Retreat to home** when a direct path fails
- **Skip** unreachable pins and continue
- Match behavior on **mock hardware** and the **real robot**

---



## 2. System architecture

```
┌─────────────────────────────────────────────────────────────┐
│  HyperFusion (Windows, Qt/C++)                              │
│  • Ur3eHemisphereScanSettingsWidget  — grid params, Plan/Execute │
│  • Ur3eHemisphereScanPreviewWidget   — 3D dome + pin preview │
│  • Ur3ePanelController               — orchestration, logging  │
│  • Ur3eClient                        — HTTP via wsl.exe curl │
└──────────────────────────┬──────────────────────────────────┘
                           │ HTTP JSON
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  WSL sidecar (ur3e_server.py)                               │
│  • ros_bridge.py — /plan_hemisphere_scan                    │
│                  /execute_scan_waypoint                       │
│                  /execute_move_home                           │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│  scan_planner.py (MoveIt)                                   │
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

---



## 3. Geometry and coordinates



### 3.1 Dome grid

Pins lie on a **spherical cap** above the tray:


| Parameter         | Meaning                                               |
| ----------------- | ----------------------------------------------------- |
| **θ (theta)**     | Polar angle from dome apex (0°) toward tray rim (90°) |
| **φ (phi)**       | Azimuth around the dome (0°–360°)                     |
| **Sphere radius** | Distance from sphere center to each pin               |


Grid generation (`generateHemisphereScanPoints`):

- θ: linear samples from `thetaMinDeg` → `thetaMaxDeg` (`verticalPoints` rings)
- φ: evenly spaced per ring (`horizontalPoints` per ring)
- Cartesian position on sphere:
  - `x = R sin(θ) cos(φ)`
  - `y = R sin(θ) sin(φ)`
  - `z = R cos(θ)` (above tray center)



### 3.2 TCP orientation

For each grid point, `tcpPoseForHemispherePoint` sets:

- **Position:** pin on dome, offset by tray height
- **Tool +Z:** unit vector **inward** toward sphere center (camera looks at sample)
- **Rotation:** UR rotation vector (axis-angle) from that frame



### 3.3 Workspace boundary

From `hyperfusion.cfg` — a **600 × 600 × 650 mm** cube (defaults):

- Tray centered at origin
- Z = 0: tray bottom
- Z = height: robot mount plane (ceiling)
- MoveIt adds thin collision slabs (floor, ceiling, four walls) during plan/execute



### 3.4 Home pose

From `hyperfusion.cfg` `[ur3e]`:

```ini
home_joints_deg = 0,-130,120,0,90,0
```

Six joint angles (degrees): `pan, lift, elbow, wrist_1, wrist_2, wrist_3`.

Used for:

- Plan IK reference (every pin seeded from home)
- Execute retreat target
- Move to home **before** first pin and **after** scan completes

---



## 4. Configuration summary


| Setting             | Source            | Role                     |
| ------------------- | ----------------- | ------------------------ |
| Grid size (H × V)   | UI                | Pin count                |
| Sphere radius       | UI                | Dome size                |
| θ range             | UI                | Which part of hemisphere |
| Workspace box       | `hyperfusion.cfg` | Collision limits         |
| Home joints         | `hyperfusion.cfg` | Reference + retreat pose |
| `use_mock_hardware` | `hyperfusion.cfg` | Mock vs real robot       |


---



## 5. Plan phase

**Trigger:** User presses **Plan** in the Scanning route panel.

### 5.1 App (C++)

1. Read scan params from `Ur3eHemisphereScanSettingsWidget`
2. Build grid locally: `generateHemisphereScanPoints`
3. Compute TCP pose per pin: `tcpPoseForHemispherePoint`
4. POST all poses to sidecar: `/plan_hemisphere_scan`
  - Includes workspace box and `home_joints_deg`
5. Requires robot **connected** (`initial_seed` = current joints — proves link only; **not** used as IK reference)



### 5.2 Sidecar — home-centric IK (`plan_poses`)

For **each pin independently** (no route chaining):

```
For each pin:
  1. Build 9 IK seeds from home
  2. For each seed → /compute_ik (collision-aware)
  3. Normalize solution to home branch
  4. /check_state_validity (static collision + limits)
  5. Keep best valid solution nearest home
  6. Store joints or error reason
```

**Important:** Plan does **not** check whether a **motion path** exists — only that the arm can **hold** the pose at the home branch without collision.

### 5.3 Multi-seed IK (9 seeds per pin)


| #   | Seed                     |
| --- | ------------------------ |
| 1   | Home (`home_joints_deg`) |
| 2–3 | Shoulder lift ±45°       |
| 4–5 | Elbow ±45°               |
| 6–7 | Wrist 1 ±45°             |
| 8–9 | Wrist 2 ±45°             |


Perturbations are **small nudges** (±45°) from home — not full ±180° flips. Large perturbations tended to put the IK seed in self-collision with the arm or workspace box before a solution was even attempted.

Seeds that fail static collision (`/check_state_validity`) are **skipped** before calling `/compute_ik`. The best valid solution **closest to home** (plan) or **current pose** (execute) is kept.

### 5.4 Joint branch normalization

UR joints have multiple valid representations (e.g. 209° vs −151°). The planner:

- Wraps angles to UR3e limits
- Picks the branch **nearest** the reference (home at plan time)
- Avoids branch drift that broke execute when pins were chained



### 5.5 Plan output


| Result          | Meaning                                           |
| --------------- | ------------------------------------------------- |
| **Green pin**   | Reachable — valid IK + static collision OK        |
| **Red pin**     | Unreachable — no IK, collision, or invalid joints |
| **Plan joints** | 6 stored joint angles (rad) per reachable pin     |


---



## 6. Execute phase

**Trigger:** User presses **Execute** (after successful Plan).

### 6.1 Visit order (fixed)

**Top → bottom, ring-by-ring:**

1. θ ascending (upper/apex rings first)
2. φ ascending within each ring

Only **reachable** pins are visited.

### 6.2 Execute sequence (full)

```
Execute pressed
    │
    ▼
① Move to HOME (MoveIt, collision-aware)
    │  fail → abort entire scan
    ▼
② For each pin (in order):
    │
    ├─ Live IK refresh (9 seeds from current pose)
    │     success → use refreshed joints
    │     fail    → fall back to Plan joints
    │
    ├─ Validate start state + goal state
    │
    ├─ Try DIRECT: current → pin (MoveIt)
    │     success → dwell 2 s → next pin
    │     fail    → go to retreat path
    │
    ├─ RETREAT: current → HOME (MoveIt)
    │     fail → skip pin, continue
    │
    ├─ Live IK again from home (9 seeds)
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


| When                 | Behavior                                            |
| -------------------- | --------------------------------------------------- |
| **Before first pin** | Always move to home; abort if it fails              |
| **After last pin**   | Always return to home (unless fatal mid-scan error) |
| **Mid-scan retreat** | Only when direct path fails                         |




### 6.4 Per-pin timing

- **2 s dwell** at each successful pin (`kScanExecuteDwellMs`)
- Joint poll ~**100 ms** during motion (async, non-blocking UI)
- MoveIt motion timeout up to **120 s** per leg



### 6.5 Skip behavior

If a pin cannot be reached (no path even via home):

- Log: `skipped — <reason>`
- **Continue** to next pin
- Summary at end: `N pin(s) skipped`

Scan is **best-effort**, not all-or-nothing.

---



## 7. Live IK vs Plan joints


|                  | **Plan joints**        | **Live IK (execute)**                   |
| ---------------- | ---------------------- | --------------------------------------- |
| **When**         | Plan time              | Each pin before motion                  |
| **Seeds**        | 9 from **home**        | 9 from **home** (same perturbations)    |
| **Pick nearest** | Home branch            | **Current arm** pose                    |
| **On failure**   | Pin marked unreachable | **Fall back to Plan joints**            |
| **Motion?**      | No                     | No — only sets goal; MoveIt moves after |


**After retreat to home:** live IK runs again from home; if that fails, Plan joints are used for the home→pin leg.

### 7.1 When does the robot go home?


| Situation            | Goes home?                            |
| -------------------- | ------------------------------------- |
| Plan                 | No — home is reference only           |
| Execute start        | Yes — before first pin                |
| Execute end          | Yes — after last pin                  |
| Direct move succeeds | No — stays at pin (2 s dwell)         |
| Direct move fails    | Yes — retreat, then approach          |
| Skipped pin          | No — stays at current pose, continues |


---



## 8. Motion planning (MoveIt execute)



### 8.1 Planner order (per leg)

1. OMPL **RRTConnect**
2. OMPL **RRTstar**
3. Pilz **PTP**

MoveIt only — no direct `joint_trajectory_controller` fallback.

### 8.2 Speed scaling

Joint travel distance sets velocity/acceleration scale:


| Travel (rad sum) | Scale |
| ---------------- | ----- |
| > 5.0            | 0.05  |
| > 3.0            | 0.08  |
| else             | 0.15  |


Larger moves run slower for safety.

### 8.3 Common MoveIt error codes


| Code    | Name                | Typical cause                   |
| ------- | ------------------- | ------------------------------- |
| **-2**  | INVALID_MOTION_PLAN | No collision-free path          |
| **-4**  | CONTROL_FAILED      | Controller rejected trajectory  |
| **-26** | START_STATE_INVALID | Start joints invalid for MoveIt |


---



## 9. UI and visualization



### 9.1 Settings panel

- Sphere radius, grid (H × V), θ range
- **Plan** / **Execute** buttons



### 9.2 3D preview (`Ur3eHemisphereScanPreviewWidget`)

- Dome, tray, workspace boundary, pin normals
- **Green** = reachable, **red** = unreachable
- During execute: active pin **color pulse** (80 ms timer)
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
| **Home-centric plan**            | Every pin judged on same branch as retreat; fixes chained-branch bugs |
| **No route chaining at plan**    | Avoids −304° pan drift from pin-to-pin seeds                          |
| **Direct first, home fallback**  | Fast when nearby pins work; home when not                             |
| **Skip on failure**              | Partial scans better than aborting entire route                       |
| **Static plan, dynamic execute** | Plan is fast; execute re-validates paths                              |
| **Configurable home in cfg**     | Match real robot home; same for mock and field                        |
| **Top-to-bottom rings**          | Predictable scan order for dome imaging                               |


---



## 11. What Plan does *not* guarantee

A **green** pin means:

> There exists a collision-free **static pose** on the home branch.

It does **not** guarantee:

> There is a collision-free **path** from wherever the arm will be during execute.

Some green pins may **skip at execute** if motion planning fails. Optional future work: **Phase B** — `plan_only` home→pin check at plan time (slower plan, fewer execute surprises).

---



## 12. Operational checklist

1. Start sidecar / connect UR3e
2. Set `home_joints_deg` in `hyperfusion.cfg` to match robot
3. **Plan** — review green/red pins
4. **Execute** — home → pins → home
5. After **Python** changes → restart sidecar
6. After **C++** changes → rebuild app
7. Avoid running two `move_group` instances without restart

---



## 13. Glossary


| Term             | Definition                                                     |
| ---------------- | -------------------------------------------------------------- |
| **Pin**          | One dome grid point + TCP pose + (if reachable) joint solution |
| **Ring**         | All pins at the same θ (elevation)                             |
| **Plan joints**  | Joint angles stored at Plan for a pin                          |
| **Home**         | Configured safe reference pose; retreat and bookends           |
| **Live IK**      | Re-solve IK at execute from current (or home) pose             |
| **Direct leg**   | MoveIt path: current → pin                                     |
| **Retreat leg**  | MoveIt path: current → home                                    |
| **Approach leg** | MoveIt path: home → pin                                        |
| **Skip**         | Pin unreachable; scan continues                                |


---



## 14. File reference


| Layer          | Main files                                                                          |
| -------------- | ----------------------------------------------------------------------------------- |
| Config         | `app/hyperfusion.cfg`, `HyperFusionConfig.cpp`                                      |
| Grid / TCP     | `Ur3eHemisphereScan.cpp`, `Ur3eHemisphereScanReachability.cpp`                      |
| UI             | `Ur3eHemisphereScanSettingsWidget.cpp`, `Ur3eHemisphereScanPreviewWidget.cpp`       |
| Orchestration  | `Ur3ePanelController.cpp`                                                           |
| HTTP client    | `Ur3eClient.cpp`                                                                    |
| Sidecar        | `hyperfusion_ur3e/bridge/ros_bridge.py`, `hyperfusion_ur3e/sidecar/http_handler.py` |
| MoveIt planner | `hyperfusion_ur3e/moveit/scan_planner.py`                                           |


---



## 15. HTTP API (sidecar)


| Endpoint                      | Purpose                                |
| ----------------------------- | -------------------------------------- |
| `POST /plan_hemisphere_scan`  | IK + static validity for all poses     |
| `POST /execute_scan_waypoint` | MoveIt plan+execute one pin            |
| `POST /execute_move_home`     | MoveIt plan+execute to configured home |


Request bodies include `workspace` and `home_joints_deg` from `hyperfusion.cfg`.