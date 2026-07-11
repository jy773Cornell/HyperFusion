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

- **Position:** pin on dome, offset by tray height, then mount transform
- **Tool +Z:** unit vector **inward** toward the dome / floor-circle center `(0, 0, tray_height)`
- **Rotation:** UR rotation vector (axis-angle) from that frame
- **IK filter:** MoveIt solutions whose tool0 +Z dot product with the desired inward axis is below 0.95 are rejected (prevents 180° “facing outward” wrist flips)



### 3.3 Workspace boundary

From `hyperfusion.cfg` — a **600 × 600 × 650 mm** cube (defaults):

- Tray centered at origin
- Z = 0: tray bottom
- Z = height: robot mount plane (ceiling)
- MoveIt adds thin collision slabs (floor, ceiling, four walls) during plan/execute

### 3.3.1 Mount orientation (align RViz with real robot)

The robot base is placed in the URDF via a **world → base_link** transform. Defaults match a ceiling mount (roll = 180°). If RViz/simulation looks rotated or mirrored vs your real install, tune these keys in `hyperfusion.cfg` `[ur3e]`:

```ini
# Degrees / mm. Z height comes from workspace_height_mm.
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

**Also applied to:** URDF robot base (full roll/pitch/yaw + offset), hemisphere **pin preview** (tray, dome, workspace box, pins), and MoveIt scan TCP targets (yaw/pitch/offset — roll is ceiling robot flip only).

Restart the UR3e sidecar and reopen MoveIt/RViz after changing mount values (URDF is built at launch). Restart the app to reload `hyperfusion.cfg` for the scan-route preview.


From `hyperfusion.cfg` `[ur3e]`:

```ini
home_joints_deg = 0,-130,120,0,90,0
```

Six joint angles (degrees): `pan, lift, elbow, wrist_1, wrist_2, wrist_3`.

Used for:

- Primary IK seed at plan and execute
- Execute retreat target
- Move to home **before** first pin and **after** scan completes
- Mock-hardware startup pose (`initial_positions.yaml`)

**Important:** Home must pass MoveIt **static collision** checks inside the workspace box, not just match joint numbers. Joint tolerance alone is not enough — see §6.6 and §11.

### 3.5 IK seeds (home + current, 45° permutations)

IK seeds are generated automatically — there are **no cfg-defined midpoints**. For each pin, the seed set is built from two base poses:

1. **`home_joints_deg`**
2. **Current pin position** — robot joints at plan start; after the first reachable pin, the last reachable pin's solution

For each base pose, every joint is also offset individually by **45°, 90°, 135°, 180°, 225°, 270°, and 315°** (one joint at a time). Duplicate seeds are removed.

This replaces the old `ik_seed_midpoints_deg` cfg key (removed) and the earlier 90°/180°/270°-only permutations.

---



## 4. Configuration summary


| Setting             | Source            | Role                                      |
| ------------------- | ----------------- | ----------------------------------------- |
| Grid size (H × V)   | UI                | Pin count                                 |
| Sphere radius       | UI                | Dome size                                 |
| θ range             | UI                | Which part of hemisphere                  |
| Workspace box       | `hyperfusion.cfg` | Collision limits                          |
| Mount RPY / offset  | `hyperfusion.cfg` | URDF base pose (RViz vs real alignment)   |
| Home joints         | `hyperfusion.cfg` | Primary IK seed + retreat pose            |
| IK seeds            | auto              | home + current pose, 45° joint permutations |
| `use_mock_hardware` | `hyperfusion.cfg` | Mock vs real robot                        |


---



## 5. Plan phase

**Trigger:** User presses **Plan** in the Scanning route panel.

### 5.1 App (C++)

1. Read scan params from `Ur3eHemisphereScanSettingsWidget`
2. Build grid locally: `generateHemisphereScanPoints`
3. Compute TCP pose per pin: `tcpPoseForHemispherePoint`
4. POST all poses to sidecar: `/plan_hemisphere_scan`
  - Includes `workspace` and `home_joints_deg`
5. Requires robot **connected** (`initial_seed` = current joints — used as the first pin’s “current pin” seed until a reachable pin is found)



### 5.2 Sidecar — home-centric IK (`plan_poses`)

For **each pin independently** (no route chaining for motion, but IK seeds can chain):

```
For each pin:
  1. Build IK seeds: (home + current pin) × 45° per-joint permutations
  2. For each seed → validate seed, then /compute_ik (collision-aware)
  3. Normalize solution to home branch
  4. /check_state_validity (static collision + limits + pinch guard)
  5. Keep best valid solution nearest home
  6. Store joints or error reason
  7. If reachable, use this pin’s joints as “current pin” seed for the next pin
```

**Important:** Plan does **not** check whether a **motion path** exists — only that the arm can **hold** the pose at the home branch without collision.

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
| **Reachable**   | Green         | Valid IK + static collision OK                    |
| **Unreachable** | Blue        | No IK, collision, or invalid joints               |
| **Plan joints** | —             | 6 stored joint angles (rad) per reachable pin     |


---



## 6. Execute phase

**Trigger:** User presses **Execute** (after successful Plan).

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
    ├─ Validate start state + goal state
    │     start invalid → skip pin (common if home pose collides)
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
| **Mid-scan retreat** | Only when direct path fails                                      |
| **After connect**    | MoveIt verify/move to home; dialog on failure                    |
| **Before disconnect**| MoveIt verify/move to home; dialog on failure                    |
| **Before app close** | MoveIt verify/move to home if robot connected; Cancel keeps app open |


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

MoveIt `max_velocity_scaling_factor` from joint travel (before planning):

| Travel (rad sum) | Scale |
| ---------------- | ----- |
| > 5.0            | 0.05  |
| > 3.0            | 0.08  |
| else             | 0.15  |

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
- Same trajectory prep as scan execute (multi-planner, ±π unwrap, 190 deg/s cap)
- **No home fallback** if planning fails — the MoveIt error is shown in the log

**Base-only** moves (only `shoulder_pan` changes) and pinch-escape moves to a safer goal still apply for manual jogging — see §11.2.


---



## 9. UI and visualization



### 9.1 Settings panel

- Sphere radius, grid (H × V), θ range
- **Plan** / **Execute** buttons



### 9.2 3D preview (`Ur3eHemisphereScanPreviewWidget`)

- Dome, tray, workspace boundary, pin normals
- **Plan view:** green = reachable, blue = unreachable (plan)
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
| **Home-centric plan**            | Every pin judged on same branch as retreat; fixes chained-branch bugs |
| **No route chaining at plan**    | Avoids −304° pan drift from pin-to-pin seeds                          |
| **Direct first, home fallback**  | Fast direct path; retreat via home is the only fallback              |
| **Easiest trajectory wins**      | Plan-only all pipelines; pick shortest joint travel + pinch-safe path |
| **Skip on failure**              | Partial scans better than aborting entire route                       |
| **Static plan, dynamic execute** | Plan is fast; execute re-validates paths                              |
| **Configurable home in cfg**     | Match real robot home; same for mock and field                        |
| **Auto IK seeds (home + current, 45° perms)** | Simple, dense seed set; no cfg midpoints to maintain    |
| **MoveIt home validity**         | Joint match alone is insufficient — home must pass collision check    |
| **Homing dialog**                | Manual jog + retry when home invalid or unreachable                   |
| **Home-nearest ring sweep**      | Short, simple trajectories; fewer C403A0 pinch stops                  |


---



## 11. What Plan does *not* guarantee

A **green** pin means:

> There exists a collision-free **static pose** on the home branch.

It does **not** guarantee:

> There is a collision-free **path** from wherever the arm will be during execute.

Some green pins may **skip at execute** if motion planning fails. Optional future work: **Phase B** — `plan_only` home→pin check at plan time (slower plan, fewer execute surprises).

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

If the pendant shows **External Control speed limit** on **joint 5** (wrist_3) or another joint, consecutive trajectory samples were too close in time (often **0.002 s** at the 500 Hz External Control rate) or had a branch jump. HyperFusion **unwraps** trajectories on the hardware joint branch, snaps the first waypoint to live feedback, strips velocity/acceleration fields, enforces a minimum **20 ms** between samples, and caps peak joint speed via `max_joint_velocity_deg_s` in `hyperfusion.cfg` (default **60 deg/s**; UR hardware allows up to **190**). Lower the cap if warnings persist; restart the sidecar after changing config.

---



## 12. Operational checklist

1. Start sidecar / connect UR3e
2. Set `home_joints_deg` in `hyperfusion.cfg` to match robot (6 values; must be collision-free and pinch-safe in workspace)
3. **Connect** — confirm homing succeeds (or fix pose via dialog)
4. **Plan** — review green/blue pins
5. **Execute** — home → pins → home (top ring first, home-nearest entry on top ring)
6. After **Python** changes → restart sidecar
7. After **C++** changes → rebuild app
8. Avoid running two `move_group` instances without restart

---



## 13. Glossary


| Term             | Definition                                                     |
| ---------------- | -------------------------------------------------------------- |
| **Pin**          | One dome grid point + TCP pose + (if reachable) joint solution |
| **Ring**         | All pins at the same θ (elevation)                             |
| **Plan joints**  | Joint angles stored at Plan for a pin                          |
| **Home**         | Configured safe reference pose; retreat and bookends               |
| **IK seed**      | home or current pose (+ 45° per-joint permutations) used for IK    |
| **Current pin**  | Robot joints at plan start, or last reachable pin’s plan solution  |
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