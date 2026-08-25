# BFS + UR3e calibration (eye-in-hand, fixed board)

Working directory for **BFS camera intrinsics**, **flange → camera** (hand–eye), and **base → board / tray**. Not FX10e/SWIR `.scp` packs.

## Current results (2026-08-17)

Written into `hyperfusion.cfg` and `hyperfusion_tcp` (xyz **and** rpy). Dummy `fx=4638` and CAD TCP are retired.

**Dataset:** `checkerboard/` — 72 stills (clipped frames and 180° flip `00082` removed). 4096×3000. 72/72 detected (`10×7`). Fit 64, hold-out last 8. `00108` (glare) already gone.

### Intrinsics `K`, `D`

| | value |
|---|---|
| RMS | **0.96 px** (target was &lt; 0.5; usable, not the spec) |
| `bfs_camera_fx` | 1787.820905328422 |
| `bfs_camera_fy` | 1787.499380533722 |
| `bfs_camera_cx` | 2084.4011955271963 |
| `bfs_camera_cy` | 1526.0259879957282 |
| `bfs_camera_distortion` | −0.16223465, 0.10156067, 0.0025228506, 0.00068692294, −0.029189458 |

Free `calibrateCamera` (not locked to 4638). `fx ≈ 1788` matches ~200 mm working distance. YAML: `results/camera_intrinsics.yaml`. Undistort before/after: `results/undistort_preview/`.

Old still JSONs in `checkerboard/` still say `fx=4638` (capture log). The solver used the images. New stills write the measured `K,D` from cfg.

### Hand–eye Tsai (`tool0` → camera)

| | old CAD | calibrated (in cfg / URDF) |
|---|---|---|
| xyz mm | (0, −56.035, 81.825) | **(0.715, −54.197, 73.755)** |
| rpy deg | 0, 0, 0 | **(−1.9138, 0.7450, 0.2868)** ≈ 2° |

Optical +Z is ~2° off `tool0` +Z. `hyperfusion_tcp` uses the full SE(3). YAML: `results/flange_T_camera.yaml`.

Fit-set `base_T_board` cluster: median **0.29 mm / 0.10°**, max **2.5 mm / 0.71°** (after dropping `00082`).

### Board in `base_link`

Origin ≈ **(12.5, −95.6, 629) mm**, yaw ≈ 90°. That Z is the printed-corner stand-off, not tray center. Cfg `ceiling_mount_height_mm = 629` matches this plane. YAML: `results/base_T_board.yaml`.

### Hold-out (last 8)

| | result | target |
|---|---|---|
| translation | **0.57 mm** | &lt; 2 mm |
| rotation | **0.15°** | &lt; 0.5° |
| PnP RMS | **0.67 px** | &lt; 0.5 px |

YAML: `results/validation.yaml`.

### Live check (`multiview_20260817_160212`)

Apex `00000`: optical TCP **199 mm** above tray (plan 200 mm). Intrinsics in that JSON match cfg.

**After this calibration:** re-Plan Semi routes. Old `R200_*` joints were IK’d for CAD TCP and will not match the cfg fingerprint.

---

## What this is for

| Quantity | Now (cfg / URDF) |
| -------- | ---------------- |
| Intrinsics `K` | Measured `bfs_camera_fx/fy/cx/cy` |
| Distortion `D` | Brown–Conrady in `bfs_camera_distortion`; stills store `D`; live view stays raw |
| Optical TCP | Tsai `tool_tcp_*_mm` + `tool_tcp_roll/pitch/yaw_deg` on `hyperfusion_tcp` |
| Board / tray | Averaged `base_link → board` in `results/base_T_board.yaml` |

Do **not** hand–eye against `hyperfusion_tcp`. That frame already includes `tool_tcp_*`. Gripper frame is **`tool0` (flange)**.

---

## Frames (lock these names)

```
world  (tray / URDF, ceiling mount)
  └─ base_link          robot base
       └─ tool0         flange  =  Base T Flange
            └─ camera   OpenCV  =  Flange T Camera   ← hand–eye result
                 └─ board       =  Camera T Board    ← solvePnP

board is fixed on the tray for the whole capture.
```

- **OpenCV camera:** X right, Y down, Z optical (out of the BFS).
- **`tool0`:** ROS-Industrial flange. Calibrated optical +Z is ~2° from `tool0` +Z.
- **`board`:** origin at a chosen inner corner (see §1), Z out of the printed face toward the camera when the board is face-up on the tray.
- **`world`:** tray frame from `mount_*` / `ceiling_mount_height_mm`. Reconstruction can stay in `base_link`.
- **Not this problem:** HSI linear rail `[sample_stage_position]`.

OpenCV `calibrateHandEye(...)` returns **`Flange T Camera`**.

---

## Folder layout

```
app/calibration/multiview/
  board.yaml
  checkerboard/          # current stills + live tool0 JSON (this calibration)
  captures/              # extra stamped sets
  results/
    camera_intrinsics.yaml
    undistort_preview/
    eval_preview/
    flange_T_camera.yaml
    base_T_board.yaml
    validation.yaml
  scripts/
    calibrate.py
    requirements.txt
```

---

## Pipeline (recapture)

### 1. Prepare the target

- Rigid, flat board. A4 sheet (~297 × 210 mm) is fine; printed area is not the OpenCV pattern span.
- **Squares vs inner corners:** an `11 × 8` square grid has **`10 × 7` inner corners**.
- **Measured square size: 22 mm.** Inner-corner span is **220 × 154 mm**. Store this in `board.yaml`; object points use metres (`0.022`).
- Board frame (write it in `board.yaml` and do not change it):
  - origin = top-left **inner** corner as seen in a nadir image
  - +X along the long side (width)
  - +Y along the short side (height)
  - +Z out of the board (toward the camera at nadir)

OpenCV `findChessboardCorners` uses **inner corners**, not outer squares.

### 2. Fix the board on the tray

Tape/clamp so it **cannot move** for the whole sequence plus hold-out poses.

### 3. Collect one synchronized dataset

**~30 imaging poses + ~6–8 hold-out poses.**

At every still, after `scan_capture_stabilize_ms`, save TIFF + live `base_link → tool0` + joints.

**Motion:** translation (center / edges; close and far), rotation (~±10°, ±20°, ±30° about more than one axis), board near image edges. A look-at-center hemisphere at one range is a weak `K` set (this capture: RMS 0.96 px).

Drop a pose if detection fails (`00082` was a 180° origin flip — 238 mm / 180° off the cluster).

### 4. Calibrate intrinsics (`K` and `D`)

1. Detect corners on **raw** TIFFs.
2. Object points from **22 mm** squares and `(10, 7)`.
3. `cv2.calibrateCamera` → 5-coeff Brown–Conrady. Do not lock fx to 4638.
4. Write `results/camera_intrinsics.yaml`.
5. Copy `bfs_camera_*` into `hyperfusion.cfg` after reviewing RMS and undistort previews.

Today the app **stores** `D` in still JSON; it does **not** remap live pixels.

**Pass:** RMS **&lt; 0.5 px** (this set: 0.96 px).

### 4b. Distortion correction (apply `D`)

Keep raw TIFFs. Remap with `getOptimalNewCameraMatrix` + `initUndistortRectifyMap`. PnP / hand–eye on **raw** with `K,D`. Reconstruction uses the same maps / `newK`.

### 5–6. PnP and hand–eye

`solvePnP` then `calibrateHandEye` (Tsai, Park, Horaud, Daniilidis). Keep the tightest `Base T Board` cluster. This set: **Tsai**.

Write `flange_T_camera.yaml` (metres, OpenCV axes) and URDF `origin xyz rpy` for `hyperfusion_tcp`. Do not drop rotation.

### 7. Stage / board transform

```
Base T Board,i = Base T Flange,i · Flange T Camera · Camera T Board,i
```

Robust average → `base_T_board.yaml`. Report spread (mm / deg).

### 8. Hold-out

Targets: **&lt; 2 mm**, **&lt; 0.5°**, **&lt; 0.5 px**. This set: 0.57 mm, 0.15°, 0.67 px.

### 9. Use during scanning

1. New stills get `K,D` from cfg in pose JSON.
2. Camera pose: live TF `hyperfusion_tcp` (includes calibrated xyz+rpy) or `Base T Flange · Flange T Camera`. JSON `extrinsics` are camera → `base_link`.
3. Re-Plan Semi / hemisphere routes after any TCP change.

---

## Run the offline pipeline (Windows)

```powershell
cd app\calibration\multiview
python -m venv .venv
.\.venv\Scripts\pip install -r scripts\requirements.txt
.\.venv\Scripts\python scripts\calibrate.py --images checkerboard --board board.yaml --out results --holdout 8
```

Results: `camera_intrinsics.yaml`, `undistort_preview/`, `flange_T_camera.yaml`, `base_T_board.yaml`, `validation.yaml`.

No motion on app start. Calibration capture is an explicit, armed sequence like scan Execute.
