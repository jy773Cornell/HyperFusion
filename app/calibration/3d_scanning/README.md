# BFS + UR3e calibration (eye-in-hand, fixed board)

Working directory for **BFS camera intrinsics**, **flange → camera** (hand–eye), and **base → board / tray**. Not FX10e/SWIR `.scp` packs.

## What this is for

Scanning today uses two CAD/nominal numbers:

| Quantity | Current source | After this calibration |
| -------- | -------------- | ---------------------- |
| Intrinsics `K` | `bfs_camera_fx/fy/cx/cy` (nominal) | Measured `fx, fy, cx, cy` |
| Distortion `D` | `bfs_camera_distortion` **empty** — stills are not undistorted | Brown–Conrady `k1,k2,p1,p2,k3` + remap for reconstruction |
| Optical TCP | `tool_tcp_*_mm` translation in `tool0`, **same orientation as `tool0`** | Full `tool0 → camera` SE(3) |
| Board / tray | Assumed at tray center | Averaged `base_link → board` |

Do **not** hand–eye against `hyperfusion_tcp`. That frame already includes CAD `tool_tcp_*`. Solving against it bakes CAD error into the result. Gripper frame is **`tool0` (flange)**.

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
- **`tool0`:** ROS-Industrial flange; HyperFusion CAD assumes `tool0` +Z = optical +Z. Hand–eye is allowed to find a small extra rotation.
- **`board`:** origin at a chosen inner corner (see §1), Z out of the printed face toward the camera when the board is face-up on the tray.
- **`world`:** tray frame from `mount_*` / `ceiling_mount_height_mm`. Reconstruction can stay in `base_link`; convert with the mount transform only when you need tray coordinates.
- **Not this problem:** HSI linear rail `[sample_stage_position]`.

OpenCV `calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)` returns **`R_cam2gripper`, `t_cam2gripper`** = **`Flange T Camera`**.

---

## Folder layout

```
app/calibration/3d_scanning/
  checkerboard/          # current stills (no poses) — detector smoke test only
  captures/<stamp>/      # synchronized dataset (required for hand–eye)
    images/              # cb_XXXX.tif
    poses.jsonl          # one record per still
    board.yaml           # measured square size + inner-corner size
  results/
    camera_intrinsics.yaml   # K, D, image size, RMS
    undistort_preview/       # a few before/after TIFFs
    flange_T_camera.yaml
    base_T_board.yaml
    validation.yaml
  scripts/               # OpenCV pipeline (to add)
```

The 39 TIFFs already in `checkerboard/` are hemisphere stills with **no flange poses** (JSON removed). Use them to tune corner detection. **Do not** use them for hand–eye or `base_T_board`.

---

## Pipeline

### 1. Prepare the target

- Rigid, flat board. A4 sheet (~297 × 210 mm) is fine; printed area is not the OpenCV pattern span.
- **Squares vs inner corners:** an `11 × 8` square grid has **`10 × 7` inner corners**.
- **Measured square size: 22 mm.** Inner-corner span is **220 × 154 mm**. Store this in `board.yaml`; object points use metres (`0.022`).
- Board frame (example, write it down in `board.yaml` and do not change it):
  - origin = top-left **inner** corner as seen in a nadir image
  - +X along the long side (width)
  - +Y along the short side (height)
  - +Z out of the board (toward the camera at nadir)

OpenCV `findChessboardCorners` uses **inner corners**, not outer squares.

### 2. Fix the board on the tray

- Tape/clamp so it **cannot move** for the whole sequence plus hold-out poses.
- A fixed board is the tray/world reference. If it shifts, `Base T Board` estimates will disagree and the average is meaningless.

### 3. Collect one synchronized dataset (new capture)

**~30 imaging poses + ~6 hold-out poses** (hold-out not used in `calibrateCamera` / `calibrateHandEye`).

At every still, after `scan_capture_stabilize_ms`, save:

| Field | Required | Source |
| ----- | -------- | ------ |
| BFS TIFF | yes | same still pipeline as scan |
| `base_link → tool0` | **yes** | live TF (not planned pin, not `hyperfusion_tcp`) |
| 6 joint angles | yes | `joint_states` (FK fallback / audit) |
| timestamp + index | yes | capture clock |
| `base_link → hyperfusion_tcp` | optional | CAD TCP, for before/after comparison only |

**Motion (do not reuse a look-at-center hemisphere as the only set):**

- Translation: center / left / right / top / bottom of the board in the image; close and far working distances the lens actually uses for scans.
- Rotation: roughly **±10°, ±20°, ±30°** about more than one axis (not a single θ-ring).
- Distortion: board **near image edges and corners** on several frames. Hemisphere pins that stare at tray center keep the board in the middle — that is a weak intrinsic set.
- Coverage: every inner corner detected on as many frames as possible; drop a pose if detection fails rather than forcing it.

Reachability/collision still go through the existing UR3e MoveIt path. Calibration poses can be a **manual/semi set**, not Auto dome pins.

### 4. Calibrate intrinsics (`K` and `D`)

1. Detect corners (`findChessboardCorners` + `cornerSubPix`) on **raw** TIFFs.
2. Object points from **22 mm** squares and `(cols, rows) = (10, 7)` if using 11×8 squares.
3. `cv2.calibrateCamera` → `K` and `D` (5-coeff Brown–Conrady `k1,k2,p1,p2,k3`). BFS 4096×3000 with fx ~4.6k px is a moderate FOV; do not fit a 14-coeff model on 30 frames.
4. Write `results/camera_intrinsics.yaml` (`K`, `D`, width, height, RMS).
5. Copy `fx, fy, cx, cy` and `bfs_camera_distortion = k1,k2,p1,p2,k3` into `hyperfusion.cfg` only after validation.

Today the app **stores** `D` in still JSON if cfg is set; it does **not** remap pixels. Distortion correction is a required product of this calibration, not optional metadata.

**Pass:** RMS reprojection **&lt; 0.5 px** (prefer ~0.2–0.3 px).

### 4b. Distortion correction (apply `D`)

Keep raw TIFFs as the archive. Build a remap once:

```
newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0)
mapx, mapy = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_32FC1)
undistorted = cv2.remap(raw, mapx, mapy, cv2.INTER_LINEAR)
```

- **Validate:** undistort several calibration frames (center + edge-heavy). Checkerboard lines must be straight out to the corners. Save before/after under `results/undistort_preview/`.
- **PnP / hand–eye:** run on **raw** images with `K,D` (OpenCV models distortion). Do **not** undistort first unless you switch to `newK` and `D = 0`.
- **Scanning / reconstruction:** undistort every BFS still with the same maps (or equivalent `cv2.undistort`). Use `newK` as the camera matrix for geometry on undistorted pixels. Live view can stay raw.

`alpha=0` crops invalid borders (preferred for recon). `alpha=1` keeps the full FOV with empty edges.

### 5. Board pose per image

With locked `K`,`D` on **raw** images, `solvePnP` (iterative or IPPE_SQUARE, then refine) → **`Camera T Board,i`** (`rvec`, `tvec`).

Store next to the flange pose. Reject frames with high PnP reprojection.

### 6. Hand–eye: `Flange T Camera`

Inputs per pose `i`:

- `Base T Flange,i` from live TF `tool0`
- `Camera T Board,i` from PnP

Run `cv2.calibrateHandEye` with Tsai, Park, Horaud, and Daniilidis. Compare:

- translation spread (mm)
- rotation spread (deg)
- residual of `Base T Board` in §7

Keep the method whose `Base T Board` cluster is tightest and whose translation is close to CAD (`tool_tcp_*` ≈ 0, −56, +82 mm in `tool0`, plus a small rotation). A result that disagrees with CAD by tens of millimetres or ~90° about Z is a **frame-convention bug**, not a better calibration.

Write `results/flange_T_camera.yaml` as a 4×4 (metres, OpenCV camera axes). Also export URDF `origin xyz rpy` for `hyperfusion_tcp` if you keep that tip for IK.

**Scan stack today assumes translation-only TCP.** After calibration, either:

- update URDF `hyperfusion_tcp` with **xyz + rpy**, or
- apply `Flange T Camera` only in reconstruction and leave MoveIt on CAD TCP.

Do not silently replace `tool_tcp_*` with the translation part and drop rotation.

### 7. Stage / board transform

Board never moved, so for every pose:

```
Base T Board,i = Base T Flange,i · Flange T Camera · Camera T Board,i
```

These must agree. Robust average (e.g. translation median + rotation chordal / quaternion median) → `results/base_T_board.yaml`.

Optional tray frame:

```
World T Board = World T Base · Base T Board
```

`World T Base` is the ceiling mount already in cfg. Report the spread of `Base T Board,i` (mm / deg) as a health check. Outliers usually mean a bumped board or a bad detection.

### 8. Validate on hold-out poses

For poses **not** used in §4–6:

1. Predict `Base T Camera = Base T Flange · Flange T Camera`.
2. From the image, PnP → `Camera T Board`, hence `Base T Board` observed.
3. Compare to the averaged `Base T Board` from §7.

Report in `results/validation.yaml`:

- translation error (mm)
- rotation error (deg)
- reprojection (px)

Targets (first pass): **&lt; 2 mm**, **&lt; 0.5°**, **&lt; 0.5 px**. Tighten once the capture set is diverse.

### 9. Use during scanning

1. **Undistort** each BFS still with the calibrated `K`,`D` maps before reconstruction / fusion. Write `D` into still JSON from cfg so a later pipeline can remap even if the TIFF on disk is raw.
2. Camera pose: `Base T Camera = Base T Flange · Flange T Camera` (live `tool0` TF × calibrated hand–eye). Stop treating CAD `hyperfusion_tcp` as the optical origin once the URDF/cfg is updated.
3. Sample / tray: `Base T Board` (or `World T Board`) as the fixed reconstruction origin if the sample is registered to the board.

---

## Implementation order

1. `board.yaml` + print/measure the real grid (inner-corner count confirmed on one image from `checkerboard/`).
2. Capture writer: TIFF + **`tool0` TF** + joints (keep optional CAD TCP).
3. Offline scripts: detect → `K,D` → undistort preview → PnP → hand–eye → average `Base T Board` → validation report.
4. Write results yaml; patch `hyperfusion.cfg` (`bfs_camera_*` including distortion); then remap stills in the scan/recon path.

No motion on app start. Calibration capture is an explicit, armed sequence like scan Execute.
