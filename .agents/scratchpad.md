# Scratchpad — Simulation & Track Work (2026-03-01)

## Context: Running Claude Code on the UTM VM

### The problem
Claude Code runs on the Mac. The Gazebo simulator runs on the UTM VM (Ubuntu 22.04, GNOME on Wayland). Taking screenshots and launching Gazebo over SSH is painful:
- GNOME runs **Wayland**, so X11 tools (`scrot`, `xdotool`) can't see windows
- `gnome-screenshot` works but requires `DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus`
- If GNOME Activities overlay gets stuck, there's no reliable way to dismiss it over SSH
- Launching Gazebo via `nohup ... &` over SSH causes Ignition Transport disconnection (bridge sees `blues=0 yellows=0`). Must use `gnome-terminal --` to launch within the GNOME session.
- Camera sensor images don't publish without a display (headless EGL fails on llvmpipe)

### The plan
**Run Claude Code directly inside the UTM VM.** This eliminates all SSH/Wayland/DBUS issues:
- Same GNOME session → `gnome-screenshot` works natively
- Same terminal → Gazebo launches properly with full Ignition Transport connectivity
- Direct filesystem access → no SCP needed
- Can commit and push from the VM (git is already set up there)

### VM specs (should be enough)
- 8 GB RAM, 4 CPU cores
- ~17 GB free disk
- Node.js needed for Claude Code (install via `nvm` or `apt`)
- The workspace `~/kart_brain` is already there and built

### What to try
1. Install Claude Code on the VM
2. Open a terminal in UTM, run `claude`
3. Test: launch Gazebo, take screenshot, validate rendering
4. Edit code, commit, push — all from the VM

## Current state: Autocross track

### What's done
- **New autocross track created** — ~80x60m field, mix of left/right turns, S-chicane, long straights
- `scripts/sim2d/track.py` — `AUTOCROSS_TRACK` with 70 blue + 70 yellow + 4 orange cones
- `scripts/sim2d/generate_sdf.py` — generates SDF from track.py (keeps 2D sim and Gazebo in sync)
- `src/kart_sim/worlds/autocross_track.sdf` — generated, deployed to VM
- `src/kart_sim/launch/simulation.launch.py` — `autocross` entry added to `_TRACKS`
- **2D sim validated** — track plot viewed, smooth curves, no self-intersections
- **Gazebo validated by user** — user opened Gazebo from UTM GUI, shared screenshot showing cones rendered correctly
- **Controller runs** — logs showed `neural_v2` steering with `blues=5-7 yellows=4-6`, kart navigating the track

### What's NOT done
- **Gazebo screenshot by Claude** — blocked by Wayland/SSH issues (see above)
- **Neural controller retraining** — current weights are trained on the oval track. Need to retrain on autocross for optimal performance (the kart drives but may not handle all turns well)
- **`visualize.py` bug** — `GeometricController.control()` missing `current_speed` kwarg. Pre-existing, not blocking.

## What we want to do with the simulator

### Immediate goals
1. **Validate the neural_v2 controller on the autocross track in Gazebo** — visually confirm the kart completes laps without leaving the track. This requires taking a screenshot/video from Gazebo (blocked until Claude runs on VM or user provides screenshots).
2. **Retrain on autocross** — run `python train.py` with the autocross track in the 2D sim to get weights optimized for this layout. Current weights are oval-trained.
3. **Deploy retrained weights** — copy best weights to `src/kart_sim/config/neural_v2_weights.json`, rebuild on VM, test in Gazebo.

### Longer term
- Test on real hardware (Orin) once controller is validated in simulation
- Try the YOLO perception pipeline (`use_yolo:=true`) instead of perfect perception
- Consider adding more track layouts for robustness testing

---

## TODO
- [ ] **Document Claude Code on VM setup** once confirmed working (add to `vm_environment.md`)
- [ ] Retrain neural_v2 on autocross track
- [ ] Fix `visualize.py` GeometricController bug (low priority)

---

## Previous: YOLOv11 Migration (2026-02-25)

### Status
- YOLO + CUDA: **WORKING** (cuBLAS fix confirmed)
- Camera (`/dev/video0`): Needs USB reset or reconnection
- Full pipeline test with real camera: **NOT YET DONE**

### What's left
1. Test with real camera — USB reset the ZED, run pipeline, verify cone detection end-to-end
2. Verify colored labels — confirm per-class colors render correctly with YOLOv11 class names
3. Measure GPU FPS — benchmark with real frames
4. Check class name mapping — YOLOv11 model has class `unknown_cone` (id 3) that YOLOv5 didn't have
