# TODO

## Kart Still Oversteers with YOLO Pipeline (Simulation)

**Status:** CameraInfo intrinsics fixed, midpoint-angle steering implemented, but kart still doesn't drive properly.

**What was fixed (2026-02-22):**

- `camera_info_fix_node.py` corrects Gazebo's wrong intrinsics (FX 277→381.5, CX 160→320, CY 120→180) — verified working
- `cone_follower_node.py` rewritten: optical→camera_link frame conversion, midpoint-angle steering instead of PID-on-Y-offset
- `simulation.launch.py` updated: bridge CameraInfo→`_raw`, fix node republishes corrected version

**What's still broken:**

- Steering saturates at -0.5 rad (max right) continuously — kart doesn't recover
- `steering_gain=2.0` is too aggressive: any midpoint angle >14° saturates the steering clamp
- The midpoint angle is consistently ~19-45° in testing, so it's always clamped

**Next steps to try:**

1. Lower `steering_gain` to ~1.0 and retest
2. Check if the kart has already drifted off-course by the time the first YOLO detections arrive (cone_follower starts at t=6s, detections may come later)
3. Verify depth image quality — cone forward distances seem short (~1-3m for cones that should be ~5m away), which could be a depth image issue separate from CameraInfo
4. Consider adding a ramp-up delay or pausing Gazebo until all nodes are ready

## Rebuild kart_brain on Orin

**Status:** Pending — build failed due to missing `ackermann_msgs` (now installed).

**What's needed:**
```bash
source /opt/ros/humble/setup.bash && cd ~/kart_brain && colcon build
```
`ros-humble-ackermann-msgs` was installed but the build hasn't been re-run yet.

## AnyDesk Remote Access

**Status:** Not working — Orin shows "Client Offline" from Mac.

**Problem:** AnyDesk service is running and ID is `721489674`, but the Orin can't reach AnyDesk relay servers. The Xorg config (`/etc/X11/xorg.conf.d/10-virtual-display.conf` with `ConnectedMonitor DFP-0`) was added but needs a reboot to take effect (X server must restart to load it).

**What's needed:**
1. Power on the Orin (reboot will apply the Xorg config)
2. Verify AnyDesk comes online: connect from Mac using ID `721489674`
3. If still offline, check if the university network (Robots_urjc) blocks AnyDesk relay traffic
4. Fallback: use SSH (`ssh orin`) — always works on the same network

## Fix PyTorch CUDA Support on Orin

**Status:** Blocked — Jetson AI Lab wheel index (`pypi.jetson-ai-lab.dev`) is unreachable (DNS doesn't resolve from either Orin or Mac as of 2026-02-22).

**Problem:** PyTorch installed as `torch 2.10.0+cpu` (CPU-only) because pip fell back to PyPI when the Jetson-specific index was down. `torch.cuda.is_available()` returns `False`.

**What's needed:**
1. Wait for `pypi.jetson-ai-lab.dev` to come back online, then:
   ```bash
   pip3 install --no-cache-dir --force-reinstall \
     --extra-index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 \
     torch torchvision
   pip3 install --no-cache-dir 'numpy<2'  # force-reinstall may pull numpy>=2
   ```
2. Verify: `python3 -c "import torch; print(torch.cuda.is_available())"` → `True`
3. If the index stays down, look for alternative Jetson PyTorch wheels at https://developer.download.nvidia.com/compute/redist/jp/ or build from source

**Important:** Always pin `numpy<2` after any torch reinstall — torch's dependencies may pull numpy 2.x which breaks `cv2` and `pyzed`.

## Install Gazebo Simulation on Orin

**Status:** Ready — NVMe is now root with 419 GB free.

**What's needed:**
```bash
sudo apt install ros-humble-ros-gz  # ~3-4 GB with all dependencies
```

**Once installed:**
- All simulation code is already in `src/kart_sim/` (worlds, models, launch files)
- See `.agents/simulation.md` for how to run
- The sim was developed and tested in a UTM VM (see `.agents/vm_environment.md`)

## ESP32 Communication

**Status:** Not started on Orin

**What's needed:**
1. Explore `src/kb_coms_micro/` package (ROS2 serial comms to ESP32)
2. Check `kart_medulla` firmware at `~/Desktop/kart_medulla` (ESP32, PlatformIO)
3. Understand actuation protocol — see `docs/ACTUATION_PROTOCOL.md`
4. Wire ESP32 via USB serial (`/dev/ttyTHS1` or USB)
5. Test sending steering + throttle commands via ROS2 topics

## Document Hardware/Software Setup

**Status:** Done

**Context:** The [UM-Driverless repo](https://github.com/UM-Driverless/driverless) documents the same physical kart hardware but with a different software stack (Xavier NX, KVASER CAN, FSDS simulator). That info was used as reference to document our current setup (AGX Orin, ESP32/UART, Gazebo).

**What was done:**
1. UM-Driverless repo studied — hardware details extracted and adapted for our current setup
2. Hardware overview table added to this repo's README
3. Extensive docs in `kart_docs/`:
    - [Assembly overview](https://um-driverless.github.io/kart_docs/assembly/) — hardware overview table with all subsystems
    - [Getting Started](https://um-driverless.github.io/kart_docs/software/ros2/getting-started/) — full sim + real hardware setup guide
    - [Architecture](https://um-driverless.github.io/kart_docs/software/ros2/architecture/) — system diagrams and topic maps
    - [Orin Setup](https://um-driverless.github.io/kart_docs/assembly/electronics/orin-setup/) — complete flash + software install guide

## Investigate Zombie/Stale Process Accumulation

**Status:** Not started

**Problem:** ROS2/Gazebo processes accumulate over time (zombie processes, orphaned nodes, leftover `ign gazebo` or `ros2` processes from previous runs). This clutters the system and can cause port conflicts or resource issues.

**What's needed:**
1. Investigate which processes are leaking — ROS2 nodes, Gazebo, bridge, YOLO, etc.
2. Figure out why they aren't cleaned up on shutdown (missing signal handling? launch file issues?)
3. Consider solutions: cleanup script on start, proper `on_exit` handlers in launch files, a wrapper script that kills stale processes before launching, or a systemd-style approach
4. Implement and document whatever works

## Create Reproducible Setup Script / Guide for Orin

**Status:** Not started

**Goal:** Make it easy to set up a fresh Orin (or reinstall) by creating either an install script, a detailed step-by-step doc, or both — written so an AI agent can follow it autonomously.

**What to cover:**
1. Flash JetPack / base OS
2. Install ROS2 Humble + colcon
3. Install Jetson-specific PyTorch + torchvision (CUDA wheels, numpy<2 pin)
4. Install ZED SDK + ROS2 wrapper
5. Clone and build `kart_brain` (including `ackermann_msgs` and other deps)
6. Install Gazebo Fortress + `ros-humble-ros-gz` (optional, for sim)
7. AnyDesk / remote access setup
8. Any system config (udev rules for ESP32 serial, network, etc.)

**Format options (pick when starting):**
- `setup_orin.sh` script that does everything non-interactively
- A detailed `.md` doc in `kart_docs/` or `.agents/` written for AI agents to follow
- Both: script for the happy path, doc for context and troubleshooting

**Reference:** Consolidate info already scattered across this TODO, `.agents/` docs, and `kart_docs/`.

## Long-Term

- Explore YOLO acceleration via ONNX/TensorRT on Jetson
- Set up ZED ROS2 wrapper for proper depth + 3D cone localization (instead of webcam mode)
- Trajectory planning from cone positions
- Full autonomous loop: camera → detection → planning → actuation
