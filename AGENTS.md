# Agent Quick Reference

## Key Files (Read Before Working)
Before making any changes to the kart_sw workspace, consult:
1. **`.agents/architecture.md`** — System architecture, packages, topic map, message types
2. **`.agents/simulation.md`** — Gazebo simulation setup, known issues, how to run
3. **`.agents/error_log.md`** — Past errors and added preventions
4. **`.agents/vm_environment.md`** — UTM VM specifics, SSH, sudo, installed software

## Environment
- **Dev machine:** macOS, SSH into UTM VM via `ssh utm` (192.168.65.2)
- **VM:** Ubuntu 22.04 arm64, 8GB RAM, 4 cores, no GPU (LLVMpipe software rendering)
- **ROS 2:** Humble (desktop install)
- **Simulator:** Gazebo Fortress (ign-gazebo 6.16.0) via `ros-humble-ros-gz`
- **Workspace:** `~/kart_sw/` (colcon workspace, git repo)
- **sudo password:** `0` (pipe via `echo "0" | sudo -S <cmd>`)

## Build & Run
```bash
# Build everything
source /opt/ros/humble/setup.bash
cd ~/kart_sw && colcon build
source install/setup.bash

# Build single package
colcon build --packages-select kart_sim

# Run simulation (perfect perception + cone follower)
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix kart_sim)/share/kart_sim/models
ros2 launch kart_sim simulation.launch.py

# Run simulation with YOLO pipeline instead
ros2 launch kart_sim simulation.launch.py use_perception:=true use_perfect_perception:=false
```

## Key Paths
| Path | Description |
|---|---|
| `~/kart_sw/src/kart_sim/` | Gazebo simulation package (this session's work) |
| `~/kart_sw/src/kart_perception/` | Perception pipeline (YOLO + depth + viz) |
| `~/kart_sw/src/kart_bringup/` | Launch files and config for real hardware |
| `~/kart_sw/src/joy_to_cmd_vel/` | Joystick teleop (C++) |
| `~/kart_sw/src/msgs_to_micro/` | ESP32 serial comms (C++) |
| `~/kart_sw/models/perception/yolo/best_adri.pt` | YOLO weights |

## Topic Map (Simulation Mode)
```
Gazebo Server
  ├─→ /kart/rgbd/image          ──bridge──→ /zed/zed_node/rgb/image_rect_color
  ├─→ /kart/rgbd/depth_image    ──bridge──→ /zed/zed_node/depth/depth_registered
  ├─→ /kart/rgbd/camera_info    ──bridge──→ /zed/zed_node/rgb/camera_info
  ├─→ /world/fs_track/clock     ──bridge──→ /clock
  └─→ /model/kart/odometry      ──bridge──→ /model/kart/odometry

Perfect Perception Node (or YOLO pipeline)
  └─→ /perception/cones_3d      (Detection3DArray)

Cone Follower Node
  ├─← /perception/cones_3d
  └─→ /kart/cmd_vel             ──bridge──→ Gazebo Ackermann plugin
```

## Cone Class IDs
Used in `Detection3DArray.detections[].results[].hypothesis.class_id`:
- `blue_cone` — left track boundary
- `yellow_cone` — right track boundary
- `orange_cone` — start/finish markers
- `large_orange_cone` — large start/finish markers

## Commit Protocol
1. `git status` — check what will be committed
2. `git diff --cached` — review changes
3. Build and verify before committing
4. If a recurring mistake happens, document it in `.agents/error_log.md`
