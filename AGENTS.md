# Agent Quick Reference

## Key Files (Read Before Working)
Before making any changes to the kart_brain workspace, consult:
1. **`.agents/architecture.md`** — System architecture, packages, topic map, message types
2. **`.agents/error_log.md`** — Past errors and added preventions
3. **Environment-specific guide:**
   - Working on **real hardware (Orin)?** → `.agents/orin_environment.md`
   - Working on **simulation (VM)?** → `.agents/simulation.md` + `.agents/vm_environment.md`

## Environments

### Jetson Orin (Real Hardware)
- **Connection:** `ssh orin` (WiFi 10.7.20.142) or AnyDesk
- **Workspace:** `/mnt/data/kart_brain`
- **Camera:** ZED 2 stereo (USB)
- **sudo password:** `0`
- **Full details:** `.agents/orin_environment.md`

### UTM VM (Simulation)
- **Connection:** `ssh utm` (192.168.65.2)
- **Workspace:** `~/kart_sw/`
- **Simulator:** Gazebo Fortress (headless, CPU rendering)
- **sudo password:** `0`
- **Full details:** `.agents/vm_environment.md`

## Build & Run
```bash
# Build everything
source /opt/ros/humble/setup.bash
cd /mnt/data/kart_brain && colcon build
source install/setup.bash

# Build single package
colcon build --packages-select kart_perception

# Live perception on Orin
/mnt/data/kart_brain/run_live.sh

# Simulation in VM
ros2 launch kart_sim simulation.launch.py
```

## Key Paths
| Path | Description |
|---|---|
| `src/kart_perception/` | Perception pipeline (YOLO + depth + viz) |
| `src/kart_sim/` | Gazebo simulation package |
| `src/kart_bringup/` | Launch files and config for real hardware |
| `src/joy_to_cmd_vel/` | Joystick teleop (C++) |
| `src/msgs_to_micro/` | ESP32 serial comms (C++) |
| `models/perception/yolo/best_adri.pt` | YOLO weights |

## Cone Class IDs
Used everywhere — YOLO class names, Detection messages, visualization:
- `blue_cone` — left track boundary
- `yellow_cone` — right track boundary
- `orange_cone` — start/finish markers
- `large_orange_cone` — large start/finish markers

## Commit Protocol
1. `git status` — check what will be committed
2. `git diff --cached` — review changes
3. Build and verify before committing
4. If a mistake occurred, document it in `.agents/error_log.md`
5. If recurring, create a postmortem in `.agents/postmortems/`
