# Kart SW — Claude Code Instructions

Read `.agents/` documentation before making changes. Start with `AGENTS.md` for the quick reference.

## Critical Rules
- **Gazebo Fortress uses `ign` CLI**, not `gz`. Message types are `ignition.msgs.*`, not `gz.msgs.*`.
- **No `<cone>` geometry** in SDF — use `<cylinder>` instead (Fortress limitation).
- **Odom is relative to spawn** — always account for the kart's initial world position.
- **sudo password is `0`** — use `echo "0" | sudo -S <cmd>` for non-interactive SSH.
- **No GPU on the VM** — keep camera resolution at 640x360, disable shadows, use headless rendering.
- **Cone class IDs:** `blue_cone`, `yellow_cone`, `orange_cone`, `large_orange_cone` — must be consistent across YOLO, perception, and control nodes.
- When something goes wrong, document it in `.agents/error_log.md`.

## Build
```bash
source /opt/ros/humble/setup.bash
cd ~/kart_sw && colcon build && source install/setup.bash
```

## Run Simulation
```bash
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix kart_sim)/share/kart_sim/models
ros2 launch kart_sim simulation.launch.py
```
