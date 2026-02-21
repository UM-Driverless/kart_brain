# Error Log

Tracks mistakes made during development and the prevention mechanisms added. Every recurring or painful error should be documented here.

## Format
```markdown
## YYYY-MM-DD - Brief title
**What happened:** Description of the error
**Prevention added:**
- List of changes made
```

---

## 2026-02-21 - Cone geometry invisible in Gazebo Fortress
**What happened:** Used `<cone>` SDF geometry for track cones. Gazebo Fortress (SDF 1.6) does not support cone geometry — it silently renders nothing and logs `Geometry type [0] not supported` for every cone (44 errors). The cones were invisible in the sim.
**Prevention added:**
- Replaced all `<cone>` with `<cylinder>` geometry in world SDF and model files
- Documented in `.agents/simulation.md` under "Known Issues"
- Rule: Only use geometries supported by Fortress: box, sphere, cylinder, capsule, ellipsoid, plane, mesh

## 2026-02-21 - Odometry at (0,0) despite kart spawned at (20,0)
**What happened:** The `perfect_perception_node.py` used raw odometry (x,y) as world position. But Gazebo odometry is relative to the spawn pose — it reports (0,0) at startup regardless of world placement. All cones appeared >20m away, so zero detections.
**Prevention added:**
- Added `kart_start_x`, `kart_start_y`, `kart_start_yaw` parameters to `perfect_perception_node.py`
- The node transforms odom into world frame: `world_pos = start_pos + rotate(odom_pos, start_yaw)`
- These parameters MUST match the `<pose>` in `fs_track.sdf` (currently: 20, 0, yaw=1.5708)
- Documented in `.agents/simulation.md`

## 2026-02-21 - Kart drifts 1000+ meters during Gazebo startup
**What happened:** With `real_time_factor: 0` (unlimited speed), Gazebo simulated thousands of seconds during the ~12s wall-clock startup. Even tiny physics artifacts (wheel clipping ground) accumulated into massive odometry drift. By the time the controller started, the kart was kilometers off-track.
**Prevention added:**
- Changed physics to `real_time_factor: 1` in `fs_track.sdf`
- Alternatively: start Gazebo paused (omit `-r`), start all nodes, then unpause via `ign service`
- Rule: Never use `real_time_factor: 0` unless all control nodes are already running

## 2026-02-21 - Steering joints log velocity control warnings
**What happened:** Gazebo logged `Velocity control does not respect positional limits` for steering joints that had position limits but no effort limits.
**Prevention added:**
- Added `<effort>1e6</effort>` to both steering joint limits in `kart/model.sdf`

## 2026-02-21 - Wheels start below ground plane
**What happened:** With the kart model at z=0.15, wheel centers were at z=0.10, and with radius 0.15, wheel bottoms were at z=-0.05 — embedded in the ground. This caused physics jitter.
**Prevention added:**
- Increased model z-offset to 0.22 so wheel bottoms are at z≈0.02 (above ground)
- Rule: When changing wheel radius or chassis height, verify `model_z - wheel_z_offset - wheel_radius > 0`

## 2026-02-21 - sudo requires password over non-interactive SSH
**What happened:** `ssh utm "sudo apt install ..."` failed because sudo needs a TTY for password input. `-t` flag doesn't help from non-interactive contexts.
**Prevention added:**
- Use `ssh utm 'echo "0" | sudo -S <command>'` for all sudo operations
- Documented in `.agents/vm_environment.md`
