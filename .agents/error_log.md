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

## 2026-02-22 - BGR/RGB swap in YOLO annotated image
**What happened:** The live camera feed in rqt_image_view showed inverted colors (blue sky appeared orange). Root cause: `image_source_node` publishes BGR (from OpenCV), `yolo_detector_node` converts to RGB for inference, then `results.render()` draws on the RGB buffer, but the result was published with `encoding="bgr8"` without converting back.
**Prevention added:**
- Added `cv2.cvtColor(rendered[0], cv2.COLOR_RGB2BGR)` before publishing the debug image in `yolo_detector_node.py`
- Rule: **OpenCV uses BGR, ROS Image with "bgr8" expects BGR, but YOLO/PIL/matplotlib work in RGB.** Whenever passing images between these systems, always verify the channel order matches the declared encoding. If you convert to RGB for inference, convert back to BGR before publishing as "bgr8".

## 2026-02-22 - Duplicate YOLO detector processes consuming all GPU
**What happened:** Multiple `yolo_detector` instances were running (from both `run_live.sh` and manual launches), each consuming ~500-800% CPU and GPU memory. The system was sluggish.
**Prevention added:**
- Rule: Before launching nodes, always check for existing instances: `ps aux | grep yolo_detector | grep -v grep`
- Rule: Kill stale processes before launching new ones. Use `kill -9` if SIGTERM doesn't work within a few seconds.

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
- Use `ssh <host> 'echo "0" | sudo -S <command>'` for all sudo operations
- Documented in `.agents/vm_environment.md` and `.agents/orin_environment.md`

## 2026-02-22 - Wrong IP for y540 laptop, wasted time on SSH setup
**What happened:** The laptop (y540) was given IP 10.7.20.136 but DHCP had assigned 10.7.20.138. Spent multiple attempts trying to connect to the wrong IP. Also referenced IPs without labeling which machine they belonged to, causing confusion.
**Prevention added:**
- Rule: **All machines on Robots_urjc use DHCP — IPs can change.** Always verify the current IP with `hostname -I` on the target machine before attempting SSH.
- Rule: **Always label IPs with the machine name** (e.g., "Mac at 10.7.20.28", "Orin at 10.7.20.142") — never mention bare IPs.
- Rule: **SSH config hostnames are the source of truth** (`ssh orin`, `ssh y540`). When an IP changes, update `~/.ssh/config` on the Mac.

## 2026-02-22 - SSH server not installed on fresh Ubuntu laptop
**What happened:** Tried to SSH into the y540 laptop but got "Connection refused" — openssh-server was not installed. Cannot install it remotely without SSH access.
**Prevention added:**
- Rule: When setting up a new machine for remote access, the **first step is always installing openssh-server** on it physically: `sudo apt install -y openssh-server`
