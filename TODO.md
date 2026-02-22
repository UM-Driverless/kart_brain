# TODO

## Kart Still Oversteers with YOLO Pipeline

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

## Long-Term Tasks

- Explore YOLO acceleration via ONNX/TensorRT on Jetson (C++ node optional).
