# System Architecture

## Workspace Structure

```
~/kart_brain/                          (colcon ROS 2 workspace)
├── AGENTS.md                       ← Agent entry point
├── .agents/                        ← This directory
├── src/
│   ├── kart_sim/                   (ament_cmake) Gazebo simulation
│   │   ├── worlds/fs_track.sdf    44-cone oval track
│   │   ├── models/kart/           Ackermann kart + RGBD camera
│   │   ├── models/cone_{blue,yellow,orange}/
│   │   ├── scripts/
│   │   │   ├── perfect_perception_node.py
│   │   │   └── cone_follower_node.py
│   │   └── launch/simulation.launch.py
│   │
│   ├── kart_perception/            (ament_python) Perception pipeline
│   │   ├── kart_perception/
│   │   │   ├── yolo_detector_node.py         YOLO 2D detection
│   │   │   ├── cone_depth_localizer_node.py  Depth → 3D projection
│   │   │   ├── cone_marker_viz_3d_node.py    RViz markers
│   │   │   ├── cone_marker_viz_node.py       2D markers (legacy)
│   │   │   └── image_source_node.py          File/video publisher
│   │   └── launch/
│   │       ├── perception_3d.launch.py       Full 3D pipeline
│   │       └── perception_test.launch.py     Offline testing
│   │
│   ├── kart_bringup/               (ament_cmake) Hardware launch files
│   │   ├── launch/teleop_launch.py
│   │   └── config/teleop_params.yaml
│   │
│   ├── joy_to_cmd_vel/             (ament_cmake, C++) Joystick → Twist
│   ├── msgs_to_micro/              (ament_cmake, C++) ROS → ESP32 serial
│   └── ThirdParty/
│
├── models/perception/yolo/best_adri.pt   YOLO weights (YOLOv5 custom)
├── test_data/driverless_test_media/      Test images/videos
├── scripts/                              Workspace utility scripts
├── build/ install/ log/                  colcon output (gitignored)
└── pyproject.toml
```

## Node Graph

### Simulation Mode (kart_sim)

```
┌─────────────────────────────────────────────────────────┐
│  Gazebo Fortress (ign gazebo -s --headless-rendering)   │
│                                                         │
│  World: fs_track.sdf                                    │
│  ├── ground_plane                                       │
│  ├── sun (no shadows)                                   │
│  ├── kart (AckermannSteering + RGBD camera)             │
│  └── 44 cone models (static cylinders)                  │
│                                                         │
│  Publishes (Ignition topics):                           │
│    /kart/rgbd/image, /depth_image, /camera_info         │
│    /model/kart/odometry                                 │
│    /world/fs_track/clock                                │
│  Subscribes:                                            │
│    /kart/cmd_vel (Twist → AckermannSteering)            │
└────────────────────┬────────────────────────────────────┘
                     │ ros_gz_bridge
                     ▼
┌─────────────────────────────────────────────────────────┐
│  ROS 2 Topics                                           │
│                                                         │
│  /zed/zed_node/rgb/image_rect_color  (remapped)         │
│  /zed/zed_node/depth/depth_registered (remapped)        │
│  /zed/zed_node/rgb/camera_info       (remapped)         │
│  /model/kart/odometry                                   │
│  /clock                              (remapped)         │
│  /kart/cmd_vel                       (ROS→Gazebo)       │
└────────────────────┬────────────────────────────────────┘
                     │
        ┌────────────┴────────────┐
        ▼                         ▼
┌──────────────────┐   ┌──────────────────────┐
│ Perfect Percep.  │   │ YOLO Pipeline        │
│ (ground truth)   │   │ (camera-based)       │
│                  │   │                      │
│ Reads SDF cones  │   │ yolo_detector        │
│ + odom → 3D det  │   │ → cone_depth_local.  │
│                  │   │                      │
│ Publishes:       │   │ Publishes:           │
│ /perception/     │   │ /perception/         │
│   cones_3d       │   │   cones_3d           │
└────────┬─────────┘   └──────────┬───────────┘
         │    (one or the other)  │
         └────────────┬───────────┘
                      ▼
         ┌────────────────────────┐
         │  Cone Follower         │
         │                        │
         │  Subscribes:           │
         │    /perception/cones_3d│
         │  Publishes:            │
         │    /kart/cmd_vel       │
         │                        │
         │  Algorithm:            │
         │  1. Separate blue/     │
         │     yellow cones       │
         │  2. Find nearest pair  │
         │  3. Steer to midpoint  │
         │  4. Speed ∝ straightness│
         └────────────────────────┘
```

### Real Hardware Mode (kart_bringup)

```
ZED Camera → /zed/zed_node/rgb/image_rect_color
           → /zed/zed_node/depth/depth_registered
           → /zed/zed_node/rgb/camera_info

Perception pipeline (same nodes, same topics)
  → /perception/cones_3d

Controller (cone_follower or future planner)
  → /actuation_cmd (AckermannDriveStamped)

msgs_to_micro → ESP32 serial → actuators
```

## Message Types

| Topic | ROS 2 Type | Key Fields |
|---|---|---|
| `/perception/cones_2d` | `vision_msgs/Detection2DArray` | bbox center, class_id, score |
| `/perception/cones_3d` | `vision_msgs/Detection3DArray` | 3D position, class_id, score |
| `/kart/cmd_vel` | `geometry_msgs/Twist` | linear.x (speed), angular.z (steering) |
| `/model/kart/odometry` | `nav_msgs/Odometry` | pose (position + orientation), twist |
| Camera topics | `sensor_msgs/Image` | RGB 640x360, Depth 32FC1 |
| `/zed/.../camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (fx, fy, cx, cy) |

## Cone Class IDs (String Constants)

These strings are used everywhere — in YOLO class names, Detection messages, and visualization:

| Class ID | Color | Role | YOLO class name |
|---|---|---|---|
| `blue_cone` | Blue (0.1, 0.3, 1.0) | Left track boundary | Same |
| `yellow_cone` | Yellow (1.0, 0.9, 0.1) | Right track boundary | Same |
| `orange_cone` | Orange (1.0, 0.5, 0.1) | Start/finish markers | Same |
| `large_orange_cone` | Dark orange (1.0, 0.3, 0.0) | Large start/finish | Same |

## Track Layout (fs_track.sdf)

Oval track centered at (0, 0) in world coordinates:
- **Right straight:** x=20, y from -10 to +10 (blue at x=18.5, yellow at x=21.5)
- **Left straight:** x=-20, y from +10 to -10 (blue at x=-18.5, yellow at x=-21.5)
- **Top curve:** semicircle center (0, 10), radius 18.5 (blue inner) / 21.5 (yellow outer)
- **Bottom curve:** semicircle center (0, -10), same radii
- **Start/finish:** 4 orange cones at y≈0 on right straight
- **Kart spawn:** (20, 0) facing +Y (yaw = π/2), drives counterclockwise

Track width: 3m. Cone spacing: ~5m on straights, ~8m on curves.
