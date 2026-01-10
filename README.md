# kart_sw

ROS workspace for the UM-Driverless kart software stack.

## Install (ROS 2 Humble)
Assumes Ubuntu 22.04 with ROS 2 Humble already installed and sourced.

```bash
./scripts/install_deps.sh
```

## Build
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Startup (no hardware)
This launches just the teleop processing node; you can publish fake `/joy` input.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run joy_to_cmd_vel joy_to_cmd_vel --ros-args --params-file src/kart_bringup/config/teleop_params.yaml
```

```bash
ros2 topic echo /actuation_cmd
```

```bash
ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 1.0, -1.0], buttons: [0,0,0,0,0,1]}"
```

## Startup (with hardware)
Requires a joystick at `/dev/input/js0` and the kart microcontroller on `/dev/ttyTHS1`.

```bash
ros2 launch kart_bringup teleop_launch.py
```

## References
- Kart Docs: https://github.com/UM-Driverless/kart_docs
- Kart Docs site: https://um-driverless.github.io/kart_docs/
- ROS 2 installation: https://docs.ros.org/en/rolling/Installation.html
- ROS 2 Humble docs: https://docs.ros.org/en/humble/

## Test Media
Pulled from https://github.com/UM-Driverless/driverless and stored locally at
`test_data/driverless_test_media`.

## YOLO Weights
Pulled from https://github.com/UM-Driverless/driverless and stored locally at
`models/perception/yolo/best_adri.pt`.

## YOLO Quick Test
Run YOLO on a test image or video and save annotated output.

```bash
python3 scripts/run_yolo_on_media.py \
  --source test_data/driverless_test_media/cones_test.png \
  --weights models/perception/yolo/best_adri.pt \
  --output outputs/yolo
```

First run will download the YOLOv5 code via Torch Hub and cache it locally.

## Running ROS Nodes
Launch the image source and YOLO detector nodes on test media.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select kart_perception
source install/setup.bash

ros2 launch kart_perception perception_test.launch.py \
  source:=test_data/driverless_test_media/cones_test.png \
  weights:=models/perception/yolo/best_adri.pt
```
