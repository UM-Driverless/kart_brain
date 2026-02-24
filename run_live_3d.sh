#!/bin/bash
# Full 3D pipeline: ZED native → YOLO → depth localizer → cone follower → steering HUD
export LD_LIBRARY_PATH=/home/orin/.local/lib/python3.10/site-packages/nvidia/nvjitlink/lib:$LD_LIBRARY_PATH
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export PATH=/usr/local/cuda-12.6/bin:$PATH
cd ~/kart_brain
source /opt/ros/humble/setup.bash
source install/setup.bash

# ZED wrapper (publishes RGB, depth, camera_info natively)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 &

# YOLO detector (on ZED RGB)
ros2 run kart_perception yolo_detector --ros-args \
  -p image_topic:=/zed/zed_node/rgb/image_rect_color \
  -p detections_topic:=/perception/cones_2d \
  -p debug_image_topic:=/perception/yolo/annotated \
  -p weights_path:=models/perception/yolo/best_adri.pt &

# 3D cone localizer (depth + 2D detections → 3D)
ros2 run kart_perception cone_depth_localizer --ros-args \
  -p detections_topic:=/perception/cones_2d \
  -p depth_topic:=/zed/zed_node/depth/depth_registered \
  -p camera_info_topic:=/zed/zed_node/rgb/camera_info \
  -p output_topic:=/perception/cones_3d &

# Cone follower (steering controller)
ros2 run kart_sim cone_follower --ros-args \
  -p detections_topic:=/perception/cones_3d \
  -p cmd_vel_topic:=/kart/cmd_vel &

# Steering HUD overlay
ros2 run kart_perception steering_hud --ros-args \
  -p annotated_topic:=/perception/yolo/annotated \
  -p cones_3d_topic:=/perception/cones_3d \
  -p cmd_vel_topic:=/kart/cmd_vel \
  -p camera_info_topic:=/zed/zed_node/rgb/camera_info \
  -p output_topic:=/perception/hud &

sleep 30
echo 'Opening HUD viewer...'
ros2 run rqt_image_view rqt_image_view /perception/hud &
wait
