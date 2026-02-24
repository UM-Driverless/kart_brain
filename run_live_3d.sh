#!/bin/bash
# Live pipeline: ZED (webcam mode) → YOLO → HUD
# For full 3D (depth + cone follower), the ZED ROS2 wrapper is needed.
# Include all pip-installed NVIDIA libs (nvjitlink, cusparselt, cudss, cublas, etc.)
NVIDIA_LIBS=$(find ~/.local/lib/python3.10/site-packages/nvidia -name "lib" -type d 2>/dev/null | tr "\n" ":")
export LD_LIBRARY_PATH="${NVIDIA_LIBS}${LD_LIBRARY_PATH}"
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export PATH=/usr/local/cuda-12.6/bin:$PATH
cd ~/kart_brain
source /opt/ros/humble/setup.bash
source install/setup.bash

# Image source (ZED as webcam, left eye only)
ros2 run kart_perception image_source --ros-args \
  -p source:=/dev/video0 \
  -p stereo_crop:=true \
  -p publish_rate:=15.0 \
  -p image_topic:=/image_raw &

# YOLO detector
ros2 run kart_perception yolo_detector --ros-args \
  -p image_topic:=/image_raw \
  -p detections_topic:=/perception/cones_2d \
  -p debug_image_topic:=/perception/yolo/annotated \
  -p weights_path:=models/perception/yolo/nava_yolov11_2026_02.pt &

# Steering HUD overlay (shows HUD even without 3D cones)
ros2 run kart_perception steering_hud --ros-args \
  -p annotated_topic:=/perception/yolo/annotated \
  -p output_topic:=/perception/hud &

sleep 30
echo 'Opening HUD viewer...'
ros2 run rqt_image_view rqt_image_view /perception/hud &
wait
