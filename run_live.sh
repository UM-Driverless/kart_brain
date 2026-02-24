#!/bin/bash
export LD_LIBRARY_PATH=/home/orin/.local/lib/python3.10/site-packages/nvidia/nvjitlink/lib:/home/orin/.local/lib/python3.10/site-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export PATH=/usr/local/cuda-12.6/bin:$PATH
cd ~/kart_brain
source /opt/ros/humble/setup.bash
source install/setup.bash

# Image source (ZED as webcam, left eye only)
ros2 run kart_perception image_source --ros-args   -p source:=/dev/video0   -p stereo_crop:=true   -p publish_rate:=10.0   -p image_topic:=/image_raw &

# YOLO detector
ros2 run kart_perception yolo_detector --ros-args   -p image_topic:=/image_raw   -p detections_topic:=/perception/cones_2d   -p debug_image_topic:=/perception/yolo/annotated   -p weights_path:=models/perception/yolo/best_adri.pt &

sleep 25
echo 'Opening viewer...'
ros2 run rqt_image_view rqt_image_view /perception/yolo/annotated &
wait
