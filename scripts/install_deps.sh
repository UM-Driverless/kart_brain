#!/usr/bin/env bash
set -euo pipefail

# ROS 2 Humble workspace dependencies for this repo.
# Requires: Ubuntu 22.04 with ROS 2 Humble already installed.

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  ros-humble-ackermann-msgs \
  ros-humble-joy \
  ros-humble-zed-msgs
