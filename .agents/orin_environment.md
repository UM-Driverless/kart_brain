# Jetson Orin Environment

## Hardware
| Property | Value |
|---|---|
| Board | NVIDIA Jetson Orin (AGX) |
| Architecture | aarch64 (ARM64) |
| RAM | 62 GB |
| CPUs | 12 cores |
| GPU | Ampere (CUDA 12.6) |
| Storage | 476 GB NVMe SSD (root filesystem) + 57 GB eMMC (bootloader only) |
| Display | DisplayPort only (no HDMI). DP-to-HDMI adapter + dummy plug for AnyDesk |
| Camera | ZED 2 stereo (USB, SN 21983349) |
| CAN bus | `can0`, `can1` interfaces (for ESP32 comms) |

## Access
```bash
ssh orin          # WiFi (10.7.20.142, DHCP — may change)
ssh orin_wire     # Wired (10.0.255.177, ethernet must be connected)
# AnyDesk for GUI (needs dummy HDMI plug)
# sudo password: 0
```

For non-interactive sudo:
```bash
ssh orin 'echo "0" | sudo -S <command>'
```

## Software
| Software | Version / Path |
|---|---|
| OS | Ubuntu 22.04 (L4T R36.5, JetPack 6.2.2) |
| ROS 2 | Humble (full desktop + vision_msgs + dev tools) |
| CUDA | 12.6 (via nvidia-jetpack) |
| cuDNN | 9.3 (via nvidia-jetpack) |
| TensorRT | 10.3 (via nvidia-jetpack) |
| PyTorch | 2.10.0 (CUDA works with LD_LIBRARY_PATH set — see Environment Setup) |
| Gazebo | Fortress 6.16.0 (`ros-humble-ros-gz`) |
| ZED SDK | 4.2 (`/usr/local/zed/`, L4T 36.4 build, compatible with L4T 36.5) |
| Python | 3.10.12 (system) |
| numpy | 1.26.4 (must be <2, cv2 compiled against numpy 1.x) |
| ultralytics | 8.4.14 |
| AnyDesk | Installed |

## Environment Setup

The following are already in `~/.bashrc` and sourced automatically on login/terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/kart_brain/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix kart_sim 2>/dev/null)/share/kart_sim/models
```

**Not in `.bashrc`** — must be set manually when running PyTorch/YOLO (the `run_live_3d.sh` script handles this):
```bash
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/targets/aarch64-linux/lib:$(find ~/.local/lib/python3.10/site-packages/nvidia -name "lib" -type d 2>/dev/null | tr "\n" ":"):$LD_LIBRARY_PATH
```

**Note:** After `colcon build`, you need to re-source `install/setup.bash` (or open a new terminal) for changes to take effect.

## Workspace
| Path | Description |
|---|---|
| `/home/orin/kart_brain` | Main ROS2 workspace (this repo) |
| `~/Desktop/KART_SW` | Old copy of kart_sw (can be deleted) |
| `~/Desktop/kart_medulla` | ESP32 firmware (PlatformIO) — **not present**, needs cloning from https://github.com/UM-Driverless/kart_medulla |

## ZED Camera
- USB device: `2b03:f780 STEREOLABS ZED 2`
- Appears as `/dev/video0` + `/dev/video1` when connected
- **OpenCV webcam mode**: 1344x376 stereo (left+right side by side). With `stereo_crop=true`, left eye only = 672x376
- **ZED SDK mode** (`pyzed.sl`): Full HD + depth maps
- May need re-plugging after reboot
- Calibration file already downloaded for SN 21983349

## Live Perception Pipeline
```bash
# All-in-one script
~/kart_brain/run_live.sh

# Or manually:
ros2 run kart_perception image_source --ros-args \
  -p source:=/dev/video0 -p stereo_crop:=true -p publish_rate:=10.0 &
ros2 run kart_perception yolo_detector --ros-args \
  -p weights_path:=models/perception/yolo/best_adri.pt &
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  ros2 run rqt_image_view rqt_image_view /perception/yolo/annotated &
```

## Known Issues
1. **torch needs LD_LIBRARY_PATH** set for `libnvJitLink.so.12` (see Environment Setup above)
2. **numpy must be <2** — cv2 was compiled against numpy 1.x, numpy 2 breaks it
3. **ZED camera may need re-plug** after reboot
4. **Webcam mode resolution is low** (672x376 after stereo crop). Consider ZED SDK for full HD + depth
5. **AnyDesk display**: Requires Xorg config at `/etc/X11/xorg.conf.d/10-virtual-display.conf` with `Option "ConnectedMonitor" "DFP-0"` to force a framebuffer on the DisplayPort output. Without this, the NVIDIA driver sees DFP-0 and DFP-1 as "disconnected" (dummy plug via DP-to-HDMI adapter doesn't provide proper EDID), so Xorg has no screen and AnyDesk gets a black framebuffer.
6. **PyTorch is CPU-only**: `pypi.jetson-ai-lab.dev` was unreachable during install. Needs CUDA-enabled wheel. See TODO.md.
7. **ZED SDK installer breaks pip permissions**: The installer runs pip as root, leaving `.dist-info` dirs with bad permissions. Fix: `sudo chmod -R a+rX /usr/local/lib/python3.10/dist-packages/`
