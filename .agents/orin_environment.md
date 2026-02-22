# Jetson Orin Environment

## Hardware
| Property | Value |
|---|---|
| Board | NVIDIA Jetson Orin (AGX) |
| Architecture | aarch64 (ARM64) |
| RAM | 62 GB |
| CPUs | 12 cores |
| GPU | Ampere (CUDA 12.6) |
| Storage | 57 GB eMMC (root) + 476 GB NVMe SSD at `/mnt/data` |
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
| OS | Ubuntu 22.04 (L4T R36.4, JetPack 6) |
| ROS 2 | Humble (`/opt/ros/humble/setup.bash`) |
| CUDA | 12.6 (`/usr/local/cuda-12.6/`) |
| PyTorch | 2.5.0a0+872d972e41.nv24.08 (Jetson build) |
| ZED SDK | 4.2.5 (`/usr/local/zed/`) |
| Python | 3.10.12 (system) |
| numpy | Must be <2 (cv2 compiled against numpy 1.x) |

## Environment Setup
```bash
# Required before running anything with PyTorch/YOLO
export LD_LIBRARY_PATH=/home/orin/.local/lib/python3.10/site-packages/nvidia/nvjitlink/lib:$LD_LIBRARY_PATH

# ROS 2 workspace
cd /mnt/data/kart_brain
source /opt/ros/humble/setup.bash && source install/setup.bash
```

## Workspace
| Path | Description |
|---|---|
| `/mnt/data/kart_brain` | Main ROS2 workspace (this repo) |
| `/mnt/data/driverless` | Legacy Python prototype |
| `~/Desktop/KART_SW` | Old copy of kart_sw |
| `~/Desktop/kart_medulla` | ESP32 firmware (PlatformIO) |

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
/mnt/data/kart_brain/run_live.sh

# Or manually:
ros2 run kart_perception image_source --ros-args \
  -p source:=/dev/video0 -p stereo_crop:=true -p publish_rate:=10.0 &
ros2 run kart_perception yolo_detector --ros-args \
  -p weights_path:=models/perception/yolo/best_adri.pt &
DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  ros2 run rqt_image_view rqt_image_view /perception/yolo/annotated &
```

## Known Issues
1. **Root filesystem nearly full** (~5 GB free). Install large packages to NVMe `/mnt/data`
2. **NVMe may not auto-mount** after reboot: `echo '0' | sudo -S mount /dev/nvme0n1p1 /mnt/data`
3. **torch needs LD_LIBRARY_PATH** set for `libnvJitLink.so.12` (see Environment Setup above)
4. **numpy must be <2** — cv2 was compiled against numpy 1.x, numpy 2 breaks it
5. **ZED camera may need re-plug** after reboot
6. **Webcam mode resolution is low** (672x376 after stereo crop). Consider ZED SDK for full HD + depth
7. **AnyDesk display**: Xorg forces DFP-0 (DisplayPort) via `/etc/X11/xorg.conf.d/10-virtual-display.conf`
