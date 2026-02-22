# Orin NVMe Flash & Setup Guide

Complete guide for flashing the Jetson AGX Orin to boot from NVMe SSD with JetPack 6.2.2 (L4T R36.5).

## Overview

| Item | Value |
|---|---|
| Target | Jetson AGX Orin Developer Kit |
| JetPack | 6.2.2 (L4T R36.5, released 2026-02-06) |
| Target OS | Ubuntu 22.04 |
| Flash host | y540 laptop (Ubuntu 24.04, x86_64) — `ssh y540` |
| Connection | USB-C from y540 → Orin flashing port (next to 40-pin GPIO header) |

## Prerequisites

### On the flash host (y540)

```bash
# Install flash dependencies
sudo apt-get install -y abootimg binfmt-support binutils cpio cpp \
  device-tree-compiler dosfstools lbzip2 libxml2-utils nfs-kernel-server \
  python3-yaml sshpass udev

# Download L4T R36.5 BSP and root filesystem
mkdir -p ~/jetson-flash && cd ~/jetson-flash
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/release/Jetson_Linux_r36.5.0_aarch64.tbz2
wget https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/release/Tegra_Linux_Sample-Root-Filesystem_r36.5.0_aarch64.tbz2
```

### Extract and prepare

```bash
cd ~/jetson-flash
tar xf Jetson_Linux_r36.5.0_aarch64.tbz2
sudo tar xpf Tegra_Linux_Sample-Root-Filesystem_r36.5.0_aarch64.tbz2 -C Linux_for_Tegra/rootfs/
cd Linux_for_Tegra/
sudo ./tools/l4t_flash_prerequisites.sh
sudo ./apply_binaries.sh
```

## Put Orin in Recovery Mode

The Orin must be in Force Recovery Mode (RCM) for flashing.

### If Orin is powered on:
1. Press and hold the **middle button** (Force Recovery)
2. While holding it, press and release the **Reset button** (leftmost)
3. Release the Force Recovery button after ~2 seconds

### If Orin is powered off:
1. Press and hold the **Force Recovery button** (middle, labeled 2)
2. Power on: plug USB-C power into port 4 (above DC jack), or plug DC into jack 5
3. If the white LED (0) is not lit, press the **Power button** (1)
4. Release both buttons

### Verify recovery mode
On the y540:
```bash
lsusb | grep -i nvidia
# Should show: 0955:7023 (recovery mode)
# NOT: 0955:7020 (normal mode)
```

### USB-C cable placement
The USB-C cable from the y540 goes to the Orin's **flashing port** — the USB-C port **next to the 40-pin GPIO header**, NOT the USB-C power port above the DC jack.

## Flash to NVMe

```bash
cd ~/jetson-flash/Linux_for_Tegra

# Flash to NVMe SSD (entire root filesystem on NVMe)
sudo ./tools/kernel_flash/l4t_initrd_flash.sh \
  --external-device nvme0n1p1 \
  -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
  --showlogs \
  -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
  --network usb0 \
  jetson-agx-orin-devkit \
  nvme0n1p1
```

This takes ~10-20 minutes. The Orin will reboot automatically when done.

## After First Boot

The Orin boots into Ubuntu 22.04 setup wizard (language, user, password). Complete it, then:

### Set up SSH access
On the Orin (via monitor/keyboard or AnyDesk):
```bash
sudo apt install -y openssh-server
```

Then from your Mac, copy your SSH key:
```bash
sshpass -p "<password>" ssh-copy-id -o StrictHostKeyChecking=accept-new orin@<new-ip>
```

Update `~/.ssh/config` on the Mac with the new IP if it changed.

### Install everything
Copy and run the setup script:
```bash
scp jetson-orin-setup.sh orin:/tmp/
ssh orin "bash /tmp/jetson-orin-setup.sh"
```

The script installs (in order):
1. **nvidia-jetpack** — CUDA 12.6, cuDNN, TensorRT
2. **ROS 2 Humble** — full desktop + vision_msgs + dev tools
3. **ZED SDK 5.2** — for ZED 2 stereo camera
4. **PyTorch 2.5** — from NVIDIA's Jetson AI Lab wheels
5. **Python deps** — numpy <2, ultralytics, etc.
6. **kart_brain** — clone, build with colcon

### Verify
```bash
nvcc --version                    # CUDA
python3 -c "import torch; print(torch.cuda.is_available())"  # PyTorch GPU
ros2 --help                       # ROS 2
/mnt/data/kart_brain/run_live.sh  # Live perception
```

## Post-Setup Checklist

- [ ] NVMe is root (`df -h /` shows nvme0n1p1)
- [ ] CUDA working (`nvcc --version`)
- [ ] PyTorch sees GPU
- [ ] ROS 2 Humble sourced in `.bashrc`
- [ ] ZED camera detected (`ls /dev/video*`)
- [ ] kart_brain built and perception pipeline runs
- [ ] SSH key access from Mac (`ssh orin`)
- [ ] AnyDesk working (needs DP dummy plug)
- [ ] Deploy key for GitHub push (already added to kart_brain repo as "Jetson Orin")

## Troubleshooting

### Flash fails with "No Jetson device found"
- Orin is not in recovery mode. Check `lsusb | grep 0955:7023`
- Wrong USB-C port. Use the one next to the 40-pin header, not the power port.

### Flash fails with USB errors
- Try a different USB-C cable (data-capable, not charge-only)
- Connect directly (no hub)

### Orin doesn't boot after flash
- Enter recovery mode and reflash
- If persistent, the NVMe may have issues — try flashing to eMMC first to verify hardware

### "nvidia-jetpack" install fails
- Ensure the NVIDIA apt repo is configured. If not: `sudo apt-get install -y nvidia-l4t-core` first, which adds the repo.

## Notes

- **JetPack 7** does not support AGX Orin as of Feb 2026. JetPack 7.2 (with Orin support) is expected Q2 2026.
- The eMMC still has the old OS. After confirming NVMe boot works, the eMMC can serve as fallback.
- DHCP IPs may change after reflash. Always verify with `hostname -I` on the Orin.
