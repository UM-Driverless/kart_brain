# UTM VM Environment

## Connection
```bash
ssh utm                    # configured in ~/.ssh/config → 192.168.65.2
ssh -X utm                 # with X11 forwarding (for RViz, Gazebo GUI)
```

## Specs
| Property | Value |
|---|---|
| OS | Ubuntu 22.04 LTS (Jammy), arm64/aarch64 |
| Kernel | 5.15.x |
| RAM | 8 GB |
| CPUs | 4 cores (emulated) |
| Disk | ~38 GB total, ~17 GB free (as of 2026-02-21) |
| GPU | virtio-gpu-pci (**virgl disabled**, llvmpipe software rendering) |
| Hypervisor | UTM (QEMU with HVF) on macOS Apple Silicon |

## GPU Acceleration Status

### Current: Software Rendering (llvmpipe)
The VM uses `virtio-gpu-pci` **without** the `-gl` suffix, meaning virgl is disabled:
```
dmesg: features: -virgl +edid -resource_blob -host_visible
glxinfo: OpenGL renderer string: llvmpipe (LLVM 15.0.7, 128 bits)
```
All 3D rendering (Gazebo camera, RViz) runs on CPU via Mesa's llvmpipe.

### Can the Mac GPU be used?

**Short answer: theoretically yes via virgl, but not practical for Gazebo.**

UTM with the QEMU backend supports paravirtualized GPU acceleration through **virgl**:
```
Guest Mesa (virgl) → virtio-gpu → QEMU → virglrenderer → ANGLE → Metal → Apple GPU
```

However, there are significant limitations:

| Issue | Impact |
|---|---|
| **Max OpenGL 2.1** via virgl on macOS (ANGLE caps it) | Gazebo's OGRE2 needs OpenGL 3.3+ |
| **Crashes with 3D apps** | UTM docs warn "3D applications may lock up or crash UTM" |
| **Apple Virtualization Framework has no 3D** | Only QEMU backend supports virgl |
| **No Vulkan** | Venus not supported in UTM |

**Sources:**
- [UTM #4285](https://github.com/utmapp/UTM/issues/4285): OpenGL limited to 2.1 with virgl
- [UTM #5482](https://github.com/utmapp/UTM/discussions/5482): "No GPU acceleration under AVF"
- [UTM v4.1 docs](https://docs.getutm.app/updates/v4.1/): 3D apps may crash UTM
- [UTM #5987](https://github.com/utmapp/UTM/issues/5987): Crashes with virtio-gpu-gl-pci

### How to enable virgl (if you want to try)
1. Shut down the VM in UTM
2. VM Settings → Display → change card to **`virtio-gpu-gl-pci`** (note the `-gl`)
3. UTM Preferences → Renderer Backend → **ANGLE (Metal)**
4. Boot VM, verify: `dmesg | grep virgl` should show `+virgl`
5. `glxinfo | grep renderer` should show `virgl (ANGLE ...)` instead of `llvmpipe`

**Even if virgl works, Gazebo OGRE2 will likely fail** (needs GL 3.3, virgl gives 2.1). You'd need OGRE1 or a different render engine. The headless llvmpipe setup we have now is actually more reliable for Gazebo.

### Better alternatives for GPU-accelerated simulation
- **Native Gazebo on macOS**: Gazebo Garden/Harmonic have ARM64 macOS support. Run Gazebo natively, bridge to ROS in the VM.
- **Dedicated x86 machine**: A desktop with an NVIDIA GPU runs Gazebo at full speed.
- **Docker + Rosetta**: Run an x86_64 Gazebo container with GPU passthrough (requires compatible Docker setup).

### Bottom line
**Stick with llvmpipe for now.** The headless rendering at 640x360@10Hz works. The perfect_perception_node bypasses rendering entirely for the control loop. GPU acceleration via virgl is not worth the effort for Gazebo — the OpenGL 2.1 cap makes it incompatible with OGRE2.

## sudo
Password is `0`. For non-interactive SSH commands:
```bash
ssh utm 'echo "0" | sudo -S apt-get install -y <package>'
```
Note: `ssh -t` for pseudo-terminal allocation does NOT work from non-interactive contexts. Always pipe the password with `-S`.

## Installed Software

### ROS 2
- `ros-humble-desktop` (full desktop install including RViz)
- `ros-humble-ros-gz` (Gazebo Fortress bridge, sim, image)
- `ros-humble-vision-msgs` (Detection2DArray, Detection3DArray)
- `ros-humble-xacro` (URDF/SDF macros)
- `ros-humble-tf2-ros` (TF transforms)

### Gazebo
- Gazebo Fortress (ign-gazebo 6.16.0) — the Ignition-era release
- CLI is `ign`, not `gz`
- Installed via `ros-humble-ros-gz` which pulls in all Ignition Fortress libs
- Plugins at `/usr/lib/aarch64-linux-gnu/ign-gazebo-6/plugins/`

### Python
- Python 3.10 (system)
- PyTorch (for YOLO) — check with `python3 -c "import torch; print(torch.__version__)"`
- cv_bridge, image_geometry, numpy (ROS dependencies)

### Mesa / Rendering
- `mesa-utils`, `libegl1-mesa-dev`, `libgles2-mesa-dev`
- OGRE2 headless rendering works via EGL on LLVMpipe
- OpenGL 4.5 (Compatibility) via llvmpipe — sufficient for OGRE2 headless
- No hardware acceleration — keep resolutions low

## Disk Space Concerns
Gazebo + ros-gz consumed ~3-4 GB. With ~17 GB free, there's room for additional packages but be mindful of:
- colcon build artifacts (`build/`, `install/`)
- Gazebo model cache (`~/.ignition/`)
- ROS bag recordings

## Network
- VM accessible at 192.168.65.2 from host Mac
- Host Mac accessible from VM at 192.168.65.1
- Internet access works (for apt, pip, etc.)

## Known Quirks
1. **SSH connections can hang** if a background process (like Gazebo) holds the terminal. Always redirect output (`&>/tmp/log`) when running long processes over SSH.
2. **X11 forwarding is slow** — expect 1-5 FPS for GUI apps like RViz or Gazebo GUI.
3. **No systemd user services** — cron or manual launches only.
4. **Colcon build is slow** for C++ packages (~minutes) but fast for Python packages (~seconds).
