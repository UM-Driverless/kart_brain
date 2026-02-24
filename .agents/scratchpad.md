# Scratchpad — Steering HUD + YOLO CUDA (2026-02-24)

## What's done
- `steering_hud_node.py` — created, registered in setup.py, working. Direct subscriber (no sync), FPS EMA, cone highlights, gauge, status.
- `yolo_detector_node.py` — colored per-class boxes (two-pass: boxes then labels), auto-detect CUDA device, queue_size=1.
- `run_live_3d.sh` — uses `image_source` (webcam mode), NVIDIA lib wildcard for LD_LIBRARY_PATH.
- `run_live.sh` — same LD_LIBRARY_PATH fix.

## Current blocker: YOLO + torch 2.10 on Orin
The original Jetson PyTorch 2.5.0a0 (NVIDIA build) got overwritten by CPU-only torch.
Installed torch 2.10 + torchvision 0.25 from `pypi.jetson-ai-lab.io/jp6/cu126` — CUDA works.

**But**: YOLOv5 via `torch.hub.load('ultralytics/yolov5', 'custom', ...)` is broken with torch 2.10:
- `model(numpy_array)` → `conv2d() got numpy.ndarray` (AutoShape doesn't convert)
- `model(pil_image)` → same issue
- `model(tensor)` → size mismatch (bypasses letterbox)
- Cleared `~/.cache/torch/hub/ultralytics_yolov5_master` — not yet tested if fresh download fixes it

**Root cause**: The cached YOLOv5 hub code was written for older torch. Fresh download MIGHT fix it.
If not, switching to `ultralytics` (YOLOv8/v11) pip package would be the cleanest fix — it has native torch 2.10 support.

## Attempted torch install paths (all failed or partial)
1. `pip install torch==2.5.0 --index-url nvidia.../jp/v60` — no wheels found
2. NVIDIA Jetson wheel `torch-2.5.0a0+nv24.08` — CUDA works but torchvision 0.20 overrides it with CPU torch
3. `--no-deps` reinstall of NV wheel + torchvision 0.20 from PyPI — `torchvision::nms does not exist`
4. `torch+torchvision from pypi.jetson-ai-lab.io/jp6/cu126` — torch 2.10 + torchvision 0.25, CUDA works, but YOLOv5 hub code is incompatible

## What's left
1. **Fix YOLO inference** — either:
   - Fresh YOLOv5 hub download (cache was cleared, untested)
   - Switch to `ultralytics` pip package (YOLOv8/v11) — teammate may have a newer model
   - Pin a known-good YOLOv5 commit
2. **Verify colored labels** — couldn't test since YOLO keeps crashing
3. **Measure GPU FPS** — never got a clean CUDA run to benchmark
4. **Document torch install** in README and kart_docs (user requested)
5. **Add to TODO.md** if not completing docs this session

## Key env facts
- Orin: JetPack 6, L4T R36.4, CUDA 12.6, Python 3.10
- Current torch: 2.10.0 from jetson-ai-lab (CUDA works)
- torchvision: 0.25.0 from jetson-ai-lab
- numpy: 1.26.4 (downgraded for pyzed compat)
- YOLO weights: `models/perception/yolo/best_adri.pt` (YOLOv5 format)
- `nvidia-cusparselt-cu12`, `nvidia-cudss-cu12` installed for torch 2.10 deps
