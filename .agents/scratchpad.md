# Scratchpad — YOLOv11 Migration (2026-02-25)

## What's done
- Migrated `yolo_detector_node.py` to ultralytics API (YOLOv11 primary, torch.hub YOLOv5 fallback)
- Added `nava_yolov11_2026_02.pt` (5.2MB, YOLOv11) as new default weights
- Renamed `best_adri.pt` → `adri_yolov5_2025.pt` (kept as legacy)
- Added `models/perception/yolo/README.md` documenting weight files
- Updated all launch files and scripts to use new weights
- Fixed cuBLAS crash: system CUDA 12.6 libs must precede pip cuBLAS 12.9 in LD_LIBRARY_PATH
- YOLOv11 loads and runs on CUDA successfully (tested with dummy image)

## Current status
- YOLO + CUDA: **WORKING** (cuBLAS fix confirmed)
- Camera (`/dev/video0`): Needs USB reset or reconnection (got `Failed to open device`)
- Full pipeline test with real camera: **NOT YET DONE**

## What's left
1. **Test with real camera** — USB reset the ZED, run pipeline, verify cone detection works end-to-end
2. **Verify colored labels** — confirm per-class colors render correctly with YOLOv11 class names
3. **Measure GPU FPS** — benchmark with real frames
4. **Check class name mapping** — YOLOv11 model has class `unknown_cone` (id 3) that YOLOv5 didn't have
5. **Document torch install** in kart_brain README (briefly) and kart_docs (extensively)

## Key env facts
- Orin: JetPack 6, L4T R36.4, CUDA 12.6, Python 3.10
- torch 2.10.0 from jetson-ai-lab (CUDA works WITH system cuBLAS)
- torchvision 0.25.0 from jetson-ai-lab
- ultralytics pip package installed
- numpy 1.26.4 (pinned for pyzed compat)
- **LD_LIBRARY_PATH must start with** `/usr/local/cuda-12.6/targets/aarch64-linux/lib`
- YOLO weights: `models/perception/yolo/nava_yolov11_2026_02.pt` (default)
