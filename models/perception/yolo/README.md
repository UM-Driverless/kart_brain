# YOLO Weights

| File | Architecture | Trained by | Notes |
|---|---|---|---|
| `nava_yolov11_2026_02.pt` | YOLOv11 | Nava (Feb 2026) | **Current default.** Loaded via `ultralytics` pip package. |
| `adri_yolov5_2025.pt` | YOLOv5 | Adri (2025) | Legacy. Loaded via `torch.hub.load("ultralytics/yolov5", "custom", ...)`. |

All models detect: `blue_cone`, `yellow_cone`, `orange_cone`, `large_orange_cone`.

## Switching weights

Set the `weights_path` ROS parameter on `yolo_detector`:

```bash
ros2 run kart_perception yolo_detector --ros-args \
  -p weights_path:=models/perception/yolo/nava_yolov11_2026_02.pt
```

The node auto-detects YOLOv5 vs YOLOv11 format (tries `ultralytics` first, falls back to `torch.hub`).
