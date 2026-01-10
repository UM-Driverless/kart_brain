# Stuff

- Source selection: choose ZED vs media replay at launch, or add a mux later that republishes to `/image_raw`.
- Safety: watchdog that zeros `/actuation_cmd` if perception/control inputs are stale or a node dies.
- Health signals: heartbeat topics from critical nodes for a supervisor to monitor.
- Command interface: `/actuation_cmd` carries `ackermann_msgs/AckermannDriveStamped` for now; rename/type can evolve.
- Perception pipeline: ZED -> YOLO 2D detections -> 3D triangulation -> planner -> controller -> `/actuation_cmd`.
- UV on aarch64 lacks `uv sync --system`; use `uv pip compile pyproject.toml` then `uv pip install --system -r <requirements>` so ROS can import system Python deps.
- Python packaging prefers centralizing metadata in `pyproject.toml`, but ROS packages still require `package.xml` for build/test/runtime dependencies; `pyproject.toml` does not replace ROS metadata.
