#!/usr/bin/env python3
import os
import pathlib
import warnings
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose


# Suppress YOLOv5 autocast deprecation warnings globally.
warnings.filterwarnings(
    "ignore",
    message=".*autocast\\(args...\\).*deprecated.*",
    category=FutureWarning,
)


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("detections_topic", "/perception/cones_2d")
        self.declare_parameter("debug_image_topic", "/perception/yolo/annotated")
        self.declare_parameter("weights_path", "models/perception/yolo/best_adri.pt")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("publish_debug_image", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.weights_path = pathlib.Path(self.get_parameter("weights_path").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.debug_publisher = self.create_publisher(Image, self.debug_image_topic, 10)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self._on_image, 10
        )

        self.model = self._load_model()
        self.class_names = None if self.model is None else self.model.names

    def _load_model(self):
        if not self.weights_path.exists():
            self.get_logger().error(f"Weights not found: {self.weights_path}")
            return None
        # Ensure matplotlib uses a non-GUI backend and suppress Axes3D warning.
        os.environ.setdefault("MPLBACKEND", "Agg")
        warnings.filterwarnings(
            "ignore",
            message="Unable to import Axes3D.*",
            module="matplotlib",
        )
        # Keep additional filters near model load for clarity.
        try:
            import torch
        except ImportError:
            self.get_logger().error("torch not installed; cannot load YOLO model")
            return None
        try:
            model = torch.hub.load(
                "ultralytics/yolov5", "custom", path=str(self.weights_path)
            )
            model.conf = self.conf_threshold
            model.iou = self.iou_threshold
            model.to(self.device)
            return model
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            return None

    def _on_image(self, msg: Image) -> None:
        if self.model is None:
            return
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        results = self.model(frame_rgb, size=self.imgsz)
        detections = Detection2DArray()
        detections.header = msg.header

        for det in results.xyxy[0].tolist():
            x1, y1, x2, y2, conf, cls_id = det
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.center.theta = 0.0
            bbox.size_x = max(0.0, x2 - x1)
            bbox.size_y = max(0.0, y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            if self.class_names is not None:
                hypothesis.hypothesis.class_id = str(self.class_names[int(cls_id)])
            else:
                hypothesis.hypothesis.class_id = str(int(cls_id))
            hypothesis.hypothesis.score = float(conf)

            detection = Detection2D()
            detection.bbox = bbox
            detection.results.append(hypothesis)
            detections.detections.append(detection)

        self.publisher.publish(detections)

        if self.publish_debug_image:
            rendered = results.render()
            if rendered:
                debug_bgr = rendered[0]
                debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_publisher.publish(debug_msg)


def main() -> None:
    rclpy.init()
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
