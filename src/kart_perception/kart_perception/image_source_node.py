#!/usr/bin/env python3
import pathlib
from typing import List, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".webm"}


class ImageSourceNode(Node):
    def __init__(self) -> None:
        super().__init__("image_source")

        self.declare_parameter(
            "source", "test_data/driverless_test_media/cones_test.png"
        )
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("image_topic", "/image_raw")

        self.source = pathlib.Path(self.get_parameter("source").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.image_topic, 10)

        self._image_paths: List[pathlib.Path] = []
        self._image_index = 0
        self._video_capture: Optional[cv2.VideoCapture] = None

        if self.source.is_dir():
            self._image_paths = sorted(
                [p for p in self.source.iterdir() if p.suffix.lower() in IMAGE_EXTS]
            )
            if not self._image_paths:
                self.get_logger().error(f"No images found in {self.source}")
        elif self.source.is_file():
            if self.source.suffix.lower() in IMAGE_EXTS:
                self._image_paths = [self.source]
            elif self.source.suffix.lower() in VIDEO_EXTS:
                self._video_capture = cv2.VideoCapture(str(self.source))
                if not self._video_capture.isOpened():
                    self.get_logger().error(f"Failed to open video {self.source}")
            else:
                self.get_logger().error(f"Unsupported source: {self.source}")
        else:
            self.get_logger().error(f"Source not found: {self.source}")

        if publish_rate <= 0:
            self.get_logger().error("publish_rate must be > 0")
            return

        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)

    def _publish_image(self, frame) -> None:
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)

    def _next_image(self):
        if not self._image_paths:
            return None
        path = self._image_paths[self._image_index]
        frame = cv2.imread(str(path))
        if frame is None:
            self.get_logger().warning(f"Failed to read image {path}")
            return None
        self._image_index += 1
        if self._image_index >= len(self._image_paths):
            if self.loop:
                self._image_index = 0
            else:
                self.timer.cancel()
        return frame

    def _next_video_frame(self):
        if self._video_capture is None:
            return None
        ok, frame = self._video_capture.read()
        if ok:
            return frame
        if self.loop:
            self._video_capture.release()
            self._video_capture = cv2.VideoCapture(str(self.source))
            return None
        self.timer.cancel()
        return None

    def _on_timer(self) -> None:
        frame = None
        if self._video_capture is not None:
            frame = self._next_video_frame()
        else:
            frame = self._next_image()
        if frame is not None:
            self._publish_image(frame)


def main() -> None:
    rclpy.init()
    node = ImageSourceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
