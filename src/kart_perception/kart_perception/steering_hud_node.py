#!/usr/bin/env python3
"""Steering HUD overlay node.

Composites steering visualizations onto the YOLO-annotated image:
- Cone highlights with distance labels on nearest blue/yellow cones
- Midpoint crosshair between the target cones
- Steering arrow from bottom-center toward the midpoint
- Horizontal steering gauge at the bottom
- Text overlay with steering angle and speed
- Status indicator (BLUE + YLW / BLUE ONLY / YLW ONLY / NO CONES)
"""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3DArray

# Colors (BGR)
BLUE_CONE_COLOR = (255, 150, 0)   # bright blue
YELLOW_CONE_COLOR = (0, 230, 255)  # yellow
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (0, 0, 255)
DARK_BG = (30, 30, 30)


class SteeringHudNode(Node):
    def __init__(self):
        super().__init__("steering_hud")

        self.declare_parameter("annotated_topic", "/perception/yolo/annotated")
        self.declare_parameter("cones_3d_topic", "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/rgb/camera_info")
        self.declare_parameter("output_topic", "/perception/hud")
        self.declare_parameter("half_track_width", 1.5)
        self.declare_parameter("lookahead_max", 15.0)
        self.declare_parameter("sync_slop", 0.15)
        self.declare_parameter("fallback_rate", 5.0)

        self.half_track_width = float(self.get_parameter("half_track_width").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)

        self.bridge = CvBridge()
        self.latest_cmd = Twist()
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_ready = False
        self.latest_annotated = None
        self._fps_last_time = time.monotonic()
        self._fps_count = 0
        self._fps = 0.0

        # Publisher
        self.pub = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), 10
        )

        # Synced subscribers: annotated image + 3D cones
        self.img_sub = Subscriber(
            self, Image, str(self.get_parameter("annotated_topic").value)
        )
        self.cones_sub = Subscriber(
            self, Detection3DArray, str(self.get_parameter("cones_3d_topic").value)
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.img_sub, self.cones_sub],
            queue_size=10,
            slop=float(self.get_parameter("sync_slop").value),
        )
        self.sync.registerCallback(self._on_synced)

        # Standalone subscriber for cmd_vel (cache latest)
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd_vel,
            10,
        )

        # One-shot camera info
        self._info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            10,
        )

        # Fallback timer: republish annotated image with just steering gauge
        fallback_rate = float(self.get_parameter("fallback_rate").value)
        self.fallback_timer = self.create_timer(1.0 / fallback_rate, self._fallback)

        self.get_logger().info("SteeringHudNode ready")

    # ---- Callbacks ----

    def _on_cmd_vel(self, msg: Twist):
        self.latest_cmd = msg

    def _on_camera_info(self, msg: CameraInfo):
        if not self.camera_info_ready:
            K = msg.k
            self.fx, self.fy = K[0], K[4]
            self.cx, self.cy = K[2], K[5]
            self.camera_info_ready = True
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f}"
            )

    def _update_fps(self):
        self._fps_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_last_time
        if elapsed >= 1.0:
            self._fps = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_last_time = now

    def _on_synced(self, img_msg: Image, cones_msg: Detection3DArray):
        self._update_fps()
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        self.latest_annotated = img.copy()

        # Find nearest blue/yellow cones (replicate cone_follower logic)
        nearest_blue = None
        nearest_yellow = None
        min_blue_dist = float("inf")
        min_yellow_dist = float("inf")

        for det in cones_msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos = det.results[0].pose.pose.position
            # Optical frame: Z=forward, X=right, Y=down
            fwd = pos.z
            left = -pos.x
            if fwd < 0.5:
                continue
            dist = math.sqrt(fwd**2 + left**2)
            if dist > self.lookahead_max:
                continue
            if class_id == "blue_cone" and dist < min_blue_dist:
                min_blue_dist = dist
                nearest_blue = (pos.x, pos.y, pos.z, dist)
            elif class_id == "yellow_cone" and dist < min_yellow_dist:
                min_yellow_dist = dist
                nearest_yellow = (pos.x, pos.y, pos.z, dist)

        # Compute midpoint in optical frame
        mid_opt = None  # (X, Y, Z) in optical frame
        status = "NO CONES"

        if nearest_blue and nearest_yellow:
            bx, by, bz, _ = nearest_blue
            yx, yy, yz, _ = nearest_yellow
            mid_opt = ((bx + yx) / 2.0, (by + yy) / 2.0, (bz + yz) / 2.0)
            status = "BLUE + YLW"
        elif nearest_blue:
            bx, by, bz, _ = nearest_blue
            # Offset right (positive X in optical) by half_track_width
            mid_opt = (bx + self.half_track_width, by, bz)
            status = "BLUE ONLY"
        elif nearest_yellow:
            yx, yy, yz, _ = nearest_yellow
            # Offset left (negative X in optical) by half_track_width
            mid_opt = (yx - self.half_track_width, yy, yz)
            status = "YLW ONLY"

        # Draw overlays
        h, w = img.shape[:2]

        if self.camera_info_ready:
            # Draw cone highlights
            if nearest_blue:
                self._draw_cone(img, nearest_blue, BLUE_CONE_COLOR, "B")
            if nearest_yellow:
                self._draw_cone(img, nearest_yellow, YELLOW_CONE_COLOR, "Y")
            # Draw midpoint crosshair and steering arrow
            if mid_opt is not None and mid_opt[2] > 0:
                mu, mv = self._project(mid_opt[0], mid_opt[1], mid_opt[2])
                mu, mv = int(mu), int(mv)
                # Crosshair
                size = 15
                cv2.line(img, (mu - size, mv), (mu + size, mv), GREEN, 2)
                cv2.line(img, (mu, mv - size), (mu, mv + size), GREEN, 2)
                cv2.circle(img, (mu, mv), 4, GREEN, -1)
                # Steering arrow from bottom-center
                arrow_start = (w // 2, h - 30)
                arrow_end = (mu, mv)
                cv2.arrowedLine(img, arrow_start, arrow_end, GREEN, 2, tipLength=0.04)

        # Steering gauge
        self._draw_gauge(img, self.latest_cmd.angular.z)

        # Text overlay
        self._draw_text_overlay(img, self.latest_cmd)

        # Status
        self._draw_status(img, status)

        # Publish
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        out_msg.header = img_msg.header
        self.pub.publish(out_msg)

    def _fallback(self):
        """Republish latest annotated image with just the steering gauge."""
        if self.latest_annotated is None:
            return
        img = self.latest_annotated.copy()
        self._draw_gauge(img, self.latest_cmd.angular.z)
        self._draw_text_overlay(img, self.latest_cmd)
        self._draw_status(img, "NO 3D")
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub.publish(out_msg)

    # ---- Drawing helpers ----

    def _project(self, x, y, z):
        """Project optical-frame 3D point (X=right,Y=down,Z=fwd) to pixel."""
        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return u, v

    def _draw_cone(self, img, cone_data, color, label):
        """Draw a highlighted circle + distance label at the cone's pixel position."""
        x, y, z, dist = cone_data
        if z <= 0 or not self.camera_info_ready:
            return
        u, v = self._project(x, y, z)
        u, v = int(u), int(v)
        h, w = img.shape[:2]
        if 0 <= u < w and 0 <= v < h:
            cv2.circle(img, (u, v), 20, color, 3)
            text = f"{label} {dist:.1f}m"
            cv2.putText(img, text, (u + 24, v + 5), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2, cv2.LINE_AA)

    def _draw_gauge(self, img, steer_rad):
        """Horizontal bar gauge at bottom of frame."""
        h, w = img.shape[:2]
        gauge_y = h - 12
        gauge_w = min(300, w - 40)
        gauge_x0 = (w - gauge_w) // 2

        # Background bar
        cv2.rectangle(img, (gauge_x0, gauge_y - 6), (gauge_x0 + gauge_w, gauge_y + 6),
                      DARK_BG, -1)
        # Center tick
        center_x = gauge_x0 + gauge_w // 2
        cv2.line(img, (center_x, gauge_y - 8), (center_x, gauge_y + 8), WHITE, 1)

        # Indicator: steer is in radians, max ±0.5 rad
        max_steer = 0.5
        frac = max(-1.0, min(1.0, steer_rad / max_steer))
        ind_x = int(center_x + frac * (gauge_w // 2))
        cv2.circle(img, (ind_x, gauge_y), 7, RED, -1)
        cv2.circle(img, (ind_x, gauge_y), 7, WHITE, 1)

    def _draw_text_overlay(self, img, cmd: Twist):
        """Steering angle + speed text at top-left."""
        steer_deg = math.degrees(cmd.angular.z)
        speed = cmd.linear.x
        lines = [
            f"Steer: {steer_deg:+.1f} deg",
            f"Speed: {speed:.1f} m/s",
            f"FPS: {self._fps:.1f}",
        ]
        y0 = 25
        for i, line in enumerate(lines):
            y = y0 + i * 22
            # Dark background
            (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(img, (8, y - th - 4), (16 + tw, y + 4), DARK_BG, -1)
            cv2.putText(img, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, WHITE, 2, cv2.LINE_AA)

    def _draw_status(self, img, status):
        """Status text at bottom-left."""
        h = img.shape[0]
        y = h - 20
        (tw, th), _ = cv2.getTextSize(status, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(img, (8, y - th - 4), (16 + tw, y + 4), DARK_BG, -1)
        color = GREEN if "+" in status else YELLOW_CONE_COLOR if "ONLY" in status else RED
        cv2.putText(img, status, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 2, cv2.LINE_AA)


def main():
    rclpy.init()
    node = SteeringHudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
