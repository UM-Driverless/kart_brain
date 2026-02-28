#!/usr/bin/env python3
"""Midpoint-angle cone-following controller for the kart.

Finds the nearest blue (left) and yellow (right) cones, computes the
midpoint between them, and steers toward that midpoint using the angle
from the kart's forward axis. Speed scales down in proportion to
steering magnitude (slow in curves, fast on straights).

The input Detection3DArray positions are in the camera optical frame
(Z=forward, X=right, Y=down) as produced by projectPixelTo3dRay.
This node converts to camera_link convention (X=forward, Y=left, Z=up)
before computing steering.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection3DArray


class ConeFollowerNode(Node):
    def __init__(self):
        super().__init__("cone_follower")

        self.declare_parameter("detections_topic", "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
        self.declare_parameter("steering_gain", 1.0)
        self.declare_parameter("max_steer", 0.5)
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("lookahead_max", 15.0)
        self.declare_parameter("half_track_width", 1.5)
        self.declare_parameter("speed_curve_factor", 1.0)
        self.declare_parameter("no_cone_timeout", 1.0)

        det_topic = str(self.get_parameter("detections_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.half_track_width = float(self.get_parameter("half_track_width").value)
        self.speed_curve_factor = float(self.get_parameter("speed_curve_factor").value)
        self.no_cone_timeout = float(self.get_parameter("no_cone_timeout").value)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )

        self._last_steer = 0.0
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._safety_check)

    def _on_detections(self, msg: Detection3DArray):
        self.last_detection_time = self.get_clock().now()

        nearest_blue = None
        nearest_yellow = None
        min_blue_dist = float("inf")
        min_yellow_dist = float("inf")

        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos = det.results[0].pose.pose.position

            # Convert optical frame (Z=fwd, X=right, Y=down) to
            # camera_link frame (X=fwd, Y=left, Z=up)
            fwd = pos.z
            left = -pos.x
            # up = -pos.y  (unused)

            if fwd < 0.5:
                continue

            dist = math.sqrt(fwd**2 + left**2)
            if dist > self.lookahead_max:
                continue

            if class_id == "blue_cone" and dist < min_blue_dist:
                min_blue_dist = dist
                nearest_blue = (fwd, left)
            elif class_id == "yellow_cone" and dist < min_yellow_dist:
                min_yellow_dist = dist
                nearest_yellow = (fwd, left)

        # Compute midpoint in camera_link frame
        mid_fwd = None
        mid_left = None

        if nearest_blue and nearest_yellow:
            mid_fwd = (nearest_blue[0] + nearest_yellow[0]) / 2.0
            mid_left = (nearest_blue[1] + nearest_yellow[1]) / 2.0
        elif nearest_blue:
            # Only left boundary visible — offset right by half track width
            mid_fwd = nearest_blue[0]
            mid_left = nearest_blue[1] - self.half_track_width
        elif nearest_yellow:
            # Only right boundary visible — offset left by half track width
            mid_fwd = nearest_yellow[0]
            mid_left = nearest_yellow[1] + self.half_track_width

        cmd = Twist()

        if mid_fwd is not None:
            # Angle from forward axis to the midpoint (positive = left)
            angle = math.atan2(mid_left, mid_fwd)

            steer = self.steering_gain * angle
            steer = max(-self.max_steer, min(self.max_steer, steer))
            cmd.angular.z = steer
            self._last_steer = steer

            # Speed: slow down in curves
            speed = self.max_speed * (1.0 - self.speed_curve_factor * abs(steer))
            speed = max(self.min_speed, min(self.max_speed, speed))
            cmd.linear.x = speed

            self.get_logger().info(
                f"angle={math.degrees(angle):.1f}deg steer={steer:.3f} "
                f"speed={speed:.1f} "
                f"blue={nearest_blue} yellow={nearest_yellow} "
                f"mid=({mid_fwd:.1f},{mid_left:.1f})"
            )
        else:
            # No cones seen — coast with last steering
            cmd.linear.x = self.min_speed
            cmd.angular.z = self._last_steer

        self.cmd_pub.publish(cmd)

    def _safety_check(self):
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > self.no_cone_timeout:
            cmd = Twist()
            cmd.linear.x = self.min_speed
            cmd.angular.z = self._last_steer
            self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ConeFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
