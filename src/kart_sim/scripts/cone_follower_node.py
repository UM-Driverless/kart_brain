#!/usr/bin/env python3
"""Simple midpoint-follower controller for the kart.

Subscribes to 3D cone detections, separates blue (left) and yellow (right),
finds the nearest pair, steers toward the midpoint with proportional control.
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
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("steering_gain", 1.5)
        self.declare_parameter("speed_curvature_gain", 2.0)
        self.declare_parameter("lookahead_min", 2.0)
        self.declare_parameter("lookahead_max", 10.0)
        self.declare_parameter("no_cone_timeout", 1.0)

        det_topic = str(self.get_parameter("detections_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.speed_curv_gain = float(self.get_parameter("speed_curvature_gain").value)
        self.lookahead_min = float(self.get_parameter("lookahead_min").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.no_cone_timeout = float(self.get_parameter("no_cone_timeout").value)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )

        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._safety_check)

    def _on_detections(self, msg: Detection3DArray):
        self.last_detection_time = self.get_clock().now()

        blue_cones = []
        yellow_cones = []

        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos = det.results[0].pose.pose.position
            # Only consider cones in front of the camera (x > 0)
            if pos.x <= 0.5:
                continue

            dist = math.sqrt(pos.x ** 2 + pos.y ** 2)
            if dist > self.lookahead_max:
                continue

            if class_id == "blue_cone":
                blue_cones.append((pos.x, pos.y, dist))
            elif class_id == "yellow_cone":
                yellow_cones.append((pos.x, pos.y, dist))
            # Orange cones treated as both sides at start/finish

        cmd = Twist()

        if not blue_cones and not yellow_cones:
            # No cones visible — keep creeping forward with last steering
            cmd.linear.x = self.min_speed
            cmd.angular.z = getattr(self, '_last_steer', 0.0)
            self.cmd_pub.publish(cmd)
            return

        # Find target point: midpoint between nearest blue and yellow
        target_x = 0.0
        target_y = 0.0
        has_target = False

        if blue_cones and yellow_cones:
            # Sort by distance, pick nearest of each
            blue_cones.sort(key=lambda c: c[2])
            yellow_cones.sort(key=lambda c: c[2])

            # Use the nearest pair within lookahead range
            bx, by, _ = blue_cones[0]
            yx, yy, _ = yellow_cones[0]
            target_x = (bx + yx) / 2.0
            target_y = (by + yy) / 2.0
            has_target = True
        elif blue_cones:
            # Only blue (left boundary) visible — steer right of them
            blue_cones.sort(key=lambda c: c[2])
            bx, by, _ = blue_cones[0]
            target_x = bx
            target_y = by - 1.5  # Offset to the right
            has_target = True
        elif yellow_cones:
            # Only yellow (right boundary) visible — steer left of them
            yellow_cones.sort(key=lambda c: c[2])
            yx, yy, _ = yellow_cones[0]
            target_x = yx
            target_y = yy + 1.5  # Offset to the left
            has_target = True

        if not has_target or target_x < 0.5:
            cmd.linear.x = self.min_speed
            self.cmd_pub.publish(cmd)
            return

        # Proportional steering: angle to target
        angle_to_target = math.atan2(target_y, target_x)
        steer = self.steering_gain * angle_to_target

        # Speed: inversely proportional to curvature
        curvature = abs(angle_to_target)
        speed = self.max_speed - self.speed_curv_gain * curvature
        speed = max(self.min_speed, min(self.max_speed, speed))

        cmd.linear.x = speed
        cmd.angular.z = steer
        self._last_steer = steer
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"target=({target_x:.1f},{target_y:.1f}) "
            f"steer={steer:.2f} speed={speed:.2f} "
            f"blue={len(blue_cones)} yellow={len(yellow_cones)}"
        )

    def _safety_check(self):
        """Stop the kart if no detections received for too long."""
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > self.no_cone_timeout:
            cmd = Twist()
            cmd.linear.x = self.min_speed
            cmd.angular.z = getattr(self, '_last_steer', 0.0)
            self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ConeFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
