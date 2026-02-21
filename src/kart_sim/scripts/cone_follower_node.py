#!/usr/bin/env python3
"""PID cone-following controller for the kart.

Finds the closest blue (left) and yellow (right) cones in front,
computes the average Y offset as the error signal, and applies
PID control to steer the kart back to center.

Long-term goal: replace with a neural net trained via imitation learning.
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
        self.declare_parameter("speed", 2.0)
        self.declare_parameter("lookahead_max", 10.0)
        self.declare_parameter("no_cone_timeout", 1.0)
        # PID gains (error = avg_y in meters, output = steering in degrees)
        self.declare_parameter("kp", 0.2)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.15)

        det_topic = str(self.get_parameter("detections_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.speed = float(self.get_parameter("speed").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.no_cone_timeout = float(self.get_parameter("no_cone_timeout").value)
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )

        # PID state
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None
        self._last_steer = 0.0

        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._safety_check)

    def _on_detections(self, msg: Detection3DArray):
        self.last_detection_time = self.get_clock().now()

        nearest_blue = None
        nearest_yellow = None
        min_blue_dist = float('inf')
        min_yellow_dist = float('inf')

        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos = det.results[0].pose.pose.position

            if pos.x <= 0.5:
                continue

            dist = math.sqrt(pos.x ** 2 + pos.y ** 2)
            if dist > self.lookahead_max:
                continue

            if class_id == "blue_cone" and dist < min_blue_dist:
                min_blue_dist = dist
                nearest_blue = pos
            elif class_id == "yellow_cone" and dist < min_yellow_dist:
                min_yellow_dist = dist
                nearest_yellow = pos

        # Compute error (Y offset from track center, in meters)
        error = None
        if nearest_blue and nearest_yellow:
            error = (nearest_blue.y + nearest_yellow.y) / 2.0
        elif nearest_blue:
            error = nearest_blue.y - 1.5
        elif nearest_yellow:
            error = nearest_yellow.y + 1.5

        cmd = Twist()
        cmd.linear.x = self.speed

        if error is not None:
            # PID
            now = self.get_clock().now()
            if self._last_time is not None:
                dt = (now - self._last_time).nanoseconds / 1e9
                dt = max(dt, 0.001)  # avoid division by zero
            else:
                dt = 0.1

            self._integral += error * dt
            # Anti-windup: clamp integral
            self._integral = max(-5.0, min(5.0, self._integral))

            derivative = (error - self._prev_error) / dt

            steer = self.kp * error + self.ki * self._integral + self.kd * derivative
            # Clamp to max steering (0.5 rad ≈ 29 deg)
            steer = max(-0.5, min(0.5, steer))
            cmd.angular.z = steer

            self._prev_error = error
            self._last_time = now
            self._last_steer = cmd.angular.z

            self.get_logger().info(
                f"err={error:.2f}m steer={math.degrees(steer):.1f}deg "
                f"P={self.kp*error:.1f} I={self.ki*self._integral:.2f} D={self.kd*derivative:.1f} "
                f"blue={'%.1f'%nearest_blue.y if nearest_blue else '-'} "
                f"yellow={'%.1f'%nearest_yellow.y if nearest_yellow else '-'}"
            )
        else:
            cmd.angular.z = self._last_steer

        self.cmd_pub.publish(cmd)

    def _safety_check(self):
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if elapsed > self.no_cone_timeout:
            cmd = Twist()
            cmd.linear.x = self.speed
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
