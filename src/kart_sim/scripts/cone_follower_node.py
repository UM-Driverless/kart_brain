#!/usr/bin/env python3
"""Cone-following controller for the kart.

Supports two controller types (selected via the ``controller_type`` param):

**geometric** (default)
    Nearest blue/yellow midpoint → atan2 → steer.  Six tunable params.

**neural**
    Small feed-forward net (8→8→2) whose weights are loaded from a JSON
    file produced by the 2D-sim GA trainer (``scripts/sim2d/train.py``).

Both controllers receive Detection3DArray in the camera *optical* frame
(Z=forward, X=right, Y=down) and publish Twist on ``/kart/cmd_vel``.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection3DArray


class ConeFollowerNode(Node):
    def __init__(self):
        super().__init__("cone_follower")

        # --- common params ---
        self.declare_parameter("detections_topic", "/perception/cones_3d")
        self.declare_parameter("cmd_vel_topic", "/kart/cmd_vel")
        self.declare_parameter("no_cone_timeout", 1.0)
        self.declare_parameter("controller_type", "geometric")  # "geometric" | "neural"
        self.declare_parameter("weights_json", "")               # path for neural

        # --- geometric params ---
        self.declare_parameter("steering_gain", 1.0)
        self.declare_parameter("max_steer", 0.5)
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("lookahead_max", 15.0)
        self.declare_parameter("half_track_width", 1.5)
        self.declare_parameter("speed_curve_factor", 1.0)

        det_topic = str(self.get_parameter("detections_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.no_cone_timeout = float(self.get_parameter("no_cone_timeout").value)
        self.controller_type = str(self.get_parameter("controller_type").value)

        # geometric fields
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.lookahead_max = float(self.get_parameter("lookahead_max").value)
        self.half_track_width = float(self.get_parameter("half_track_width").value)
        self.speed_curve_factor = float(self.get_parameter("speed_curve_factor").value)

        # neural net weights (loaded if controller_type == "neural")
        self._nn_W1 = self._nn_b1 = self._nn_W2 = self._nn_b2 = None
        self._nn_max_steer = 0.5
        self._nn_max_speed = 5.0

        if self.controller_type == "neural":
            self._load_neural_weights()

        # ROS plumbing
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.sub = self.create_subscription(
            Detection3DArray, det_topic, self._on_detections, 10
        )
        self._last_steer = 0.0
        self.last_detection_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._safety_check)

        self.get_logger().info(f"Controller type: {self.controller_type}")

    # ── neural net loading ────────────────────────────────────────────

    def _load_neural_weights(self):
        path = str(self.get_parameter("weights_json").value)
        if not path:
            self.get_logger().error("controller_type=neural but weights_json not set")
            raise SystemExit(1)

        with open(path) as f:
            data = json.load(f)

        genes = np.array(data["genes"], dtype=np.float64)
        self.get_logger().info(
            f"Loaded neural weights from {path} "
            f"(fitness={data.get('fitness', '?')})"
        )

        # Parse: W1 (8×8), b1 (8), W2 (8×2), b2 (2) — 90 genes total
        i = 0
        self._nn_W1 = genes[i:i + 64].reshape(8, 8);   i += 64
        self._nn_b1 = genes[i:i + 8];                   i += 8
        self._nn_W2 = genes[i:i + 16].reshape(8, 2);    i += 16
        self._nn_b2 = genes[i:i + 2]

    # ── detection callback ────────────────────────────────────────────

    def _on_detections(self, msg: Detection3DArray):
        self.last_detection_time = self.get_clock().now()

        # Parse detections into (class_id, fwd, left) in camera_link frame
        cones = []
        for det in msg.detections:
            if not det.results:
                continue
            class_id = det.results[0].hypothesis.class_id
            pos = det.results[0].pose.pose.position
            fwd = pos.z
            left = -pos.x
            if fwd < 0.5:
                continue
            cones.append((class_id, fwd, left))

        if self.controller_type == "neural":
            steer, speed = self._control_neural(cones)
        else:
            steer, speed = self._control_geometric(cones)

        cmd = Twist()
        cmd.angular.z = steer
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)

    # ── geometric controller ──────────────────────────────────────────

    def _control_geometric(self, cones):
        nearest_blue = None
        nearest_yellow = None
        min_bd = float("inf")
        min_yd = float("inf")

        for cls, fwd, left in cones:
            dist = math.hypot(fwd, left)
            if dist > self.lookahead_max:
                continue
            if cls == "blue_cone" and dist < min_bd:
                min_bd = dist
                nearest_blue = (fwd, left)
            elif cls == "yellow_cone" and dist < min_yd:
                min_yd = dist
                nearest_yellow = (fwd, left)

        if nearest_blue and nearest_yellow:
            mid_f = (nearest_blue[0] + nearest_yellow[0]) / 2.0
            mid_l = (nearest_blue[1] + nearest_yellow[1]) / 2.0
        elif nearest_blue:
            mid_f = nearest_blue[0]
            mid_l = nearest_blue[1] - self.half_track_width
        elif nearest_yellow:
            mid_f = nearest_yellow[0]
            mid_l = nearest_yellow[1] + self.half_track_width
        else:
            return self._last_steer, self.min_speed

        angle = math.atan2(mid_l, mid_f)
        steer = max(-self.max_steer,
                     min(self.max_steer, self.steering_gain * angle))
        self._last_steer = steer

        speed = self.max_speed * (1.0 - self.speed_curve_factor * abs(steer))
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.get_logger().info(
            f"[geo] angle={math.degrees(angle):.1f}° steer={steer:.3f} "
            f"speed={speed:.1f} blue={nearest_blue} yellow={nearest_yellow}"
        )
        return steer, speed

    # ── neural net controller ─────────────────────────────────────────

    def _control_neural(self, cones):
        blues = []
        yellows = []
        for cls, fwd, left in cones:
            dist = math.hypot(fwd, left)
            angle = math.atan2(left, fwd)
            if cls == "blue_cone":
                blues.append((dist, angle))
            elif cls == "yellow_cone":
                yellows.append((dist, angle))
        blues.sort()
        yellows.sort()

        inp = np.zeros(8)
        for j, (d, a) in enumerate(blues[:2]):
            inp[j * 2] = d / 15.0
            inp[j * 2 + 1] = a / np.pi
        for j, (d, a) in enumerate(yellows[:2]):
            inp[4 + j * 2] = d / 15.0
            inp[4 + j * 2 + 1] = a / np.pi

        hidden = np.tanh(inp @ self._nn_W1 + self._nn_b1)
        out = hidden @ self._nn_W2 + self._nn_b2

        steer = float(np.tanh(out[0])) * self._nn_max_steer
        speed = float(1.0 / (1.0 + np.exp(-out[1]))) * self._nn_max_speed

        self._last_steer = steer
        self.get_logger().info(
            f"[nn] steer={steer:.3f} speed={speed:.1f} "
            f"blues={len(blues)} yellows={len(yellows)}"
        )
        return steer, speed

    # ── safety timeout ────────────────────────────────────────────────

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
