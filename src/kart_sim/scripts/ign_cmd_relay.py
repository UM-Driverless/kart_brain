#!/usr/bin/env python3
"""Relay ROS /kart/cmd_vel to Gazebo via ign topic subprocess.

Workaround for ros_gz_bridge not delivering Twist messages to the
Ackermann steering plugin on some platforms (aarch64 UTM VM).
"""
import os
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class IgnCmdRelay(Node):
    def __init__(self):
        super().__init__("ign_cmd_relay")
        self.sub = self.create_subscription(
            Twist, "/kart/cmd_vel", self._on_cmd, 10
        )
        self._latest_msg = None
        self._lock = threading.Lock()
        self._running = True
        # Background thread publishes at ~10Hz using os.system (fast, fire-and-forget)
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("IgnCmdRelay active — relaying cmd_vel via ign topic")

    def _on_cmd(self, msg: Twist):
        with self._lock:
            self._latest_msg = (msg.linear.x, msg.angular.z)

    def _publish_loop(self):
        while self._running:
            with self._lock:
                msg = self._latest_msg
            if msg is not None:
                lx, az = msg
                cmd = (
                    f'ign topic -t /kart/cmd_vel -m ignition.msgs.Twist '
                    f'-p "linear: {{x: {lx}}}, angular: {{z: {az}}}"'
                )
                os.system(cmd + " &>/dev/null &")
            time.sleep(0.1)  # 10 Hz


def main():
    rclpy.init()
    node = IgnCmdRelay()
    rclpy.spin(node)
    node._running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
