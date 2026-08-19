#!/usr/bin/env python3
"""Demo: subscribe to /vision/hands, stop_robot whenever a hand is detected.

Closes the loop the team designed: vision detects a hand -> ROS2 -> robot
stops. Talks to the robot directly (case1/ur_client.py's URClient.stop()),
NOT through the MCP tool layer -- a safety reaction has no business going
through an LLM's tool-call loop, and the team's own low-latency argument
for safety functions (see meeting notes) is exactly why.

NOT real spatial safety fencing. /vision/hands positions are in
camera-frame normalized image coordinates (see vision_human_track/README.md
and ros2_vision_bridge/README.md's "Coordinate frame" sections) -- there is
no camera<->robot extrinsic calibration in this repo, so there's no way to
compute a real "is this hand actually near the robot" distance yet. This
reacts to hand *presence* in the camera frame, not real proximity. It
demonstrates the reactive pipeline end-to-end; treat the trigger condition
itself as a placeholder until real calibration exists.

    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 safety_stop_demo.py
"""
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "case1"))
from ur_client import URClient  # noqa: E402


class SafetyStopDemo(Node):
    def __init__(self):
        super().__init__("safety_stop_demo")

        self.declare_parameter("cooldown_s", 2.0)
        self._cooldown_s = self.get_parameter("cooldown_s").value
        self._last_stop_t = 0.0

        self._robot = URClient()
        self._robot.connect()  # fails fast if the simulator/robot is unreachable

        self.create_subscription(PoseArray, "/vision/hands", self._on_hands, 10)
        self.get_logger().info(
            "safety_stop_demo: watching /vision/hands, will stop_robot on any "
            f"hand detection (cooldown {self._cooldown_s}s between stops)"
        )

    def _on_hands(self, msg: PoseArray) -> None:
        if not msg.poses:
            return
        now = time.monotonic()
        if now - self._last_stop_t < self._cooldown_s:
            return  # a hand staying in frame shouldn't spam stopj every poll
        self._last_stop_t = now
        self.get_logger().warning(f"{len(msg.poses)} hand(s) detected -> stopping robot")
        self._robot.stop()


def main():
    rclpy.init()
    node = SafetyStopDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
