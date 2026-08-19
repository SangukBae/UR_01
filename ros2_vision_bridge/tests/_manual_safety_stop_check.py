"""Live integration check for safety_stop_demo.py: starts a slow real robot
move in a background thread, publishes one synthetic /vision/hands message
partway through (standing in for a real camera detection, which needs an
actual hand in front of an actual webcam - not something this script can
do), and confirms the robot actually stopped short of its target.

Not an automated test (needs the live simulator) - a debugging aid, same
category as _manual_subscriber_check.py.
"""
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "case1"))
from ur_client import URClient  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety_stop_demo import SafetyStopDemo  # noqa: E402


class FakeHandPublisher(Node):
    def __init__(self):
        super().__init__("fake_hand_publisher")
        self.pub = self.create_publisher(PoseArray, "/vision/hands", 10)

    def publish_one_hand(self):
        msg = PoseArray()
        msg.header.frame_id = "camera_link"
        msg.poses.append(Pose())  # content doesn't matter, only that a hand exists
        self.pub.publish(msg)


def main():
    robot = URClient()
    robot.connect()

    HOME = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
    FAR = [0.5236, -1.5708, 0.0, -1.5708, 0.0, 0.0]  # base +30deg
    robot.move_joint(HOME, 1.0, 1.4)  # known start

    move_done = threading.Event()

    def do_slow_move():
        robot.move_joint(FAR, 0.15, 0.3)
        move_done.set()

    move_thread = threading.Thread(target=do_slow_move, daemon=True)

    rclpy.init()
    safety_node = SafetyStopDemo()
    fake_pub = FakeHandPublisher()

    move_thread.start()
    print("Move started toward +30deg base, speed=0.15 (should take ~2.25s)...")
    time.sleep(0.5)  # let real motion actually begin

    print("Publishing a fake hand detection...")
    for _ in range(5):  # a few publishes so a slow subscriber match isn't missed
        fake_pub.publish_one_hand()
        rclpy.spin_once(safety_node, timeout_sec=0.2)

    for _ in range(20):
        rclpy.spin_once(safety_node, timeout_sec=0.1)
        if move_done.is_set():
            break

    move_thread.join(timeout=5)
    final_state = robot.get_state()
    final_base_deg = final_state.q_rad[0] * 180 / 3.14159265

    print(f"\nFinal base angle: {final_base_deg:.1f} deg (target was 30 deg)")
    if abs(final_base_deg - 30.0) > 2.0:
        print("PASS: robot stopped well short of the target -- safety stop worked")
    else:
        print("FAIL: robot reached (or nearly reached) the target -- stop didn't happen in time")

    safety_node.destroy_node()
    fake_pub.destroy_node()
    rclpy.shutdown()
    robot.move_joint(HOME, 1.0, 1.4)  # park home


if __name__ == "__main__":
    main()
