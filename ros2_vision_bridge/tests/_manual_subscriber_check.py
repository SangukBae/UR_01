"""One-off manual check: subscribe to vision_bridge's topics for a few
seconds and print what arrives. Not an automated test (needs the bridge
node + vision API actually running) - a debugging aid for live
verification, matching the project's practice of confirming things
against the real system rather than trusting code review alone."""
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from std_msgs.msg import String


class Checker(Node):
    def __init__(self):
        super().__init__("vision_bridge_manual_check")
        self.hands_count = 0
        self.json_count = 0
        self.create_subscription(PoseArray, "/vision/hands", self._on_hands, 10)
        self.create_subscription(String, "/vision/humans_json", self._on_json, 10)

    def _on_hands(self, msg):
        self.hands_count += 1
        if self.hands_count == 1:
            print(f"/vision/hands: received, {len(msg.poses)} pose(s)")
            for p in msg.poses:
                print(f"  pos=({p.position.x:.3f},{p.position.y:.3f},{p.position.z:.3f}) "
                      f"quat=({p.orientation.x:.3f},{p.orientation.y:.3f},"
                      f"{p.orientation.z:.3f},{p.orientation.w:.3f})")

    def _on_json(self, msg):
        self.json_count += 1
        if self.json_count == 1:
            print(f"/vision/humans_json: received, {len(msg.data)} bytes")
            print(f"  preview: {msg.data[:200]}")


def main():
    rclpy.init()
    node = Checker()
    end_time = node.get_clock().now().nanoseconds + 8_000_000_000
    while node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=0.5)
    print(f"\nTotal: {node.hands_count} /vision/hands messages, "
          f"{node.json_count} /vision/humans_json messages in ~8s")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
