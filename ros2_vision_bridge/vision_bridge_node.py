#!/usr/bin/env python3
"""ROS2 bridge for the vision_human_track REST service.

Polls POST /detect/live on a timer and republishes the result as ROS2
topics, so any ROS2 node (obstacle avoidance, digital twin, safety
fencing) can consume it without knowing the vision stack is a plain HTTP
service under the hood.

No colcon package here on purpose, matching this repo's existing
ros2_ur_driver/ros2_client.py convention (a plain rclpy script, not an
ament package) -- run directly, nothing to build:

    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # see ros2_ur_driver/README.md
    pip install requests    # only non-ROS dependency
    python3 vision_bridge_node.py

Needs vision_human_track's API reachable (default http://localhost:8000) --
run it locally (`docker compose up` or the venv) with a camera attached.

Topics published:
  /vision/hands         geometry_msgs/PoseArray   palm center + orientation
                         per detected hand (position.z / orientation are a
                         Left/Right-corrected palm normal - see
                         vision_human_track/README.md's known gaps for what
                         that orientation does and doesn't mean).
  /vision/humans_markers  visualization_msgs/MarkerArray
                         one LINE_LIST per detected person's skeleton - Rviz
                         can show this with zero custom-message parsing.
  /vision/humans_json    std_msgs/String
                         the vision service's raw JSON, one message per
                         poll -- IDs, full 33-point skeleton, per-hand
                         21-point landmarks, unassigned_hands -- for any
                         consumer that wants more than the two topics above
                         give without a custom .msg package.
  /vision/objects_json   std_msgs/String
                         only published when the polled response carries an
                         `objects` field (i.e. the service was started with
                         YOLO_ENABLED=true) -- same shape
                         realsense_vision_node.py publishes:
                         {"objects": [...], "frame_width": W,
                         "frame_height": H}. No TF here, unlike
                         realsense_vision_node.py's per-object TF -- this
                         node never owns the camera, so it has no
                         intrinsics/deproject() to turn a pixel + depth
                         into a 3D point; use realsense_vision_node.py
                         directly instead if you need that.

Coordinate frame: everything above is in the vision service's own output
frame (camera-relative, normalized [0,1] image x/y, MediaPipe's relative
z) -- NOT the robot/world frame. Per the team's first architecture
meeting, transforming that into robot coordinates is a separate ROS-side
job, deliberately not done in this node.
"""
import json
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import quat_from_z_axis  # noqa: E402

# Simplified skeleton connections for the 33-point MediaPipe Pose model --
# just the limbs/torso, not every face landmark, so the RViz marker stays
# readable. Indices per MediaPipe's PoseLandmarker point ordering.
POSE_CONNECTIONS = [
    (11, 12),               # shoulder to shoulder
    (11, 13), (13, 15),     # left arm
    (12, 14), (14, 16),     # right arm
    (11, 23), (12, 24),     # shoulders to hips
    (23, 24),               # hip to hip
    (23, 25), (25, 27),     # left leg
    (24, 26), (26, 28),     # right leg
]


def build_hands_msg(data: dict, stamp, frame_id: str) -> PoseArray:
    """Shared with live_demo.py's --ros2 mode, which builds this in-process
    instead of polling the REST API."""
    msg = PoseArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    all_hands = list(data.get("unassigned_hands", []))
    for human in data.get("humans", []):
        all_hands.extend(human.get("hands", []))

    for hand in all_hands:
        pose = Pose()
        px, py, pz = hand["palm_center"]
        pose.position = Point(x=px, y=py, z=pz)
        qx, qy, qz, qw = quat_from_z_axis(hand["normal"])
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw
        msg.poses.append(pose)

    return msg


def build_markers_msg(data: dict, stamp, frame_id: str) -> MarkerArray:
    """Shared with live_demo.py's --ros2 mode, which builds this in-process
    instead of polling the REST API."""
    markers = MarkerArray()
    for human in data.get("humans", []):
        skeleton = human["skeleton"]
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "vision_humans"
        marker.id = human["id"]
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1  # auto-clear if this ID stops publishing

        for a, b in POSE_CONNECTIONS:
            if skeleton[a]["visibility"] < 0.3 or skeleton[b]["visibility"] < 0.3:
                continue
            marker.points.append(Point(x=skeleton[a]["x"], y=skeleton[a]["y"], z=skeleton[a]["z"]))
            marker.points.append(Point(x=skeleton[b]["x"], y=skeleton[b]["y"], z=skeleton[b]["z"]))

        markers.markers.append(marker)
    return markers


def build_object_tfs(detections: list[dict], cap, frame_id: str, stamp) -> list[TransformStamped]:
    """One TransformStamped per detected object with a valid depth
    reading (parented to `frame_id`, expected to be the camera's own
    optical frame -- see camera_tf_publisher.py). Shared by
    realsense_vision_node.py and live_demo.py's --ros2 mode.

    ``cap`` must have RealSenseCapture's `deproject(x, y, depth_m)` (a
    plain cv2.VideoCapture has no depth, so this is only called when
    `--realsense` is on); ``detections`` are YOLO dicts with `xyxy`,
    `instance_name`, and (from add_distances) `distance_m`.
    """
    transforms = []
    for det in detections:
        distance = det.get("distance_m")
        if distance is None:
            continue
        x1, y1, x2, y2 = det["xyxy"]
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        point = cap.deproject(cx, cy, distance)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = frame_id
        tf.child_frame_id = f"object_{det['instance_name']}"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = point
        tf.transform.rotation.w = 1.0  # no orientation from a 2D detector -- axis-aligned to the camera
        transforms.append(tf)
    return transforms


class VisionBridgeNode(Node):
    def __init__(self):
        import requests  # only this class polls the REST API; keeps the
                          # module importable (e.g. by live_demo.py --ros2,
                          # which never calls this class) without requests.

        super().__init__("vision_bridge")

        self.declare_parameter("vision_api_url", "http://localhost:8000")
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("camera_frame_id", "camera_link")

        self._api_url = self.get_parameter("vision_api_url").value.rstrip("/")
        rate_hz = self.get_parameter("poll_rate_hz").value
        self._frame_id = self.get_parameter("camera_frame_id").value

        self._hands_pub = self.create_publisher(PoseArray, "/vision/hands", 10)
        self._markers_pub = self.create_publisher(MarkerArray, "/vision/humans_markers", 10)
        self._json_pub = self.create_publisher(String, "/vision/humans_json", 10)
        # Only actually publishes if a poll response ever contains an
        # "objects" field (YOLO_ENABLED=true on the service side) -- see
        # this module's docstring.
        self._objects_pub = self.create_publisher(String, "/vision/objects_json", 10)

        self._session = requests.Session()
        self._warned_unreachable = False

        self.create_timer(1.0 / rate_hz, self._poll)
        self.get_logger().info(
            f"vision_bridge: polling {self._api_url}/detect/live at {rate_hz} Hz"
        )

    def _poll(self):
        import requests

        try:
            resp = self._session.post(f"{self._api_url}/detect/live", timeout=2.0)
            resp.raise_for_status()
        except requests.RequestException as exc:
            if not self._warned_unreachable:
                self.get_logger().warning(
                    f"vision API unreachable at {self._api_url}: {exc} "
                    "(will keep retrying quietly)"
                )
                self._warned_unreachable = True
            return
        self._warned_unreachable = False

        data = resp.json()
        stamp = self.get_clock().now().to_msg()

        self._json_pub.publish(String(data=resp.text))
        self._publish_hands(data, stamp)
        self._publish_markers(data, stamp)
        if "objects" in data:
            self._objects_pub.publish(String(data=json.dumps({
                "objects": data["objects"],
                "frame_width": data.get("frame_width"),
                "frame_height": data.get("frame_height"),
            })))

    def _publish_hands(self, data: dict, stamp) -> None:
        self._hands_pub.publish(build_hands_msg(data, stamp, self._frame_id))

    def _publish_markers(self, data: dict, stamp) -> None:
        self._markers_pub.publish(build_markers_msg(data, stamp, self._frame_id))


def main():
    rclpy.init()
    node = VisionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
