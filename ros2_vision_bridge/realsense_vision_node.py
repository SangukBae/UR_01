#!/usr/bin/env python3
"""ROS2 publisher for person/hand/object recognition, reading directly from
an attached Intel RealSense camera (D435 etc.) -- no vision_human_track
REST service or plain-webcam camera_index involved.

Unlike vision_bridge_node.py (which polls vision_human_track's HTTP API,
so it works with whatever camera that service happens to have -- typically
a plain UVC webcam), this node owns the camera itself via pyrealsense2 and
runs MediaPipe (person skeleton + hand palm-center/orientation) and,
optionally, YOLO (object detection) in-process on every frame. Use this
one when the camera in question is specifically a RealSense -- e.g. the
one mounted on the robot arm -- and you want its detections on ROS2
topics without standing up the separate REST service first.

No colcon package here on purpose, matching this repo's existing
ros2_ur_driver/ros2_client.py and vision_bridge_node.py convention (a
plain rclpy script, not an ament package) -- run directly, nothing to
build:

    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # see ros2_ur_driver/README.md
    cd vision_human_track && source venv/bin/activate && cd ..
    python3 ros2_vision_bridge/realsense_vision_node.py
    python3 ros2_vision_bridge/realsense_vision_node.py --no-yolo   # skip object detection

Needs a RealSense attached and visible to Linux (on WSL2: usbipd bind +
attach, see vision_human_track/README.md's known gaps) and, for object
detection, ../yolo/yolo26n.pt + `pip install ultralytics` in the same venv
(a teammate's module -- see live_demo.py's --yolo flag, same model).

Topics published (same names/shapes as vision_bridge_node.py for the
person/hand ones, so any existing consumer -- RViz, safety_stop_demo.py --
works unchanged regardless of which bridge is running):
  /vision/hands           geometry_msgs/PoseArray   palm center + orientation
                           per detected hand.
  /vision/humans_markers   visualization_msgs/MarkerArray
                           one LINE_LIST per detected person's skeleton.
  /vision/humans_json      std_msgs/String
                           raw person/hand detection data as JSON (same
                           shape vision_human_track's REST API returns).
  /vision/objects_json     std_msgs/String
                           YOLO detections as JSON: {"objects": [...],
                           "frame_width": W, "frame_height": H} -- boxes
                           are in pixel xyxy at that resolution (2D only,
                           no depth fusion -- see repo CLAUDE.md's "Known
                           gaps"). Only published when YOLO is enabled.

Coordinate frame: everything above is camera-relative (MediaPipe's
normalized [0,1] image x/y + its own relative z for hands/skeleton, pixel
xyxy for YOLO objects) -- NOT the robot/world frame, matching the team's
architecture decision that camera->robot transforms are ROS's job, not
the vision stack's (see vision_bridge_node.py's docstring).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(_REPO_ROOT / "vision_human_track" / "src"))

from detector import HumanHandTracker  # noqa: E402
from realsense_camera import RealSenseCapture, list_realsense_devices  # noqa: E402
from vision_bridge_node import build_hands_msg, build_markers_msg  # noqa: E402


class RealsenseVisionNode(Node):
    def __init__(self, use_yolo: bool = True):
        super().__init__("realsense_vision_bridge")

        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("camera_frame_id", "camera_link")
        rate_hz = self.get_parameter("poll_rate_hz").value
        self._frame_id = self.get_parameter("camera_frame_id").value

        devices = list_realsense_devices()
        if not devices:
            raise RuntimeError(
                "No RealSense device found by pyrealsense2. On WSL2, attach "
                "it first: `usbipd bind --busid <BUSID>` (admin, once) then "
                "`usbipd attach --wsl --busid <BUSID>` -- `usbipd list` on "
                "Windows shows the busid, look for 'Intel(R) RealSense'."
            )
        self.get_logger().info(f"Using RealSense: {devices[0][1]} ({devices[0][0]})")

        self._cap = RealSenseCapture()
        if not self._cap.isOpened():
            raise RuntimeError("RealSense device found but its color/depth "
                                "pipeline failed to start.")
        for _ in range(10):  # let auto-exposure settle, same as live_demo.py
            self._cap.read()

        self._tracker = HumanHandTracker()

        self._yolo_model = None
        if use_yolo:
            from ultralytics import YOLO

            sys.path.insert(0, str(_REPO_ROOT / "yolo"))
            from smoke_test import serialize_detections  # noqa: E402

            self._serialize_detections = serialize_detections
            self.get_logger().info("Loading YOLO model...")
            self._yolo_model = YOLO(str(_REPO_ROOT / "yolo" / "yolo26n.pt"))

        self._hands_pub = self.create_publisher(PoseArray, "/vision/hands", 10)
        self._markers_pub = self.create_publisher(MarkerArray, "/vision/humans_markers", 10)
        self._json_pub = self.create_publisher(String, "/vision/humans_json", 10)
        self._objects_pub = None
        if self._yolo_model is not None:
            self._objects_pub = self.create_publisher(String, "/vision/objects_json", 10)

        self._frame_count = 0
        self._fps_t0 = time.time()

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f"realsense_vision_bridge: publishing at {rate_hz} Hz "
            f"(objects: {'on' if self._yolo_model is not None else 'off'})"
        )

    def _tick(self):
        ok, bgr = self._cap.read()
        if not ok:
            self.get_logger().warning("Failed to read a frame from the RealSense.")
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._tracker.detect(mp_image)

        stamp = self.get_clock().now().to_msg()
        self._hands_pub.publish(build_hands_msg(result, stamp, self._frame_id))
        self._markers_pub.publish(build_markers_msg(result, stamp, self._frame_id))
        self._json_pub.publish(String(data=json.dumps(result)))

        if self._yolo_model is not None:
            yolo_result = self._yolo_model.predict(
                source=bgr, conf=0.25, device="cpu", verbose=False
            )[0]
            detections = self._serialize_detections(yolo_result)
            height, width = bgr.shape[:2]
            self._objects_pub.publish(String(data=json.dumps({
                "objects": detections, "frame_width": width, "frame_height": height,
            })))

        self._frame_count += 1
        elapsed = time.time() - self._fps_t0
        if elapsed >= 5.0:
            fps = self._frame_count / elapsed
            self.get_logger().info(f"~{fps:.1f} fps")
            self._frame_count = 0
            self._fps_t0 = time.time()

    def destroy_node(self):
        self._cap.release()
        self._tracker.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-yolo", action="store_true",
                         help="skip object detection, only publish person/hand topics")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RealsenseVisionNode(use_yolo=not args.no_yolo)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
