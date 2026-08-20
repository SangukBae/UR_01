"""TF-based lookup of a detected object's position in the robot's base
frame, for move_robot_to_object in server.py.

Composes three transforms already published elsewhere in this project's
vision/robot stack, via tf2 -- no manual forward-kinematics/matrix math
needed here, tf2 does the composition:

    base_link -> ... -> flange       ur_robot_driver's robot_state_publisher,
                                      from live joint states (this is the
                                      same "base_link" motion_planner.py
                                      plans against -- PLANNING_FRAME there).
    flange -> camera_optical_frame   ros2_vision_bridge/camera_tf_publisher.py,
                                      or live_demo.py/live_demo_rtdetr.py
                                      --realsense --ros2 (same CAMERA_R/
                                      CAMERA_T either way, one source of
                                      truth -- see that module).
    camera_optical_frame ->          realsense_vision_node.py, or
      object_<name>                  live_demo.py/live_demo_rtdetr.py
                                      --realsense --detect/--yolo --ros2.

All three need to be live in the same ROS graph for a lookup to succeed --
see move_robot_to_object's docstring in server.py for the checklist.

CAUTION: flange -> camera_optical_frame (CAMERA_R in camera_tf_publisher.py)
is not yet empirically confirmed against a real depth reading -- see that
module's comments. A position from this module could be systematically
wrong (wrong side, wrong distance) until that's verified with a real
object at a known position.
"""
from __future__ import annotations

import atexit
import re
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, ExtrapolationException, LookupException, TransformListener

BASE_FRAME = "base_link"  # matches motion_planner.py's PLANNING_FRAME


class VisionTF:
    """Connection to the TF tree for looking up object_<name> frames.

    A separate rclpy node/connection from ros2_client.py and
    motion_planner.py, same pattern as both (own node, background spin
    thread, atexit cleanup) -- works under UR_BACKEND=socket too, since
    this only reads TF, never commands motion itself.
    """

    def __init__(self) -> None:
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._stop_spinning = threading.Event()
        self._buffer = None

    def connect(self) -> None:
        if self._node is not None:
            return
        if not rclpy.ok():
            rclpy.init(args=[])
        self._node = Node("vision_tf_lookup")
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self._node)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        atexit.register(self.disconnect)

    def _spin(self) -> None:
        while not self._stop_spinning.is_set():
            self._executor.spin_once(timeout_sec=0.1)

    def disconnect(self) -> None:
        if self._node is None:
            return
        self._stop_spinning.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._node.destroy_node()
        self._node = None

    def find_object(self, name: str, timeout_s: float = 3.0) -> dict:
        """Find a currently-published object_<name>* TF frame and return
        its position in the base frame.

        ``name`` matches the detector's class name (e.g. "apple"), not one
        specific instance -- multiple detected instances get suffixed
        counts (object_apple1, object_apple2, ... -- see
        ../yolo/smoke_test.py's serialize_detections). If more than one
        matches, returns whichever is closest to the robot's base origin
        (a reasonable default when the caller doesn't care which specific
        instance).

        Args:
            name: Object class name to search for, e.g. "apple". Matched
                case-insensitively as a prefix against object_<name><n>.
            timeout_s: How long to wait for a matching frame to appear (the
                detector may not have published one yet this tick, or the
                object may just now be out of view).

        Returns:
            {"frame_id": "object_apple1", "position_m": [x, y, z]} in the
            base frame.

        Raises:
            RuntimeError: no matching frame found within timeout_s, or the
                TF tree is missing a link in the chain (camera or robot
                driver TF not being published) -- the message says which.
        """
        self.connect()
        pattern = re.compile(rf"^object_{re.escape(name)}\d*$", re.IGNORECASE)

        deadline = time.monotonic() + timeout_s
        matches: list[str] = []
        while time.monotonic() < deadline:
            for line in self._buffer.all_frames_as_yaml().splitlines():
                if not line or line[0].isspace():
                    continue
                frame_name = line.split(":", 1)[0].strip()
                if pattern.match(frame_name):
                    matches.append(frame_name)
            if matches:
                break
            time.sleep(0.1)

        if not matches:
            raise RuntimeError(
                f"No object matching '{name}' found in the TF tree within "
                f"{timeout_s}s. Checklist: is a camera node with object "
                "detection + --ros2 running (live_demo_rtdetr.py --realsense "
                f"--detect --ros2, or realsense_vision_node.py); is '{name}' "
                "actually in view right now; is camera_tf_publisher.py (or "
                "the equivalent --ros2 run) publishing the static "
                "flange -> camera_optical_frame TF?"
            )

        best_frame_id = None
        best_position = None
        best_dist = None
        for frame_id in matches:
            try:
                tf = self._buffer.lookup_transform(BASE_FRAME, frame_id, rclpy.time.Time())
            except (LookupException, ExtrapolationException):
                continue
            t = tf.transform.translation
            dist = (t.x ** 2 + t.y ** 2 + t.z ** 2) ** 0.5
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_frame_id = frame_id
                best_position = [t.x, t.y, t.z]

        if best_frame_id is None:
            raise RuntimeError(
                f"Found object frame(s) {matches} but couldn't resolve "
                f"'{BASE_FRAME} -> <frame>' -- is the robot driver's "
                "robot_state_publisher running (base_link -> ... -> flange)?"
            )

        return {"frame_id": best_frame_id, "position_m": [round(v, 4) for v in best_position]}
