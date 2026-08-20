#!/usr/bin/env python3
"""Static TF: the RealSense camera's mounting pose relative to the robot's
flange.

The camera is bolted to the flange at a fixed offset/orientation, so this
is published once with a StaticTransformBroadcaster (latched -- any late
subscriber, e.g. a fresh RViz, still gets it) rather than re-published
every tick like the per-object TFs in realsense_vision_node.py.

No colcon package here on purpose, matching this repo's existing
ros2_ur_driver/ros2_client.py and vision_bridge_node.py convention (a
plain rclpy script, not an ament package) -- run directly:

    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # see ros2_ur_driver/README.md
    python3 ros2_vision_bridge/camera_tf_publisher.py

Combined with realsense_vision_node.py's per-object TFs
(camera_optical_frame -> object_<name>), this lets any TF2 consumer ask
for an object's pose directly in the flange frame (flange ->
camera_optical_frame -> object_<name>) with no extra code -- tf2 composes
the chain automatically.

Published frame is named "camera_optical_frame" (not the more common ROS
"camera_link") on purpose: CAMERA_R below is defined directly in the
camera's optical convention (X right, Y down, Z forward -- same as
RealSenseCapture.deproject()'s output), not the REP-103 physical-mount
convention "camera_link" usually means (X forward, Y left, Z up). Naming
it accurately avoids a future mix-up between the two.
"""
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parent))
from geometry import matrix_to_quaternion  # noqa: E402

# Camera mounting extrinsics relayed by a teammate (2026-08-20 message):
# rotation of the camera's optical frame (X right, Y down, Z forward --
# same convention RealSenseCapture.deproject() outputs points in, see
# vision_human_track/src/realsense_camera.py) expressed in the flange
# frame, plus the camera's translation offset from the flange origin, in
# metres.
#
# As given, det(CAMERA_R) = -1 -- a reflection, not a rotation, so it has
# no quaternion representation (matrix_to_quaternion below would raise
# ValueError rather than publish something silently wrong). Very likely a
# single mistyped sign; two equally-plausible single-sign fixes both give
# a valid rotation (det=+1):
#   (a) flip row 3, col 1:  [-1, 0, 0] -> [1, 0, 0]
#   (b) flip row 1, col 3:  [0, 0, -1] -> [0, 0, 1]   <- used below
#
# Disambiguated 2026-08-20 using /opt/ros/humble/share/ur_description/
# urdf/ur_macro.xacro (ROS-Industrial's standard UR description, not a
# guess) -- computing flange -> tool0's rotation (rpy = (pi/2, 0, pi/2))
# and using its documented "X+ left, Y+ up, Z+ front" tool0 axes shows
# **flange's own +X is the standard "front"/tool-forward direction**
# (the direction the arm reaches outward, away from the robot body).
# A photo of the real mount shows the RealSense extending straight out
# from wrist_3 in that same outward direction -- i.e. the camera's
# forward axis (optical Z, this matrix's 3rd column) must map to flange
# +X. Column 3 of the literal matrix is (-1, 0, 0) -- backward -- so (a)
# (which leaves column 3 unchanged) is wrong, and (b) (which flips it to
# (1, 0, 0)) is the one consistent with both the photo and the standard
# flange convention. Still NOT empirically confirmed against a real
# depth reading -- verify once with a real object: place something on
# the side the camera looks (away from the robot body), run
# realsense_vision_node.py, and check with `ros2 run tf2_ros tf2_echo
# flange object_<name>` that the reported position actually matches
# where the object physically is.
CAMERA_R = [
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
]
CAMERA_T = (0.0, -0.06, 0.02)


class CameraTfPublisher(Node):
    def __init__(self):
        super().__init__("camera_tf_publisher")
        self.declare_parameter("parent_frame", "flange")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        parent = self.get_parameter("parent_frame").value
        child = self.get_parameter("camera_frame").value

        qx, qy, qz, qw = matrix_to_quaternion(CAMERA_R)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tx, ty, tz = CAMERA_T
        tf.transform.translation.x = tx
        tf.transform.translation.y = ty
        tf.transform.translation.z = tz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(tf)
        self.get_logger().info(f"Published static TF {parent!r} -> {child!r}")


def main():
    rclpy.init()
    node = CameraTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
