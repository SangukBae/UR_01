"""Same as live_demo.py, but the object-detection overlay uses RT-DETR
(a transformer-based detector, ultralytics.RTDETR) instead of ../yolo/'s
YOLO26 model. See live_demo.py's own docstring for the full option list --
only the detection backend differs, everything else (camera handling,
--realsense distance, --ros2 topics, HumanHandTracker) is identical.

    python3 live_demo_rtdetr.py              # camera index 0
    python3 live_demo_rtdetr.py --detect      # also run RT-DETR object
                                               # detection (needs `pip
                                               # install ultralytics` in
                                               # this venv, same package
                                               # YOLO26 uses)
    python3 live_demo_rtdetr.py --realsense --detect --ros2

RTDETR_DEVICE (default "cpu") and RTDETR_MODEL (default "rtdetr-x.pt")
env vars mirror live_demo.py's YOLO_DEVICE/YOLO_MODEL -- set
RTDETR_DEVICE=0 to run on a CUDA GPU. The weights auto-download to
../yolo/ (same directory YOLO26's weights live in) if not already there.
rtdetr-l.pt is the only smaller variant Ultralytics ships for this
architecture (no n/s/m sizes like YOLO26 has) -- x is what's benchmarked
in this repo's notes.

Reuses ../yolo/smoke_test.py's serialize_detections/plot_instance_names
(same as live_demo.py does) rather than duplicating them -- both just
operate on an ultralytics Results object's .boxes, which RTDETR produces
in the same shape YOLO does, so no changes needed there.

With --realsense --ros2 together, also publishes TF (via /tf): the fixed
flange -> camera_optical_frame mount (StaticTransformBroadcaster, same
CAMERA_R/CAMERA_T as ros2_vision_bridge/camera_tf_publisher.py -- imported
from there, not duplicated) published once at startup, plus (with --detect
too) a dynamic camera_optical_frame -> object_<instance_name> TF per
detected object every frame (real 3D point from RealSenseCapture.deproject,
same as realsense_vision_node.py's _publish_object_tfs -- shared via
vision_bridge_node.build_object_tfs, not duplicated). Same feature as
live_demo.py's --ros2 mode, just with RT-DETR standing in for YOLO26.

Press 'q' in the window to quit.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent / "tests"))
from detector import HumanHandTracker  # noqa: E402
from distance_utils import add_distances, hand_pixel  # noqa: E402
from test_real_image import draw_result  # noqa: E402


POINTS_TO_PRINT = [0, 1, 4, 5, 8, 17, 20]


def draw_distances(image, result, detections, width, height):
    """Separate overlay pass, after draw_result()/det_plot() have already
    drawn the skeleton/box overlays -- keeps this module's own distance
    text independent of test_real_image.py's draw_result and RT-DETR's
    plot function, rather than reaching into either."""
    all_hands = list(result["unassigned_hands"])
    for human in result["humans"]:
        all_hands.extend(human["hands"])
    for hand in all_hands:
        distance = hand.get("distance_m")
        if distance is None:
            continue
        x, y = hand_pixel(hand, width, height)
        cv2.putText(image, f"{distance:.2f}m", (x + 8, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    if detections:
        for det in detections:
            distance = det.get("distance_m")
            if distance is None:
                continue
            x1, _, _, y2 = (int(v) for v in det["xyxy"])
            cv2.putText(image, f"{distance:.2f}m", (x1, int(y2) + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    return image


def _print_hand(hand, prefix="    "):
    distance = hand.get("distance_m")
    distance_str = f"{distance:.2f}m" if distance is not None else "n/a"
    print(f"{prefix}hand={hand['handedness']} "
          f"palm_center={[round(v, 3) for v in hand['palm_center']]} "
          f"normal={[round(v, 3) for v in hand['normal']]} "
          f"distance={distance_str}")
    for idx in POINTS_TO_PRINT:
        lm = hand["landmarks"][idx]
        print(f"{prefix}  point {idx}: "
              f"x={lm['x']:.3f} y={lm['y']:.3f} z={lm['z']:.3f}")


def print_coords(result, detections=None):
    if not result["humans"] and not result["unassigned_hands"]:
        print("  (no people/hands detected)")
    for human in result["humans"]:
        print(f"  person {human['id']}")
        for hand in human["hands"]:
            _print_hand(hand)
    for hand in result["unassigned_hands"]:
        print("  unassigned")
        _print_hand(hand, prefix="    ")

    if detections is not None:
        if not detections:
            print("  (no objects detected)")
        for det in detections:
            distance = det.get("distance_m")
            distance_str = f"{distance:.2f}m" if distance is not None else "n/a"
            print(f"  object {det['instance_name']} "
                  f"confidence={det['confidence']} xyxy={det['xyxy']} "
                  f"distance={distance_str}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("camera_index", nargs="?", type=int, default=0)
    parser.add_argument(
        "--realsense", action="store_true",
        help="use an attached Intel RealSense camera (via pyrealsense2) "
             "instead of camera_index/cv2.VideoCapture",
    )
    parser.add_argument(
        "--ros2", action="store_true",
        help="also publish /vision/hands, /vision/humans_markers, "
             "/vision/humans_json (same topics as ros2_vision_bridge)",
    )
    parser.add_argument(
        "--detect", action="store_true",
        help="also detect objects with RT-DETR (ultralytics.RTDETR, "
             "rtdetr-x.pt by default)",
    )
    args = parser.parse_args()
    camera_index = args.camera_index

    det_model = None
    det_serialize = None
    det_plot = None
    det_device = "cpu"
    if args.detect:
        import os

        yolo_dir = Path(__file__).resolve().parent.parent / "yolo"
        sys.path.insert(0, str(yolo_dir))
        from smoke_test import plot_instance_names, serialize_detections  # noqa: E402
        from ultralytics import RTDETR  # noqa: E402

        det_serialize = serialize_detections
        det_plot = plot_instance_names
        det_device = os.environ.get("RTDETR_DEVICE", "cpu")
        det_model_name = os.environ.get("RTDETR_MODEL", "rtdetr-x.pt")
        print(f"Loading RT-DETR model {det_model_name} ({det_device})...")
        det_model = RTDETR(str(yolo_dir / det_model_name))

    ros_node = None
    ros_pubs = None
    ros_tf_broadcaster = None
    camera_frame_id = "camera_optical_frame"
    if args.ros2:
        import rclpy
        from geometry_msgs.msg import PoseArray, TransformStamped
        from std_msgs.msg import String
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        from visualization_msgs.msg import MarkerArray

        sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ros2_vision_bridge"))
        from camera_tf_publisher import CAMERA_R, CAMERA_T  # noqa: E402
        from geometry import matrix_to_quaternion  # noqa: E402
        from vision_bridge_node import (  # noqa: E402
            build_hands_msg,
            build_markers_msg,
            build_object_tfs,
        )

        rclpy.init()
        ros_node = rclpy.create_node("live_demo_rtdetr_vision_publisher")
        ros_pubs = {
            "hands": ros_node.create_publisher(PoseArray, "/vision/hands", 10),
            "markers": ros_node.create_publisher(MarkerArray, "/vision/humans_markers", 10),
            "json": ros_node.create_publisher(String, "/vision/humans_json", 10),
        }
        topics = "/vision/hands, /vision/humans_markers, /vision/humans_json"
        if args.detect:
            ros_pubs["objects"] = ros_node.create_publisher(String, "/vision/objects_json", 10)
            topics += ", /vision/objects_json"
        print(f"ROS2 publishing enabled: {topics}")

        if args.realsense:
            # Same flange -> camera_optical_frame mount as
            # ros2_vision_bridge/camera_tf_publisher.py -- imported from
            # there rather than redefined, so there's one source of truth
            # for this calibration. See that module for the derivation and
            # the "not yet empirically confirmed" caveat.
            qx, qy, qz, qw = matrix_to_quaternion(CAMERA_R)
            static_tf = TransformStamped()
            static_tf.header.stamp = ros_node.get_clock().now().to_msg()
            static_tf.header.frame_id = "flange"
            static_tf.child_frame_id = camera_frame_id
            tx, ty, tz = CAMERA_T
            static_tf.transform.translation.x = tx
            static_tf.transform.translation.y = ty
            static_tf.transform.translation.z = tz
            static_tf.transform.rotation.x = qx
            static_tf.transform.rotation.y = qy
            static_tf.transform.rotation.z = qz
            static_tf.transform.rotation.w = qw
            # Held in a variable (not a throwaway) so it isn't garbage
            # collected -- StaticTransformBroadcaster's latched publisher
            # needs to stay alive for the life of the node, same reason
            # camera_tf_publisher.py keeps it on `self`.
            static_tf_broadcaster = StaticTransformBroadcaster(ros_node)
            static_tf_broadcaster.sendTransform(static_tf)
            print(f"Published static TF 'flange' -> {camera_frame_id!r}")

            if args.detect:
                ros_tf_broadcaster = TransformBroadcaster(ros_node)

    if args.realsense:
        from realsense_camera import RealSenseCapture, list_realsense_devices

        devices = list_realsense_devices()
        if not devices:
            print("No RealSense device found by pyrealsense2.")
            print("If this is WSL2: attach it first with usbipd-win, e.g.\n"
                  "  usbipd bind --busid <BUSID>      # once, as admin\n"
                  "  usbipd attach --wsl --busid <BUSID>\n"
                  "(`usbipd list` on Windows shows the busid/VID:PID, "
                  "look for 'Intel(R) RealSense').")
            sys.exit(1)
        print(f"Using RealSense: {devices[0][1]} ({devices[0][0]})")
        cap = RealSenseCapture()
        if not cap.isOpened():
            print("Found the RealSense device but pyrealsense2 could not "
                  "start its color+depth pipeline.")
            sys.exit(1)
    else:
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print(f"Could not open camera index {camera_index}.")
            print("If this is WSL2: the camera needs to be attached first via "
                  "usbipd-win (see README.md's 'Known gaps' section). Check "
                  "`ls /dev/video*` - if nothing's there, the camera isn't "
                  "visible to Linux yet.")
            sys.exit(1)

        # Forcing MJPG (instead of the default raw YUYV) fixed a solid-green-
        # frame bug seen over the usbipd-win/vhci_hcd USB passthrough in WSL2
        # - raw YUYV apparently doesn't survive that transport reliably, MJPG
        # does. Doesn't apply to RealSense, which streams via pyrealsense2's
        # own librealsense pipeline, not cv2's V4L2 backend.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

    # A few frames are also just genuinely blank/dark while the sensor's
    # auto-exposure ramps up (true for both UVC webcams and RealSense), so
    # read-and-discard a handful before showing anything - verified live for
    # the webcam case: mean pixel value climbed frame over frame instead of
    # a flat green until frame ~5-10.
    print("Warming up the camera...")
    for _ in range(10):
        cap.read()

    print("Loading MediaPipe models...")
    tracker = HumanHandTracker()
    print("Ready. Press 'q' in the video window to quit.")

    cv2.namedWindow("UR_01 human+hand tracking (RT-DETR)", cv2.WINDOW_NORMAL)

    frame_count = 0
    fps_t0 = time.time()

    try:
        while True:
            ok, bgr = cap.read()
            if not ok:
                print("Failed to read a frame - camera may have disconnected.")
                break

            height, width = bgr.shape[:2]
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = tracker.detect(mp_image)

            detections = None
            if det_model is not None:
                det_result = det_model.predict(
                    source=bgr, conf=0.25, device=det_device, verbose=False
                )[0]
                detections = det_serialize(det_result)

            if args.realsense:
                add_distances(result, detections, cap, width, height)

            if ros_node is not None:
                stamp = ros_node.get_clock().now().to_msg()
                ros_pubs["hands"].publish(build_hands_msg(result, stamp, camera_frame_id))
                ros_pubs["markers"].publish(build_markers_msg(result, stamp, camera_frame_id))
                ros_pubs["json"].publish(String(data=json.dumps(result)))
                if "objects" in ros_pubs and detections is not None:
                    ros_pubs["objects"].publish(String(data=json.dumps({
                        "objects": detections, "frame_width": width, "frame_height": height,
                    })))
                    if ros_tf_broadcaster is not None:
                        transforms = build_object_tfs(detections, cap, camera_frame_id, stamp)
                        if transforms:
                            ros_tf_broadcaster.sendTransform(transforms)
                rclpy.spin_once(ros_node, timeout_sec=0)

            annotated = draw_result(bgr, result)
            if detections is not None:
                annotated = det_plot(det_result, detections, image=annotated)
            if args.realsense:
                annotated = draw_distances(annotated, result, detections, width, height)

            frame_count += 1
            elapsed = time.time() - fps_t0
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                cv2.setWindowTitle("UR_01 human+hand tracking (RT-DETR)", f"live ({fps:.1f} fps)")
                frame_count = 0
                fps_t0 = time.time()
                print_coords(result, detections)

            cv2.imshow("UR_01 human+hand tracking (RT-DETR)", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        tracker.close()
        if ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
