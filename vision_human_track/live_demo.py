"""Live webcam demo: opens a camera, runs HumanHandTracker on every frame,
draws skeleton/palm-center/orientation overlay, shows it in a window.

    python3 live_demo.py              # camera index 0
    python3 live_demo.py 1            # camera index 1
    python3 live_demo.py --realsense  # use an attached Intel RealSense
                                       # (D435 etc.) via pyrealsense2 instead
                                       # of a plain UVC index -- needed
                                       # because a RealSense exposes several
                                       # /dev/video* nodes (depth, IR x2,
                                       # color) and a bare index guess is
                                       # unreliable; `pip install
                                       # pyrealsense2` in this venv first
    python3 live_demo.py --ros2       # also publish /vision/hands,
                                       # /vision/humans_markers, /vision/humans_json
                                       # (source /opt/ros/humble/setup.bash +
                                       # RMW_IMPLEMENTATION first, same as
                                       # ros2_vision_bridge/README.md)
    python3 live_demo.py --yolo       # also detect objects with ../yolo/'s
                                       # YOLO26 model (needs `pip install
                                       # ultralytics` in this venv)

Press 'q' in the window to quit.

--ros2 builds and publishes the same topics as
ros2_vision_bridge/vision_bridge_node.py, but in-process from this script's
own camera read/detect loop instead of polling vision_human_track's REST API
-- so it works without api.py running, and without two processes fighting
over the same camera device. Don't run this alongside vision_bridge_node.py
at the same time -- both would publish onto the same topic names.

--yolo runs ../yolo/'s object-detection model (a teammate's module, not
part of this user's human/hand assignment) on every frame alongside
PoseLandmarker/HandLandmarker, drawing both overlays on the same window and
printing both to the console. Two CPU models per frame is heavier than
either alone -- expect a real FPS drop versus running just one.
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
from test_real_image import draw_result  # noqa: E402


POINTS_TO_PRINT = [0, 1, 4, 5, 8, 17, 20]


def _print_hand(hand, prefix="    "):
    print(f"{prefix}hand={hand['handedness']} "
          f"palm_center={[round(v, 3) for v in hand['palm_center']]} "
          f"normal={[round(v, 3) for v in hand['normal']]}")
    for idx in POINTS_TO_PRINT:
        lm = hand["landmarks"][idx]
        print(f"{prefix}  point {idx}: "
              f"x={lm['x']:.3f} y={lm['y']:.3f} z={lm['z']:.3f}")


def print_coords(result, yolo_detections=None):
    if not result["humans"] and not result["unassigned_hands"]:
        print("  (no people/hands detected)")
    for human in result["humans"]:
        print(f"  person {human['id']}")
        for hand in human["hands"]:
            _print_hand(hand)
    for hand in result["unassigned_hands"]:
        print("  unassigned")
        _print_hand(hand, prefix="    ")

    if yolo_detections is not None:
        if not yolo_detections:
            print("  (no objects detected)")
        for det in yolo_detections:
            print(f"  object {det['instance_name']} "
                  f"confidence={det['confidence']} xyxy={det['xyxy']}")


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
        "--yolo", action="store_true",
        help="also detect objects with ../yolo/'s YOLO26 model",
    )
    args = parser.parse_args()
    camera_index = args.camera_index

    yolo_model = None
    yolo_serialize = None
    yolo_plot = None
    yolo_device = "cpu"
    if args.yolo:
        import os

        yolo_dir = Path(__file__).resolve().parent.parent / "yolo"
        sys.path.insert(0, str(yolo_dir))
        from smoke_test import plot_instance_names, serialize_detections  # noqa: E402
        from ultralytics import YOLO  # noqa: E402

        yolo_serialize = serialize_detections
        yolo_plot = plot_instance_names
        yolo_device = os.environ.get("YOLO_DEVICE", "cpu")
        print(f"Loading YOLO model ({yolo_device})...")
        yolo_model = YOLO(str(yolo_dir / "yolo26n.pt"))

    ros_node = None
    ros_pubs = None
    if args.ros2:
        import rclpy
        from geometry_msgs.msg import PoseArray
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray

        sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ros2_vision_bridge"))
        from vision_bridge_node import build_hands_msg, build_markers_msg  # noqa: E402

        rclpy.init()
        ros_node = rclpy.create_node("live_demo_vision_publisher")
        ros_pubs = {
            "hands": ros_node.create_publisher(PoseArray, "/vision/hands", 10),
            "markers": ros_node.create_publisher(MarkerArray, "/vision/humans_markers", 10),
            "json": ros_node.create_publisher(String, "/vision/humans_json", 10),
        }
        print("ROS2 publishing enabled: /vision/hands, /vision/humans_markers, "
              "/vision/humans_json")

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

    cv2.namedWindow("UR_01 human+hand tracking", cv2.WINDOW_NORMAL)

    frame_count = 0
    fps_t0 = time.time()

    try:
        while True:
            ok, bgr = cap.read()
            if not ok:
                print("Failed to read a frame - camera may have disconnected.")
                break

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = tracker.detect(mp_image)

            if ros_node is not None:
                stamp = ros_node.get_clock().now().to_msg()
                ros_pubs["hands"].publish(build_hands_msg(result, stamp, "camera_link"))
                ros_pubs["markers"].publish(build_markers_msg(result, stamp, "camera_link"))
                ros_pubs["json"].publish(String(data=json.dumps(result)))
                rclpy.spin_once(ros_node, timeout_sec=0)

            annotated = draw_result(bgr, result)

            yolo_detections = None
            if yolo_model is not None:
                yolo_result = yolo_model.predict(
                    source=bgr, conf=0.25, device=yolo_device, verbose=False
                )[0]
                yolo_detections = yolo_serialize(yolo_result)
                annotated = yolo_plot(yolo_result, yolo_detections, image=annotated)

            frame_count += 1
            elapsed = time.time() - fps_t0
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                cv2.setWindowTitle("UR_01 human+hand tracking", f"live ({fps:.1f} fps)")
                frame_count = 0
                fps_t0 = time.time()
                print_coords(result, yolo_detections)

            cv2.imshow("UR_01 human+hand tracking", annotated)
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
