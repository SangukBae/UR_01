"""Live webcam demo: opens a camera, runs HumanHandTracker on every frame,
draws skeleton/palm-center/orientation overlay, shows it in a window.

    python3 live_demo.py              # camera index 0
    python3 live_demo.py 1            # camera index 1

Press 'q' in the window to quit.
"""
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent / "tests"))
from detector import HumanHandTracker  # noqa: E402
from test_real_image import draw_result  # noqa: E402


def main():
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Could not open camera index {camera_index}.")
        print("If this is WSL2: the camera needs to be attached first via "
              "usbipd-win (see README.md's 'Known gaps' section). Check "
              "`ls /dev/video*` - if nothing's there, the camera isn't "
              "visible to Linux yet.")
        sys.exit(1)

    # Forcing MJPG (instead of the default raw YUYV) fixed a solid-green-frame
    # bug seen over the usbipd-win/vhci_hcd USB passthrough in WSL2 - raw
    # YUYV apparently doesn't survive that transport reliably, MJPG does.
    # A few frames are also just genuinely blank/dark while the sensor's
    # auto-exposure ramps up, so read-and-discard a handful before showing
    # anything - verified live: mean pixel value climbed frame over frame
    # instead of a flat green until frame ~5-10.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
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

            annotated = draw_result(bgr, result)

            frame_count += 1
            elapsed = time.time() - fps_t0
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                cv2.setWindowTitle("UR_01 human+hand tracking", f"live ({fps:.1f} fps)")
                frame_count = 0
                fps_t0 = time.time()

            cv2.imshow("UR_01 human+hand tracking", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        tracker.close()


if __name__ == "__main__":
    main()
