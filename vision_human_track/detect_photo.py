"""Single-image human/hand (+ optional object) detection.

    python3 detect_photo.py path/to/image.jpg
    python3 detect_photo.py path/to/image.jpg --yolo

Unlike live_demo.py (continuous webcam loop, GUI only), this runs detection
once on a single image and writes the result to disk: an annotated image
(<stem>_detected.jpg) and the full detection data as JSON
(<stem>_detected.json), both saved next to the input image.

--yolo adds ../yolo/'s YOLO26 object detection on top of the usual
PoseLandmarker/HandLandmarker output, same as live_demo.py --yolo.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import mediapipe as mp

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent / "tests"))
from detector import HumanHandTracker  # noqa: E402
from test_real_image import draw_result  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path, help="input image path")
    parser.add_argument(
        "--yolo", action="store_true",
        help="also detect objects with ../yolo/'s YOLO26 model",
    )
    args = parser.parse_args()

    image_path = args.image.resolve()
    if not image_path.is_file():
        raise FileNotFoundError(f"Image not found: {image_path}")

    bgr = cv2.imread(str(image_path))
    if bgr is None:
        raise ValueError(f"Could not read image (unsupported format?): {image_path}")

    print("Loading MediaPipe models...")
    tracker = HumanHandTracker()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    result = tracker.detect(mp_image)
    tracker.close()

    annotated = draw_result(bgr, result)

    output = dict(result)
    if args.yolo:
        yolo_dir = Path(__file__).resolve().parent.parent / "yolo"
        sys.path.insert(0, str(yolo_dir))
        from smoke_test import plot_instance_names, serialize_detections  # noqa: E402
        from ultralytics import YOLO  # noqa: E402

        yolo_device = os.environ.get("YOLO_DEVICE", "cpu")
        print(f"Loading YOLO model ({yolo_device})...")
        yolo_model = YOLO(str(yolo_dir / "yolo26n.pt"))
        yolo_result = yolo_model.predict(
            source=bgr, conf=0.25, device=yolo_device, verbose=False
        )[0]
        objects = serialize_detections(yolo_result)
        annotated = plot_instance_names(yolo_result, objects, image=annotated)
        output["objects"] = objects

    n_humans = len(result["humans"])
    n_hands = sum(len(h["hands"]) for h in result["humans"]) + len(result["unassigned_hands"])
    n_objects = len(output["objects"]) if "objects" in output else None
    print(f"{image_path.name}: humans={n_humans} hands={n_hands}"
          + (f" objects={n_objects}" if n_objects is not None else ""))

    image_out_path = image_path.with_name(f"{image_path.stem}_detected.jpg")
    json_out_path = image_path.with_name(f"{image_path.stem}_detected.json")

    cv2.imwrite(str(image_out_path), annotated)
    with open(json_out_path, "w") as f:
        json.dump(output, f, indent=2)

    print(f"  -> saved {image_out_path}")
    print(f"  -> saved {json_out_path}")


if __name__ == "__main__":
    main()
