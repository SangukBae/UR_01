"""Minimal Ultralytics YOLO inference smoke test.

The test passes when the model loads, inference completes, at least one object
is detected, and an annotated image is written successfully.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
from typing import Any

import cv2
from ultralytics import YOLO


HERE = Path(__file__).resolve().parent
MODEL = os.environ.get("YOLO_MODEL", "yolo26n.pt")


def serialize_detections(result: Any) -> list[dict[str, object]]:
    """Convert every YOLO box into a JSON-serializable detection."""
    detections = []
    class_counts: dict[str, int] = defaultdict(int)
    for box in result.boxes:
        class_id = int(box.cls.item())
        class_name = result.names[class_id]
        class_counts[class_name] += 1
        detections.append(
            {
                "class_id": class_id,
                "class_name": class_name,
                "instance_name": f"{class_name}{class_counts[class_name]}",
                "confidence": round(float(box.conf.item()), 3),
                "xyxy": [round(value, 1) for value in box.xyxy[0].tolist()],
            }
        )
    return detections


def plot_instance_names(
    result: Any, detections: list[dict[str, object]], image: Any = None
) -> Any:
    """Draw boxes labelled with per-class instance names.

    Draws onto `image` if given (e.g. a frame with other overlays already
    on it), otherwise a fresh copy of `result.orig_img`.
    """
    if image is None:
        image = result.orig_img.copy()
    color = (0, 200, 0)
    font = cv2.FONT_HERSHEY_SIMPLEX

    for detection in detections:
        x1, y1, x2, y2 = (int(value) for value in detection["xyxy"])
        label = (
            f"{detection['instance_name']} "
            f"{float(detection['confidence']):.2f}"
        )

        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        (text_width, text_height), baseline = cv2.getTextSize(
            label, font, 0.6, 2
        )
        label_top = max(0, y1 - text_height - baseline - 6)
        cv2.rectangle(
            image,
            (x1, label_top),
            (x1 + text_width + 6, y1),
            color,
            thickness=-1,
        )
        cv2.putText(
            image,
            label,
            (x1 + 3, max(text_height, y1 - baseline - 3)),
            font,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    return image


def main() -> None:
    parser = argparse.ArgumentParser(description="Run YOLO on one image")
    parser.add_argument(
        "image",
        nargs="?",
        type=Path,
        default=HERE / "test.png",
        help="input image (default: test.png beside this script)",
    )
    args = parser.parse_args()
    image_path = args.image.resolve()
    output_path = image_path.with_name(f"{image_path.stem}_detected.jpg")

    if not image_path.is_file():
        raise FileNotFoundError(f"Test image not found: {image_path}")

    device = os.environ.get("YOLO_DEVICE", "cpu")
    print(f"Loading model: {MODEL}")
    model = YOLO(MODEL)
    result = model.predict(
        source=str(image_path),
        conf=0.25,
        device=device,
        verbose=False,
    )[0]

    detections = serialize_detections(result)

    if not detections:
        raise AssertionError("Inference completed but detected no objects")

    annotated_image = plot_instance_names(result, detections)
    if not cv2.imwrite(str(output_path), annotated_image):
        raise AssertionError(f"Could not write annotated output: {output_path}")
    if output_path.stat().st_size == 0:
        raise AssertionError(f"Annotated output was not written: {output_path}")

    smoke_test_result = {
        "status": "passed",
        "model": MODEL,
        "device": device,
        "input_image": str(image_path),
        "annotated_image": str(output_path),
        "detection_count": len(detections),
        "detections": detections,
    }
    print(json.dumps(smoke_test_result, indent=2))


if __name__ == "__main__":
    main()
