"""Real-photo detection test: runs the actual detector on real person photos
and draws the results, so output can be visually verified (not just 'ran
without crashing'). Saves annotated images to tests/sample_images/*_out.jpg.
"""
import sys
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from detector import HumanHandTracker  # noqa: E402

SAMPLES_DIR = Path(__file__).resolve().parent / "sample_images"

# Standard MediaPipe 21-point hand skeleton bone connections (wrist=0,
# 4 joints per finger: MCP/PIP/DIP/TIP, thumb/index/middle/ring/pinky).
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),        # index
    (5, 9), (9, 10), (10, 11), (11, 12),   # middle
    (9, 13), (13, 14), (14, 15), (15, 16), # ring
    (13, 17), (17, 18), (18, 19), (19, 20),# pinky
    (0, 17),                               # palm base
]


def _draw_hand(out, hand, w, h):
    pts = [(int(lm["x"] * w), int(lm["y"] * h)) for lm in hand["landmarks"]]
    for a, b in HAND_CONNECTIONS:
        cv2.line(out, pts[a], pts[b], (0, 200, 255), 2)
    for x, y in pts:
        cv2.circle(out, (x, y), 3, (0, 200, 255), -1)

    px, py = hand["palm_center"][0], hand["palm_center"][1]
    cx, cy = int(px * w), int(py * h)
    cv2.circle(out, (cx, cy), 8, (0, 0, 255), -1)
    nx, ny = hand["normal"][0], hand["normal"][1]
    tip = (int(cx + nx * 100), int(cy + ny * 100))
    cv2.arrowedLine(out, (cx, cy), tip, (255, 0, 0), 2)
    cv2.putText(out, hand["handedness"], (cx + 10, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)


def draw_result(bgr_image, result):
    h, w = bgr_image.shape[:2]
    out = bgr_image.copy()

    for human in result["humans"]:
        for lm in human["skeleton"]:
            if lm["visibility"] < 0.3:
                continue
            cx, cy = int(lm["x"] * w), int(lm["y"] * h)
            cv2.circle(out, (cx, cy), 3, (0, 255, 0), -1)
        for hand in human["hands"]:
            _draw_hand(out, hand, w, h)
        label_lm = human["skeleton"][0]
        cv2.putText(out, f"ID {human['id']}",
                    (int(label_lm["x"] * w), int(label_lm["y"] * h) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    for hand in result["unassigned_hands"]:
        _draw_hand(out, hand, w, h)

    return out


def run_on(tracker, image_path):
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        print(f"COULD NOT READ {image_path}")
        return
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    result = tracker.detect(mp_image)
    n_humans = len(result["humans"])
    n_hands_assigned = sum(len(h["hands"]) for h in result["humans"])
    n_hands_unassigned = len(result["unassigned_hands"])
    print(f"{image_path.name}: humans={n_humans} "
          f"hands_assigned={n_hands_assigned} hands_unassigned={n_hands_unassigned}")

    for h in result["humans"]:
        for hand in h["hands"]:
            print(f"  person {h['id']} hand={hand['handedness']} "
                  f"palm_center={[round(v, 3) for v in hand['palm_center']]} "
                  f"normal={[round(v, 3) for v in hand['normal']]}")
    for hand in result["unassigned_hands"]:
        print(f"  unassigned hand={hand['handedness']} "
              f"palm_center={[round(v, 3) for v in hand['palm_center']]} "
              f"normal={[round(v, 3) for v in hand['normal']]}")

    annotated = draw_result(bgr, result)
    out_path = image_path.with_name(image_path.stem + "_out.jpg")
    cv2.imwrite(str(out_path), annotated)
    print(f"  -> saved {out_path}")
    return result


def main():
    tracker = HumanHandTracker()
    for name in ("pose.jpg", "woman_hands.jpg"):
        run_on(tracker, SAMPLES_DIR / name)
    tracker.close()


if __name__ == "__main__":
    main()
