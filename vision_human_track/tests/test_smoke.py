"""Smoke test: models actually load and run inference without crashing.
Uses a blank image, so zero detections are expected - this only proves the
pipeline runs end-to-end, NOT that detection quality is good. Real accuracy
needs a live camera or a real photo of a person (not available yet in this
environment - see README known gaps).
"""
import sys
from pathlib import Path

import numpy as np
import mediapipe as mp

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from detector import HumanHandTracker  # noqa: E402


def main():
    print("Loading PoseLandmarker + HandLandmarker models...")
    tracker = HumanHandTracker()
    print("Models loaded OK.")

    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=blank)

    result = tracker.detect(mp_image)
    print(f"Inference ran OK. humans={len(result['humans'])} "
          f"unassigned_hands={len(result['unassigned_hands'])}")
    assert result["humans"] == []
    assert result["unassigned_hands"] == []

    tracker.close()
    print("\nSMOKE TEST PASSED (pipeline runs end-to-end).")
    print("NOTE: this does not verify detection accuracy - needs a real "
          "camera/photo test before trusting the output on an actual person.")


if __name__ == "__main__":
    main()
