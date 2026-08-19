"""Deterministic unit tests for palm-center/normal math - no camera or model needed."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from detector import _palm_center_and_normal, HumanHandTracker  # noqa: E402


def _lm(x, y, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _flat_right_hand_facing_camera():
    """21 landmarks approximating a flat right hand, palm facing the camera
    (i.e. facing -z, toward the viewer), fingers pointing up (+y is down in
    image coords, so 'up' is smaller y)."""
    lm = [None] * 21
    lm[0] = _lm(0.5, 0.6, 0.0)   # WRIST
    lm[5] = _lm(0.45, 0.4, 0.0)  # INDEX_MCP (thumb side, screen-left of wrist)
    lm[9] = _lm(0.5, 0.38, 0.0)  # MIDDLE_MCP
    lm[13] = _lm(0.55, 0.4, 0.0) # RING_MCP
    lm[17] = _lm(0.6, 0.42, 0.0) # PINKY_MCP (screen-right of wrist)
    for i in (1, 2, 3, 4, 6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20):
        lm[i] = _lm(0.5, 0.5, 0.0)
    return lm


def test_palm_center_is_mean_of_palm_landmarks():
    hand = _flat_right_hand_facing_camera()
    center, _ = _palm_center_and_normal(hand, "Right")
    expected = np.mean(
        [[hand[i].x, hand[i].y, hand[i].z] for i in (0, 5, 9, 13, 17)], axis=0
    )
    assert np.allclose(center, expected, atol=1e-6)


def test_normal_is_unit_length():
    hand = _flat_right_hand_facing_camera()
    _, normal = _palm_center_and_normal(hand, "Right")
    n = np.array(normal)
    assert abs(np.linalg.norm(n) - 1.0) < 1e-5


def test_normal_flips_sign_between_handedness():
    hand = _flat_right_hand_facing_camera()
    _, normal_right = _palm_center_and_normal(hand, "Right")
    _, normal_left = _palm_center_and_normal(hand, "Left")
    assert np.allclose(np.array(normal_right), -np.array(normal_left), atol=1e-6)


def test_degenerate_hand_returns_zero_normal_not_crash():
    hand = [_lm(0.5, 0.5, 0.0)] * 21  # all landmarks collapsed to one point
    center, normal = _palm_center_and_normal(hand, "Right")
    assert normal == [0.0, 0.0, 0.0]


def test_tracker_assigns_stable_id_to_similar_position():
    tracker = HumanHandTracker.__new__(HumanHandTracker)  # skip model loading
    tracker._tracks = []
    tracker._next_id = 1

    id_frame1 = tracker._assign_ids([np.array([0.5, 0.5, 0.0])])
    id_frame2 = tracker._assign_ids([np.array([0.51, 0.49, 0.0])])
    assert id_frame1 == id_frame2 == [1]


def test_tracker_assigns_new_id_to_far_away_person():
    tracker = HumanHandTracker.__new__(HumanHandTracker)
    tracker._tracks = []
    tracker._next_id = 1

    tracker._assign_ids([np.array([0.1, 0.1, 0.0])])
    ids = tracker._assign_ids([np.array([0.1, 0.1, 0.0]), np.array([0.9, 0.9, 0.0])])
    assert ids == [1, 2]


if __name__ == "__main__":
    import inspect
    failures = 0
    tests = {n: f for n, f in globals().items() if n.startswith("test_")}
    for name, fn in tests.items():
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as e:
            failures += 1
            print(f"FAIL {name}: {e}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    sys.exit(1 if failures else 0)
