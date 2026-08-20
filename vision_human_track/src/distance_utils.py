"""Depth lookup shared by every camera-owning entry point (live_demo.py,
live_demo_rtdetr.py, realsense_vision_node.py, and the Docker API service
in api.py) -- previously duplicated between live_demo.py and
live_demo_rtdetr.py, consolidated here so there's one place to fix a bug.
"""


def hand_pixel(hand, width, height):
    """Palm center's normalized (x, y) -> color-frame pixel coords, for a
    RealSense depth lookup (get_distance needs pixel indices, not [0,1])."""
    px, py, _ = hand["palm_center"]
    return int(px * width), int(py * height)


def add_distances(result, detections, cap, width, height):
    """Fills in a `distance_m` field (float or None) on every hand and
    object, using `cap.get_distance` -- only present when `cap` is a
    RealSenseCapture (plain cv2.VideoCapture has no depth). Mutates the
    dicts in place, same convention as the rest of this module's data
    flow (e.g. HumanHandTracker.detect's own result dict)."""
    if not hasattr(cap, "get_distance"):
        return
    all_hands = list(result["unassigned_hands"])
    for human in result["humans"]:
        all_hands.extend(human["hands"])
    for hand in all_hands:
        x, y = hand_pixel(hand, width, height)
        hand["distance_m"] = cap.get_distance(x, y)

    if detections:
        for det in detections:
            x1, y1, x2, y2 = det["xyxy"]
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            det["distance_m"] = cap.get_distance(cx, cy)
