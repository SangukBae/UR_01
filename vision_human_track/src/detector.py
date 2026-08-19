"""Core detection logic: MediaPipe PoseLandmarker (human skeleton + ID)
combined with HandLandmarker (palm center + orientation), no camera I/O here.
"""
import time
from dataclasses import dataclass
from pathlib import Path

import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    HandLandmarker,
    HandLandmarkerOptions,
    PoseLandmarker,
    PoseLandmarkerOptions,
    RunningMode,
)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
POSE_MODEL_PATH = str(_PROJECT_ROOT / "models" / "pose_landmarker_lite.task")
HAND_MODEL_PATH = str(_PROJECT_ROOT / "models" / "hand_landmarker.task")

# Hand landmark indices (MediaPipe 21-point hand model)
WRIST, THUMB_MCP = 0, 2
INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP = 5, 9, 13, 17

# Palm-plane landmarks used for the center-of-palm estimate
PALM_LANDMARKS = [WRIST, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP]

# Max pixel distance (in a normalized 0-1 image) between a hand's wrist and a
# pose's wrist landmark to consider them the same person.
HAND_TO_POSE_MATCH_THRESHOLD = 0.35

# Max centroid distance between frames to keep the same tracked human ID.
TRACK_MATCH_THRESHOLD = 0.2
# Frames a track can go unmatched before it's dropped.
TRACK_MAX_MISSES = 15


def _landmark_to_xyz(lm):
    return np.array([lm.x, lm.y, lm.z], dtype=np.float32)


def _palm_center_and_normal(hand_landmarks, handedness_label):
    pts = [_landmark_to_xyz(hand_landmarks[i]) for i in PALM_LANDMARKS]
    center = np.mean(pts, axis=0)

    wrist = _landmark_to_xyz(hand_landmarks[WRIST])
    index_mcp = _landmark_to_xyz(hand_landmarks[INDEX_MCP])
    pinky_mcp = _landmark_to_xyz(hand_landmarks[PINKY_MCP])

    v1 = index_mcp - wrist
    v2 = pinky_mcp - wrist
    normal = np.cross(v1, v2)
    norm = np.linalg.norm(normal)
    if norm < 1e-8:
        normal = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    else:
        normal = normal / norm
        # Winding is mirrored between hands; flip so "normal" consistently
        # points out of the palm face regardless of handedness.
        # NOT YET VISUALLY VERIFIED against a real camera - re-check sign
        # once a live feed is available before trusting this for safety use.
        if handedness_label == "Left":
            normal = -normal

    return center.tolist(), normal.tolist()


@dataclass
class _Track:
    track_id: int
    centroid: np.ndarray
    last_seen: float
    misses: int = 0


class HumanHandTracker:
    """Wraps PoseLandmarker + HandLandmarker and assigns persistent human IDs."""

    def __init__(self, pose_model_path=POSE_MODEL_PATH, hand_model_path=HAND_MODEL_PATH,
                 num_poses=4, num_hands=8):
        self._pose = PoseLandmarker.create_from_options(
            PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=pose_model_path),
                running_mode=RunningMode.IMAGE,
                num_poses=num_poses,
            )
        )
        self._hand = HandLandmarker.create_from_options(
            HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=hand_model_path),
                running_mode=RunningMode.IMAGE,
                num_hands=num_hands,
            )
        )
        self._tracks: list[_Track] = []
        self._next_id = 1

    def close(self):
        self._pose.close()
        self._hand.close()

    def _assign_ids(self, centroids):
        now = time.time()
        assigned = [None] * len(centroids)
        used_tracks = set()

        for i, c in enumerate(centroids):
            best_track, best_dist = None, TRACK_MATCH_THRESHOLD
            for t in self._tracks:
                if id(t) in used_tracks:
                    continue
                dist = float(np.linalg.norm(c[:2] - t.centroid[:2]))
                if dist < best_dist:
                    best_track, best_dist = t, dist
            if best_track is not None:
                best_track.centroid = c
                best_track.last_seen = now
                best_track.misses = 0
                assigned[i] = best_track.track_id
                used_tracks.add(id(best_track))
            else:
                new_track = _Track(track_id=self._next_id, centroid=c, last_seen=now)
                self._next_id += 1
                self._tracks.append(new_track)
                assigned[i] = new_track.track_id
                used_tracks.add(id(new_track))

        for t in self._tracks:
            if id(t) not in used_tracks:
                t.misses += 1
        self._tracks = [t for t in self._tracks if t.misses <= TRACK_MAX_MISSES]

        return assigned

    def detect(self, mp_image: "mp.Image") -> dict:
        pose_result = self._pose.detect(mp_image)
        hand_result = self._hand.detect(mp_image)

        poses = pose_result.pose_landmarks or []
        pose_centroids = []
        for landmarks in poses:
            xs = np.array([lm.x for lm in landmarks], dtype=np.float32)
            ys = np.array([lm.y for lm in landmarks], dtype=np.float32)
            pose_centroids.append(np.array([float(xs.mean()), float(ys.mean()), 0.0]))

        human_ids = self._assign_ids(pose_centroids) if pose_centroids else []

        # Wrist position per detected pose, for hand->person association.
        pose_wrists = []
        for landmarks in poses:
            left_wrist = _landmark_to_xyz(landmarks[15])
            right_wrist = _landmark_to_xyz(landmarks[16])
            pose_wrists.append((left_wrist, right_wrist))

        hands_by_pose: dict[int, list] = {i: [] for i in range(len(poses))}
        unassigned_hands = []

        hand_landmarks_list = hand_result.hand_landmarks or []
        handedness_list = hand_result.handedness or []
        for landmarks, handedness in zip(hand_landmarks_list, handedness_list):
            label = handedness[0].category_name if handedness else "Unknown"
            center, normal = _palm_center_and_normal(landmarks, label)
            wrist_xyz = _landmark_to_xyz(landmarks[WRIST])

            best_pose_idx, best_dist = None, HAND_TO_POSE_MATCH_THRESHOLD
            for pose_idx, (lw, rw) in enumerate(pose_wrists):
                for candidate in (lw, rw):
                    dist = float(np.linalg.norm(wrist_xyz[:2] - candidate[:2]))
                    if dist < best_dist:
                        best_pose_idx, best_dist = pose_idx, dist

            hand_entry = {
                "handedness": label,
                "palm_center": center,
                "normal": normal,
                # All 21 MediaPipe hand landmarks (wrist + 4 joints per
                # finger: MCP/PIP/DIP/TIP) - the model already tracks these,
                # palm_center/normal above are just a derived summary of a
                # 5-point subset of this same list.
                "landmarks": [
                    {"x": lm.x, "y": lm.y, "z": lm.z} for lm in landmarks
                ],
            }
            if best_pose_idx is not None:
                hands_by_pose[best_pose_idx].append(hand_entry)
            else:
                unassigned_hands.append(hand_entry)

        humans = []
        for i, landmarks in enumerate(poses):
            skeleton = [
                {"x": lm.x, "y": lm.y, "z": lm.z, "visibility": lm.visibility}
                for lm in landmarks
            ]
            humans.append({
                "id": human_ids[i],
                "skeleton": skeleton,
                "hands": hands_by_pose.get(i, []),
            })

        return {"humans": humans, "unassigned_hands": unassigned_hands}
