"""FastAPI REST wrapper around HumanHandTracker (+ optional YOLO object
detection), per the team's "Docker Compose + HTTP REST POST" convention --
this is the module's one supported plug-in point for the rest of the
pipeline (MCP passthrough, ros2_vision_bridge/vision_bridge_node.py). Any
consumer that wants person/hand/object detections talks to this endpoint,
regardless of what camera or detector is running behind it.

Two ways to get a frame in:
  - POST /detect/image  - caller uploads an image file, stateless.
  - POST /detect/live    - service grabs one frame from its own local
                            camera (useful once this runs next to the
                            robot cell's camera). Returns 503 if no
                            camera is attached.

Camera backend (env var CAMERA_BACKEND, default "cv2"):
  - "cv2"       - a plain UVC webcam via cv2.VideoCapture(camera_index).
  - "realsense" - an attached Intel RealSense (D435 etc.) via
                   pyrealsense2/RealSenseCapture -- adds a real `distance_m`
                   (meters) to every hand and object, and ignores
                   `camera_index` (RealSense enumerates its own device,
                   not a /dev/video* index).

Object detection (env var YOLO_ENABLED, default "false"): when enabled,
every response also carries an `objects` list (YOLO26 detections, a
teammate's model -- see ../../yolo/README.md), sharing the same
serialize_detections() ../../yolo/smoke_test.py already uses so both
paths stay identical in shape. YOLO_DEVICE (default "cpu", set "0" for a
CUDA GPU) and YOLO_MODEL (default "yolo26n.pt") mirror the same env vars
live_demo.py already reads.
"""
import os
import sys
import threading
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse

from detector import HumanHandTracker
from distance_utils import add_distances

# Two candidate layouts: Docker bakes yolo/ as a sibling of src/'s parent
# (see Dockerfile's `COPY yolo/ yolo/` into /app), while running api.py
# straight from a repo checkout has it one level further up
# (UR_01/yolo, sibling of vision_human_track/). Try the Docker layout
# first since that's the primary deployment target.
_CANDIDATE_YOLO_DIRS = [
    Path(__file__).resolve().parent.parent / "yolo",
    Path(__file__).resolve().parent.parent.parent / "yolo",
]
YOLO_DIR = next((p for p in _CANDIDATE_YOLO_DIRS if p.is_dir()), _CANDIDATE_YOLO_DIRS[0])

CAMERA_BACKEND = os.environ.get("CAMERA_BACKEND", "cv2")
YOLO_ENABLED = os.environ.get("YOLO_ENABLED", "false").lower() == "true"
YOLO_DEVICE = os.environ.get("YOLO_DEVICE", "cpu")
YOLO_MODEL = os.environ.get("YOLO_MODEL", "yolo26n.pt")

app = FastAPI(title="UR_01 Vision Stack - Human & Hand Tracking")
_tracker: HumanHandTracker | None = None
_yolo_model = None
_yolo_serialize = None
_camera = None
_camera_index: int | None = None
_camera_lock = threading.Lock()


@app.on_event("startup")
def _load_models():
    global _tracker, _yolo_model, _yolo_serialize
    _tracker = HumanHandTracker()

    if YOLO_ENABLED:
        sys.path.insert(0, str(YOLO_DIR))
        from smoke_test import serialize_detections  # noqa: E402
        from ultralytics import YOLO  # noqa: E402

        _yolo_serialize = serialize_detections
        weight_path = YOLO_DIR / YOLO_MODEL
        # A weight baked into the image at build time (Dockerfile) is used
        # by path; otherwise fall back to the bare filename, which
        # ultralytics auto-downloads on first use (same convention
        # ../yolo/smoke_test.py and live_demo.py already rely on).
        _yolo_model = YOLO(str(weight_path) if weight_path.is_file() else YOLO_MODEL)


@app.on_event("shutdown")
def _unload_models():
    global _camera
    if _tracker is not None:
        _tracker.close()
    with _camera_lock:
        if _camera is not None:
            _camera.release()
            _camera = None


def _open_camera(camera_index: int):
    if CAMERA_BACKEND == "realsense":
        from realsense_camera import RealSenseCapture, list_realsense_devices

        if not list_realsense_devices():
            raise HTTPException(
                status_code=503,
                detail="No RealSense device found by pyrealsense2 (check USB "
                "passthrough into the container -- see docker-compose.yml).",
            )
        cap = RealSenseCapture()
        if not cap.isOpened():
            raise HTTPException(
                status_code=503,
                detail="RealSense device found but its color/depth pipeline "
                "failed to start.",
            )
        # A few frames are genuinely dark/blank while auto-exposure ramps
        # up, same as the plain-webcam warm-up below.
        for _ in range(10):
            cap.read()
        return cap

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise HTTPException(
            status_code=503,
            detail=f"No camera available at index {camera_index}",
        )
    # Forcing MJPG (instead of the default raw YUYV) fixed a solid-green-
    # frame bug seen over usbipd-win/vhci_hcd USB passthrough in WSL2 -
    # verified in vision_human_track/live_demo.py. A handful of frames are
    # also just genuinely dark/blank while auto-exposure ramps up, so
    # discard some before trusting the feed.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    for _ in range(10):
        cap.read()
    return cap


def _get_camera_locked(camera_index: int):
    """Open the camera once and keep reusing it (a fresh open + warm-up on
    every single request would be far too slow for a polled endpoint).
    Re-opens if the index changes (cv2 backend) or the device dropped.
    RealSense ignores `camera_index` -- there's only ever one instance.
    Caller must hold `_camera_lock`."""
    global _camera, _camera_index
    if _camera is not None:
        if CAMERA_BACKEND == "realsense":
            return _camera
        if _camera_index == camera_index and _camera.isOpened():
            return _camera
        _camera.release()

    _camera = _open_camera(camera_index)
    _camera_index = camera_index
    return _camera


@app.get("/health")
def health():
    return {
        "status": "ok",
        "models_loaded": _tracker is not None,
        "camera_backend": CAMERA_BACKEND,
        "yolo_enabled": _yolo_model is not None,
    }


def _run_detection(bgr_image: np.ndarray, cap=None) -> dict:
    if bgr_image is None:
        raise HTTPException(status_code=400, detail="Could not decode image")

    height, width = bgr_image.shape[:2]
    rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    result = _tracker.detect(mp_image)
    result["frame_width"] = width
    result["frame_height"] = height

    objects = None
    if _yolo_model is not None:
        yolo_result = _yolo_model.predict(
            source=bgr_image, conf=0.25, device=YOLO_DEVICE, verbose=False
        )[0]
        objects = _yolo_serialize(yolo_result)
        result["objects"] = objects

    # Only a RealSenseCapture has get_distance; add_distances() no-ops
    # otherwise (e.g. cv2 backend, or a stateless /detect/image upload
    # with cap=None).
    if cap is not None:
        add_distances(result, objects, cap, width, height)

    return result


@app.post("/detect/image")
async def detect_image(file: UploadFile = File(...)):
    raw = await file.read()
    arr = np.frombuffer(raw, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    result = _run_detection(bgr, cap=None)
    return JSONResponse(result)


@app.post("/detect/live")
def detect_live(camera_index: int = 0):
    # FastAPI dispatches sync `def` endpoints to a threadpool, so concurrent
    # requests (e.g. a poller plus a retry) could otherwise interleave one
    # thread's cap.release() with another's cap.read()/cap.set() on the same
    # camera handle. Hold the lock across get+read, not just the
    # open/reopen, to close that window.
    with _camera_lock:
        cap = _get_camera_locked(camera_index)
        ok, bgr = cap.read()
    if not ok:
        raise HTTPException(status_code=503, detail="Failed to read frame from camera")
    result = _run_detection(bgr, cap=cap)
    return JSONResponse(result)
