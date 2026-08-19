"""FastAPI REST wrapper around HumanHandTracker.

Two ways to get a frame in, per the team's "Docker Compose + HTTP REST POST"
convention:
  - POST /detect/image  - caller uploads an image file, stateless.
  - POST /detect/live    - service grabs one frame from its own local camera
                            (useful once this runs next to the robot cell's
                            camera). Returns 503 if no camera is attached.
"""
import threading

import cv2
import mediapipe as mp
import numpy as np
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse

from detector import HumanHandTracker

app = FastAPI(title="UR_01 Vision Stack - Human & Hand Tracking")
_tracker: HumanHandTracker | None = None
_camera: cv2.VideoCapture | None = None
_camera_index: int | None = None
_camera_lock = threading.Lock()


@app.on_event("startup")
def _load_models():
    global _tracker
    _tracker = HumanHandTracker()


@app.on_event("shutdown")
def _unload_models():
    global _camera
    if _tracker is not None:
        _tracker.close()
    with _camera_lock:
        if _camera is not None:
            _camera.release()
            _camera = None


def _get_camera_locked(camera_index: int) -> cv2.VideoCapture:
    """Open the camera once and keep reusing it (a fresh open + fourcc set +
    warm-up on every single request would be far too slow for a polled
    endpoint). Re-opens if the index changes or the device dropped. Caller
    must hold `_camera_lock`."""
    global _camera, _camera_index
    if _camera is not None and _camera_index == camera_index and _camera.isOpened():
        return _camera

    if _camera is not None:
        _camera.release()

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

    _camera, _camera_index = cap, camera_index
    return _camera


@app.get("/health")
def health():
    return {"status": "ok", "models_loaded": _tracker is not None}


def _decode_and_detect(bgr_image: np.ndarray) -> dict:
    if bgr_image is None:
        raise HTTPException(status_code=400, detail="Could not decode image")
    rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    return _tracker.detect(mp_image)


@app.post("/detect/image")
async def detect_image(file: UploadFile = File(...)):
    raw = await file.read()
    arr = np.frombuffer(raw, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    result = _decode_and_detect(bgr)
    return JSONResponse(result)


@app.post("/detect/live")
def detect_live(camera_index: int = 0):
    # FastAPI dispatches sync `def` endpoints to a threadpool, so concurrent
    # requests (e.g. a poller plus a retry) could otherwise interleave one
    # thread's cap.release() with another's cap.read()/cap.set() on the same
    # cv2.VideoCapture handle. Hold the lock across get+read, not just the
    # open/reopen, to close that window.
    with _camera_lock:
        cap = _get_camera_locked(camera_index)
        ok, bgr = cap.read()
    if not ok:
        raise HTTPException(status_code=503, detail="Failed to read frame from camera")
    result = _decode_and_detect(bgr)
    return JSONResponse(result)
