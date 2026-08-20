"""Minimal cv2.VideoCapture-compatible wrapper around an Intel RealSense
camera (D435 etc.), so live_demo.py/api.py can swap it in without touching
the rest of the read/warm-up/annotate loop.

Only the color stream is used -- MediaPipe/YOLO both just want an RGB/BGR
frame, same as any UVC webcam. Depth is available on `.last_depth_frame`
for future use (e.g. camera-frame Z for palm centers / object distance)
but nothing in this repo consumes it yet.
"""
import numpy as np
import pyrealsense2 as rs


class RealSenseCapture:
    """Duck-types the subset of cv2.VideoCapture's interface live_demo.py
    and api.py actually call: isOpened(), set() (no-op -- RealSense's
    format/resolution is fixed at pipeline start, not via cv2 flags),
    read(), release()."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        try:
            self._profile = self._pipeline.start(config)
            self._opened = True
        except RuntimeError:
            self._profile = None
            self._opened = False
        self.last_depth_frame = None

    def isOpened(self) -> bool:
        return self._opened

    def set(self, *_args, **_kwargs) -> bool:
        # No-op: resolution/format/fps are fixed at pipeline start above,
        # not settable per-property like cv2.VideoCapture (e.g. the
        # MJPG-fourcc trick api.py/live_demo.py use for usbipd webcams
        # doesn't apply here).
        return True

    def read(self):
        if not self._opened:
            return False, None
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=5000)
        except RuntimeError:
            return False, None
        color_frame = frames.get_color_frame()
        if not color_frame:
            return False, None
        self.last_depth_frame = frames.get_depth_frame()
        bgr = np.asanyarray(color_frame.get_data())
        return True, bgr

    def release(self):
        if self._opened:
            self._pipeline.stop()
            self._opened = False


def list_realsense_devices():
    """Returns [(serial, name), ...] for every connected RealSense device
    -- useful for a clear error message when none is attached (e.g. still
    needs `usbipd attach` on WSL2)."""
    ctx = rs.context()
    return [(d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.name))
            for d in ctx.query_devices()]
