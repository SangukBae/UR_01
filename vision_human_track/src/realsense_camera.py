"""Minimal cv2.VideoCapture-compatible wrapper around an Intel RealSense
camera (D435 etc.), so live_demo.py/api.py can swap it in without touching
the rest of the read/warm-up/annotate loop.

Depth is aligned to the color frame on every read() (`rs.align` -- the
color and depth sensors sit a few cm apart with different fields of view,
so a raw depth frame's (x, y) doesn't line up with the same (x, y) in the
color frame without this) and exposed via `.last_depth_frame` /
`get_distance(x, y)`, so a color-frame pixel from a YOLO box or a
MediaPipe landmark can be looked up directly for real-world distance.
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
        self._align = rs.align(rs.stream.color)
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
        frames = self._align.process(frames)
        color_frame = frames.get_color_frame()
        if not color_frame:
            return False, None
        self.last_depth_frame = frames.get_depth_frame()
        bgr = np.asanyarray(color_frame.get_data())
        return True, bgr

    def get_distance(self, x: int, y: int, radius: int = 3) -> float | None:
        """Real-world distance in meters at color-frame pixel (x, y) --
        median over a (2*radius+1)^2 ROI, not a single-pixel read, since
        individual depth pixels are commonly 0 ("no valid depth": object
        edges, reflective/dark surfaces, out of the sensor's ~0.2-10m
        range). Returns None if there's no depth frame yet, (x, y) is out
        of bounds, or every sample in the ROI is invalid.

        Needs `.last_depth_frame` from a prior read() -- and that frame is
        already aligned to the color frame, so `x, y` here are the same
        pixel coordinates as in the color image (a YOLO box center, a
        MediaPipe landmark scaled to pixels), not raw depth-sensor pixels.
        """
        depth_frame = self.last_depth_frame
        if depth_frame is None:
            return None
        width, height = depth_frame.get_width(), depth_frame.get_height()
        if not (0 <= x < width and 0 <= y < height):
            return None
        samples = []
        for dy in range(-radius, radius + 1):
            py = y + dy
            if not (0 <= py < height):
                continue
            for dx in range(-radius, radius + 1):
                px = x + dx
                if 0 <= px < width:
                    d = depth_frame.get_distance(px, py)
                    if d > 0:
                        samples.append(d)
        if not samples:
            return None
        samples.sort()
        return samples[len(samples) // 2]

    def get_intrinsics(self):
        """The color stream's pyrealsense2 intrinsics (depth is aligned to
        color on every read(), so these apply to depth pixels too) --
        needed to turn a color-frame pixel + depth into a real 3D point."""
        return self._profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def deproject(self, x: int, y: int, depth_m: float) -> tuple[float, float, float]:
        """3D point (X, Y, Z) in metres, in the color camera's optical
        frame (X right, Y down, Z forward -- rs2's standard convention),
        for a color-frame pixel at a known depth. Takes ``depth_m`` rather
        than sampling it itself so callers that already called
        get_distance(x, y) (e.g. to fill in a `distance_m` field) reuse
        that same number instead of a second, possibly-different sample.
        """
        return tuple(rs.rs2_deproject_pixel_to_point(self.get_intrinsics(), [float(x), float(y)], depth_m))

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
