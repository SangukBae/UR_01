# vision_human_track — Human Detection & Hand Tracking

Team's shared Vision Stack: this service is the "Human Detection & Hand
Tracking" half (SANGUK BAE's assignment from the 2nd team meeting). Object
detection (apple/cup/etc.) is a separate teammate's module — not built here.

Detects people in a frame (skeleton + a per-person ID across frames) and,
for each detected hand, the full 21-point finger-joint skeleton plus a
derived palm center point and orientation (a unit normal vector) — not
full gesture/sign-language recognition, just enough for robot-interaction
and safety-fencing use cases.

See [`../ros2_vision_bridge/`](../ros2_vision_bridge/) for the ROS2 side
that republishes this service's output as topics.

## Why two models, not one

`PoseLandmarker` alone (the model originally considered) only carries 4
hand-related points per arm (wrist, thumb, index, pinky) — enough to know
roughly where a hand is, not enough to reliably compute a palm-plane normal.
`HandLandmarker` gives 21 points per hand and is used specifically for the
palm-center/orientation math; `PoseLandmarker` is used for the skeleton and
per-person ID. Each detected hand is associated to the nearest pose's wrist
landmark (`HAND_TO_POSE_MATCH_THRESHOLD` in `src/detector.py`, currently
0.35 in normalized image coordinates — tuned against the one test photo
below, not a large multi-person test set).

## Architecture

- `src/detector.py` — `HumanHandTracker`: loads both MediaPipe Tasks models,
  runs inference, computes palm center/normal, does simple centroid-based
  ID tracking across calls (see caveats below).
- `src/api.py` — FastAPI wrapper. `POST /detect/image` (upload a frame,
  stateless) and `POST /detect/live` (service grabs one frame from its own
  local camera — 503 if none attached). `GET /health`. The camera is opened
  once and kept open across `/detect/live` calls, not reopened per request
  (see Known gaps — reopening every time was both slow and silently broken).
- `Dockerfile` + `docker-compose.yml` — per the team's agreed convention,
  packaged as a standalone HTTP service for the ROS/MCP bridge to call.
  `download_models.sh` runs as part of the image build (and can be run
  standalone for local dev) so `models/*.task` (~13MB, gitignored) never
  needs to be committed or hand-carried — a plain `git clone` +
  `docker compose up -d --build` is enough for anyone to reproduce this.
- `live_demo.py` — local-only GUI demo (not part of the API), opens a
  camera and shows the detection overlay in a live window (`q` to quit).
  Needs the non-headless `opencv-contrib-python` this venv already has
  (mediapipe pulls it in) and a display (WSLg on Windows, or any real X11/
  Wayland session) — won't work through the Docker image, which stays
  headless since it's a server with no display.
- `detect_photo.py` — local-only single-image CLI (not part of the API):
  `python3 detect_photo.py path/to/image.jpg` runs this service's own
  detection once and writes `<stem>_detected.jpg` (annotated) +
  `<stem>_detected.json` (full result) next to the input. `--yolo` also
  runs `../yolo/`'s YOLO26 object detection on the same image and adds an
  `"objects"` array to the JSON — see "Output coordinates in detail" below
  for why that array's coordinates are NOT in the same frame as everything
  else in the file. This is a convenience script for eyeballing detection
  on a still photo, not a REST endpoint.

## Coordinate frame — scope boundary

Output is in **camera-frame, normalized [0,1] image coordinates** (plus
MediaPipe's relative z). Converting these to robot/world coordinates is
ROS's job, per the team's first architecture meeting — not handled here.

## API contract

```
POST /detect/image   multipart form field "file" = image bytes
POST /detect/live     optional query param "camera_index" (default 0)
GET  /health
```

Response shape:
```json
{
  "humans": [
    {
      "id": 1,
      "skeleton": [{"x":.., "y":.., "z":.., "visibility":..}, ...33 pts],
      "hands": [
        {
          "handedness": "Right", "palm_center": [x,y,z], "normal": [nx,ny,nz],
          "landmarks": [{"x":.., "y":.., "z":..}, ...21 pts, wrist + 4 joints/finger]
        }
      ]
    }
  ],
  "unassigned_hands": [ ...same shape as a hand entry, no matching pose... ]
}
```

## Output coordinates in detail

Two different coordinate systems appear in this service's output — don't
mix them up:

**`humans[].skeleton`, `humans[].hands[]`, `unassigned_hands[]` — all
normalized [0,1] image coordinates + relative depth** (this is the
"Coordinate frame — scope boundary" above, broken down field by field):

- **`x`, `y`** (skeleton points, hand `landmarks`, and `palm_center`'s
  first two values): fraction of image width/height, `(0,0)` = top-left of
  the frame, `(1,1)` = bottom-right. NOT pixels.
- **`z`**: relative depth, roughly the same scale as x/y, zeroed at a
  reference point (hip midpoint for the pose skeleton; the hand's own
  wrist for hand landmarks) — smaller (more negative) means closer to the
  camera than that reference. Not metres, not comparable across a pose
  skeleton and a hand's landmarks (different reference points).
- **`visibility`** (skeleton points only): 0-1 confidence the model has
  that joint, not occluded. Low values (e.g. legs out of frame) mean
  "estimated, possibly unreliable," not "wrong coordinates" — the x/y/z
  are still MediaPipe's best guess, just flagged as uncertain.
- **`palm_center`**: mean of the wrist + 4 finger-MCP landmarks (5 of the
  21 hand `landmarks`) — a derived summary point, same coordinate system
  as everything else here.
- **`normal`**: a unit vector (length 1, not a position) — which way the
  palm faces. Computed as `cross(wrist->index_mcp, wrist->pinky_mcp)`,
  sign-flipped for `"Left"` hands so it points out of the palm regardless
  of handedness (see Known gaps — this flip isn't yet confirmed against a
  real hand of known orientation).
- **`id`** (per human): a tracking ID across frames/calls, not a
  coordinate — see Known gaps for its stability caveats.

**`detect_photo.py --yolo`'s `"objects"` array — raw pixel coordinates,
a different frame entirely:**

- **`xyxy`**: `[x1, y1, x2, y2]` bounding box corners in the ORIGINAL
  IMAGE'S PIXELS (e.g. `2108.0` for a ~2000px-wide photo) — not
  normalized to [0,1] like everything above. Values over 1 are the
  giveaway if you're not sure which array you're looking at.
- This array only appears via `detect_photo.py --yolo` (a local
  convenience script combining this service's own output with a
  teammate's separate YOLO module for one still image) — it is NOT part
  of `/detect/image` or `/detect/live`'s response; the actual REST API
  never returns object detections (see "Object detection is intentionally
  not here" in Known gaps).

## WSL2: attach a USB webcam first

Skip this if you're not on WSL2, or if `ls /dev/video*` already shows a
device. Otherwise, the camera needs `usbipd-win` to pass it through from
Windows before OpenCV can see it (`can't open camera by index` / `Camera
index out of range` means this step hasn't been done yet).

```powershell
# Windows PowerShell, as Administrator:
usbipd list                            # find the webcam's BUSID
usbipd bind --busid <BUSID>            # one-time per device
usbipd attach --wsl --busid <BUSID>    # needed again after every reboot/sleep
```

```bash
# Back in WSL2:
ls /dev/video*   # should now show at least /dev/video0
```

If `ls /dev/video*` still shows nothing even though `usbipd list` says the
device is `Attached`, the `uvcvideo` kernel module likely isn't loaded —
check with `lsmod | grep uvc` and load it if missing:

```bash
sudo modprobe uvcvideo
ls /dev/video*   # should now show at least /dev/video0
```

This resets on every WSL2 restart along with the `usbipd attach`, so both
steps are needed again after a reboot/sleep, not just the Windows-side one.

If the camera never shows up in `usbipd list` at all, it's likely a
built-in MIPI CSI camera, not USB — `usbipd` can't pass those through;
run this outside WSL2/Docker on native Windows Python instead.

## Running it

```bash
# Local (no Docker), from this directory:
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
./download_models.sh   # fetches models/*.task (~13MB, not committed to git)
cd src && uvicorn api:app --host 0.0.0.0 --port 8000

# Docker (matches the team's packaging convention):
docker compose up -d --build
curl http://localhost:8000/health

# Live GUI demo (local venv only, not through Docker - needs a display):
python3 live_demo.py
python3 live_demo.py --realsense --yolo   # RealSense camera + object detection too
```

For continuous headless ROS2 publishing from a RealSense (no GUI, no
Docker) instead of this local demo, see `../ros2_vision_bridge/
realsense_vision_node.py`.

## Tests

```bash
source venv/bin/activate
python3 tests/test_math.py         # palm-center/normal/tracker math, no models needed
python3 tests/test_smoke.py        # models load + run inference (blank image, 0 detections expected)
python3 tests/test_real_image.py   # runs on real photos, saves annotated *_out.jpg for visual check
```

`test_real_image.py` downloads-free — sample photos are already in
`tests/sample_images/` (from `storage.googleapis.com/mediapipe-assets/`,
Google's own MediaPipe example images). Verified live (2026-08-19): 1 human
detected in `pose.jpg` (full-body skeleton correctly on shoulders/elbows/
wrists/hips/knees/ankles); 2 hands correctly detected and matched to 1
person in `woman_hands.jpg`, palm centers landing on the visible palms and
normal-vector arrows pointing in a geometrically plausible direction.

## Known gaps / not yet verified

- **Live camera: working, verified (2026-08-19), WSL2 + `usbipd-win`.**
  Attached a real USB webcam (`usbipd bind`/`attach --wsl` from an admin
  Windows PowerShell) and confirmed real frames flow through to detection —
  a real face's landmarks landed correctly in a captured frame. One real
  bug found along the way: the very first frames (and, separately, the
  default raw-YUYV format over this specific USB-passthrough transport)
  came back as a solid green image — fixed by forcing `CAP_PROP_FOURCC` to
  MJPG and discarding ~10 warm-up frames before trusting the feed (see
  `live_demo.py` and `src/api.py`'s `_get_camera`). Also found: even after
  `usbipd attach` shows the device as `Attached`, OpenCV can still fail with
  "can't open camera by index" / "Camera index out of range" if the
  `uvcvideo` kernel module isn't loaded (`lsmod | grep uvc` empty,
  `/dev/video*` missing) — `sudo modprobe uvcvideo` fixes it immediately;
  see the WSL2 section above. If a teammate's laptop
  camera doesn't show up in `usbipd list` at all, it's likely MIPI CSI, not
  USB — `usbipd` categorically can't pass those through; live-camera work
  would need to happen outside WSL2/Docker on native Windows Python instead.
- **RealSense camera: working, verified (2026-08-20), WSL2 + `usbipd-win`.**
  Same passthrough dance as the plain webcam (`usbipd bind`/`attach --wsl`,
  admin PowerShell) but a different busid/VID:PID — `usbipd list` shows it
  as "Intel(R) RealSense(TM) Depth Camera 435 ...", not a generic webcam
  name. `src/realsense_camera.py`'s `RealSenseCapture` wraps
  `pyrealsense2`'s color stream behind the same `read()`/`release()`
  interface `live_demo.py`/`api.py` already use for `cv2.VideoCapture`, so
  none of the MJPG-fourcc/warm-up-frame logic needed for the plain-webcam
  case applies here — RealSense streams through its own librealsense
  pipeline, not cv2's V4L2 backend. Verified end-to-end: real color frames
  in, correct hand detection out (`live_demo.py --realsense --yolo`, and
  headless via `../ros2_vision_bridge/realsense_vision_node.py`). Depth
  stream is also opened (`RealSenseCapture.last_depth_frame`) but nothing
  in this repo consumes it yet. Known cosmetic issue: the camera is
  mounted vertically on the robot arm, so frames come out ~90° rotated
  from upright — not corrected in code, see `ros2_vision_bridge/
  README.md`'s known gaps.
- **Hand-orientation sign convention (Left vs Right flip in
  `_palm_center_and_normal`) is still a geometric assumption, not
  confirmed against a real hand with known orientation.** The live demo
  ran successfully end-to-end, but nobody has yet held up a hand in a
  known pose and checked the arrow points the expected way — re-check
  before using this for anything safety-related.
- **`HAND_TO_POSE_MATCH_THRESHOLD = 0.35`** was widened from an initial
  0.15 after the tighter value failed to associate hands with a test
  photo's pose (pose landmarker's own wrist estimate was imprecise under
  partial occlusion). Untested with multiple people close together, where
  a loose threshold risks mis-assigning a hand to the wrong person.
- **Person-ID stability is shakier than intended.** Confirmed live via
  `ros2_vision_bridge` (see its README): two different track IDs appeared
  within the same 8-second window for what was very likely one real
  person. The centroid-based tracker (`_assign_ids` in `src/detector.py`)
  can spawn a new ID before an old one ages out — either the person's
  detected centroid moved more than `TRACK_MATCH_THRESHOLD` (0.2) between
  polls, or `num_poses=4` produced a low-confidence duplicate pose. Not
  fixed — don't trust the `id` field to stay constant over more than a
  few seconds without re-checking this, especially with multiple real
  people (still never tested).
- **Object detection is intentionally not here** — separate teammate's
  module per the 2nd team meeting.
