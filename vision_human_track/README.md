# vision_human_track — Human Detection & Hand Tracking (+ Object Detection)

Team's shared Vision Stack, packaged as **the** modular plug-in container
for the rest of the pipeline — per the team's agreed convention (Docker
Compose + HTTP REST POST endpoints), this is the one place another module
(MCP passthrough, `ros2_vision_bridge`, anything else) should talk to for
person/hand/object detections. It does not assume a specific camera or
deployment host: `CAMERA_BACKEND` picks a plain UVC webcam or an attached
Intel RealSense (real depth), and `YOLO_ENABLED` folds in object detection
(a teammate's YOLO26 model, `../yolo/`) alongside this service's own
human/hand detection (SANGUK BAE's assignment from the 2nd team meeting) —
**one container, one endpoint, both detectors** — see "Modular deployment"
below for the exact knobs. Runs real-time on this machine's GPU (see
"Running it").

Detects people in a frame (skeleton + a per-person ID across frames) and,
for each detected hand, the full 21-point finger-joint skeleton plus a
derived palm center point and orientation (a unit normal vector) — not
full gesture/sign-language recognition, just enough for robot-interaction
and safety-fencing use cases. With `YOLO_ENABLED=true`, every response also
carries an `objects` array (bounding boxes + class) from the teammate's
YOLO26 model.

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
  `CAMERA_BACKEND` env var picks `cv2` (default, any UVC webcam via
  `camera_index`) or `realsense` (an attached Intel RealSense via
  `src/realsense_camera.py`'s `RealSenseCapture`, real `distance_m` on
  every hand/object, ignores `camera_index`). `YOLO_ENABLED=true` loads
  `../yolo/`'s YOLO26 model at startup and adds an `objects` array to every
  response (`YOLO_DEVICE`/`YOLO_MODEL` env vars pick the compute device/
  model size, same convention `live_demo.py --yolo` already uses).
- `src/distance_utils.py` — `add_distances()`: fills in a real `distance_m`
  (metres) on every hand/object from a RealSense's depth sensor. Shared by
  `api.py`, `live_demo.py`, `live_demo_rtdetr.py`, and
  `../ros2_vision_bridge/realsense_vision_node.py` — one implementation,
  not four copies.
- `Dockerfile` + `docker-compose.yml` — per the team's agreed convention,
  packaged as a standalone HTTP service for the ROS/MCP bridge to call.
  Build context is the repo root (`..`), not this directory, so the
  Dockerfile can also pull in `../yolo/smoke_test.py` for the `objects`
  path above. `download_models.sh` runs as part of the image build (and can
  be run standalone for local dev) so `models/*.task` (~13MB, gitignored)
  never needs to be committed or hand-carried — a plain `git clone` +
  `docker compose up -d --build` is enough for anyone to reproduce this.
  GPU passthrough (`deploy.resources.reservations.devices`, nvidia) and
  RealSense USB passthrough (`privileged` + `/dev/bus/usb` mount, commented
  out by default) are both in `docker-compose.yml` — see "Modular
  deployment" below.
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

## Modular deployment

The one supported integration point is this REST API — not a shared
library, not a ROS node someone else has to also run. `docker-compose.yml`
exposes every knob as an env var so a deployer picks the camera/detector
mix without touching code:

| env var          | default    | meaning                                                          |
|-------------------|-----------|-------------------------------------------------------------------|
| `CAMERA_BACKEND`  | `cv2`      | `cv2` (any UVC webcam) or `realsense` (Intel RealSense, real depth) |
| `YOLO_ENABLED`    | `true`     | fold in the teammate's YOLO26 object detection                    |
| `YOLO_DEVICE`     | `0`        | GPU index for YOLO, or `cpu`                                      |
| `YOLO_MODEL`      | `yolo26n.pt` | which YOLO26 size (`n/s/m/l/x`) — baked into the image at build time via the matching `--build-arg` |

```bash
# Zero-hardware default: human/hand + object detection, cv2 camera backend,
# GPU YOLO -- works out of the box for anyone without a RealSense attached.
docker compose up -d --build

# The real deployment target -- RealSense mounted on the robot arm.
# Uncomment `privileged`/`/dev/bus/usb` in docker-compose.yml first (see
# the comments there for why -- a RealSense isn't a single /dev/video*
# node, so a plain --device mapping doesn't work), then:
CAMERA_BACKEND=realsense docker compose up -d --build

# CPU-only YOLO (no nvidia-container-toolkit on the host):
YOLO_DEVICE=cpu docker compose up -d --build   # also remove the deploy.resources
                                                 # GPU block in docker-compose.yml
```

Verified live (2026-08-20) against this machine's own Docker + nvidia
runtime: built image runs `torch.cuda.is_available() == True` *inside* the
container, `/detect/image` correctly detects real objects (banana/apples on
`../yolo/test.png`) via the GPU, and `CAMERA_BACKEND=realsense` with
`--privileged -v /dev/bus/usb:/dev/bus/usb` starts cleanly and returns a
clean `503` (not a crash) when no RealSense happens to be attached — same
graceful-degradation behavior as the plain-webcam path. Not yet verified:
an actual RealSense detected *from inside* the container (none was attached
to this sandbox when this was built — re-verify the moment one is).

## API contract

```
POST /detect/image   multipart form field "file" = image bytes
POST /detect/live     optional query param "camera_index" (default 0, ignored for CAMERA_BACKEND=realsense)
GET  /health
```

Response shape (`objects` only present with `YOLO_ENABLED=true`;
`distance_m` only present with `CAMERA_BACKEND=realsense`):
```json
{
  "humans": [
    {
      "id": 1,
      "skeleton": [{"x":.., "y":.., "z":.., "visibility":..}, ...33 pts],
      "hands": [
        {
          "handedness": "Right", "palm_center": [x,y,z], "normal": [nx,ny,nz],
          "landmarks": [{"x":.., "y":.., "z":..}, ...21 pts, wrist + 4 joints/finger],
          "distance_m": 1.42
        }
      ]
    }
  ],
  "unassigned_hands": [ ...same shape as a hand entry, no matching pose... ],
  "frame_width": 640,
  "frame_height": 480,
  "objects": [
    {
      "class_id": 47, "class_name": "apple", "instance_name": "apple1",
      "confidence": 0.669, "xyxy": [44.6, 25.2, 314.4, 288.1], "distance_m": 0.87
    }
  ]
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
- **`distance_m`** (hands only): real depth in metres at the palm center's
  pixel, from the RealSense's depth sensor — the one field here that ISN'T
  normalized/relative, a genuine metric measurement. Present on
  `CAMERA_BACKEND=realsense` (the REST API itself, both endpoints), plus
  `live_demo.py --realsense` and `../ros2_vision_bridge/
  realsense_vision_node.py`. `None`/`null` if no valid depth was available
  there (out of the sensor's ~0.2-10m range, a reflective/dark surface,
  etc.). Not present at all on `CAMERA_BACKEND=cv2`/plain-webcam runs —
  there's no depth sensor to read.

**`objects[]` (`YOLO_ENABLED=true`) — raw pixel coordinates, a different
frame entirely:**

- **`xyxy`**: `[x1, y1, x2, y2]` bounding box corners in the ORIGINAL
  IMAGE'S PIXELS (e.g. `2108.0` for a ~2000px-wide photo) — not
  normalized to [0,1] like everything above. Values over 1 are the
  giveaway if you're not sure which array you're looking at. Use the
  response's own `frame_width`/`frame_height` to interpret them, not an
  assumed resolution.
- Present directly on `/detect/image` and `/detect/live` when the service
  was started with `YOLO_ENABLED=true` (see "Modular deployment" above) —
  this is the team's single agreed integration point for object
  detection, same endpoint as human/hand detection, not a separate
  service or a local-only script. `detect_photo.py --yolo` (a local
  single-image CLI convenience, not part of the API) and `live_demo.py
  --yolo`/`--realsense --yolo` produce the same shape for eyeballing on a
  still photo or a live GUI window. Each object also gets `distance_m`
  (bounding-box-center pixel) when the camera backend is `realsense`,
  same meaning as the hands' field above.

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

# Docker (matches the team's packaging convention) -- see "Modular
# deployment" above for CAMERA_BACKEND/YOLO_ENABLED/YOLO_DEVICE/YOLO_MODEL:
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
  headless via `../ros2_vision_bridge/realsense_vision_node.py`). Known
  cosmetic issue: the camera is mounted vertically on the robot arm, so
  frames come out ~90° rotated from upright — not corrected in code, see
  `ros2_vision_bridge/README.md`'s known gaps.
- **Real depth (`distance_m`), added 2026-08-20.** `RealSenseCapture.
  get_distance(x, y)` aligns the depth stream to the color stream
  (`rs.align` — the two sensors are physically offset, so a raw depth
  frame's pixel (x,y) isn't the same point as the color frame's (x,y)
  without this) and reads a median over a small pixel neighborhood rather
  than one raw sample, since individual depth pixels are commonly 0 ("no
  valid depth") at edges/reflective surfaces. `live_demo.py --realsense`
  and `realsense_vision_node.py` both call this for every detected hand's
  palm center and (with `--yolo`/YOLO enabled) every object's box center,
  adding a real `distance_m` (metres) field — see "Output coordinates in
  detail" above. **Logic verified with synthetic depth data (median/outlier
  rejection, out-of-bounds, no-data-yet all covered) — not yet verified
  against the real sensor**, since the RealSense was physically
  disconnected from this sandbox (`usbipd list` stopped showing it,
  unrelated to this change) when this was built; re-verify with a real
  object at a known distance once it's reattached.
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
- **Object detection now IS here (2026-08-20)** — folded into this same
  REST endpoint via `YOLO_ENABLED=true` (still a teammate's YOLO26 model,
  `../yolo/`, reused as-is via `../yolo/smoke_test.py`'s
  `serialize_detections`, not reimplemented). Earlier versions of this
  README said the API deliberately never returns object detections — that
  was true until the team's Docker/integration review found the
  REST-endpoint convention and object detection weren't actually meeting
  at one integration point yet; fixed by combining both detectors behind
  the one endpoint the team already agreed on. See "Modular deployment"
  above.
- **RealSense-in-Docker: passthrough verified, no live device tested.**
  `--privileged -v /dev/bus/usb:/dev/bus/usb` gets `pyrealsense2` inside
  the container talking to the host's USB bus at all (confirmed:
  `list_realsense_devices()` runs without error and correctly reports zero
  devices, matching this sandbox's actual state) — but no RealSense was
  physically attached to this sandbox when this was built, so an actual
  in-container detection has not been run. Re-verify with a real device
  attached before trusting this for the robot cell.
