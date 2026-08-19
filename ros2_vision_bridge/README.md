# ros2_vision_bridge

Bridges `vision_human_track`'s HTTP REST service into ROS2 topics, so any
ROS2 node (obstacle avoidance, digital twin, safety fencing) can consume
human/hand detections without knowing it's a plain HTTP service under the
hood. Polls `POST /detect/live` on a timer and republishes the JSON as
ROS2 messages.

**No colcon package, on purpose** — matches this repo's existing
`ros2_ur_driver/ros2_client.py` convention (plain rclpy script, not an
ament package). Nothing to build; just run it. This is also what makes it
realistic to hand to a teammate as "a couple of files."

## Handing this off to a teammate — what actually travels vs. what doesn't

The two files here (`vision_bridge_node.py`, `geometry.py`) are genuinely
just "send them the files, they run it" *if* they already have ROS2
installed and can get `vision_human_track` running with a camera on their
own machine. That second part is the real friction, and it's OS-dependent:

- **Native Linux**: easy, `docker compose up` in `vision_human_track/`
  and the camera just works.
- **WSL2**: needs the same `usbipd-win` camera-passthrough dance this repo's
  CLAUDE.md documents for `vision_human_track` — not a given, needs doing
  once per machine, and won't work at all if their laptop's camera is MIPI
  CSI rather than USB (`usbipd list` won't show it).
- **Mac**: Docker Desktop doesn't support USB/camera passthrough in any
  normal setup — a Mac teammate would need to run `vision_human_track`
  outside Docker (its own venv) rather than the container.

So: send the two files, but tell them "you also need `vision_human_track`
running with your own camera first" — it isn't a single self-contained
drop-in until they have that.

## Running it

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # see ros2_ur_driver/README.md - required in this sandbox
pip install requests                            # only non-ROS dependency
python3 vision_bridge_node.py
```

Needs `vision_human_track`'s API already running and reachable (default
`http://localhost:8000`) — `docker compose up -d` or its venv, with a
camera attached. Override the URL, poll rate, or frame id via ROS
parameters:

```bash
python3 vision_bridge_node.py --ros-args -p vision_api_url:=http://192.168.1.50:8000 -p poll_rate_hz:=5.0
```

## Topics published

| Topic | Type | Contents |
|---|---|---|
| `/vision/hands` | `geometry_msgs/PoseArray` | One `Pose` per detected hand: `position` = palm center, `orientation` = a quaternion whose local +Z axis points along the palm normal (see `geometry.py`'s `quat_from_z_axis` docstring — this fixes 2 of 3 rotational DOF, not the hand's roll). |
| `/vision/humans_markers` | `visualization_msgs/MarkerArray` | One `LINE_LIST` marker per detected person (simplified limb/torso skeleton) — viewable directly in RViz, zero custom-message parsing needed. |
| `/vision/humans_json` | `std_msgs/String` | The vision service's raw JSON response, one message per poll — full 33-point skeleton, all 21 hand landmarks per hand, IDs, unassigned hands. Everything the two typed topics above don't carry, without needing a custom `.msg` package. |

**Coordinate frame**: everything is in the vision service's own output
frame — camera-relative, normalized `[0,1]` image x/y, MediaPipe's
relative z. **Not** robot/world coordinates. Per the team's first
architecture meeting, transforming this into robot coordinates is a
separate ROS-side job, deliberately not done here.

## Verified live (2026-08-19)

Ran against the real webcam (WSL2 + usbipd-win passthrough) and the real
`vision_human_track` API: `/vision/hands` and `/vision/humans_json`
carried real detections at the configured 10 Hz (measured: 80 messages in
~8s), `/vision/humans_markers` published real `LINE_LIST` markers with
plausible point counts. `geometry.py`'s quaternion math has its own
6-test unit suite (`tests/test_geometry.py`, plain `python3`, no ROS
needed).

**Found and fixed along the way**: `vision_human_track`'s `/detect/live`
endpoint was reopening the camera from scratch on every single request —
slow (~0.5s/call) and, worse, missing the MJPG-fourcc fix
`live_demo.py` already had, so it was silently returning zero detections
(a solid-green first frame, same bug as before, just not visible since
this endpoint never showed you the frame). Fixed by keeping the camera
device open across requests in `vision_human_track/src/api.py`; warm
calls dropped to ~40ms.

## safety_stop_demo.py — closing the loop

Subscribes to `/vision/hands`; stops the robot the moment any hand shows
up. Talks to the robot directly via `case1/ur_client.py`'s `URClient.stop()`
— **not** through the MCP tool layer, deliberately: a safety reaction has
no business going through an LLM's tool-call loop, which is exactly the
low-latency argument the team's own supervisor made for safety functions
specifically (see meeting notes).

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 safety_stop_demo.py
```

**This is a presence trigger, not real spatial safety fencing.** `/vision/
hands` positions are camera-frame, normalized image coordinates — there's
no camera↔robot extrinsic calibration anywhere in this repo, so there's no
way yet to compute "is this hand actually near the robot." Any hand
anywhere in the camera's view triggers a stop. Real proximity-based
fencing needs that calibration built first (ROS-side work, per the first
architecture meeting) — this demo exists to prove the reactive pipeline
end-to-end (vision → ROS2 → robot stop), not to be a finished safety system.

**Verified live (2026-08-19)**, but not with a real hand in front of a real
camera — that part needs a person physically doing it, which an
automated/scripted check can't do. Instead verified the actual reaction
logic directly: started a slow real robot move (target 30°, speed 0.15
rad/s, simulator), published a synthetic `/vision/hands` message partway
through, and confirmed the robot stopped well short of the target (ended
at 2.0°) — see `tests/_manual_safety_stop_check.py`. The piece that's
still genuinely unverified is the camera→detection half in a real-time
loop with a real hand; try `vision_human_track/live_demo.py` alongside
this running to see the whole thing end-to-end with a real hand.

## Known gaps

- **Person-ID churn.** In one live run, two different track IDs appeared
  simultaneously in an 8-second window for what was very likely one real
  person — the centroid-based tracker in `vision_human_track/src/
  detector.py` can spawn a new ID before an old one ages out if the
  detected pose centroid jumps by more than `TRACK_MATCH_THRESHOLD` (0.2)
  between polls, or if MediaPipe's `num_poses=4` setting produces a
  low-confidence duplicate pose. Not investigated further — the tracker
  was already flagged as untested with real multi-person scenes; this is
  the same class of issue showing up even single-person. Don't trust
  `/vision/humans_markers`'/`_json`'s `id` field to stay stable over more
  than a few seconds without re-checking this.
- **DDS discovery is flaky in this sandbox.** `ros2 topic echo`/`ros2 node
  list` intermittently saw nothing even with the node running and
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` set and the daemon restarted —
  a stale node process (left over from an earlier restart attempt) was
  the actual cause once, but the log also fills with continuous
  `ddsi_udp_conn_write ... failed` noise pointing at a stale peer address,
  which looks like a broader multicast/networking quirk in this
  environment (same category as the `ros2_ur_driver/README.md` FastRTPS
  note, just showing up differently). If topics seem to vanish, kill and
  restart the node fresh (`pgrep -af vision_bridge_node`, `kill -9`) before
  assuming the code is wrong.
- **No obstacle-avoidance/safety-fencing consumer built yet** — this node
  only publishes; nothing currently subscribes to `/vision/hands` to
  actually stop or slow the robot. That's the ROS/digital-twin side's
  work per the team's architecture.
