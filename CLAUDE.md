# UR_01 — Project Notes for Claude Code

UR Robotics Summer School 2026, Case 1 (MCP Server for Robot Tools) plus a
ROS2 driver integration and a custom LLM chat wrapper (text + voice), all
against the PolyScope X (URSim) simulator. This file is durable project
context for future sessions — architecture, verified findings, known gaps —
not a changelog; see `git log` for that.

## What the team is actually building

Not just this repo's own scope — this is the user's (SANGUK BAE's) piece
of a larger team project. The team's end goal (from two recorded planning
meetings, `~/meeting.txt`, and the user's own `~/block_diagram.excalidraw`
architecture sketch): let someone who can't program operate a UR robot
arm by talking to it in natural language, for basic logistics tasks
("grab that, put it there, give it to me"). **Low latency is the team's
top design priority** — voice command to robot reacting, target <1500ms —
because a slow response breaks trust and forces the user to repeat
themselves (this was argued explicitly to the team by their supervisor).
Safety (stopping fast if a person is near) is the other latency-driven
concern. Architecture: user speaks → LLM wrapper (STT/intent/TTS) → MCP
server (translates intent into robot commands, tracks queue/state) → ROS
(executes safely, would check paths/obstacles) → robot; a vision stack
watches people/objects in the scene for both safety and non-destructive
"is this actually what I think it is" checks before acting.

This repo covers: the MCP server + both robot backends (this user's Case 1
assignment, done), the vision stack's human/hand half (this user's team
assignment, done), a ROS2 bridge + safety-stop demo tying vision to the
robot (built past-assignment, to close the loop end to end), an LLM chat
client with per-turn latency instrumentation and opt-in wake-word
listening (both added 2026-08-20), and MoveIt-backed path checking +
obstacle avoidance (also added 2026-08-20, see below). It does NOT cover:
a real Digital Twin (3D visualization/mirrored simulation state — path
checking and obstacle avoidance now exist, but there's no 3D scene view),
object detection (a different teammate's module), or real spatial safety
fencing (needs camera↔robot calibration, not built) — see "Known gaps" and
the block-diagram cross-check below for the full picture of what's covered
vs. not.

## Status

All four official tiers (Bronze → Diamond, per the course's
`ESSRE2026_UR_Industry_Cases_tiers.docx`) are implemented and verified
against the live simulator, on both robot backends. Plus, beyond the
official spec: a `stop_robot` tool, a non-blocking command queue
(`move_robot_queued` + `get_queue`), a waypoint database + free-drive tool
(`save_waypoint`/`list_waypoints`/`delete_waypoint`/
`move_robot_to_waypoint`/`free_drive`, socket backend only) — both per the
team's own architecture-meeting design — a standalone LLM chat client with
voice I/O, per-turn latency reporting (`latency.py`), and opt-in wake-word
listening (`wakeword.py`, `--wake-word`), a ROS2 driver integration, a
standalone Vision Stack service (human detection + hand tracking), a ROS2
bridge republishing it as topics, a safety-stop demo closing the loop
(vision → ROS2 → robot stop), and MoveIt-backed path checking + obstacle
avoidance (`check_path`/`add_obstacle`/`remove_obstacle`/`list_obstacles`,
`motion_planner.py`) — all verified live, not just written (wake-word is
the one exception: verified live against real mic audio with no crash, but
not against an actual spoken wake word — see its Known gaps entry).

Repo layout:
- `case1/` — the MCP server (`server.py`), socket backend (`ur_client.py`),
  FK safety check (`kinematics.py`), smoke test (`test_server.py`), opt-in
  sim-then-real shadow wrapper (`shadow_client.py`, `UR_REAL_HOST` env var —
  see its README section), MoveIt-backed path checking + obstacle avoidance
  (`motion_planner.py` — talks to `move_group`'s planning services directly
  via rclpy, since `moveit_py` isn't packaged for ROS2 Humble; needs both
  `ur_robot_driver` and `ur_moveit_config`'s `move_group` launched
  separately, see Running Things below).
- `ros2_ur_driver/` — ROS2 backend (`ros2_client.py`), calibration config,
  driver setup notes.
- `llm_client/` — standalone chat wrapper: `chat.py` (text/voice loop),
  `voice.py` (mic capture + STT), `tts.py` (spoken replies).
- `vision_human_track/` — standalone Vision Stack service (Docker Compose +
  HTTP REST, per the team's packaging convention): human detection +
  skeleton + per-person ID (MediaPipe PoseLandmarker) and full 21-point
  hand skeleton + derived palm-center/orientation (MediaPipe HandLandmarker).
  This user's team assignment; object detection is a separate teammate's
  module (`../yolo/`), not built here, but `live_demo.py --yolo` can run it
  alongside for a combined demo. `live_demo.py` is a local-only live-camera
  GUI preview (not part of the API) — `--realsense` points it at an
  attached Intel RealSense (`src/realsense_camera.py`, via pyrealsense2)
  instead of a plain `cv2.VideoCapture` index. With `--realsense`, every
  hand and (with `--yolo`) every object also gets a real `distance_m`
  (metres, from the depth sensor, aligned to the color frame via
  `rs.align` — `RealSenseCapture.get_distance` reads a median over a small
  pixel ROI rather than one raw sample, since individual depth pixels are
  commonly 0/invalid at edges and reflective surfaces) — see "Verified
  findings" below for what's actually been confirmed against real
  hardware vs. only logic-tested. Verified live against both a
  real webcam and a real RealSense D435 (both WSL2 + usbipd-win
  passthrough, different busid/VID:PID per device) — see its README for
  the API contract and known gaps (handedness sign convention still not
  empirically verified, person-ID stability shakier than intended,
  RealSense frames come out ~90° rotated since it's mounted vertically on
  the robot arm).
- `ros2_vision_bridge/` — two plain rclpy scripts (no colcon package,
  matches `ros2_ur_driver/`'s convention), same output topics either way.
  `vision_bridge_node.py` polls `vision_human_track`'s `/detect/live` and
  republishes it as ROS2 topics (`/vision/hands`, `/vision/humans_markers`,
  `/vision/humans_json`) — verified live at 10Hz against the real webcam +
  vision service. `realsense_vision_node.py` instead owns an attached
  RealSense directly (no REST service in between) and runs MediaPipe +
  YOLO in-process, adding a fourth topic (`/vision/objects_json`, also
  carrying `distance_m`) for object detections — verified live against the
  real RealSense D435 at ~5.8Hz (person+hand+object inference on CPU is
  heavier than person+hand alone). `vision_human_track/live_demo.py
  --realsense --yolo --ros2` publishes this exact same four-topic set
  in-process, so one command gets both the GUI window and the full topic
  set — no separate node needed if you also want to see the picture (the
  two scripts still can't run at once, though — one camera, one process).
  Both bridge scripts share their message-building code
  (`vision_bridge_node.py`'s `build_hands_msg`/`build_markers_msg`, and
  `live_demo.py`'s `add_distances`), so any existing consumer works
  unchanged regardless of which one is running. `cyclonedds_localhost.xml`
  is an opt-in CycloneDDS config that fixes this sandbox's DDS-discovery
  hang for vision-only work — see "Verified findings" below, do NOT set it
  globally. Also has `safety_stop_demo.py`
  — subscribes to `/vision/hands` and calls `stop_robot` on any hand
  presence, closing the vision→ROS2→robot-stop loop end to end (talks to
  the robot directly, not through the MCP layer — a safety reaction
  shouldn't go through an LLM tool-call loop). A presence trigger, not
  real spatial fencing — no camera↔robot calibration exists yet. See its
  README for the hand-off caveats (what a teammate needs on their own
  machine, which varies a lot by OS) and known gaps (DDS discovery is
  flaky in this sandbox — same class of issue as `ros2_ur_driver/
  README.md`'s FastRTPS note).

Each folder's own README has setup/run commands and more detail than this
file repeats. Read those first for "how do I run X"; read this file for
"why does X work this way" and "what's already been tried and found true
or false."

## Working style: don't ask, just run

The user has said explicitly, more than once, not to pause and ask before
running commands needed for the work in this repo — diagnostic/environment
checks, tests, builds, and (as of 2026-08-19) git commit/push too. Just run
them. Still stop and confirm before anything genuinely destructive/hard to
reverse (force-push, `git reset --hard`, deleting volumes/branches, etc.) —
this preference is about not over-asking for routine and forward-progress
commands, not a blanket override of normal safety judgment.

## Architecture

`server.py` exposes 32 MCP tools over stdio (8 from the original four
tiers, the rest additive -- queue, waypoints, programs, relative moves,
vision passthrough, tracking -- see the tool list below). Two interchangeable backends
behind the same `RobotState` shape (`q_rad`, `qd_rad`, `tcp_pose`,
`robot_mode`, `safety_status`, `digital_output_bits` + `.gripper_closed`
property) and method names (`connect`, `get_state`, `move_joint`,
`move_linear`, `move_waypoints`, `set_gripper`, `stop`), picked by
`UR_BACKEND` env var (`socket` default, `ros2`):

- **`ur_client.py`** — plain TCP sockets, stdlib only. Primary interface
  (port 30001) for motion: uploads a small URScript program per call
  (`movej`, `movel`, `stopj`, `set_digital_out`). RTDE (port 30004) for
  state, including `actual_digital_output_bits` for the gripper readback.
- **`ros2_client.py`** — through `ros_humble_ur_robot_driver`.
  `FollowJointTrajectory` action for motion, `SetIO` service for gripper,
  `/joint_states` + `/tcp_pose_broadcaster/pose` +
  `/io_and_status_controller/{robot_mode,safety_mode,io_states}` topics
  for state.

Tools, by tier: Bronze `move_robot_to_position`; Silver `get_robot_state` +
`move_robot_linear`; Gold `move_through_waypoints` (streams live via MCP
`notifications/progress`, not just a post-hoc trace); Diamond
`move_robot_to_position_safe` + `set_gripper`. Extra (all additive, block-
diagram-driven -- see CLAUDE.md's block-diagram cross-check below):
`stop_robot`; `move_robot_queued` + `get_queue` (non-blocking move with
reject/queue/override semantics, `queue_manager.py`'s background worker
thread, makes `stop_robot` genuinely interruptible mid-turn); waypoint DB
(`save_waypoint`/`list_waypoints`/`delete_waypoint`/
`move_robot_to_waypoint`) + `free_drive` (socket-only); relative-move
variants (`move_robot_to_position_relative`/`move_robot_linear_relative`/
`move_robot_linear_sequence`); programs (`program_new`/`list_programs`/
`program_delete`/`program_start`/`program_stop` -- named waypoint
sequences, run via the same queue); vision passthrough (`get_vision`/
`get_environment`/`get_environment_shadow` -- calls `vision_human_track`'s
REST API over HTTP, `requests` dependency, `VISION_API_URL` env var);
`track` (deliberately crude 1-DOF demo -- no camera<->robot calibration
exists, so it only turns the base toward a hand's left/right position in
frame); path checking + obstacle avoidance (`check_path` -- collision-free
path query, no motion; `add_obstacle`/`remove_obstacle`/`list_obstacles` --
box obstacles in MoveIt's planning scene; `motion_planner.py`, needs
`move_group` launched separately, lazy-connects on first call so it
doesn't block runs that never use it); `example` (template).

`llm_client/chat.py` connects to `server.py` in-process (like
`test_server.py` — no subprocess/stdio), converts its tools to OpenAI
function-calling schema, talks to Cerebras (`gpt-oss-120b`) by default.
`--voice` swaps typed `input()` for `voice.listen()` (mic → energy-VAD →
local Whisper) and adds spoken replies via `tts.py` (`espeak-ng` →
`paplay`).

## Verified findings (don't re-derive these — re-verify only if something
## seems to contradict them)

- **Remote Control / Operational Mode don't need touching, on this
  simulator.** Tested directly: with Operational Mode on `Manual` (which
  forces Remote Control to `Local`), both the socket backend and the ROS2
  driver moved the robot exactly on target. Earlier notes claiming
  Remote+Automatic was required were wrong. Scoped as this-simulator-only,
  not a general PolyScope X claim.
- **Singularities break `move_robot_linear`, and they're not obvious from
  joint values alone.** Reproduced at `HOME_DEG` (wrist2=0°, a real wrist
  singularity) *and* at a pose picked specifically to avoid that
  (`[20,-90,30,-90,60,0]`, a different shoulder-adjacent singularity) —
  both tripped a protective stop. `[30,-100,40,-80,70,10]` is verified
  clean over repeated round trips and is what `test_server.py` uses.
  Moral: verify a new pose the same way before trusting it for a linear
  move, don't just eyeball the joint values.
- **The socket backend's `move_waypoints` had a real bug** (fixed): no
  completion signal from the controller, so it polled and guessed
  "arrived" by proximity to the last waypoint — which could pass on the
  very first poll for a path looping back near its start, before the
  robot had moved at all. Fixed with `_estimate_path_duration`, a
  trapezoidal-profile minimum-duration floor.
- **`parecord` with raw output to a pipe never flushes in this sandbox**
  (0 bytes no matter how long you wait), even though identical output to
  a real file works. `parec` (lower-level, always raw) streams to a pipe
  correctly — that's why `voice.py` uses it, not `parecord`.
- **A copied API key can carry an invisible non-ASCII character** (smart
  quote, zero-width space) that breaks the `Authorization` header with a
  `UnicodeEncodeError` 8 frames deep in `httpx2`, no hint it's the key.
  `chat.py` now checks `api_key.isascii()` upfront with a clear message.
- **This sandbox's `httpx2` (openai's HTTP client dep) has a broken
  brotli decoder** — `Accept-Encoding: identity` on the OpenAI client
  works around it (see `chat.py`).
- **The model (gpt-oss-120b via Cerebras) does not reliably call
  `move_robot_to_position_safe`**, even when asked by exact tool name —
  it defaults to `move_robot_to_position` most of the time, inconsistently
  across otherwise-identical prompts. Don't rely on natural language to
  demo the Diamond safety gate; use `test_server.py` instead, which calls
  it directly and deterministically shows the speed-cap rejection.
- **The tool-call loop had no round cap** (fixed): reproduced live, the
  model called one tool 8–31 times in a row with identical args instead
  of ever stopping or switching. `chat.py` now caps at `MAX_TOOL_ROUNDS=8`.
- **Vague requests get clarifying questions, not invented values** — e.g.
  "safely move somewhere new" made the model ask for joint angles rather
  than pick any; "grab something" asked for a target pose. This is
  correct/expected behavior, not a bug, but means demo phrasing needs
  real numbers (or context the model can compute from, e.g. "move up 5cm
  from here" right after a state read) for anything that isn't a
  documented default like home.
- **`CommandQueue`'s cancellation only works because it stops waiting on
  the blocking call, not because it stops the call itself.** Found live:
  after `stop_robot()`, the worker's `robot.move_joint()` call for the
  cancelled command doesn't return early -- `move_joint` has no
  cancellation hook, so it keeps polling toward the original (now
  unreachable) target for its own full ~20s `timeout_s` before finally
  raising `TimeoutError`. Originally the worker awaited `cmd.fn()`
  directly, so the whole queue stayed stuck for up to 20s after every
  interruption. Fixed by running `cmd.fn()` on a throwaway per-command
  thread and having the worker loop move on as soon as
  `stop_requested` fires, leaving the stale thread to finish (harmlessly
  read-only polling) in the background. Verified live: `move_robot_queued`
  → `stop_robot` mid-motion → `get_queue` shows idle within ~0.1s, not 20s.
- **A stale/leftover `rclpy` process can silently break DDS discovery for
  everyone else, even under the correct RMW.** Found live building
  `ros2_vision_bridge`: with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` set
  and the daemon restarted, `ros2 topic list`/`ros2 topic echo` still saw
  nothing from a running publisher node, and its log filled with continuous
  `ddsi_udp_conn_write ... failed` noise aimed at a stale peer address.
  Killing the old process and starting a fresh one (`pgrep -af <node>`,
  `kill -9`, relaunch) fixed it immediately — the actual publisher/
  subscriber code was fine throughout. If ROS2 topics seem to vanish for
  no code reason, suspect a stale process before the RMW/daemon config.
  Also: `pkill -f <pattern>` in this sandbox's Bash tool sometimes reports
  a nonzero/odd exit code even when it *did* kill the process (and
  sometimes doesn't kill it at all) — always verify with `pgrep -af
  <pattern>` afterward rather than trusting pkill's own exit status.
- **Root cause of the DDS-discovery flakiness found (2026-08-20): this
  WSL2 sandbox's only non-loopback interface can't send UDP to itself.**
  `ros2 topic list`/`echo` would hang indefinitely (not error, just never
  return), with the log filling with `ddsi_udp_conn_write ... failed`
  aimed at this host's own eth1 address and the DDS multicast group
  (`239.255.0.1`) — a hairpin-NAT/virtual-switch limitation of this WSL2
  network, not a code bug. Every SPDP discovery packet CycloneDDS sends
  over that interface fails, so nodes can't find each other until a lucky
  retry (the old `ros2 daemon stop && ros2 daemon start` workaround just
  got lucky by forcing a fresh discovery round, not by fixing anything).
  Real fix, scoped to work that doesn't need to interoperate with nodes
  outside this shell's own loopback: `ros2_vision_bridge/
  cyclonedds_localhost.xml` forces CycloneDDS onto `lo` only (multicast
  disabled, unicast-peers to `localhost`) — `export CYCLONEDDS_URI="file://
  $(pwd)/cyclonedds_localhost.xml"` (run from that directory) alongside
  the usual `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Verified: 10
  consecutive `ros2 topic echo --once` calls against a live publisher, all
  instant, zero hangs. **Deliberately not set in `~/.bashrc`** — confirmed
  live that a shell with this config can't see nodes started under the
  default config on another interface (tested against this sandbox's
  already-running `ur_robot_driver`/`ur_moveit_config` stack: `/joint_states`
  invisible with the fix on, visible with it off). Fine for the vision
  stack (`safety_stop_demo.py` talks to the robot over a socket, not the
  ROS graph) but wrong for anything that needs to share a graph with
  `move_group`/the UR driver — use the plain daemon-restart workaround for
  those instead, or restart that whole graph fresh under the same config.
- **Webcam frames over `usbipd-win`/WSL2 need MJPG forced, not the default
  raw format, plus a handful of discarded warm-up frames.** Found live: the
  first `cv2.VideoCapture` reads after attaching a real USB webcam this way
  came back as a solid green image — `cap.set(cv2.CAP_PROP_FOURCC,
  cv2.VideoWriter_fourcc(*"MJPG"))` plus reading-and-discarding ~10 frames
  before trusting the feed fixed it (see `vision_human_track/live_demo.py`
  and `src/api.py`'s `_get_camera`). Also: opening the camera device fresh
  on every single request (as `/detect/live` originally did) is both slow
  (~0.5s/call) and re-triggers this same green-frame issue every time since
  the warm-up never happened — keep the device open across requests instead
  (~40ms/call once warm).
- **A tool returning an empty top-level dict (`{}`) deserializes as `None`
  on the FastMCP client side, not `{}`.** Found live building
  `list_waypoints`: with zero waypoints saved, `client.call_tool(...).data`
  came back `None`, breaking any caller that assumed a dict. Fixed by
  nesting under a named key (`{"waypoints": {...}}`) instead of returning
  the mapping as the tool's top-level result — guarantees a non-empty
  structure regardless of how many items are inside. Worth checking for
  any future tool whose natural return type could legitimately be empty.
- **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is required** in this sandbox
  — the default RMW (FastRTPS) hangs on multicast discovery for any
  `ros2`/`rclpy` process. Set for every such process, mixing RMWs on the
  same topics is unreliable. Already in `~/.bashrc` on this machine.
- **WSLg bridges the Windows host mic/speakers in as PulseAudio**
  (`RDPSource`/`RDPSink`, `$PULSE_SERVER` pre-set) — this is what makes
  both `voice.py` (capture) and `tts.py` (playback) work at all in WSL2.
  Code never hardcodes these names; works on any Linux with a real
  PulseAudio default source/sink too.
- **`moveit_py` isn't packaged for ROS2 Humble via apt** (only iron/
  rolling) — `motion_planner.py` talks to `move_group`'s plain ROS2
  services (`GetMotionPlan`, `GetStateValidity`, `ApplyPlanningScene`,
  `GetPlanningScene`) directly via rclpy instead. `ros-humble-moveit` +
  `ros-humble-ur-moveit-config` (apt) were enough — no source build needed.
- **`ApplyPlanningScene`'s `CollisionObject` normalizes the pose you send**
  — found live building `list_obstacles`: the pose passed to
  `add_box_obstacle` comes back as `obj.pose` (the object's own origin) on
  a later `GetPlanningScene` read, with `obj.primitive_poses[0]` reset to
  identity (relative to `obj.pose`, not the world). Reading
  `primitive_poses[0]` for world position (the natural first guess) silently
  returned `(0, 0, 0)` for every obstacle regardless of where it was placed.
  Fixed by reading `obj.pose` instead.
- **PolyScope X (URSim) has no dashboard-server protocol on port 29999** —
  confirmed again live: connects, then the socket closes with no banner and
  no reply to any command. Powering the robot on / releasing brakes only
  works through the browser UI (`http://localhost`), same as `README.md`
  already said — there's no way to script it from this sandbox.
- **`openwakeword` (via scikit-learn -> scipy) crashes on import against
  this sandbox's numpy 2.2.6** — `pip install openwakeword` pulls numpy>=2
  as a transitive dependency, but the OS's pre-installed
  `/usr/lib/python3/dist-packages/scipy` (1.8.0) was compiled against
  numpy<1.25, so the moment anything imports it: `AttributeError: _ARRAY_
  API not found` -> `ImportError: numpy.core.multiarray failed to import`.
  Fixed with `pip install --upgrade scipy` (installs 1.15+ to
  `/usr/local`, which shadows the broken OS one on `sys.path`) — no numpy
  downgrade needed, and nothing else in this repo (faster-whisper included)
  broke from the scipy upgrade.
- **An un-shut-down `rclpy` node/executor can abort the interpreter on exit
  even after the actual test has already passed** — reproduced live:
  `test_server.py` printed `ALL PASSED`, then crashed with `terminate
  called without an active exception` + a core dump, because
  `motion_planner.py`'s `MotionPlanner` had no `atexit` cleanup hook (unlike
  `ros2_client.py`'s `ROS2URClient`, which already had one for the same
  reason). Fixed by registering `atexit.register(self.disconnect)` at the
  end of `connect()`, mirroring `ROS2URClient` exactly — same pattern,
  independently rediscovered. If a new class wraps an `rclpy` node, give it
  this from the start rather than waiting to hit the crash.
- **Running a vision script without activating `vision_human_track/venv`
  first crashes with a numpy/cv2 ABI mismatch, not an obvious "wrong
  python" error.** Found live: `python3 realsense_vision_node.py` from a
  plain shell (no venv activated) failed importing `cv2` with `AttributeError:
  _ARRAY_API not found` → `ImportError: numpy.core.multiarray failed to
  import`. Root cause: the system's `python3` (`/usr/bin/python3`) picks up
  `/usr/lib/python3/dist-packages/cv2...so` — an apt-installed cv2 (pulled
  in by a ROS package) built against numpy 1.x — while the system's own
  numpy is 2.2.6. Same *category* of bug as the openwakeword/scipy ABI
  mismatch above, different package, different fix: there's no upgrade
  path for the apt cv2, so the fix is just "activate the venv" (it has a
  matched numpy+opencv-contrib-python pair) — `source vision_human_track/
  venv/bin/activate` before running any script that imports `cv2`/
  `mediapipe`, exactly as every README's run commands already show. Check
  `which python3` if this error shows up again — if it says `/usr/bin/
  python3`, the venv isn't active.
- **Only one process can hold the RealSense at a time.** Found live: with
  `realsense_vision_node.py` already running, starting `live_demo.py
  --realsense` in another terminal raised `RuntimeError: RealSense device
  found but its color/depth pipeline failed to start.` — pyrealsense2
  finds the device fine (it's still enumerable) but a second `pipeline.
  start()` fails outright. Not a bug, just means: before switching between
  `live_demo.py`, `realsense_vision_node.py`, or any other script that
  opens the camera, stop whichever one is already running first
  (`pgrep -af realsense_vision_node.py`/`live_demo.py`, `kill`).
- **Real depth (`distance_m`) added 2026-08-20** — `RealSenseCapture.
  get_distance(x, y)` in `vision_human_track/src/realsense_camera.py`;
  `add_distances()`/`draw_distances()` in `live_demo.py` (imported and
  reused, not duplicated, by `realsense_vision_node.py`). Only two things
  get a distance: each hand's `palm_center` pixel and (with `--yolo`) each
  YOLO box's center pixel — skeleton joints don't. **Verified with
  synthetic depth data only** (median/outlier-rejection, out-of-bounds,
  no-depth-frame-yet, all covered) — the RealSense was disconnected from
  this sandbox mid-build. It has since been reattached and run live by the
  user (`realsense_vision_node.py` publishing real `/vision/objects_json`/
  `/vision/humans_json` with non-null `distance_m` values), so the
  end-to-end path is exercised, but nobody has yet checked a `distance_m`
  reading against an object at an actually-known distance — treat the
  *numbers* as unverified until that specific check happens, even though
  the plumbing clearly works.

## Team repo sync (tom-bourjala/UR-MCP)

The vision work in this repo (`vision_human_track/`, `ros2_vision_bridge/`,
and `../yolo/` minus its model weights/generated images) is also mirrored
to the team's shared GitHub repo, a completely separate remote from this
repo's own `origin` (`SangukBae/UR_01`):
`https://github.com/tom-bourjala/UR-MCP.git`, branch
`vision/human-hand-tracking`.

**This is a manual sync, not an automated one** — there's no second git
remote configured in *this* repo pointing at it. The actual process, done
several times this session: clone `tom-bourjala/UR-MCP` fresh into a
scratch directory on the `vision/human-hand-tracking` branch, `diff -q`
every relevant file against this repo's current copy to find exactly
what's new/changed (that branch's history is its own — different commit
hashes than this repo even for identical content, since it was originally
seeded by copying files, not a shared ancestry), copy over just the
delta, commit with `--author="SangukBae <halmoney956@gmail.com>"` (the
sandbox's git identity auto-detects as `root@<hostname>` otherwise — always
override it), and push. That team repo's root has its own `README.md` —
a plain command list (setup/run commands only, no explanations) covering
`vision_human_track`, `ros2_vision_bridge`, and `yolo` — kept manually in
sync with whatever's actually runnable there; update it in the same pass
whenever new run commands are added here.

**When asked to "push to the team repo" again**, don't assume a prior
session's scratch clone is still around or current — re-clone (or
`git fetch` + check) fresh, since this repo's own `origin` and that repo
diverge independently and either could have moved. Case1/CLAUDE.md/etc.
never belong on that branch — it has no root docs of its own beyond the
command-list README, and case1 isn't vision-stack scope.

## Block-diagram cross-check

`~/block_diagram.excalidraw` is the user's own hand-drawn team
architecture sketch (not part of this repo) — parsed once (2026-08-19) and
cross-checked against actual repo state. Its boxes are color-coded green
= done, orange = needs ROS integration, blue = not implemented, per the
team's own convention (see meeting notes). Summary, so a future session
doesn't have to re-parse the file from scratch:

**Matches reality:** `Low level control`, `Robot state feedback`,
`Reliable STOP Command`, `Waypoints Planning` (all green) — Bronze/Silver/
Gold/`stop_robot`. `Queue Management` (green) — `queue_manager.py`, though
it lives at the MCP layer, not an actual ROS node like the diagram's `ROS`
box implies.

**Was blue/orange (not done) when drawn, now done (this session):**
`Waypoints control/management (Joint space)` → waypoint DB tools.
`HumanID` (inside `Vision stack`) → `vision_human_track/`. The specific
tool-name legend's `waypoint_add`/`waypoint_remove`/`get_waypoints`
(orange) → `save_waypoint`/`delete_waypoint`/`list_waypoints`;
`freedrive_mode` (blue) → `free_drive`.

**Diagram says green/done but isn't actually built:** the legend's
`move_joint_relative`, `move_linear_relative`, `move_linear_sequence` were
marked green alongside the tools that *are* done — turned out to be
optimistic. Built this session (`move_robot_to_position_relative`/
`move_robot_linear_relative`/`move_robot_linear_sequence`) to close that gap.

**Was blue (not done) when drawn, now done (2026-08-20):** `Path checking`
and `Obstacle Avoidance` → `check_path`/`add_obstacle`/`remove_obstacle`/
`list_obstacles` (`motion_planner.py`, MoveIt-backed — real OMPL planning
against a live planning scene, not a stub). Verified live against the
simulator: a box obstacle placed in the arm's way makes `check_path` report
infeasible with the specific colliding links, and feasible again once
removed (`test_server.py`'s path-checking block).

**Still genuinely not built** (all need a missing dependency this repo
doesn't have, not just missing wiring): `Robot Digital Twin` in the
diagram's strict sense (a 3D scene view / mirrored sim state -- MoveIt's
planning scene *is* an internal one now, but there's no visualization or
external mirroring of it); `Environment Shadow` in the diagram's strict
sense (`get_environment_shadow` is a cache of vision+state, not a 3D map);
`ObjectID`/`grab_object`/`place_object`/`give_object` (no object-detection
model — different teammate's task); real spatial `Safety` (needs
camera↔robot calibration — `safety_stop_demo.py` only reacts to hand
*presence*, not real proximity, and obstacles in the new planning scene are
added manually via `add_obstacle`, not auto-populated from vision yet);
`program_new`/`program_delete`/`program_start`/`program_stop` were blue but
ARE now built (named waypoint sequences); `track` was blue and is now built
but deliberately crude (1-DOF, no calibration) — see its docstring.

## Shadow mode (sim-then-real), added 2026-08-19

User asked whether the LLM could drive the real robot and the simulator at
the same time. Recommended sequential ("verify in sim, then execute on
real") over simultaneous mirroring — a bad sim command has already reached
the real robot in the mirroring design, whereas gating on sim success first
means the sim can act as a genuine safety pre-check. User agreed; built
`case1/shadow_client.py`.

`ShadowClient` duck-types `URClient`'s surface (`connect`, `get_state`,
`move_joint`, `move_linear`, `move_waypoints`, `set_gripper`, `free_drive`,
`stop`) so none of `server.py`'s 31 pre-existing tools needed to change —
only the backend-selection block did (`UR_REAL_HOST` env var, socket
backend only; raises clearly if combined with `UR_BACKEND=ros2`, not
supported — one ROS2 graph talks to one controller). Target-reaching moves
run on sim first and block; only on success does the identical command go
to the real robot — a sim failure (protective stop, timeout, joint limit)
never reaches the real arm. `stop()` is the deliberate exception: sent to
both independently, each in its own try/except, never gated on the other.
`get_state()` (so `get_robot_state` and everything downstream, including
`get_environment_shadow`'s cache) reports the *real* robot once one is
configured — sim is a pre-flight gate, not what operators are told the
robot is doing. `free_drive()` skips sim entirely (no target to verify
against) and goes straight to whichever robot is meant to be hand-guided.

**Pose sync (added 2026-08-20).** The pre-flight gate above only means
anything if the simulator's current pose actually matches the real robot's
— otherwise "verified safe in sim" describes a swept path (singularities,
joint limits) the real robot, starting somewhere else entirely, would never
actually take. This is a real gap, not hypothetical: the simulator boots at
its own home pose regardless of wherever the physical arm happens to be
(powered on mid-lesson, jogged by hand, left over from a previous session).
Found live the first time a real UR10 was actually reachable from this
sandbox (`192.168.1.100`, discovered via a port probe of the workspace LAN
— has 29999/30001-30004 all open, unlike the simulator which has no
dashboard-server on 29999, see Known gaps): the real robot's joints
(~`[-78, -96, -65, -79, 58, 59]` deg) were nowhere near the sim's home pose
(`[0, -90, 0, -90, 0, 0]` deg). Fixed with a one-directional sync — never
the reverse, since moving the real robot to match an arbitrary sim pose is
exactly the unverified motion shadow mode exists to prevent:
`ShadowClient.connect()` now reads the real robot's actual joint angles and
drives the SIMULATOR to match before anything is verified.
`sync_sim_to_real()` (new `server.py` tool, same name) re-does this on
demand; it also now runs automatically right after `free_drive()`, since
hand-guiding moves only the real robot and would otherwise leave the sim
stale from that point on.

Verified: full default `test_server.py` (32 tools, live simulator, no
`UR_REAL_HOST`) still passes unchanged — confirms shadow mode is genuinely
opt-in with zero behavior change when off. `case1/test_shadow_client.py`
(network-free, fake stand-in clients) covers: sim failure blocks real
entirely; sim success replays onto real with identical args; real failure
after sim success raises a clear combined error (not silently swallowed);
`get_state()` prefers real; `stop()` attempts both independently even when
one raises; `free_drive()` skips sim for the move itself but triggers a
sim resync right after; `connect()` syncs sim to real up front;
`sync_sim_to_real()` is a no-op with `real=None`. All pass.

**Verified live against an actual physical robot (2026-08-20)** — the
first time one has been reachable from this sandbox. Confirmed via direct
socket connect + RTDE read (not just ping) that `192.168.1.100` is a real
UR controller, then ran `ShadowClient.connect()` against it: the simulator
jumped from its home pose to the real robot's actual live joint angles with
<0.01 deg residual error, and a follow-up read confirmed the real robot's
own joints were completely unchanged by the sync (only the simulator ever
moves for this). This exercised real network behavior (real controller
RTDE/primary-port connect timing) for the first time, not just the gating
logic — but deliberately state-reading and sim-only motion so far, not a
full shadow-mode move through to the real arm; that needs the user present
with hands on/near the e-stop before it's tried. Before pointing this at a
physical arm for actual motion: confirm the workspace is clear, someone has
hands on/near the physical e-stop, and target poses have already been
proven safe on the simulator alone first.

## Known gaps (deliberately not built — don't rediscover these as bugs)

- **ROS2 backend has no IK** — `move_robot_linear` raises
  `NotImplementedError` there (`scaled_joint_trajectory_controller` is
  joint-space only). Socket backend only for that tool.
- **No pick-and-place / perception** — needs a camera/object-detection
  piece from the team's wider architecture, out of scope for this repo.
  (`get_vision`/`get_environment` now exist and report people/hands via
  `vision_human_track`, but object detection specifically is still not
  built — a different teammate's task per the 2nd meeting.)
- **`track`'s "hand detected" branch was never exercised live** — the
  live `test_server.py` run that verified it (2026-08-19) had nobody in
  front of the camera, so it only proved the "no hand → skip gracefully,
  don't crash" path (0 updates over 3s). The re-aim math itself
  (`(hand_x - 0.5) * 2 * 45` → degrees) is simple enough to trust by
  inspection, but the actual robot-moves-toward-a-real-hand behavior, and
  which direction "toward" even is (camera framing/mirroring wasn't
  checked), is unverified. Also genuinely crude by design regardless —
  see its docstring on why (no camera↔robot calibration exists).
- **`get_vision`/`get_environment`/`get_environment_shadow`/`track` need
  `vision_human_track` running separately** (`VISION_API_URL` env var,
  default `http://localhost:8000`) — `case1/server.py` doesn't start or
  manage that process. `get_environment_shadow`'s background refresh
  thread degrades gracefully (returns `vision: null` +
  `vision_error`) if it's not reachable; `get_vision`/`get_environment`/
  `track` raise/skip a clear error on each call instead.
- **`stop_robot` can't interrupt a move from within the same chat turn —
  except now it can, for `move_robot_queued`.** `chat.py`'s tool-call loop
  is still sequential/blocking, and the *plain* tiered move tools
  (`move_robot_to_position`, `move_robot_linear`,
  `move_robot_to_position_safe`, `move_through_waypoints`) still don't
  return control until the robot arrives, so `stop_robot` genuinely can't
  reach one of *those* mid-flight in the same turn. But `move_robot_queued`
  (added for the team's non-blocking architecture design) returns
  immediately — verified live: submit a slow `move_robot_queued`, then
  call `stop_robot` right after in the same tool-call sequence, and the
  robot stops mid-motion, well short of the target. So the fix for this
  gap is "use `move_robot_queued` instead of the blocking tools," not a
  `chat.py` restructure — true "listen while moving" (interrupting a
  *voice* command mid-utterance, not just mid-tool-call-sequence) still
  needs running tool execution and input concurrently, not attempted.
- **Wake-word now exists (2026-08-20), opt-in.** `chat.py --voice
  --wake-word` waits silently in the background for a wake word
  (`wakeword.py`, pretrained openwakeword ONNX models) before arming a
  command capture, instead of listening for a command immediately every
  turn. Default word is openwakeword's bundled "hey jarvis" -- the team's
  earlier candidate ("Ravel") isn't one of the bundled words and would need
  training a custom model from scratch (a separate, later effort). Verified
  live against the real mic for a sustained wait with no crash; NOT verified
  against an actual spoken wake word in this sandbox (no way to speak into
  the mic here) -- the detection logic itself is exercised, the actual
  trigger-on-real-speech behavior isn't.
- **Filler-word filtering is narrow by design** — only drops an utterance
  that's *exactly* one bare interjection ("um", "uh", ...); doesn't
  filter rambling non-command sentences (the LLM itself decides those
  aren't actionable, which works but isn't a real filtering layer).
- **No gripper visualization in the simulator** — checked: no free
  PolyScope X (URCapX-format) gripper package exists yet from any vendor
  or UR's own SDK samples (those are all classic-PolyScope format). The
  digital-IO signal itself is real and now has an actual readback
  (`gripper_closed` in `get_robot_state`, sourced from
  `actual_digital_output_bits` / ROS2 `io_states` — not an echo of what
  was commanded).
- **Latency is now measured (2026-08-20), still unoptimized.** `chat.py`
  prints a per-turn breakdown after every reply (`latency.py`'s
  `TurnTimer`): each LLM call, each MCP tool call, and a total against the
  team's 1500ms budget; `--voice` additionally prints the STT split
  (record + transcribe time, with silent wait-for-speech time excluded
  from the budget — a human deciding when to talk isn't part of it). One
  real number from this session, MCP-call-only (no CEREBRAS_API_KEY
  available in this sandbox to run the full LLM loop live):
  `get_robot_state` ~20ms, `check_path` (MoveIt planning) ~373ms over an
  actual round trip against the live simulator + move_group — notably
  slower than a plain state read, worth watching if `check_path` ever
  goes in the hot path of a voice command. No actual full voice-to-robot
  number has been captured yet (needs a real API key + microphone run,
  neither available in this sandbox session) — the instrumentation is
  verified, the team's real end-to-end number is still unknown. Streaming
  responses and parallel tool calls are still not implemented. Vision
  pipeline parallelization (`vision_human_track/src/detector.py` running
  pose and hand inference sequentially) is a separate, still-open latency
  item — out of scope for this round (vision work deliberately excluded).
- **`detector.py`'s hand-to-pose wrist matching doesn't check MediaPipe's
  `.visibility` on the wrist landmark** — unlike the skeleton-drawing code
  in `tests/test_real_image.py`, which skips landmarks below 0.3. An
  occluded wrist still gets an extrapolated (low-confidence) coordinate
  that's used as-is in the nearest-wrist search, so a hand can silently
  get matched to the wrong person when that estimate happens to land
  within `HAND_TO_POSE_MATCH_THRESHOLD` of someone else. Not fixed — flag
  if hand-to-person assignment looks wrong with multiple people in frame.

## Running things

```bash
# Simulator (course repo, not duplicated here)
cd "international-summer-school-robotics-TER-UR/simulation environment" && docker compose up -d
# Power on + release brakes at http://localhost until RUNNING

# Case 1 server smoke test (all 32 tools, live)
cd case1 && python3 test_server.py                    # socket backend
UR_BACKEND=ros2 python3 test_server.py                 # ROS2 backend (driver must be launched)
UR_REAL_HOST=<real-controller-ip> python3 server.py     # shadow mode: sim-verify, then replay on the real robot
                                                         # (192.168.1.100 on this workspace's LAN, 2026-08-20 --
                                                         # confirm still live before reusing, IPs can change)
python3 test_shadow_client.py                           # shadow-gating + pose-sync unit test, no network/robot needed
# Vision-passthrough tools need vision_human_track's API running first (below) --
# test_server.py skips those specific tests gracefully if it's not reachable.
# check_path/add_obstacle/remove_obstacle/list_obstacles need move_group running too (below) --
# same graceful-skip pattern if it's not reachable.

# MoveIt: path checking + obstacle avoidance (needs ur_robot_driver already
# launched, below -- move_group reads /joint_states from it)
source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur10 \
  robot_ip:=127.0.0.1 \
  kinematics_params_file:="$(pwd)/../ros2_ur_driver/config/ur10_calibration.yaml" \
  launch_rviz:=false
python3 case1/motion_planner.py                         # standalone smoke check: path to HOME

# Chat client
cd llm_client
export CEREBRAS_API_KEY="..."   # never in a file
python3 chat.py                 # type
python3 chat.py --voice         # speak

# Vision Stack: human detection + hand tracking (own venv, see its README)
cd vision_human_track && source venv/bin/activate
python3 tests/test_math.py && python3 tests/test_smoke.py && python3 tests/test_real_image.py
docker compose up -d --build && curl http://localhost:8000/health
python3 live_demo.py                                   # live GUI, needs a camera + display
python3 live_demo.py --realsense --yolo                 # RealSense camera + object detection (+ distance_m)
python3 live_demo.py --realsense --yolo --ros2           # same, AND publishes all 4 /vision/* topics at once
                                                          # (needs /opt/ros/humble/setup.bash + RMW_IMPLEMENTATION first)

# ROS2 bridge: republishes the Vision Stack as topics
cd ros2_vision_bridge
python3 tests/test_geometry.py                          # plain python3, no ROS needed
source /opt/ros/humble/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(pwd)/cyclonedds_localhost.xml"   # optional, fixes flaky discovery - see Verified findings
python3 vision_bridge_node.py                            # polls vision_human_track's REST API (needs it running above)
python3 realsense_vision_node.py                         # OR: owns a RealSense directly, no REST API needed, adds /vision/objects_json
python3 safety_stop_demo.py                              # closes the loop: hand -> stop_robot (works with either bridge above)
```

ROS2 driver launch command and calibration are in `ros2_ur_driver/README.md`.
