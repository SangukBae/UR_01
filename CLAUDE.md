# UR_01 — Project Notes for Claude Code

UR Robotics Summer School 2026, Case 1 (MCP Server for Robot Tools) plus a
ROS2 driver integration and a custom LLM chat wrapper (text + voice), all
against the PolyScope X (URSim) simulator. This file is durable project
context for future sessions — architecture, verified findings, known gaps —
not a changelog; see `git log` for that.

## Status

All four official tiers (Bronze → Diamond, per the course's
`ESSRE2026_UR_Industry_Cases_tiers.docx`) are implemented and verified
against the live simulator, on both robot backends. Plus, beyond the
official spec: a `stop_robot` tool, a standalone LLM chat client with voice
I/O, and a ROS2 driver integration — all verified live, not just written.

Repo layout:
- `case1/` — the MCP server (`server.py`), socket backend (`ur_client.py`),
  FK safety check (`kinematics.py`), smoke test (`test_server.py`).
- `ros2_ur_driver/` — ROS2 backend (`ros2_client.py`), calibration config,
  driver setup notes.
- `llm_client/` — standalone chat wrapper: `chat.py` (text/voice loop),
  `voice.py` (mic capture + STT), `tts.py` (spoken replies).

Each folder's own README has setup/run commands and more detail than this
file repeats. Read those first for "how do I run X"; read this file for
"why does X work this way" and "what's already been tried and found true
or false."

## Architecture

`server.py` exposes 8 MCP tools over stdio. Two interchangeable backends
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
`move_robot_to_position_safe` + `set_gripper`. Extra: `stop_robot`,
`example` (template).

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
- **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is required** in this sandbox
  — the default RMW (FastRTPS) hangs on multicast discovery for any
  `ros2`/`rclpy` process. Set for every such process, mixing RMWs on the
  same topics is unreliable. Already in `~/.bashrc` on this machine.
- **WSLg bridges the Windows host mic/speakers in as PulseAudio**
  (`RDPSource`/`RDPSink`, `$PULSE_SERVER` pre-set) — this is what makes
  both `voice.py` (capture) and `tts.py` (playback) work at all in WSL2.
  Code never hardcodes these names; works on any Linux with a real
  PulseAudio default source/sink too.

## Known gaps (deliberately not built — don't rediscover these as bugs)

- **ROS2 backend has no IK** — `move_robot_linear` raises
  `NotImplementedError` there (`scaled_joint_trajectory_controller` is
  joint-space only). Socket backend only for that tool.
- **No pick-and-place / perception** — needs a camera/object-detection
  piece from the team's wider architecture, out of scope for this repo.
- **`stop_robot` can't interrupt a move from within the same chat turn.**
  `chat.py`'s tool-call loop is sequential/blocking — saying "Stop" while
  a move is still in flight in the *same* conversation doesn't reach it,
  because the mic/input loop isn't running until that call returns. Works
  fine between turns or from a second concurrent client. True "listen
  while moving" needs running tool execution and input concurrently — a
  bigger restructure, not attempted.
- **No true wake-word / continuous background listening** — `voice.py`
  is armed per `listen()` call (each chat turn), not continuously. A real
  wake-word engine (Porcupine, openWakeWord) would be its own dependency.
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
- **Latency is unmeasured/unoptimized** — Cerebras itself is fast, but no
  work has gone into streaming responses, parallel tool calls, or
  measuring actual round-trip time.

## Running things

```bash
# Simulator (course repo, not duplicated here)
cd "international-summer-school-robotics-TER-UR/simulation environment" && docker compose up -d
# Power on + release brakes at http://localhost until RUNNING

# Case 1 server smoke test (all 8 tools, live)
cd case1 && python3 test_server.py                    # socket backend
UR_BACKEND=ros2 python3 test_server.py                 # ROS2 backend (driver must be launched)

# Chat client
cd llm_client
export CEREBRAS_API_KEY="..."   # never in a file
python3 chat.py                 # type
python3 chat.py --voice         # speak
```

ROS2 driver launch command and calibration are in `ros2_ur_driver/README.md`.
