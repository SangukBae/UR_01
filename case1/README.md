# Case 1: MCP Server for Robot Tools

MCP server exposing UR robot capabilities as LLM-callable tools -- any MCP
client (Claude Code, Cursor, Case 3 agent) launches it and calls its tools.
Four-step tool shape (validate → convert units → check limits → execute),
each following `move_robot_to_position`. **Status: all four tiers
implemented and verified** (`test_server.py` passes end to end, both
backends).

## What's provided vs what you build

| File | Role |
|------|------|
| `server.py` | the MCP server (32 tools): `move_robot_to_position` (Bronze), `get_robot_state` + `move_robot_linear` (Silver), `move_through_waypoints` (Gold), `move_robot_to_position_safe` + `set_gripper` (Diamond), `stop_robot` (extra, team requirement), `move_robot_queued` + `get_queue` (extra, non-blocking move with reject/queue/override semantics), `save_waypoint`/`list_waypoints`/`delete_waypoint`/`move_robot_to_waypoint`/`free_drive` (extra, named-pose database + hand-guided teaching), `move_robot_to_position_relative`/`move_robot_linear_relative`/`move_robot_linear_sequence` (extra, delta/multi-pose variants), `program_new`/`list_programs`/`program_delete`/`program_start`/`program_stop` (extra, named waypoint sequences), `get_vision`/`get_environment`/`get_environment_shadow` (extra, MCP-level passthrough to `vision_human_track`), `track` (extra, crude 1-DOF demo -- see its docstring), `check_path`/`add_obstacle`/`remove_obstacle`/`list_obstacles` (extra, MoveIt-backed path checking + obstacle avoidance, see ./motion_planner.py), `sync_sim_to_real` (extra, shadow-mode only -- re-align the simulator to the real robot's current pose on demand, see "Shadow mode" below), `example` (template) |
| `queue_manager.py` | background-thread command queue backing `move_robot_queued`/`get_queue`/`program_start` -- the non-blocking layer that makes `stop_robot` able to genuinely interrupt a move within one chat turn |
| `waypoint_store.py` | plain JSON-file-backed waypoint database (`waypoints.json`, gitignored -- local/runtime data) backing the waypoint tools |
| `program_store.py` | plain JSON-file-backed program database (`programs.json`, gitignored) -- named ordered lists of waypoint names, backing the program tools |
| `ur_client.py` | socket seam over the robot (motion + state, plus `free_drive`), `UR_BACKEND=socket` (default) |
| `shadow_client.py` | opt-in shadow-execution wrapper: verifies every move on the simulator first, only replays it on a real robot (`UR_REAL_HOST`) if the simulator move succeeded -- see "Shadow mode" below |
| `kinematics.py` | nominal UR10 forward kinematics, used only for the Diamond workspace-bounds safety check |
| `test_server.py` | in-process smoke test for every tool above (vision-passthrough tests skip gracefully if `vision_human_track` isn't running) |
| `requirements.txt` | dependencies: the MCP framework, `requests` (for the vision-passthrough tools) |
| `../ros2_ur_driver/ros2_client.py` | ROS2 seam over the robot via `ros_humble_ur_robot_driver`, `UR_BACKEND=ros2` -- same `RobotState` shape and methods as `ur_client.py`, so `server.py`'s tools don't change either way |

### Two robot backends

`server.py` picks its backend from the `UR_BACKEND` env var:

- `UR_BACKEND=socket` (default) -- direct TCP sockets to the controller
  (`ur_client.py`). No ROS2 needed.
- `UR_BACKEND=ros2` -- routes through `ros_humble_ur_robot_driver`
  (`../ros2_ur_driver/ros2_client.py`), matching the team's target
  architecture (MCP Server -> ROS2 -> UR Driver -> robot). Needs the driver
  already launched -- see `../ros2_ur_driver/README.md` -- and `rclpy` on the
  path (`source /opt/ros/humble/setup.bash`).

### Shadow mode: simulator + a real robot together

Set `UR_REAL_HOST=<real UR controller IP>` (with `UR_BACKEND=socket`) to have
every move-tool call go through `shadow_client.py`: it runs on the
**simulator first** and blocks until that succeeds or raises; only then does
the identical command go to the **real robot**. If the simulator move fails
(protective stop, timeout, joint limit), the real robot is never touched.
`stop_robot` is the one exception -- it's sent to both robots independently
and immediately, never gated behind the other's result. `get_robot_state`
(and every other state read) reports the real robot once one is configured.
`free_drive` skips the simulator (no target to verify) and goes straight to
whichever robot is meant to be hand-guided.

**Pose sync.** The simulator boots at its own home pose regardless of
wherever the physical arm actually is (powered on mid-lesson, jogged by
hand, left over from a previous session) -- so on its own, "verified safe
in sim" would describe a swept path the real robot, starting somewhere
else, would never actually take. To fix that, `connect()` reads the real
robot's current joint angles and drives the simulator to match BEFORE
anything is verified -- never the other way; only the simulator ever moves
for this, the real robot is untouched. This re-runs automatically right
after `free_drive` too (hand-guiding only moves the real robot, so the sim
would otherwise be left out of sync the moment free-drive ends). Call the
`sync_sim_to_real` tool yourself for any other point where the two might
have drifted apart -- e.g. the real robot was jogged from the teach pendant
mid-session.

Not enabled by default -- with `UR_REAL_HOST` unset, `robot` is a plain
`URClient` exactly as before this existed. `test_shadow_client.py`
exercises the gating and pose-sync logic itself (call order,
propagate-vs-swallow, `get_state`/`stop` routing, sync-on-connect,
sync-after-free_drive) against fake stand-in clients, no network needed.
Verified live against an actual physical UR10 (2026-08-20, `192.168.1.100`
on this workspace's LAN): `connect()`'s auto-sync moved the simulator to
the real robot's live joint angles (~[-78, -96, -65, -79, 58, 59] deg, far
from the sim's own home pose) with <0.01 deg residual error, and the real
robot's own joints were confirmed unchanged by the sync. Before pointing
this at a real arm for actual MOTION (not just read/sync): confirm the
workspace is clear, someone has hands on (or near) the physical e-stop, and
the target poses have already been exercised safely on the simulator alone.

**Don't mix backends against the same live robot in one session** -- the
socket backend's raw URScript upload knocks out the ROS2 driver's External
Control program (`Controller is not running` on its next goal). Recover:
```bash
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}"
```
Also fixes External Control dropping on its own after idle.

**RMW hang:** if any `ros2` command hangs, `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
(`sudo apt install ros-humble-rmw-cyclonedds-cpp`).

**Singularities break `move_robot_linear`:** a Cartesian move through one
needs near-infinite joint speed → protective stop (`safety_status` 3).
Reproduced at `HOME_DEG` (wrist2=0°) and even a pose picked to avoid that
(a different, non-obvious singularity). `test_server.py`'s pose
(`[30, -100, 40, -80, 70, 10]`) is verified clean -- don't assume a new
pose is safe without the same check. No dashboard-server protocol on this
sim to unlock a stop from code; it also self-clears after a few seconds.

## Setup

1. **The PSX simulator is running.** Start it from `../simulation environment`:
   ```bash
   cd "../simulation environment" && docker compose up -d
   ```
   First boot takes about 40 seconds.
2. **The robot is powered on.** Open http://localhost, then power the robot on
   and release the brakes in the control panel at the bottom. It must read
   RUNNING before it will move.
3. **Python deps installed** in this folder:
   ```bash
   pip install -r requirements.txt
   ```

The server connects to `127.0.0.1` (the simulator). Set `UR_HOST` to target a
different host or a real robot.

## Run

Cheapest checks first, then connect a client.

**1. Socket seam (no MCP).** Confirms the robot is reachable and state reads:
```bash
python -c "from ur_client import URClient; r=URClient(); r.connect(); print(r.get_state())"
```

**2. MCP tools in-process.** Exercises every tool, input validation, and real
motion, without an LLM or a subprocess:
```bash
python test_server.py               # UR_BACKEND=socket (default)
UR_BACKEND=ros2 python test_server.py  # against the ROS2 driver instead
```
If the robot is off, both fail with a clear "not powered on" message. That error
is the guard working.

**3. Connect a client.** Server speaks MCP over stdio -- the client launches
`server.py`, no port to connect to.

Recommended: [`../llm_client/`](../llm_client/) (text or `--voice`, Cerebras
by default, this repo's own):
```bash
cd ../llm_client && pip install -r requirements.txt
export CEREBRAS_API_KEY="..."   # never in a file
python3 chat.py            # or --voice
```

Claude Code:
```bash
claude mcp add ur-tools -- python3 /PATH/TO/UR_01/case1/server.py
claude mcp list
```

GUI clients (Bionic, OpenClaw, Cursor, Claude Desktop): MCP entry, command
`python3`, arg the absolute path to `server.py` (31 tools on probe). Course
guides for Bionic/OpenClaw:
[`llm-client/self-hosted.md`](https://github.com/ureskr/international-summer-school-robotics-TER-UR/blob/main/llm-client/self-hosted.md),
[`llm-client/cloud-hosted.md`](https://github.com/ureskr/international-summer-school-robotics-TER-UR/blob/main/llm-client/cloud-hosted.md)
(course repo, not duplicated here).

Any of these: `move the robot home.`

## Tiers

- **Bronze, run it -- done:** `move_robot_to_position`. Bring up the sim,
  connect a client, and move the robot home.
- **Silver, read state -- done:** `get_robot_state`. Returns joints (angle +
  speed), TCP pose, mode, and safety status, so an agent can observe before it
  acts. `move_robot_linear` adds a TCP-space straight-line move (URScript
  `movel`, `UR_BACKEND=socket` only -- the ROS2 backend raises
  `NotImplementedError`, since `scaled_joint_trajectory_controller` is
  joint-space only and needs IK to accept a Cartesian target).
- **Gold, richer motion -- done:** `move_through_waypoints`. Blends a
  joint-space waypoint list into one motion, **streams state live** via MCP
  progress notifications as it moves (real push channel; `test_server.py`
  verifies notifications land before the call returns), and also returns
  the full `trace` for clients not watching progress. Blocking move runs in
  `asyncio.to_thread` so the loop stays free to flush notifications as they
  happen.

  Bug caught building this: no completion signal from the controller for a
  `movej` program, so arrival was guessed by proximity to the last
  waypoint -- false-positived instantly on a path looping back near its
  start, while the URScript kept running for real underneath. Fixed with a
  minimum-duration floor (`_estimate_path_duration`, trapezoidal estimate
  per segment).
- **Diamond, real skills -- done:** `move_robot_to_position_safe` adds a
  safety gate in front of a move -- joint limits, a speed cap, and a
  forward-kinematics workspace-bounds check (`kinematics.py`, nominal DH, a
  coarse estimate) -- and rejects an unsafe command before anything moves.
  `set_gripper` opens/closes via digital IO, and both it and
  `get_robot_state`'s `gripper_closed` report the controller's actual
  readback (`actual_digital_output_bits` over RTDE / `io_states` over
  ROS2), not just an echo of what was commanded. `safety_status` is
  surfaced by `get_robot_state`.

Next, not yet built: inverse kinematics for the ROS2 backend, so
`move_robot_linear` works there too (currently socket-only), and a compound
pick-and-place skill (needs the camera/perception piece from the team's wider
architecture, out of scope for this file alone).

## The tool pattern

Every tool has the same four steps. See `move_robot_to_position`:
1. Validate inputs. Raise `ValueError` with a plain reason (the LLM reads it).
2. Convert request units to robot units (degrees to radians).
3. Check feasibility against the joint limits.
4. Execute only after the checks pass, then report the resulting state.

Return JSON-serializable dicts with units in the key names. `example` is a
minimal template; copy it to start each new tool.

## Robot interface

Two interchangeable seams live behind the same `RobotState` shape and method
names (`connect`, `get_state`, `move_joint`, `move_linear`, `move_waypoints`,
`set_gripper`, `stop`); `server.py`'s tools call whichever one `UR_BACKEND`
picked and don't otherwise care which it is -- except `move_linear`, which
the ROS2 seam implements only as a `NotImplementedError` (see Tiers above),
and `stop`, which on the ROS2 seam only cancels a goal that `move_joint`/
`move_waypoints` is still tracking as active in the same process (the
socket seam's `stop` always works -- any upload preempts the controller).

- **`ur_client.py`** (`UR_BACKEND=socket`, default) -- plain TCP sockets, no
  ROS2:
  - Primary interface (port 30001): motion + gripper. Uploads small URScript
    programs (`movej`, `movel`, `set_digital_out`).
  - RTDE (port 30004): state. Reads joint angles, joint velocities, TCP pose,
    mode, safety status, digital output bits (gripper readback).
- **`../ros2_ur_driver/ros2_client.py`** (`UR_BACKEND=ros2`) -- through
  `ros_humble_ur_robot_driver`:
  - `/scaled_joint_trajectory_controller/follow_joint_trajectory` (action):
    motion.
  - `/io_and_status_controller/set_io` (service): gripper.
  - `/joint_states`, `/tcp_pose_broadcaster/pose`,
    `/io_and_status_controller/{robot_mode,safety_mode,io_states}` (topics): state.

Only `UR_HOST` (socket backend) changes to target a real robot instead of the
simulator; the ROS2 backend instead points at whatever `robot_ip` the driver
was launched with (see `../ros2_ur_driver/README.md`).
