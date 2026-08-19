# Case 1: MCP Server for Robot Tools

An MCP server that exposes UR robot capabilities as tools an LLM can call, so an
agent can operate the robot in natural language. It is the robot's "hands": any
MCP client (Claude Code, Cursor, or the Case 3 agent) launches this server and
calls its tools.

## The task

Turn robot capabilities into well-described, LLM-callable tools. One tool ships
fully worked, `move_robot_to_position`, and it moves the robot end to end. Your
job is to add more tools of your own (a state reader, a linear move, a gripper),
each following the same four-step shape, so a client can do more than move to a
joint pose.

**Status: all four tiers are implemented and verified** (`test_server.py` passes
end to end against the PolyScope X simulator, on both robot backends below).

## What's provided vs what you build

| File | Role |
|------|------|
| `server.py` | the MCP server: `move_robot_to_position` (Bronze), `get_robot_state` + `move_robot_linear` (Silver), `move_through_waypoints` (Gold), `move_robot_to_position_safe` + `set_gripper` (Diamond), `example` (template) |
| `ur_client.py` | socket seam over the robot (motion + state), `UR_BACKEND=socket` (default) |
| `kinematics.py` | nominal UR10 forward kinematics, used only for the Diamond workspace-bounds safety check |
| `test_server.py` | in-process smoke test for every tool above |
| `requirements.txt` | one dependency, the MCP framework |
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

**Don't mix them against the same live robot in one session.** The socket
backend uploads a raw URScript program directly to the controller, which
knocks out the "External Control" program the ROS2 driver depends on to keep
its reverse-socket connection alive -- the driver's next trajectory goal then
gets rejected with `Controller is not running`. Recover with:
```bash
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}"
```
(This simulator's External Control connection has also been observed to drop
on its own after a stretch of idle time, independent of that conflict --
same fix.)

**Sandbox note:** if `ros2 topic list` (or anything else `ros2`) hangs
instead of returning, the default RMW (FastRTPS) may be stuck on multicast
discovery in your environment. `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
(`sudo apt install ros-humble-rmw-cyclonedds-cpp` if not already installed)
fixed it here.

**Don't call `move_robot_linear` from a singularity.** A Cartesian move needs
near-infinite joint speed to track a straight line through a singular pose,
which trips a **protective stop** (`safety_status` 3) on the real
controller -- reproduced twice while building this tool: `HOME_DEG`
(`[0, -90, 0, -90, 0, 0]`, wrist2 = 0deg -- wrist1/wrist3 axes align), and
even a pose picked specifically to avoid that
(`[20, -90, 30, -90, 60, 0]`, a *different*, shoulder-adjacent singularity
that wasn't obvious from the joint values alone). `test_server.py` uses
`[30, -100, 40, -80, 70, 10]`, verified clean over repeated round-trip
`movel` calls. Moral: don't assume a pose is safe for a linear move just
because it looks unremarkable -- verify it the same way, a few round trips,
before relying on it. Recover a stopped robot from the PolyScope X UI
(there's no dashboard-server text protocol on this simulator to unlock it
from code); this simulator has also been observed to self-clear a
protective stop after a few seconds on its own.

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

**3. Connect a client.** The server speaks MCP over stdio: the client launches
`server.py` and talks to it over stdin/stdout, so you do not start it and connect
to a port. A sanity run (`python3 server.py`) only checks it imports and reaches
the robot; there is nothing to connect to there.

Set up a free LLM client with [`../llm-client`](../llm-client), then add this
server to it as an MCP named `ur-tools`. Two free paths:

- **Option A, self-hosted (Bionic, local).** Follow
  [`../llm-client/self-hosted.md`](../llm-client/self-hosted.md). In its "Add an
  MCP server" step, use Name `ur-tools`, Command the absolute path to your
  `python3`, and one Argument: the absolute path to `case 1/server.py`.
- **Option B, cloud-hosted (OpenClaw).** Follow
  [`../llm-client/cloud-hosted.md`](../llm-client/cloud-hosted.md), then register
  the server:
  ```bash
  openclaw mcp add ur-tools \
    --command /PATH/TO/python3 \
    --arg "/PATH/TO/case 1/server.py"
  openclaw mcp probe ur-tools     # expect 6 tools
  ```

With either, open the chat and ask in plain language: `move the robot home.` The
model reads the tool docstrings, calls `move_robot_to_position`, and the robot
moves.

### Using Claude Code (paid, optional)

If you already have Claude Code, register the server directly (absolute path,
quoted because of the space):
```bash
claude mcp add ur-tools -- python3 "/PATH/TO/essre2026-cases/case 1/server.py"
claude mcp list        # check it is connected
```
Then ask `Move the robot home.`; remove it with `claude mcp remove ur-tools`.
Other clients (Cursor, Claude Desktop) use the same idea: an MCP entry whose
command is `python3` and whose argument is the absolute path to `server.py`.

## Tiers

- **Bronze, run it -- done:** `move_robot_to_position`. Bring up the sim,
  connect a client, and move the robot home.
- **Silver, read state -- done:** `get_robot_state`. Returns joints (angle +
  speed), TCP pose, mode, and safety status, so an agent can observe before it
  acts. `move_robot_linear` adds a TCP-space straight-line move (URScript
  `movel`, `UR_BACKEND=socket` only -- the ROS2 backend raises
  `NotImplementedError`, since `scaled_joint_trajectory_controller` is
  joint-space only and needs IK to accept a Cartesian target).
- **Gold, richer motion -- done:** `move_through_waypoints`. Blends through a
  list of joint-space waypoints in one motion (no stop-and-restart at each
  one) and **streams state live** via MCP progress notifications
  (`notifications/progress`) as it moves -- a real push channel, not a
  trace-after-the-fact: a client watching progress sees each polled state as
  soon as it's captured, seconds before the tool call returns (verified in
  `test_server.py` by timestamping each notification against the call's own
  duration). The final result also includes the full `trace`, for a client
  that isn't watching progress. Runs the blocking move in a worker thread
  (`asyncio.to_thread`) so the event loop stays free to actually flush each
  notification as it happens, instead of queuing them all up behind the
  blocking socket/poll loop.

  Finding this also caught a real bug: the socket backend gets no
  completion signal from the controller for a `movej` program, so it polls
  and guesses "arrived" by proximity to the *last* waypoint. For a path
  that loops back near its own start (e.g. out-and-back), that check could
  pass on the very first poll -- before the robot had moved at all --
  while the uploaded URScript kept running for real underneath (reproduced
  while adding the streaming hook: a 3-waypoint round trip "completed" in
  0.14s client-side while the controller's own program log showed it ran
  ~1.3s). Fixed with a minimum-duration floor (`_estimate_path_duration`,
  a trapezoidal-profile estimate summed per segment, mirroring the
  ros2_client.py backend's own per-segment goal timing) that the tolerance
  check can't satisfy early.
- **Diamond, real skills -- done:** `move_robot_to_position_safe` adds a
  safety gate in front of a move -- joint limits, a speed cap, and a
  forward-kinematics workspace-bounds check (`kinematics.py`, nominal DH, a
  coarse estimate) -- and rejects an unsafe command before anything moves.
  `set_gripper` opens/closes via digital IO. `safety_status` is surfaced by
  `get_robot_state`.

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
`set_gripper`); `server.py`'s tools call whichever one `UR_BACKEND` picked and
don't otherwise care which it is -- except `move_linear`, which the ROS2 seam
implements only as a `NotImplementedError` (see Tiers above).

- **`ur_client.py`** (`UR_BACKEND=socket`, default) -- plain TCP sockets, no
  ROS2:
  - Primary interface (port 30001): motion + gripper. Uploads small URScript
    programs (`movej`, `movel`, `set_digital_out`).
  - RTDE (port 30004): state. Reads joint angles, joint velocities, TCP pose,
    mode, safety status.
- **`../ros2_ur_driver/ros2_client.py`** (`UR_BACKEND=ros2`) -- through
  `ros_humble_ur_robot_driver`:
  - `/scaled_joint_trajectory_controller/follow_joint_trajectory` (action):
    motion.
  - `/io_and_status_controller/set_io` (service): gripper.
  - `/joint_states`, `/tcp_pose_broadcaster/pose`,
    `/io_and_status_controller/{robot_mode,safety_mode}` (topics): state.

Only `UR_HOST` (socket backend) changes to target a real robot instead of the
simulator; the ROS2 backend instead points at whatever `robot_ip` the driver
was launched with (see `../ros2_ur_driver/README.md`).
