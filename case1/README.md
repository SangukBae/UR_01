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
`python3`, arg the absolute path to `server.py` (7 tools on probe). Course
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
