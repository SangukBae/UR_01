"""MCP server exposing UR robot capabilities as LLM-callable tools.

Case 1 baseline. Run this and connect any MCP client (Claude Code, Cursor, or
the Case 3 agent). Each ``@mcp.tool`` becomes a function the LLM can call by
name; the docstring is what the model reads to decide when and how to use it, so
write it for the model, not just for humans.

Every real tool here follows the same four-step shape (see
``move_robot_to_position``, the original worked example):
  1. Validate inputs first and raise ValueError with a plain-language reason.
     The client surfaces that text to the LLM, which can then self-correct.
  2. Convert the request units (degrees) into the robot's units (radians).
  3. Check feasibility against limits before anything moves.
  4. Execute only after the checks pass, and report back the new state.

Tools, by tier:
  * Bronze -- ``move_robot_to_position``: the one fully worked, robot-moving
    tool.
  * Silver -- ``get_robot_state``: read joints/TCP/mode/safety without moving.
    ``move_robot_linear``: a TCP-space straight-line move (URScript ``movel``,
    socket backend only -- the ROS2 backend needs IK, not yet implemented).
  * Gold -- ``move_through_waypoints``: a blended multi-point trajectory,
    streamed live via MCP progress notifications as it moves (plus a full
    trace in the final result, for a client that isn't watching progress).
  * Diamond -- ``move_robot_to_position_safe``: workspace/speed/forward-
    kinematics safety gate in front of a move; ``set_gripper``: open/close via
    digital IO.
  * ``example`` -- a do-nothing template showing the minimal tool shape. Copy
    it to start a new tool of your own.
  * ``stop_robot`` -- team requirement, not part of the four tiers:
    immediately halt any motion in progress. Socket backend always works
    (a fresh upload preempts); ROS2 backend only if a move is still
    in-flight in the same process (see ros2_client.py's ``stop``).
  * ``move_robot_queued`` / ``get_queue`` -- team architecture-meeting
    design, additive on top of the four tiers: a move that returns
    immediately (accept/reject + ETA) instead of blocking until arrival,
    with reject/queue/override semantics, backed by ``queue_manager.py``'s
    background worker thread. Makes ``stop_robot`` able to genuinely
    interrupt a move within the same chat turn (the plain tiered move
    tools above are unchanged and still block).
  * ``save_waypoint`` / ``list_waypoints`` / ``delete_waypoint`` /
    ``move_robot_to_waypoint`` / ``free_drive`` -- team architecture-
    meeting design: named poses persisted to disk (``waypoint_store.py``,
    a plain JSON file), and a free-drive tool (socket backend only) so a
    non-programmer can move the robot by hand and save the result instead
    of typing joint angles.
  * ``move_robot_to_position_relative`` / ``move_robot_linear_relative`` /
    ``move_robot_linear_sequence`` -- block-diagram tool list: relative
    (delta-from-current) variants of the absolute move tools, and a
    TCP-space multi-pose sequence that doesn't blend (each leg fully
    stops before the next), unlike move_through_waypoints.
  * ``program_new`` / ``list_programs`` / ``program_delete`` /
    ``program_start`` / ``program_stop`` -- block-diagram tool list: a
    named, ordered sequence of already-saved waypoints
    (``program_store.py``, another plain JSON file), run non-blocking via
    the same queue as ``move_robot_queued``.
  * ``get_vision`` / ``get_environment`` / ``get_environment_shadow`` --
    block-diagram tool list: MCP-level access to the ``vision_human_track``
    service (a separate HTTP service, see its README) for who/what is in
    view. ``get_environment_shadow`` reads an always-fresh background
    cache instead of paying a live round-trip.
  * ``track`` -- block-diagram tool list, deliberately crude: no camera<->
    robot calibration exists in this repo, so this only turns the base
    joint toward a hand's left/right position in frame -- a demo of the
    reactive loop, not real 3D tracking.

Return JSON-serializable dicts with explicit units in the key names.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import requests
from fastmcp import Context, FastMCP

import program_store
import waypoint_store
from kinematics import forward_kinematics
from queue_manager import CommandQueue
from ur_client import HOME_Q_RAD, JOINT_LIMIT, JOINT_NAMES, URClient

# vision_human_track's REST API (separate process/container -- see
# ../vision_human_track/README.md). Not required to be running for any
# tool except get_vision/get_environment/get_environment_shadow/track.
VISION_API_URL = os.environ.get("VISION_API_URL", "http://localhost:8000").rstrip("/")

mcp = FastMCP("ur-tools")

# UR_BACKEND=socket (default) talks to the robot directly over TCP sockets
# (ur_client.py). UR_BACKEND=ros2 routes through ros_humble_ur_robot_driver
# instead (ros2_ur_driver/ros2_client.py) -- same RobotState shape, same
# move_joint/get_state methods, so every tool below is unchanged either way.
# The ROS2 backend needs the driver already launched; see
# ../ros2_ur_driver/README.md.
_BACKEND = os.environ.get("UR_BACKEND", "socket")
if _BACKEND == "ros2":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ros2_ur_driver"))
    from ros2_client import ROS2URClient

    robot = ROS2URClient()
elif _BACKEND == "socket":
    robot = URClient()
else:
    raise ValueError(f"Unknown UR_BACKEND={_BACKEND!r}, expected 'socket' or 'ros2'.")

# Shadow mode (opt-in, see shadow_client.py): set UR_REAL_HOST to a real UR
# controller's IP to have every move verified on the simulator FIRST, then
# replayed on the real robot only if the simulator move succeeds. Unset
# (the default) -- nothing here changes, `robot` stays a plain client
# talking only to UR_HOST/the simulator, exactly as before this existed.
_REAL_HOST = os.environ.get("UR_REAL_HOST")
if _REAL_HOST:
    if _BACKEND != "socket":
        raise ValueError(
            "UR_REAL_HOST (shadow mode) is only supported with UR_BACKEND="
            "socket for now -- the ROS2 backend talks to one controller "
            "per process, not two."
        )
    from shadow_client import ShadowClient

    robot = ShadowClient(sim=robot, real=URClient(host=_REAL_HOST))

# Non-blocking command queue (team architecture meeting design) backing
# move_robot_queued/get_queue below. The original tiered tools above are
# left blocking and unchanged -- this is an additive layer, not a
# replacement, so the graded Bronze-Diamond tools keep working exactly as
# before.
_queue = CommandQueue()


def _state_summary(state) -> dict:
    return {
        "joints_deg": {n: round(math.degrees(q), 1) for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
    }


# Environment Shadow: a background thread keeps a continuously refreshed
# cache of the latest vision detection + robot state, so get_environment_
# shadow() answers instantly instead of paying a live round-trip every
# time -- the meeting's own concern that vision-on-demand alone leaves the
# LLM "blind" between calls (see meeting notes, section M). get_vision/
# get_environment below are the on-demand/live versions; this is the
# always-fresh cached one.
_shadow_lock = threading.Lock()
_shadow = {"vision": None, "vision_error": None, "robot_state": None, "updated_at": None}


def _shadow_loop() -> None:
    while True:
        update = {"updated_at": time.time()}
        try:
            resp = requests.post(f"{VISION_API_URL}/detect/live", timeout=2.0)
            resp.raise_for_status()
            update["vision"] = resp.json()
            update["vision_error"] = None
        except requests.RequestException as exc:
            update["vision"] = None
            update["vision_error"] = str(exc)
        try:
            update["robot_state"] = _state_summary(robot.get_state())
        except Exception:  # noqa: BLE001 - don't let a transient read error kill the loop
            update["robot_state"] = None
        with _shadow_lock:
            _shadow.update(update)
        time.sleep(1.0)


threading.Thread(target=_shadow_loop, daemon=True).start()


# Conservative joint-move defaults (rad/s, rad/s^2).
DEFAULT_SPEED = 1.0
DEFAULT_ACCEL = 1.4

# TCP-space (movel) defaults -- different units than the joint-move ones
# above (m/s, m/s^2), UR's own typical movel defaults.
DEFAULT_LINEAR_SPEED = 0.25
DEFAULT_LINEAR_ACCEL = 1.2

# Home pose as degrees, for readable tool output and defaults.
HOME_DEG = [round(math.degrees(a)) for a in HOME_Q_RAD]

# --- Diamond: safety-layer limits ----------------------------------------- #
# Conservative demo caps, not the robot's true mechanical limits (UR10 joints
# go well past this). The point is a gate that can actually reject something
# during a demo, not modelling the datasheet.
MAX_SAFE_SPEED_RAD_S = 2.0
# Flange-position box, base frame, metres -- generous margin around the FK
# estimate (kinematics.py is nominal/uncalibrated, see its docstring).
WORKSPACE_BOUNDS_M = {"x": (-1.3, 1.3), "y": (-1.3, 1.3), "z": (-0.3, 1.6)}


# =========================================================================== #
# WORKED TOOL  --  the one tool that actually moves the robot. Copy its
# four-step shape for every tool you add: 1) validate  2) convert units
# 3) check limits  4) execute + report.
# =========================================================================== #
@mcp.tool
def move_robot_to_position(
    joint_angles_deg: list[float] | None = None,
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
) -> dict:
    """Move the robot to an absolute joint configuration and report the result.

    Give six target joint angles in degrees, ordered base, shoulder, elbow,
    wrist1, wrist2, wrist3. Omit them to send the robot to its HOME position
    ([0, -90, 0, -90, 0, 0] degrees) -- "move the robot home" is a call with no
    arguments.

    The move blocks until the robot arrives, then returns the new robot state
    (so you can observe where it ended up before deciding the next step).

    Args:
        joint_angles_deg: Six target angles in degrees, base..wrist3. Defaults to
            the home pose when omitted.
        speed: Joint speed (rad/s).
        acceleration: Joint acceleration (rad/s^2).

    Returns:
        A dict with the target that was commanded and the resulting state:
        ``joints_deg`` (per-joint angles), ``tcp_pose`` [x, y, z, rx, ry, rz]
        in metres and radians, and ``robot_mode``.

    Raises:
        ValueError: Wrong number of angles, or a target past the joint limit.
        RuntimeError: The robot is not powered on (enable it in the UI first).
    """
    # 1. Validate inputs.
    if joint_angles_deg is None:
        joint_angles_deg = list(HOME_DEG)
    if len(joint_angles_deg) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint angles "
            f"({', '.join(JOINT_NAMES)}), got {len(joint_angles_deg)}."
        )

    # 2. Convert the request (degrees) to the robot's units (radians).
    target_rad = [math.radians(a) for a in joint_angles_deg]

    # 3. Check feasibility against the software joint limits.
    for name, angle_rad, angle_deg in zip(JOINT_NAMES, target_rad, joint_angles_deg):
        if abs(angle_rad) > JOINT_LIMIT:
            raise ValueError(
                f"Target {angle_deg} deg for {name} exceeds the +/-"
                f"{math.degrees(JOINT_LIMIT):.0f} deg limit. Choose a smaller angle."
            )

    # 4. Execute (blocks until reached), then report the new state.
    state = robot.move_joint(target_rad, speed, acceleration)
    return {
        "status": "reached",
        "target_deg": {n: round(a, 1) for n, a in zip(JOINT_NAMES, joint_angles_deg)},
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
    }


# =========================================================================== #
# SILVER  --  read state without moving, so an agent can observe before it acts.
# =========================================================================== #
@mcp.tool
def get_robot_state() -> dict:
    """Read the robot's current state without moving it.

    Returns:
        A dict with ``joints_deg`` (per-joint angles), ``joint_speeds_deg_s``
        (per-joint angular velocity), ``tcp_pose`` [x, y, z, rx, ry, rz] in
        metres and radians, ``robot_mode`` (7 = RUNNING), ``safety_status``
        (1 = NORMAL; anything else means reduced/stopped/faulted -- see UR's
        SafetyMode), and ``gripper_closed`` -- the actual digital-output-0
        state read back from the controller, not just what was last
        commanded (set_gripper is fire-and-forget; this is the honest check).
    """
    state = robot.get_state()
    return {
        "joints_deg": {n: round(math.degrees(q), 1) for n, q in zip(JOINT_NAMES, state.q_rad)},
        "joint_speeds_deg_s": {n: round(math.degrees(qd), 2)
                                for n, qd in zip(JOINT_NAMES, state.qd_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
        "safety_status": state.safety_status,
        "gripper_closed": state.gripper_closed,
    }


# =========================================================================== #
# SILVER  --  a TCP-space linear move, so an agent can move in a straight
# line (e.g. an approach/retreat) instead of the curved path a joint move
# takes.
# =========================================================================== #
@mcp.tool
def move_robot_linear(
    tcp_pose: list[float],
    speed: float = DEFAULT_LINEAR_SPEED,
    acceleration: float = DEFAULT_LINEAR_ACCEL,
) -> dict:
    """Move the TCP in a straight line to an absolute pose and report the result.

    Unlike move_robot_to_position (a joint-space move, curved TCP path), this
    interpolates the TCP itself in a straight line -- use it when the path
    matters (e.g. approaching a part without swinging sideways into it).

    Args:
        tcp_pose: Six numbers ``[x, y, z, rx, ry, rz]`` -- position in metres,
            orientation as a UR-style rotation vector in radians (axis *
            angle), base frame. Read the current pose from
            get_robot_state()'s ``tcp_pose`` to build a target relative to it.
        speed: TCP speed (m/s).
        acceleration: TCP acceleration (m/s^2).

    Returns:
        A dict with the target pose that was commanded and the resulting
        state: ``joints_deg``, ``tcp_pose``, ``robot_mode``.

    Raises:
        ValueError: ``tcp_pose`` isn't exactly 6 numbers.
        RuntimeError: The robot is not powered on.
        NotImplementedError: UR_BACKEND=ros2 -- this backend needs inverse
            kinematics for a Cartesian move, not yet implemented; use
            UR_BACKEND=socket for this tool.
    """
    # 1. Validate inputs.
    if len(tcp_pose) != 6:
        raise ValueError(f"Expected 6 values [x, y, z, rx, ry, rz], got {len(tcp_pose)}.")

    # 2-3. No unit conversion or joint-limit check for a Cartesian target --
    # the controller (movel) rejects an unreachable pose itself.
    # 4. Execute (blocks until reached), then report the new state.
    state = robot.move_linear(list(tcp_pose), speed, acceleration)
    return {
        "status": "reached",
        "target_tcp_pose": [round(v, 4) for v in tcp_pose],
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
    }


# =========================================================================== #
# RELATIVE MOVES  --  block-diagram tool list, additive: the same joint/TCP
# moves above, but "move this much from here" instead of "move to this
# absolute target" -- reads the current pose first, adds the delta, then
# reuses the exact same validate/execute path as the absolute version.
# =========================================================================== #
@mcp.tool
def move_robot_to_position_relative(
    delta_deg: list[float],
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
) -> dict:
    """Move each joint by a relative amount from wherever it is now.

    Args:
        delta_deg: Six angle changes in degrees, base..wrist3 (e.g.
            ``[0, 0, 0, 0, 0, 90]`` turns only wrist3 by +90 deg from its
            current angle). Use move_robot_to_position for an absolute target.
        speed: Joint speed (rad/s).
        acceleration: Joint acceleration (rad/s^2).

    Returns:
        Same shape as move_robot_to_position: ``status``, ``target_deg``
        (the resulting absolute target this resolved to), ``joints_deg``,
        ``tcp_pose``, ``robot_mode``.

    Raises:
        ValueError: Wrong number of deltas, or the resulting absolute
            target exceeds a joint limit.
        RuntimeError: The robot is not powered on.
    """
    # 1. Validate inputs.
    if len(delta_deg) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} deltas ({', '.join(JOINT_NAMES)}), "
            f"got {len(delta_deg)}."
        )

    # 2. Resolve to an absolute target (current + delta), then convert to radians.
    current_deg = [math.degrees(q) for q in robot.get_state().q_rad]
    target_deg = [c + d for c, d in zip(current_deg, delta_deg)]
    target_rad = [math.radians(a) for a in target_deg]

    # 3. Check feasibility against the software joint limits.
    for name, angle_rad, angle_deg in zip(JOINT_NAMES, target_rad, target_deg):
        if abs(angle_rad) > JOINT_LIMIT:
            raise ValueError(
                f"Delta for {name} resolves to {angle_deg:.1f} deg, exceeding "
                f"the +/-{math.degrees(JOINT_LIMIT):.0f} deg limit."
            )

    # 4. Execute (blocks until reached), then report the new state.
    state = robot.move_joint(target_rad, speed, acceleration)
    return {
        "status": "reached",
        "target_deg": {n: round(a, 1) for n, a in zip(JOINT_NAMES, target_deg)},
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
    }


@mcp.tool
def move_robot_linear_relative(
    delta: list[float],
    speed: float = DEFAULT_LINEAR_SPEED,
    acceleration: float = DEFAULT_LINEAR_ACCEL,
) -> dict:
    """Move the TCP in a straight line by a relative offset from its current pose.

    Args:
        delta: Six numbers ``[dx, dy, dz, drx, dry, drz]`` added to the
            current TCP pose -- metres for position, radians for the
            rotation-vector components. E.g. ``[0, 0, 0.05, 0, 0, 0]`` lifts
            5cm straight up. Note: adding rotation-vector components is only
            a good approximation for small rotations, not true rotation
            composition -- keep drx/dry/drz small.
        speed: TCP speed (m/s).
        acceleration: TCP acceleration (m/s^2).

    Returns:
        Same shape as move_robot_linear: ``status``, ``target_tcp_pose``
        (the resulting absolute pose this resolved to), ``joints_deg``,
        ``tcp_pose``, ``robot_mode``.

    Raises:
        ValueError: ``delta`` isn't exactly 6 numbers.
        RuntimeError: The robot is not powered on.
        NotImplementedError: UR_BACKEND=ros2 (see move_robot_linear).
    """
    # 1. Validate inputs.
    if len(delta) != 6:
        raise ValueError(f"Expected 6 values [dx,dy,dz,drx,dry,drz], got {len(delta)}.")

    # 2. Resolve to an absolute target pose (current + delta).
    current_pose = robot.get_state().tcp_pose
    target_pose = [c + d for c, d in zip(current_pose, delta)]

    # 3. No limit check -- the controller rejects an unreachable pose itself.
    # 4. Execute (blocks until reached), then report the new state.
    state = robot.move_linear(target_pose, speed, acceleration)
    return {
        "status": "reached",
        "target_tcp_pose": [round(v, 4) for v in target_pose],
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
    }


@mcp.tool
def move_robot_linear_sequence(
    tcp_poses: list[list[float]],
    speed: float = DEFAULT_LINEAR_SPEED,
    acceleration: float = DEFAULT_LINEAR_ACCEL,
) -> dict:
    """Move the TCP in a straight line through several absolute poses in order.

    Unlike move_through_waypoints (joint-space, blended into one smooth
    motion), this is TCP-space and does NOT blend -- the robot fully stops
    at each pose before starting the next straight line to the following
    one. Use it when every leg of the path needs to be a genuine straight
    line (move_through_waypoints' blending curves slightly through each
    intermediate point).

    Args:
        tcp_poses: A list of absolute ``[x, y, z, rx, ry, rz]`` poses, at
            least one required.
        speed: TCP speed (m/s), shared by every leg.
        acceleration: TCP acceleration (m/s^2), shared by every leg.

    Returns:
        A dict with the final ``joints_deg``/``tcp_pose``/``robot_mode``,
        plus ``trace``: a ``tcp_pose`` snapshot after each leg.

    Raises:
        ValueError: No poses given, or one isn't exactly 6 numbers.
        RuntimeError: The robot is not powered on.
        NotImplementedError: UR_BACKEND=ros2 (see move_robot_linear).
    """
    # 1. Validate inputs.
    if not tcp_poses:
        raise ValueError("Need at least one pose.")
    for i, pose in enumerate(tcp_poses):
        if len(pose) != 6:
            raise ValueError(f"Pose {i} has {len(pose)} values, expected 6.")

    # 2-3. No unit conversion or limit check -- the controller rejects an
    # unreachable pose itself, same as the single-pose version.
    # 4. Execute each leg in order (blocks until each is reached), then
    # report the final state plus a trace of every leg's result.
    trace = []
    state = None
    for pose in tcp_poses:
        state = robot.move_linear(list(pose), speed, acceleration)
        trace.append({"tcp_pose": [round(v, 4) for v in state.tcp_pose]})
    return {
        "status": "reached",
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
        "trace": trace,
    }


# =========================================================================== #
# GOLD  --  a trajectory tool with waypoint blending, streamed live via MCP
# progress notifications (a genuine push channel, not just a trace-after-the-
# fact) so a client watching progress sees the move as it happens.
# =========================================================================== #


@mcp.tool
async def move_through_waypoints(
    waypoints_deg: list[list[float]],
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
    ctx: Context | None = None,
) -> dict:
    """Move through several joint configurations as one smooth, blended motion.

    Unlike calling move_robot_to_position repeatedly, the robot doesn't stop at
    each waypoint -- it blends through them, which is faster and easier on the
    joints for a multi-point path (e.g. tracing a shape, or a pick-approach-
    place sequence).

    If your client shows MCP progress notifications, you'll see the robot's
    state pushed live as it moves (an actual push channel, via
    ``notifications/progress`` -- not just a summary after the fact).

    Args:
        waypoints_deg: A list of joint-angle sets, each six degrees in
            base..wrist3 order. At least one waypoint is required; the last
            one is where the robot ends up exactly (no blend radius there).
        speed: Joint speed (rad/s), shared by every segment.
        acceleration: Joint acceleration (rad/s^2), shared by every segment.

    Returns:
        A dict with the final ``joints_deg``/``tcp_pose``/``robot_mode``, plus
        ``trace``: a list of ``{joints_deg, tcp_pose}`` snapshots sampled while
        the robot was moving -- the same data the progress notifications
        carried, for a client that wasn't watching them live.

    Raises:
        ValueError: No waypoints, wrong angle count, or a target past a limit.
        RuntimeError: The robot is not powered on.
    """
    # 1. Validate inputs.
    if not waypoints_deg:
        raise ValueError("Need at least one waypoint.")
    for i, wp in enumerate(waypoints_deg):
        if len(wp) != len(JOINT_NAMES):
            raise ValueError(
                f"Waypoint {i} has {len(wp)} angles, expected {len(JOINT_NAMES)} "
                f"({', '.join(JOINT_NAMES)})."
            )

    # 2. Convert degrees -> radians.
    waypoints_rad = [[math.radians(a) for a in wp] for wp in waypoints_deg]

    # 3. Check feasibility against the software joint limits, every waypoint.
    for i, (wp_deg, wp_rad) in enumerate(zip(waypoints_deg, waypoints_rad)):
        for name, angle_rad, angle_deg in zip(JOINT_NAMES, wp_rad, wp_deg):
            if abs(angle_rad) > JOINT_LIMIT:
                raise ValueError(
                    f"Waypoint {i}: {angle_deg} deg for {name} exceeds the "
                    f"+/-{math.degrees(JOINT_LIMIT):.0f} deg limit."
                )

    # 4. Execute, streaming a progress notification per polled state, then
    # report the new state + trace. robot.move_waypoints blocks (plain
    # sockets/time.sleep, no asyncio) -- run it in a worker thread so the
    # event loop stays free to actually flush each notification as on_state
    # fires, instead of queuing them all up behind the blocking call.
    loop = asyncio.get_running_loop()
    step = 0

    def on_state(state) -> None:
        nonlocal step
        step += 1
        if ctx is not None:
            asyncio.run_coroutine_threadsafe(
                ctx.report_progress(progress=step, message=json.dumps(_state_summary(state))),
                loop,
            )

    final_state, trace = await asyncio.to_thread(
        robot.move_waypoints, waypoints_rad, speed, acceleration, on_state=on_state,
    )
    return {
        "status": "reached",
        **_state_summary(final_state),
        "robot_mode": final_state.robot_mode,
        "trace": [_state_summary(s) for s in trace],
    }


# =========================================================================== #
# QUEUE  --  team architecture-meeting design, additive to the four tiers:
# a move that returns immediately (accept/reject + ETA) instead of blocking
# until the robot arrives, backed by CommandQueue's background worker
# thread. This is what makes stop_robot() able to genuinely interrupt a
# move within the same chat turn -- see stop_robot's docstring.
# =========================================================================== #
@mcp.tool
def move_robot_queued(
    joint_angles_deg: list[float] | None = None,
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
    mode: str = "reject",
) -> dict:
    """Request a joint move without blocking until it arrives.

    Unlike move_robot_to_position, this returns immediately with an
    accept/reject decision and an estimated duration; the actual motion
    runs in the background. Poll get_robot_state() or get_queue() to see
    when it finishes, and call stop_robot() any time to interrupt it --
    that only works because this tool doesn't block the way the plain
    move tools do.

    Args:
        joint_angles_deg: Six target angles in degrees, base..wrist3.
            Defaults to the home pose when omitted.
        speed: Joint speed (rad/s).
        acceleration: Joint acceleration (rad/s^2).
        mode: What to do if a command is already running or queued.
            "reject" (default): refuse this request, nothing changes.
            "queue": run this one after the current queue finishes.
            "override": cancel everything running/queued and run this now.

    Returns:
        A dict with ``status`` ("accepted"/"queued"/"accepted_override"/
        "rejected"), ``command_id`` (absent on "rejected"),
        ``estimated_duration_s``, and ``queue`` -- the full current/pending
        queue state, same shape get_queue() returns.

    Raises:
        ValueError: Wrong number of angles, a target past the joint limit,
            or ``mode`` isn't one of "reject"/"queue"/"override".
    """
    # 1. Validate inputs.
    if joint_angles_deg is None:
        joint_angles_deg = list(HOME_DEG)
    if len(joint_angles_deg) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint angles "
            f"({', '.join(JOINT_NAMES)}), got {len(joint_angles_deg)}."
        )

    # 2. Convert the request (degrees) to the robot's units (radians).
    target_rad = [math.radians(a) for a in joint_angles_deg]

    # 3. Check feasibility against the software joint limits.
    for name, angle_rad, angle_deg in zip(JOINT_NAMES, target_rad, joint_angles_deg):
        if abs(angle_rad) > JOINT_LIMIT:
            raise ValueError(
                f"Target {angle_deg} deg for {name} exceeds the +/-"
                f"{math.degrees(JOINT_LIMIT):.0f} deg limit. Choose a smaller angle."
            )

    # 4. Submit to the queue (returns immediately) instead of executing here.
    current_state = robot.get_state()
    eta_s = URClient._estimate_path_duration(
        current_state.q_rad, [target_rad], speed, acceleration
    )

    def _do_move() -> dict:
        return _state_summary(robot.move_joint(target_rad, speed, acceleration))

    result = _queue.submit(
        kind="move_joint",
        fn=_do_move,
        meta={
            "target_deg": {n: round(a, 1) for n, a in zip(JOINT_NAMES, joint_angles_deg)},
            "estimated_duration_s": round(eta_s, 2),
        },
        mode=mode,
    )
    result["estimated_duration_s"] = round(eta_s, 2)
    return result


@mcp.tool
def get_queue() -> dict:
    """Check what the non-blocking command queue is doing right now.

    Returns:
        A dict with ``current`` (the in-progress command, or null if idle
        -- ``id``, ``kind``, ``meta``, ``status``), ``pending`` (queued
        commands waiting their turn, same shape), and ``queue_length``.
        Poll this after move_robot_queued instead of blocking on it.
    """
    return _queue.describe()


# =========================================================================== #
# DIAMOND  --  a safety layer (workspace + speed + forward-kinematics checks
# that reject an unsafe command before anything moves) and a gripper tool.
# =========================================================================== #
@mcp.tool
def move_robot_to_position_safe(
    joint_angles_deg: list[float] | None = None,
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
) -> dict:
    """Like move_robot_to_position, but gated by a safety layer that rejects
    the command instead of moving if it looks unsafe.

    Three checks run before anything moves:
      1. Joint limits (same as move_robot_to_position).
      2. Speed limit: rejects speed past MAX_SAFE_SPEED_RAD_S.
      3. Workspace bounds: forward-kinematics the target (nominal DH, a coarse
         estimate -- see kinematics.py) and reject it if the flange would land
         outside WORKSPACE_BOUNDS_M, so a wildly wrong joint target doesn't
         swing the arm somewhere unreasonable before you find out from the
         real robot.

    Args, Returns, Raises: same as move_robot_to_position, except ValueError
    also covers the speed and workspace checks below.
    """
    # 1. Validate inputs.
    if joint_angles_deg is None:
        joint_angles_deg = list(HOME_DEG)
    if len(joint_angles_deg) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint angles "
            f"({', '.join(JOINT_NAMES)}), got {len(joint_angles_deg)}."
        )

    # 2. Convert the request (degrees) to the robot's units (radians).
    target_rad = [math.radians(a) for a in joint_angles_deg]

    # 3. Check feasibility: joint limits, speed limit, workspace bounds.
    for name, angle_rad, angle_deg in zip(JOINT_NAMES, target_rad, joint_angles_deg):
        if abs(angle_rad) > JOINT_LIMIT:
            raise ValueError(
                f"Target {angle_deg} deg for {name} exceeds the +/-"
                f"{math.degrees(JOINT_LIMIT):.0f} deg limit. Choose a smaller angle."
            )
    if speed > MAX_SAFE_SPEED_RAD_S:
        raise ValueError(
            f"Requested speed {speed:.2f} rad/s exceeds the safety cap of "
            f"{MAX_SAFE_SPEED_RAD_S:.2f} rad/s. Choose a slower speed."
        )
    x, y, z = forward_kinematics(target_rad)
    for axis, value in (("x", x), ("y", y), ("z", z)):
        low, high = WORKSPACE_BOUNDS_M[axis]
        if not (low <= value <= high):
            raise ValueError(
                f"Target puts the flange at {axis}={value:.3f} m, outside the "
                f"safe workspace bound [{low}, {high}] m for {axis}. Rejected "
                "before moving."
            )

    # 4. Execute (blocks until reached), then report the new state.
    state = robot.move_joint(target_rad, speed, acceleration)
    return {
        "status": "reached",
        "target_deg": {n: round(a, 1) for n, a in zip(JOINT_NAMES, joint_angles_deg)},
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
        "safety_status": state.safety_status,
    }


@mcp.tool
def set_gripper(state: str) -> dict:
    """Open or close the gripper (tool digital output 0).

    Args:
        state: ``"OPEN"`` or ``"CLOSE"`` (case-insensitive).

    Returns:
        A dict with ``gripper`` (``"CLOSE"``/``"OPEN"``, read back from the
        controller's actual digital-output-0 state, not just echoing what
        was commanded) and the robot's current
        ``joints_deg``/``tcp_pose``/``robot_mode``.

    Raises:
        ValueError: ``state`` is neither "OPEN" nor "CLOSE".
        RuntimeError: The robot is not powered on.
    """
    # 1. Validate inputs.
    normalized = state.strip().upper()
    if normalized not in ("OPEN", "CLOSE"):
        raise ValueError(f'state must be "OPEN" or "CLOSE", got {state!r}.')

    # 2-3. No unit conversion or limit check needed for a digital signal.
    # 4. Execute, then report the new state.
    result = robot.set_gripper(closed=(normalized == "CLOSE"))
    return {
        "gripper": "CLOSE" if result.gripper_closed else "OPEN",
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, result.q_rad)},
        "tcp_pose": [round(v, 4) for v in result.tcp_pose],
        "robot_mode": result.robot_mode,
    }


# =========================================================================== #
# WAYPOINT DB + FREE-DRIVE  --  team architecture-meeting design, additive
# on top of the four tiers: named poses persisted to disk (waypoint_store.py,
# a plain JSON file -- "the database" from the meeting), and a free-drive
# tool so a non-programmer can move the robot by hand and save the result,
# rather than typing joint angles.
# =========================================================================== #
@mcp.tool
def save_waypoint(name: str) -> dict:
    """Save the robot's current pose under a name, for later recall with
    move_robot_to_waypoint.

    Args:
        name: A label to save this pose under. Saving again under an
            existing name overwrites it.

    Returns:
        A dict with ``name``, ``joints_deg``, and ``tcp_pose`` for what was
        just saved.
    """
    # 1-3. No inputs to validate beyond name being a string, no units to
    # convert -- this reads whatever pose the robot is already in.
    # 4. Execute (a state read, not a move), then report what was saved.
    state = robot.get_state()
    waypoint_store.save_waypoint(name, list(state.q_rad), list(state.tcp_pose))
    return {
        "name": name,
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
    }


@mcp.tool
def list_waypoints() -> dict:
    """List every saved waypoint.

    Returns:
        A dict with ``waypoints``, mapping each saved name to its
        ``joints_deg``, ``tcp_pose``, and ``saved_at`` (unix timestamp) --
        an empty dict if none are saved yet.
    """
    # Nested under "waypoints" rather than returned as the top-level dict:
    # an empty top-level dict ({}) was observed to deserialize as None on
    # the client side (verified live, see CLAUDE.md's verified findings) --
    # wrapping it guarantees a non-empty structure regardless.
    saved = waypoint_store.list_waypoints()
    return {
        "waypoints": {
            name: {
                "joints_deg": {n: round(math.degrees(q), 1)
                               for n, q in zip(JOINT_NAMES, entry["q_rad"])},
                "tcp_pose": [round(v, 4) for v in entry["tcp_pose"]],
                "saved_at": entry["saved_at"],
            }
            for name, entry in saved.items()
        }
    }


@mcp.tool
def delete_waypoint(name: str) -> dict:
    """Delete a saved waypoint.

    Args:
        name: The waypoint to remove.

    Returns:
        A dict with ``status: "deleted"`` and the ``name`` removed.

    Raises:
        ValueError: No waypoint is saved under that name.
    """
    # 1. Validate: the name must actually exist.
    try:
        waypoint_store.delete_waypoint(name)
    except KeyError as exc:
        raise ValueError(str(exc)) from exc
    # 2-4. No units, no limits, no motion -- just remove the record.
    return {"status": "deleted", "name": name}


@mcp.tool
def move_robot_to_waypoint(
    name: str,
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
) -> dict:
    """Move to a previously saved waypoint by name.

    Args:
        name: A waypoint saved earlier with save_waypoint.
        speed: Joint speed (rad/s).
        acceleration: Joint acceleration (rad/s^2).

    Returns:
        Same shape as move_robot_to_position: ``status``, ``joints_deg``,
        ``tcp_pose``, ``robot_mode``.

    Raises:
        ValueError: No waypoint is saved under that name.
        RuntimeError: The robot is not powered on.
    """
    # 1. Validate: the name must exist.
    try:
        entry = waypoint_store.get_waypoint(name)
    except KeyError as exc:
        raise ValueError(str(exc)) from exc

    # 2. Already stored in the robot's units (radians) -- no conversion.
    target_rad = entry["q_rad"]

    # 3. Feasibility already passed when the pose was saved (it was a real
    # robot state) -- no re-check needed.
    # 4. Execute (blocks until reached), then report the new state.
    state = robot.move_joint(target_rad, speed, acceleration)
    return {
        "status": "reached",
        "waypoint": name,
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
    }


@mcp.tool
def free_drive(duration_s: float = 10.0) -> dict:
    """Let the robot be moved freely by hand for a set duration (gravity-
    compensated, no target -- a human physically pushes it), then resume
    normal control and report where it ended up.

    Meant for a non-programmer workflow: call this, physically move the
    arm to where you want it while it's still running, then call
    save_waypoint once it returns to capture that pose.

    Args:
        duration_s: How long to stay in free-drive, in seconds. This call
            blocks for the full duration -- there's no target to poll
            toward, unlike the move_* tools.

    Returns:
        A dict with ``joints_deg`` and ``tcp_pose`` for the pose the robot
        was left in when free-drive ended.

    Raises:
        ValueError: duration_s isn't positive.
        RuntimeError: The robot is not powered on, or UR_BACKEND=ros2
            (socket backend only -- see ur_client.py's free_drive).
    """
    # 1. Validate inputs.
    if duration_s <= 0:
        raise ValueError(f"duration_s must be positive, got {duration_s}.")

    # 2-3. No unit conversion or limit check -- free-drive has no target.
    # 4. Execute (blocks for duration_s), then report the resulting state.
    state = robot.free_drive(duration_s)
    return {
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
    }


# =========================================================================== #
# PROGRAMS  --  block-diagram tool list: a named, ordered sequence of
# already-saved waypoints (program_store.py, another plain JSON file) --
# "run these spots in this order" as a single callable unit, distinct from
# recalling one waypoint at a time.
# =========================================================================== #
@mcp.tool
def program_new(name: str, waypoint_names: list[str]) -> dict:
    """Save a named program: an ordered list of already-saved waypoints to
    run in sequence.

    Args:
        name: A label for this program. Saving again under an existing
            name overwrites it.
        waypoint_names: Waypoint names (from save_waypoint), in run order.
            At least one required; every name must already exist.

    Returns:
        A dict with ``name`` and ``waypoint_names`` for what was just saved.

    Raises:
        ValueError: The list is empty, or one of the names isn't a saved waypoint.
    """
    # 1. Validate inputs: non-empty, and every referenced waypoint exists.
    if not waypoint_names:
        raise ValueError("Need at least one waypoint name.")
    for wp_name in waypoint_names:
        try:
            waypoint_store.get_waypoint(wp_name)
        except KeyError as exc:
            raise ValueError(str(exc)) from exc

    # 2-3. No units, no motion -- just record the sequence.
    # 4. Execute (a store write, not a move), then report what was saved.
    program_store.save_program(name, waypoint_names)
    return {"name": name, "waypoint_names": list(waypoint_names)}


@mcp.tool
def list_programs() -> dict:
    """List every saved program.

    Returns:
        A dict with ``programs``, mapping each name to its
        ``waypoint_names`` and ``saved_at`` (unix timestamp).
    """
    return {"programs": program_store.list_programs()}


@mcp.tool
def program_delete(name: str) -> dict:
    """Delete a saved program (the waypoints it references are untouched).

    Args:
        name: The program to remove.

    Returns:
        A dict with ``status: "deleted"`` and the ``name`` removed.

    Raises:
        ValueError: No program is saved under that name.
    """
    try:
        program_store.delete_program(name)
    except KeyError as exc:
        raise ValueError(str(exc)) from exc
    return {"status": "deleted", "name": name}


@mcp.tool
def program_start(
    name: str,
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
    mode: str = "reject",
) -> dict:
    """Run a saved program: move through its waypoints in order.

    Non-blocking, same as move_robot_queued -- returns immediately with an
    accept/reject decision, runs in the background, and can be interrupted
    with stop_robot (or program_stop, identical) partway through.

    Args:
        name: A program saved earlier with program_new.
        speed: Joint speed (rad/s), shared by every leg.
        acceleration: Joint acceleration (rad/s^2), shared by every leg.
        mode: Same as move_robot_queued -- "reject" (default), "queue", or
            "override" if a command is already running/queued.

    Returns:
        Same shape as move_robot_queued: ``status``, ``command_id``,
        ``estimated_duration_s``, ``queue``.

    Raises:
        ValueError: No program is saved under that name, or ``mode`` isn't
            "reject"/"queue"/"override".
    """
    # 1. Validate: the program must exist, and every waypoint it references
    # must still exist (it might have been deleted since the program was saved).
    try:
        entry = program_store.get_program(name)
    except KeyError as exc:
        raise ValueError(str(exc)) from exc
    try:
        legs = [waypoint_store.get_waypoint(wp)["q_rad"] for wp in entry["waypoint_names"]]
    except KeyError as exc:
        raise ValueError(str(exc)) from exc

    # 2. Already stored in radians -- no conversion.
    # 3. Feasibility already passed when each waypoint was saved.
    current_q = robot.get_state().q_rad
    eta_s = sum(
        URClient._estimate_path_duration(prev, [leg], speed, acceleration)
        for prev, leg in zip([current_q, *legs[:-1]], legs)
    )

    def _run_program() -> dict:
        state = None
        for leg in legs:
            state = robot.move_joint(leg, speed, acceleration)
        return _state_summary(state)

    # 4. Submit to the queue (returns immediately) instead of executing here.
    result = _queue.submit(
        kind="program",
        fn=_run_program,
        meta={"program": name, "waypoint_names": entry["waypoint_names"],
              "estimated_duration_s": round(eta_s, 2)},
        mode=mode,
    )
    result["estimated_duration_s"] = round(eta_s, 2)
    return result


@mcp.tool
def program_stop() -> dict:
    """Immediately halt a running program (and any other queued/running
    command). Identical to stop_robot -- provided under this name too since
    that's what the team's tool list calls it.

    Returns:
        Same shape as stop_robot.
    """
    return stop_robot()


# =========================================================================== #
# VISION PASSTHROUGH  --  block-diagram tool list: MCP-level access to the
# vision_human_track service (../vision_human_track/), so an LLM can ask
# "what/who is in view" without the caller needing to know that's a
# separate HTTP service. get_vision/get_environment are on-demand (pay a
# live round-trip); get_environment_shadow reads the always-fresh
# background cache above instead.
# =========================================================================== #
def _call_vision_api() -> dict:
    try:
        resp = requests.post(f"{VISION_API_URL}/detect/live", timeout=5.0)
        resp.raise_for_status()
    except requests.RequestException as exc:
        raise RuntimeError(
            f"Vision service unreachable at {VISION_API_URL} ({exc}). Is "
            "vision_human_track running? See its README (docker compose up "
            "-d, or the local venv)."
        ) from exc
    return resp.json()


@mcp.tool
def get_vision() -> dict:
    """Take a snapshot from the vision service's camera and return the raw
    detection: every detected person's skeleton and hands, full detail.

    For most purposes get_environment (a summarized list) or
    get_environment_shadow (cached, no round-trip) is more useful -- use
    this when the full per-joint skeleton/hand landmark detail is actually
    needed.

    Returns:
        The vision service's raw JSON response (see
        ../vision_human_track/README.md's API contract).

    Raises:
        RuntimeError: The vision service isn't reachable.
    """
    return _call_vision_api()


@mcp.tool
def get_environment() -> dict:
    """Take a snapshot of who/what is currently in view, summarized.

    Non-destructive check before acting -- e.g. confirm a person is
    actually visible before doing something that assumes they are.

    Note: only reports people/hands (this repo's Vision Stack scope).
    Object detection (apple/cup/etc.) is a separate, not-yet-integrated
    service -- see ../vision_human_track/README.md.

    Returns:
        A dict with ``humans``: a list of ``{id, num_hands, hands: [{
        handedness, palm_center, normal}]}`` -- position/orientation only,
        not the full per-joint skeleton (see get_vision for that).

    Raises:
        RuntimeError: The vision service isn't reachable.
    """
    data = _call_vision_api()
    humans = []
    for h in data.get("humans", []):
        humans.append({
            "id": h["id"],
            "num_hands": len(h["hands"]),
            "hands": [
                {"handedness": hand["handedness"], "palm_center": hand["palm_center"],
                 "normal": hand["normal"]}
                for hand in h["hands"]
            ],
        })
    return {"humans": humans}


@mcp.tool
def get_environment_shadow() -> dict:
    """Read the last cached vision snapshot + robot state, refreshed about
    once a second in the background -- no live round-trip, so this is
    effectively free to call often (e.g. to check "has anything changed"
    without the latency of get_vision/get_environment).

    Returns:
        A dict with ``vision`` (same shape as get_vision's result, or null
        if the vision service was unreachable at the last refresh --
        check ``vision_error`` for why), ``robot_state`` (``joints_deg``/
        ``tcp_pose``), and ``age_s`` (seconds since this snapshot was
        taken -- large means the background refresh has stalled).
    """
    with _shadow_lock:
        snapshot = dict(_shadow)
    age_s = None if snapshot["updated_at"] is None else round(time.time() - snapshot["updated_at"], 2)
    return {
        "vision": snapshot["vision"],
        "vision_error": snapshot["vision_error"],
        "robot_state": snapshot["robot_state"],
        "age_s": age_s,
    }


# =========================================================================== #
# TRACKING  --  block-diagram tool list. Deliberately crude: there is no
# camera<->robot extrinsic calibration anywhere in this repo (see
# ../vision_human_track/README.md and ../ros2_vision_bridge/README.md), so
# there's no way to compute a real 3D robot target from a camera-frame
# hand position. This maps a hand's horizontal position in frame to a
# base-joint angle (1 degree of freedom, roughly "turn toward the hand"),
# not real 3D tracking -- a demo of the loop, not a finished feature.
# =========================================================================== #
@mcp.tool
def track(duration_s: float = 10.0, poll_hz: float = 2.0) -> dict:
    """Roughly turn the robot's base to follow a hand's left/right position
    in the camera view, for a set duration.

    Crude by design (see module notes): only the base joint moves, driven
    by the hand's horizontal position in the image -- not a real 3D
    tracking of the hand's actual position, because there's no camera<->
    robot coordinate calibration in this repo yet. A demo of "vision ->
    MCP -> robot reacts continuously", not a finished tracking feature.

    Args:
        duration_s: How long to track for, in seconds. Blocks for the
            full duration.
        poll_hz: How often to sample the vision service and re-aim, in Hz.

    Returns:
        A dict with ``updates`` (how many times a hand was seen and the
        base was re-aimed) and the final ``joints_deg``.

    Raises:
        ValueError: duration_s or poll_hz isn't positive.
        RuntimeError: The robot is not powered on. (Vision-unreachable
            polls are skipped, not raised -- tracking just pauses until
            the vision service comes back or duration_s runs out.)
    """
    # 1. Validate inputs.
    if duration_s <= 0:
        raise ValueError(f"duration_s must be positive, got {duration_s}.")
    if poll_hz <= 0:
        raise ValueError(f"poll_hz must be positive, got {poll_hz}.")

    # 2-3. No unit conversion or limit check up front -- each re-aim below
    # reuses move_robot_to_position_relative's own clamp-free small delta.
    # 4. Poll-and-react loop for duration_s, then report the final state.
    updates = 0
    deadline = time.monotonic() + duration_s
    base_target_deg = math.degrees(robot.get_state().q_rad[0])
    while time.monotonic() < deadline:
        try:
            data = _call_vision_api()
        except RuntimeError:
            time.sleep(1.0 / poll_hz)
            continue
        hands = [h for human in data.get("humans", []) for h in human["hands"]]
        hands += data.get("unassigned_hands", [])
        if hands:
            hand_x = hands[0]["palm_center"][0]  # normalized [0,1], 0=left
            base_target_deg = (hand_x - 0.5) * 2 * 45  # [0,1] -> [-45, +45] deg
            current = robot.get_state()
            target_rad = list(current.q_rad)
            target_rad[0] = math.radians(base_target_deg)
            if abs(target_rad[0]) <= JOINT_LIMIT:
                robot.move_joint(target_rad, 0.5, 1.0)
                updates += 1
        time.sleep(1.0 / poll_hz)

    final_state = robot.get_state()
    return {
        "updates": updates,
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, final_state.q_rad)},
    }


# =========================================================================== #
# PATH CHECKING / OBSTACLE AVOIDANCE  --  block-diagram tool list. Closes the
# "genuinely not built, needs a real motion planner" gap from ../CLAUDE.md's
# block-diagram cross-check. Backed by MoveIt (motion_planner.py), talking to
# move_group's planning services directly -- a separate ROS2 process from
# either robot backend, so this needs move_group AND ur_robot_driver launched
# regardless of UR_BACKEND (socket or ros2) -- see ../CLAUDE.md's Running
# things section for both launch commands. Lazy-connected on first use (like
# the vision passthrough tools above) so nothing here breaks a run that never
# calls these tools -- e.g. test_server.py's default socket-backend pass.
# =========================================================================== #
_planner = None
_planner_lock = threading.Lock()


def _get_planner():
    global _planner
    with _planner_lock:
        if _planner is None:
            from motion_planner import MotionPlanner

            planner = MotionPlanner()
            try:
                planner.connect(timeout_s=5.0)
            except ConnectionError as exc:
                raise RuntimeError(
                    f"MoveIt not reachable ({exc}). Is move_group launched "
                    "(ros2 launch ur_moveit_config ur_moveit.launch.py "
                    "ur_type:=ur10 ...)? See ../CLAUDE.md's Running things "
                    "section."
                ) from exc
            _planner = planner
    return _planner


@mcp.tool
def check_path(joint_angles_deg: list[float]) -> dict:
    """Ask MoveIt whether a collision-free path exists from the robot's
    current joint configuration to a target one, WITHOUT moving the robot --
    a pre-flight check before committing to move_robot_to_position et al.

    Args:
        joint_angles_deg: Six target angles in degrees, base..wrist3 (same
            order/units as move_robot_to_position).

    Returns:
        A dict: ``feasible`` (bool). If not feasible, ``reason`` -- either
        "goal state is in collision" (+ ``colliding_pairs``, the specific
        link/obstacle names touching) or "no collision-free path found"
        (goal itself is clear, but nothing connects it to the current pose
        within the planning budget -- likely an obstacle blocking the way).
        If feasible, ``waypoints`` (planned trajectory point count).

    Raises:
        ValueError: Wrong number of angles.
        RuntimeError: MoveIt (move_group) isn't reachable.
    """
    from motion_planner import ros_joint_dict

    if len(joint_angles_deg) != len(JOINT_NAMES):
        raise ValueError(
            f"Expected {len(JOINT_NAMES)} joint angles "
            f"({', '.join(JOINT_NAMES)}), got {len(joint_angles_deg)}."
        )
    target_rad = [math.radians(a) for a in joint_angles_deg]
    return _get_planner().check_path(ros_joint_dict(target_rad))


@mcp.tool
def add_obstacle(
    obstacle_id: str, xyz_m: list[float], size_m: list[float],
) -> dict:
    """Add (or replace, same obstacle_id) a box obstacle in the planning
    scene, so check_path (and MoveIt planning generally) routes around it.

    Args:
        obstacle_id: Name for this obstacle -- reuse it to move/resize an
            existing one, or to remove_obstacle it later.
        xyz_m: [x, y, z] box centre, metres, base_link frame (same frame as
            tcp_pose's x/y/z).
        size_m: [x, y, z] full box extents, metres.

    Returns:
        ``{"status": "added", "obstacle_id": ..., "obstacles": [...]}`` --
        every obstacle currently registered, for confirmation.

    Raises:
        ValueError: xyz_m or size_m isn't length 3.
        RuntimeError: MoveIt (move_group) isn't reachable.
    """
    if len(xyz_m) != 3 or len(size_m) != 3:
        raise ValueError("xyz_m and size_m must each have exactly 3 values.")
    planner = _get_planner()
    planner.add_box_obstacle(obstacle_id, tuple(xyz_m), tuple(size_m))
    return {"status": "added", "obstacle_id": obstacle_id, "obstacles": planner.list_obstacles()}


@mcp.tool
def remove_obstacle(obstacle_id: str) -> dict:
    """Remove a previously added obstacle by id. No error if it doesn't
    exist -- MoveIt's remove is a no-op in that case, not a rejection.

    Returns:
        ``{"status": "removed", "obstacle_id": ..., "obstacles": [...]}``.

    Raises:
        RuntimeError: MoveIt (move_group) isn't reachable.
    """
    planner = _get_planner()
    planner.remove_obstacle(obstacle_id)
    return {"status": "removed", "obstacle_id": obstacle_id, "obstacles": planner.list_obstacles()}


@mcp.tool
def list_obstacles() -> dict:
    """Every obstacle currently registered in the planning scene.

    Returns:
        ``{"obstacles": [{"id", "xyz_m", "size_m"}, ...]}`` -- nested under
        a named key rather than returned as a bare list, since an empty
        result would otherwise be indistinguishable from a broken call (see
        list_waypoints's docstring for the same reasoning).

    Raises:
        RuntimeError: MoveIt (move_group) isn't reachable.
    """
    return {"obstacles": _get_planner().list_obstacles()}


# =========================================================================== #
# EMERGENCY STOP  --  team requirement, not one of the four tiers: halt any
# motion in progress right now, not the next command in a queue.
# =========================================================================== #
@mcp.tool
def stop_robot() -> dict:
    """Immediately halt any motion in progress.

    Call this the moment a motion looks wrong or was a mistake. On the
    socket backend this always works, because sending anything to the
    controller preempts whatever it's currently running -- same mechanism
    every move tool uses to start motion, just with a stop command instead.
    On the ROS2 backend this can only cancel a trajectory goal that is
    still tracked as active by the move call that started it, in this same
    process -- a stop issued after that call has already returned has
    nothing to cancel.

    Also cancels the non-blocking queue (see move_robot_queued/get_queue):
    every pending command is dropped, and the currently-running one (if
    any) is flagged cancelled -- so a stop_robot() call genuinely interrupts
    a move that was started with move_robot_queued, from the very next tool
    call, because that move didn't block the tool-call loop in the first
    place.

    Note: for the plain (blocking) move tools -- move_robot_to_position,
    move_robot_linear, move_robot_to_position_safe, move_through_waypoints
    -- tool calls within one chat turn still run one at a time, so calling
    this while still waiting on one of *those* to return won't interrupt
    it; use move_robot_queued instead when you need stop_robot to actually
    work mid-motion. On the ROS2 backend, robot.stop() itself can only
    cancel a trajectory goal still tracked as active in this same process.

    Returns:
        A dict with ``status``, ``cancelled_command_ids`` (queue entries
        dropped by this call, may be empty), and the robot's
        ``joints_deg``/``tcp_pose``/``robot_mode``/``safety_status`` right
        after the stop.
    """
    # 1-3. No inputs to validate or units to convert -- stop takes none.
    # 4. Cancel the queue, stop the robot, then report the resulting state.
    cancelled_ids = _queue.cancel_all()
    state = robot.stop()
    return {
        "status": "stopped",
        "cancelled_command_ids": cancelled_ids,
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, state.q_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
        "safety_status": state.safety_status,
    }


@mcp.tool
def sync_sim_to_real() -> dict:
    """Move the simulator to match the real robot's CURRENT joint angles.

    Shadow mode (UR_REAL_HOST set) only works as a safety pre-check if the
    simulator's pose actually matches where the physical arm is right now --
    otherwise a move "verified safe" in sim describes a path the real robot,
    starting somewhere else, will never actually take. This already runs
    automatically when the server connects and right after free_drive(); call
    it manually if the real robot was moved out of band since then (jogged by
    hand on the teach pendant, moved by another program, etc.) and you want
    the next verified move to gate on its true current pose.

    Only the simulator moves -- the real robot is never touched by this call.

    Raises:
        RuntimeError: No real robot is configured (UR_REAL_HOST unset), so
            there is nothing to sync the simulator to.

    Returns:
        A dict with the simulator's resulting ``joints_deg``/``tcp_pose``.
    """
    # 1-3. No inputs to validate or units to convert; the target is whatever
    # the real robot's own current joints are, not a value the caller picks.
    if not _REAL_HOST:
        raise RuntimeError(
            "Shadow mode is not active (UR_REAL_HOST is unset) -- there is "
            "no real robot for the simulator to sync to."
        )
    # 4. Execute (only the simulator moves) and report.
    state = robot.sync_sim_to_real()
    return _state_summary(state)


# =========================================================================== #
# TEMPLATE TOOL  --  the minimal shape of a tool, doing nothing real. Copy this
# to start a new one of your own, then follow the four-step pattern above.
# =========================================================================== #
@mcp.tool
def example() -> str:
    """A placeholder tool that performs no action.

    Returns a fixed string. Use it as the skeleton for a real tool of your own.
    """
    return "this was an example"


if __name__ == "__main__":
    robot.connect()
    mcp.run()
