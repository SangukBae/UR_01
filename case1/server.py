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
  * Gold -- ``move_through_waypoints``: a blended multi-point trajectory, with
    a state trace of how the move unfolded.
  * Diamond -- ``move_robot_to_position_safe``: workspace/speed/forward-
    kinematics safety gate in front of a move; ``set_gripper``: open/close via
    digital IO.
  * ``example`` -- a do-nothing template showing the minimal tool shape. Copy
    it to start a new tool of your own.

Return JSON-serializable dicts with explicit units in the key names.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

from fastmcp import FastMCP

from kinematics import forward_kinematics
from ur_client import HOME_Q_RAD, JOINT_LIMIT, JOINT_NAMES, URClient

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
        metres and radians, ``robot_mode`` (7 = RUNNING), and ``safety_status``
        (1 = NORMAL; anything else means reduced/stopped/faulted -- see UR's
        SafetyMode).
    """
    state = robot.get_state()
    return {
        "joints_deg": {n: round(math.degrees(q), 1) for n, q in zip(JOINT_NAMES, state.q_rad)},
        "joint_speeds_deg_s": {n: round(math.degrees(qd), 2)
                                for n, qd in zip(JOINT_NAMES, state.qd_rad)},
        "tcp_pose": [round(v, 4) for v in state.tcp_pose],
        "robot_mode": state.robot_mode,
        "safety_status": state.safety_status,
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
# GOLD  --  a trajectory tool with waypoint blending, plus a state trace so an
# agent can see how a multi-step move unfolded (there's no live push channel
# to an MCP client mid-call, so this is the trace-after-the-fact version).
# =========================================================================== #
@mcp.tool
def move_through_waypoints(
    waypoints_deg: list[list[float]],
    speed: float = DEFAULT_SPEED,
    acceleration: float = DEFAULT_ACCEL,
) -> dict:
    """Move through several joint configurations as one smooth, blended motion.

    Unlike calling move_robot_to_position repeatedly, the robot doesn't stop at
    each waypoint -- it blends through them, which is faster and easier on the
    joints for a multi-point path (e.g. tracing a shape, or a pick-approach-
    place sequence).

    Args:
        waypoints_deg: A list of joint-angle sets, each six degrees in
            base..wrist3 order. At least one waypoint is required; the last
            one is where the robot ends up exactly (no blend radius there).
        speed: Joint speed (rad/s), shared by every segment.
        acceleration: Joint acceleration (rad/s^2), shared by every segment.

    Returns:
        A dict with the final ``joints_deg``/``tcp_pose``/``robot_mode``, plus
        ``trace``: a list of ``{joints_deg, tcp_pose}`` snapshots sampled while
        the robot was moving, so you can see the path it actually took.

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

    # 4. Execute (blocks until reached), then report the new state + trace.
    final_state, trace = robot.move_waypoints(waypoints_rad, speed, acceleration)
    return {
        "status": "reached",
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, final_state.q_rad)},
        "tcp_pose": [round(v, 4) for v in final_state.tcp_pose],
        "robot_mode": final_state.robot_mode,
        "trace": [
            {
                "joints_deg": {n: round(math.degrees(q), 1)
                               for n, q in zip(JOINT_NAMES, s.q_rad)},
                "tcp_pose": [round(v, 4) for v in s.tcp_pose],
            }
            for s in trace
        ],
    }


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
        A dict with ``gripper`` (the state that was commanded) and the
        robot's current ``joints_deg``/``tcp_pose``/``robot_mode``.

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
        "gripper": normalized,
        "joints_deg": {n: round(math.degrees(q), 1)
                       for n, q in zip(JOINT_NAMES, result.q_rad)},
        "tcp_pose": [round(v, 4) for v in result.tcp_pose],
        "robot_mode": result.robot_mode,
    }


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
