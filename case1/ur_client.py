"""Thin seam over the UR robot interface, using only the Python standard library.

Two UR network interfaces are spoken directly over TCP sockets, so there is no
C++ toolchain to install and nothing to compile:

  * Primary interface (port 30001) -- MOTION. We upload a tiny URScript program
    (a ``movej``) and the controller runs it. That is all a joint move needs.
  * RTDE, Real-Time Data Exchange (port 30004) -- STATE. We speak just enough of
    the RTDE protocol to read the actual joint angles and TCP pose.

This is the seam between the robot and the MCP tools: ``server.py`` calls these
methods and never touches sockets or the RTDE byte format directly. Keeping the
boundary here means the tool layer stays portable between the PolyScope X
simulator (see ``../simulation environment``) and a real UR robot -- only the
host address changes.
"""
from __future__ import annotations

import math
import os
import socket
import struct
import time
from collections.abc import Callable
from dataclasses import dataclass, field

# UR joint order: base, shoulder, elbow, wrist1, wrist2, wrist3.
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")

# Software joint limits (radians). UR joints rotate +/- 2*pi; we keep a small
# margin so a move never parks exactly on the mechanical limit.
JOINT_LIMIT = 2 * math.pi - math.radians(2.0)

# Canonical home pose, joint angles in radians (base..wrist3): upright column,
# elbow square, tool pointing down. Same convention used across the three cases.
HOME_Q_RAD = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

# UR robot mode 7 == RUNNING (powered on, brakes released, ready to move).
ROBOT_MODE_RUNNING = 7

PRIMARY_PORT = 30001  # motion: upload URScript here
RTDE_PORT = 30004     # state: read joint angles / TCP pose here

# RTDE protocol command bytes (see the UR RTDE guide).
_RTDE_REQUEST_PROTOCOL_VERSION = 86  # 'V'
_RTDE_SETUP_OUTPUTS = 79             # 'O'
_RTDE_START = 83                     # 'S'
_RTDE_DATA_PACKAGE = 85              # 'U'
_RTDE_OUTPUTS = (
    "actual_q,actual_qd,actual_TCP_pose,robot_mode,safety_status,"
    "actual_digital_output_bits"
)

DIGITAL_OUT_GRIPPER_PIN = 0  # tool digital output 0, see ur_client.set_gripper


@dataclass
class RobotState:
    """A single snapshot of the robot, read over RTDE."""

    q_rad: list[float]        # actual joint angles (radians), base..wrist3
    qd_rad: list[float]       # actual joint velocities (rad/s), base..wrist3
    tcp_pose: list[float]     # actual TCP pose [x, y, z, rx, ry, rz] (m, rad)
    robot_mode: int           # 7 == RUNNING
    safety_status: int        # 1 == NORMAL
    digital_output_bits: int  # raw bitmask, all 18 tool+controller digital outputs

    @property
    def gripper_closed(self) -> bool:
        """Actual state of DIGITAL_OUT_GRIPPER_PIN, read back from the
        controller -- not just what was last commanded (see set_gripper's
        docstring: that call is fire-and-forget, this is the honest check)."""
        return bool(self.digital_output_bits & (1 << DIGITAL_OUT_GRIPPER_PIN))


@dataclass
class URClient:
    """Connection to a UR robot or the PolyScope X simulator.

    Args:
        host: Robot / simulator IP. Defaults to the ``UR_HOST`` env var, then to
            ``127.0.0.1`` (the simulator's published address on your machine).
    """

    host: str = field(default_factory=lambda: os.environ.get("UR_HOST", "127.0.0.1"))

    def connect(self) -> None:
        """Verify the robot is reachable, failing fast with a readable reason."""
        try:
            self.get_state()
        except OSError as exc:
            raise ConnectionError(
                f"Cannot reach the robot at {self.host}:{RTDE_PORT} ({exc}). "
                "Is the simulator up? See ../simulation environment "
                "(docker compose up -d), then open http://localhost."
            ) from exc

    # --- State (RTDE receive) ------------------------------------------- #
    def get_state(self) -> RobotState:
        """Read one RTDE snapshot: joint angles, TCP pose, mode, safety."""
        with socket.create_connection((self.host, RTDE_PORT), timeout=5) as s:
            self._rtde_send(s, _RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))
            self._rtde_recv(s)  # version-accepted reply
            self._rtde_send(
                s, _RTDE_SETUP_OUTPUTS,
                struct.pack(">d", 125.0) + _RTDE_OUTPUTS.encode(),
            )
            self._rtde_recv(s)  # recipe + variable types
            self._rtde_send(s, _RTDE_START)
            self._rtde_recv(s)  # start-accepted reply

            cmd, body = self._rtde_recv(s)
            while cmd != _RTDE_DATA_PACKAGE:  # skip any non-data control replies
                cmd, body = self._rtde_recv(s)

        off = 1  # first byte is the recipe id
        q = list(struct.unpack(">6d", body[off:off + 48])); off += 48
        qd = list(struct.unpack(">6d", body[off:off + 48])); off += 48
        tcp = list(struct.unpack(">6d", body[off:off + 48])); off += 48
        mode = struct.unpack(">i", body[off:off + 4])[0]; off += 4
        safety = struct.unpack(">i", body[off:off + 4])[0]; off += 4
        digital_out = struct.unpack(">Q", body[off:off + 8])[0]
        return RobotState(
            q_rad=q, qd_rad=qd, tcp_pose=tcp, robot_mode=mode, safety_status=safety,
            digital_output_bits=digital_out)

    def get_joint_positions(self) -> list[float]:
        """Actual joint angles in radians, base..wrist3."""
        return self.get_state().q_rad

    def get_tcp_pose(self) -> list[float]:
        """Actual TCP pose [x, y, z, rx, ry, rz] (metres, radians)."""
        return self.get_state().tcp_pose

    # --- Motion (primary interface) ------------------------------------- #
    def move_joint(
        self,
        q: list[float],
        speed: float,
        acceleration: float,
        *,
        tol_rad: float = 0.01,
        timeout_s: float = 20.0,
    ) -> RobotState:
        """Joint-space move to absolute angles ``q`` (radians), then block until
        the robot arrives (or ``timeout_s`` elapses).

        Raises:
            RuntimeError: The robot is not in RUNNING mode, so it cannot move.
                Power it on / release brakes in the PolyScope X UI first.
            TimeoutError: The robot did not reach ``q`` within ``timeout_s``.
        """
        state = self.get_state()
        if state.robot_mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(
                f"Robot is not powered on (mode {state.robot_mode}, need "
                f"{ROBOT_MODE_RUNNING}=RUNNING). Open http://localhost and power "
                "the robot on + release brakes, then try again."
            )

        joints = ", ".join(f"{v:.6f}" for v in q)
        script = (
            "def move_to():\n"
            f"  movej([{joints}], a={acceleration:.4f}, v={speed:.4f})\n"
            "end\n"
        )
        with socket.create_connection((self.host, PRIMARY_PORT), timeout=5) as s:
            s.sendall(script.encode())

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            time.sleep(0.3)
            state = self.get_state()
            if max(abs(a - b) for a, b in zip(state.q_rad, q)) <= tol_rad:
                return state
        raise TimeoutError(
            f"Robot did not reach the target within {timeout_s:.0f}s "
            "(still moving, blocked, or a protective stop?)."
        )

    def move_linear(
        self,
        pose: list[float],
        speed: float,
        acceleration: float,
        *,
        tol_m: float = 0.005,
        tol_rad: float = 0.01,
        timeout_s: float = 20.0,
    ) -> RobotState:
        """TCP-space move in a straight line to an absolute pose ``pose``
        (``[x, y, z, rx, ry, rz]``, metres + rotation-vector radians, base
        frame), then block until the robot arrives (or ``timeout_s`` elapses).

        Unlike ``move_joint`` (URScript ``movej``, curved joint-space path),
        this uses URScript ``movel``, which interpolates the TCP itself in a
        straight line -- the joints take whatever path keeps the TCP linear.

        Raises:
            RuntimeError: The robot is not in RUNNING mode.
            TimeoutError: The robot did not reach ``pose`` within ``timeout_s``.
        """
        state = self.get_state()
        if state.robot_mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(
                f"Robot is not powered on (mode {state.robot_mode}, need "
                f"{ROBOT_MODE_RUNNING}=RUNNING). Open http://localhost and power "
                "the robot on + release brakes, then try again."
            )

        pose_str = ", ".join(f"{v:.6f}" for v in pose)
        script = (
            "def move_lin():\n"
            f"  movel(p[{pose_str}], a={acceleration:.4f}, v={speed:.4f})\n"
            "end\n"
        )
        with socket.create_connection((self.host, PRIMARY_PORT), timeout=5) as s:
            s.sendall(script.encode())

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            time.sleep(0.3)
            state = self.get_state()
            pos_ok = max(abs(a - b) for a, b in zip(state.tcp_pose[:3], pose[:3])) <= tol_m
            rot_ok = max(abs(a - b) for a, b in zip(state.tcp_pose[3:], pose[3:])) <= tol_rad
            if pos_ok and rot_ok:
                return state
        raise TimeoutError(
            f"Robot did not reach the target pose within {timeout_s:.0f}s "
            "(still moving, blocked, unreachable, or a protective stop?)."
        )

    def move_waypoints(
        self,
        waypoints: list[list[float]],
        speed: float,
        acceleration: float,
        *,
        blend_radius: float = 0.05,
        tol_rad: float = 0.01,
        timeout_s: float = 40.0,
        trace_interval_s: float = 0.3,
        on_state: Callable[[RobotState], None] | None = None,
    ) -> tuple[RobotState, list[RobotState]]:
        """Move through several joint-space waypoints in one blended motion.

        Every waypoint but the last gets ``blend_radius`` so the robot doesn't
        stop at each one (URScript's ``movej ... r=``); the last is approached
        exactly. Polls state throughout so the caller gets a trace of the move
        as it happened, not just the final pose.

        Args:
            on_state: If given, called synchronously with each polled
                ``RobotState`` as it's captured (same thread, same cadence as
                the trace) -- this is the live-streaming hook: a caller with
                its own way of pushing updates out (e.g. server.py's MCP
                progress notifications) can watch the move as it happens
                instead of only seeing the trace after the fact. This method
                itself has no opinion on how -- or whether -- those updates
                reach anyone; it just calls back.

        Returns:
            ``(final_state, trace)`` -- the state on arrival, and every state
            snapshot polled while waiting.

        Raises:
            RuntimeError: The robot is not in RUNNING mode.
            TimeoutError: The last waypoint wasn't reached within ``timeout_s``.
        """
        state = self.get_state()
        if state.robot_mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(
                f"Robot is not powered on (mode {state.robot_mode}, need "
                f"{ROBOT_MODE_RUNNING}=RUNNING). Open http://localhost and power "
                "the robot on + release brakes, then try again."
            )

        lines = ["def move_path():"]
        for i, wp in enumerate(waypoints):
            joints = ", ".join(f"{v:.6f}" for v in wp)
            r = 0.0 if i == len(waypoints) - 1 else blend_radius
            lines.append(f"  movej([{joints}], a={acceleration:.4f}, v={speed:.4f}, r={r:.4f})")
        lines.append("end")
        script = "\n".join(lines) + "\n"

        with socket.create_connection((self.host, PRIMARY_PORT), timeout=5) as s:
            s.sendall(script.encode())

        target = waypoints[-1]
        # A lower bound on how long the full blended path should take, so the
        # tolerance check below can't fire before the robot has actually had
        # time to visit every waypoint. Without this, a path that loops back
        # to (near) its own start -- e.g. out-and-home -- can match the
        # target tolerance on the very first poll, before the robot has
        # moved at all, and return a stale "reached" while the uploaded
        # URScript program is still running for real on the controller
        # (reproduced while building the streaming hook above: a 3-waypoint
        # round trip "completed" in 0.14s client-side while the
        # controller's own program log showed it actually ran ~1.3s).
        min_duration_s = self._estimate_path_duration(state.q_rad, waypoints, speed, acceleration)
        move_start = time.monotonic()
        trace: list[RobotState] = []
        deadline = move_start + timeout_s
        last_poll = 0.0
        while time.monotonic() < deadline:
            time.sleep(0.1)
            state = self.get_state()
            if time.monotonic() - last_poll >= trace_interval_s:
                trace.append(state)
                last_poll = time.monotonic()
                if on_state is not None:
                    on_state(state)
            elapsed = time.monotonic() - move_start
            if (elapsed >= min_duration_s
                    and max(abs(a - b) for a, b in zip(state.q_rad, target)) <= tol_rad):
                return state, trace
        raise TimeoutError(
            f"Robot did not reach the last waypoint within {timeout_s:.0f}s "
            "(still moving, blocked, or a protective stop?)."
        )

    @staticmethod
    def _estimate_path_duration(
        q_start: list[float], waypoints: list[list[float]], speed: float, acceleration: float,
    ) -> float:
        """Trapezoidal-profile time estimate, summed across every segment
        (start -> waypoint[0] -> waypoint[1] -> ...), max joint per segment.
        Mirrors the per-segment estimate ros2_client.py already uses to time
        its own trajectory goals -- here it's a minimum-time floor instead,
        since the socket backend gets no completion signal from the
        controller and has to poll-and-guess."""
        speed = max(speed, 1e-3)
        acceleration = max(acceleration, 1e-3)
        t_accel = speed / acceleration
        d_accel = speed * speed / acceleration

        total = 0.0
        prev = q_start
        for wp in waypoints:
            worst = 0.0
            for a, b in zip(prev, wp):
                d = abs(b - a)
                t = d / speed + t_accel if d >= d_accel else (
                    2 * math.sqrt(d / acceleration) if d > 0 else 0.0)
                worst = max(worst, t)
            total += worst
            prev = wp
        return total

    # --- Emergency stop --------------------------------------------------- #
    def stop(self, deceleration: float = 2.0) -> RobotState:
        """Immediately stop any motion in progress. Uploading a new program
        preempts whatever's currently running on the controller -- same
        mechanism every other method here uses to start a move, just with
        ``stopj`` instead. No RUNNING check: stopping should never itself be
        blocked by the state it's meant to get the robot out of."""
        script = f"def stop_now():\n  stopj({deceleration:.4f})\nend\n"
        with socket.create_connection((self.host, PRIMARY_PORT), timeout=5) as s:
            s.sendall(script.encode())
        time.sleep(0.3)  # let the stop take effect before reporting state
        return self.get_state()

    # --- Tool IO ---------------------------------------------------------- #
    def set_gripper(self, closed: bool) -> RobotState:
        """Set tool digital output 0 (the convention this rig's gripper is
        wired to) ON to close, OFF to open, and report the resulting state.

        This is a fire-and-forget digital signal -- there's no RTDE feedback
        signal wired up in this recipe to confirm the gripper physically
        reached that position, only that the command was sent.
        """
        state = self.get_state()
        if state.robot_mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(
                f"Robot is not powered on (mode {state.robot_mode}, need "
                f"{ROBOT_MODE_RUNNING}=RUNNING). Open http://localhost and power "
                "the robot on + release brakes, then try again."
            )
        value = "True" if closed else "False"
        script = (
            "def set_gripper():\n"
            f"  set_digital_out({DIGITAL_OUT_GRIPPER_PIN}, {value})\n"
            "end\n"
        )
        with socket.create_connection((self.host, PRIMARY_PORT), timeout=5) as s:
            s.sendall(script.encode())
        time.sleep(0.5)  # let the signal land before reporting state
        return self.get_state()

    # --- RTDE framing helpers ------------------------------------------- #
    @staticmethod
    def _rtde_send(s: socket.socket, cmd: int, payload: bytes = b"") -> None:
        s.sendall(struct.pack(">HB", 3 + len(payload), cmd) + payload)

    @staticmethod
    def _rtde_recv(s: socket.socket) -> tuple[int, bytes]:
        hdr = b""
        while len(hdr) < 3:
            hdr += s.recv(3 - len(hdr))
        size, cmd = struct.unpack(">HB", hdr)
        body = b""
        while len(body) < size - 3:
            chunk = s.recv(size - 3 - len(body))
            if not chunk:
                break
            body += chunk
        return cmd, body
