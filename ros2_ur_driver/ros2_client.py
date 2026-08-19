"""ROS2 seam over the UR robot, a drop-in alternative to case1/ur_client.py.

Same public shape as ``case1.ur_client.URClient`` (``RobotState``, ``connect()``,
``get_state()``, ``get_joint_positions()``, ``get_tcp_pose()``, ``move_joint()``),
but instead of talking to the controller directly over sockets, it goes through
``ros_humble_ur_robot_driver`` (see ../ros2_ur_driver/README.md for the launch
command and the two setup blockers).

This exists to close the gap in the team's target architecture (MCP Server ->
ROS2 -> UR Driver -> robot) instead of MCP Server -> raw sockets -> robot. Point
``case1/server.py`` at this class instead of ``URClient`` (``UR_BACKEND=ros2``)
and every existing tool keeps working unchanged, because the shape is the same.

Requires the driver already running and connected (``ros2 launch ur_robot_driver
ur10.launch.py ...``, see this folder's README) and ``rclpy`` on the path (source
``/opt/ros/humble/setup.bash`` first).

Sandbox note: the default RMW (FastRTPS) hung during discovery in this
environment; ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`` fixed it. Set that env var
before running this module if `ros2 topic list` hangs for you too.
"""
from __future__ import annotations

import atexit
import math
import threading
import time
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

# robot_mode / safety_mode are published TRANSIENT_LOCAL (latched, once on
# change): a VOLATILE subscriber matches but never receives the retained
# sample. Depth 1 mirrors the driver's own publisher QoS exactly.
_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Canonical order (matches case1/ur_client.py): base, shoulder, elbow, wrist1-3.
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
# The driver's actual joint names, in that same canonical order.
ROS_JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)

JOINT_LIMIT = 2 * math.pi - math.radians(2.0)
HOME_Q_RAD = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
ROBOT_MODE_RUNNING = RobotMode.RUNNING  # 7, same numbering as RTDE's robot_mode
SAFETY_NORMAL = SafetyMode.NORMAL       # 1, same numbering as RTDE's safety_status

_ACTION_NAME = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
DIGITAL_OUT_GRIPPER_PIN = SetIO.Request.PIN_TOOL_DOUT0


@dataclass
class RobotState:
    """Same shape as ur_client.RobotState, so tools don't care which seam they use."""

    q_rad: list[float]
    qd_rad: list[float]
    tcp_pose: list[float]
    robot_mode: int
    safety_status: int
    digital_output_bits: int

    @property
    def gripper_closed(self) -> bool:
        """Same meaning as ur_client.RobotState.gripper_closed -- see its docstring."""
        return bool(self.digital_output_bits & (1 << DIGITAL_OUT_GRIPPER_PIN))


def _quat_to_rotvec(x: float, y: float, z: float, w: float) -> list[float]:
    """Quaternion -> UR-style rotation vector [rx, ry, rz] = axis * angle."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    if w < 0:  # shortest-path rotation
        x, y, z, w = -x, -y, -z, -w
    angle = 2 * math.acos(max(-1.0, min(1.0, w)))
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-8:
        return [0.0, 0.0, 0.0]
    return [x / s * angle, y / s * angle, z / s * angle]


class ROS2URClient:
    """Connection to a UR robot via ros_humble_ur_robot_driver.

    Args:
        controller: Joint trajectory controller action namespace. Defaults to
            ``scaled_joint_trajectory_controller`` (the driver's default).
    """

    def __init__(self, controller: str = "scaled_joint_trajectory_controller") -> None:
        self._controller = controller
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._stop_spinning = threading.Event()
        self._action_client = None
        self._set_io_client = None
        self._lock = threading.Lock()
        self._joint_state: JointState | None = None
        self._tcp_pose: PoseStamped | None = None
        self._robot_mode: int | None = None
        self._safety_mode: int | None = None
        self._io_states: IOStates | None = None
        self._active_goal_handle = None  # set while a trajectory goal is in flight

    # --- Lifecycle -------------------------------------------------------- #
    def connect(self, timeout_s: float = 10.0) -> None:
        """Bring up the ROS2 node, subscribe to state, wait for the action
        server, and fail fast if the driver isn't there.

        Raises:
            ConnectionError: No state / no action server within ``timeout_s``.
        """
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("case1_ros2_bridge")
        self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10)
        self._node.create_subscription(
            PoseStamped, "/tcp_pose_broadcaster/pose", self._on_tcp_pose, 10)
        self._node.create_subscription(
            RobotMode, "/io_and_status_controller/robot_mode",
            self._on_robot_mode, _LATCHED_QOS)
        self._node.create_subscription(
            SafetyMode, "/io_and_status_controller/safety_mode",
            self._on_safety_mode, _LATCHED_QOS)
        self._node.create_subscription(
            IOStates, "/io_and_status_controller/io_states", self._on_io_states, 10)
        self._action_client = ActionClient(
            self._node, FollowJointTrajectory, f"/{self._controller}/follow_joint_trajectory")
        self._set_io_client = self._node.create_client(
            SetIO, "/io_and_status_controller/set_io")

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if (
                self._joint_state is not None
                and self._tcp_pose is not None
                and self._robot_mode is not None
                and self._safety_mode is not None
            ):
                break
            time.sleep(0.1)
        else:
            self.disconnect()
            raise ConnectionError(
                f"No full state (joint_states / tcp_pose / robot_mode / safety_mode) "
                f"within {timeout_s:.0f}s. Is ur_robot_driver launched and connected "
                "to the robot? See ros2_ur_driver/README.md."
            )

        if not self._action_client.wait_for_server(timeout_sec=timeout_s):
            self.disconnect()
            raise ConnectionError(
                f"Action server {_ACTION_NAME} not available within {timeout_s:.0f}s. "
                "Is the driver's controller_manager up and the controller active "
                "(`ros2 control list_controllers`)?"
            )

        # Callers (e.g. case1/test_server.py) are written against the socket
        # client, which needs no cleanup -- sockets just close on exit. rclpy
        # is not that forgiving: an unshut-down context aborts the interpreter
        # on exit. Register cleanup here so this backend is a true drop-in,
        # with no cleanup call required from code that doesn't know it's ROS2.
        atexit.register(self.disconnect)

    def disconnect(self) -> None:
        """Stop spinning, tear down the node, and shut rclpy down. Idempotent
        (safe to call more than once, including from the atexit hook).

        Order matters: the spin loop must actually exit *before* the executor
        and node are torn down, or spin_once can be mid-submit against a
        thread pool that just shut down under it (an interpreter abort at
        exit, intermittent -- easy to miss in a single test run). Signalling
        the stop flag and joining first, then shutting down, closes that gap."""
        if self._node is None:
            return
        self._stop_spinning.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.0)
        self._node.destroy_node()
        self._node = None
        self._executor = None
        self._spin_thread = None
        if rclpy.ok():
            rclpy.shutdown()

    def _spin_loop(self) -> None:
        while not self._stop_spinning.is_set() and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except RuntimeError:
                # concurrent.futures.thread's own atexit hook shuts every
                # ThreadPoolExecutor in the process down as soon as interpreter
                # exit begins, process-wide, independent of our stop flag. If
                # that race wins, this loop is already on its way out anyway.
                break

    # --- Subscription callbacks (run on the executor thread) -------------- #
    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _on_tcp_pose(self, msg: PoseStamped) -> None:
        self._tcp_pose = msg

    def _on_robot_mode(self, msg: RobotMode) -> None:
        self._robot_mode = msg.mode

    def _on_safety_mode(self, msg: SafetyMode) -> None:
        self._safety_mode = msg.mode

    def _on_io_states(self, msg: IOStates) -> None:
        self._io_states = msg

    # --- State -------------------------------------------------------------#
    def get_state(self) -> RobotState:
        """Latest cached state (subscriptions are pushed continuously in the
        background, so this returns immediately, no round-trip per call)."""
        js = self._joint_state
        pose = self._tcp_pose
        if js is None or pose is None:
            raise ConnectionError("Not connected. Call connect() first.")

        pos_by_name = dict(zip(js.name, js.position))
        vel_by_name = dict(zip(js.name, js.velocity))
        q = [pos_by_name[n] for n in ROS_JOINT_NAMES]
        qd = [vel_by_name[n] for n in ROS_JOINT_NAMES]

        p, o = pose.pose.position, pose.pose.orientation
        rotvec = _quat_to_rotvec(o.x, o.y, o.z, o.w)
        tcp = [p.x, p.y, p.z, *rotvec]

        digital_out_bits = 0
        if self._io_states is not None:
            for d in self._io_states.digital_out_states:
                if d.state:
                    digital_out_bits |= 1 << d.pin

        return RobotState(
            q_rad=q,
            qd_rad=qd,
            tcp_pose=tcp,
            robot_mode=self._robot_mode if self._robot_mode is not None else -1,
            safety_status=self._safety_mode if self._safety_mode is not None else -1,
            digital_output_bits=digital_out_bits,
        )

    def get_joint_positions(self) -> list[float]:
        return self.get_state().q_rad

    def get_tcp_pose(self) -> list[float]:
        return self.get_state().tcp_pose

    # --- Motion ------------------------------------------------------------#
    def move_joint(
        self,
        q: list[float],
        speed: float,
        acceleration: float,
        *,
        tol_rad: float = 0.01,
        timeout_s: float = 20.0,
    ) -> RobotState:
        """Joint-space move to absolute angles ``q`` (radians) via the
        FollowJointTrajectory action, then block until the robot arrives.

        Raises:
            RuntimeError: Not in RUNNING mode, or the goal was rejected /
                finished with a non-zero error code.
            TimeoutError: The action didn't finish within ``timeout_s``.
        """
        state = self.get_state()
        if state.robot_mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(
                f"Robot is not powered on (mode {state.robot_mode}, need "
                f"{ROBOT_MODE_RUNNING}=RUNNING). Power it on + release brakes "
                "in the PolyScope X UI first."
            )

        duration_s = self._estimate_duration(state.q_rad, q, speed, acceleration)

        point = JointTrajectoryPoint()
        point.positions = list(q)
        point.velocities = [0.0] * len(q)
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int((duration_s % 1.0) * 1e9)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ROS_JOINT_NAMES)
        goal.trajectory.points = [point]

        deadline = time.monotonic() + timeout_s

        send_future = self._action_client.send_goal_async(goal)
        self._wait(send_future, deadline)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected by the controller.")

        self._active_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._wait(result_future, deadline)
            wrapped = result_future.result()
        finally:
            self._active_goal_handle = None
        if wrapped is None:
            raise TimeoutError(
                f"Robot did not reach the target within {timeout_s:.0f}s "
                "(still moving, blocked, or a protective stop?)."
            )
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return self.get_state()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code != 0:
            raise RuntimeError(
                f"Trajectory failed: status={wrapped.status} "
                f"error_code={wrapped.result.error_code} "
                f"({wrapped.result.error_string})"
            )

        final = self.get_state()
        if max(abs(a - b) for a, b in zip(final.q_rad, q)) > tol_rad:
            raise TimeoutError(
                f"Controller reported success but joints are still "
                f"{max(abs(a - b) for a, b in zip(final.q_rad, q)):.4f} rad off target."
            )
        return final

    def move_linear(
        self,
        pose: list[float],
        speed: float,
        acceleration: float,
        **kwargs,
    ) -> RobotState:
        """Not implemented: TCP-space linear moves need inverse kinematics,
        since scaled_joint_trajectory_controller only accepts joint-space
        goals (unlike the socket backend's URScript ``movel``, which does the
        Cartesian interpolation on the controller itself). Raises so a caller
        gets a clear, immediate reason instead of an AttributeError."""
        raise NotImplementedError(
            "move_linear needs inverse kinematics for "
            "scaled_joint_trajectory_controller (joint-space only) -- not "
            "implemented in the ROS2 backend yet. Use UR_BACKEND=socket "
            "(URScript movel) for move_robot_linear instead."
        )

    def free_drive(self, duration_s: float) -> RobotState:
        """Not implemented: free-drive is a URScript-level PolyScope
        feature (``freedrive_mode()``); the ROS2 driver would need its own
        ``freedrive_mode_controller`` activated via the controller manager
        to do the equivalent, not wired up here. Raises so a caller gets a
        clear, immediate reason instead of an AttributeError."""
        raise NotImplementedError(
            "free_drive needs the ROS2 driver's freedrive_mode_controller "
            "activated -- not implemented in the ROS2 backend yet. Use "
            "UR_BACKEND=socket (URScript freedrive_mode()) instead."
        )

    def move_waypoints(
        self,
        waypoints: list[list[float]],
        speed: float,
        acceleration: float,
        *,
        tol_rad: float = 0.01,
        timeout_s: float = 40.0,
        trace_interval_s: float = 0.3,
        on_state=None,
    ) -> tuple[RobotState, list[RobotState]]:
        """Move through several joint-space waypoints as one blended trajectory
        (a single FollowJointTrajectory goal with one point per waypoint --
        the controller's own spline interpolation blends between them, no
        stop-and-restart at each one).

        Args:
            on_state: If given, called synchronously with each polled
                ``RobotState`` as it's captured -- same live-streaming hook
                as ur_client.URClient.move_waypoints, see its docstring.

        Returns:
            ``(final_state, trace)`` -- same shape as ur_client's
            move_waypoints: the state on arrival, and every state snapshot
            polled while the goal was executing.

        Raises:
            RuntimeError: Not RUNNING, goal rejected, or a non-zero result code.
            TimeoutError: The last waypoint wasn't reached within ``timeout_s``.
        """
        state = self.get_state()
        if state.robot_mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(
                f"Robot is not powered on (mode {state.robot_mode}, need "
                f"{ROBOT_MODE_RUNNING}=RUNNING). Power it on + release brakes "
                "in the PolyScope X UI first."
            )

        points = []
        cumulative_s = 0.0
        prev_q = state.q_rad
        for wp in waypoints:
            cumulative_s += self._estimate_duration(prev_q, wp, speed, acceleration)
            point = JointTrajectoryPoint()
            point.positions = list(wp)
            point.velocities = [0.0] * len(wp)
            point.time_from_start.sec = int(cumulative_s)
            point.time_from_start.nanosec = int((cumulative_s % 1.0) * 1e9)
            points.append(point)
            prev_q = wp

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ROS_JOINT_NAMES)
        goal.trajectory.points = points

        deadline = time.monotonic() + timeout_s
        send_future = self._action_client.send_goal_async(goal)
        self._wait(send_future, deadline)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Trajectory goal was rejected by the controller.")

        self._active_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            target = waypoints[-1]
            trace: list[RobotState] = []
            last_poll = 0.0
            while not result_future.done():
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"Robot did not reach the last waypoint within {timeout_s:.0f}s "
                        "(still moving, blocked, or a protective stop?)."
                    )
                if time.monotonic() - last_poll >= trace_interval_s:
                    state = self.get_state()
                    trace.append(state)
                    last_poll = time.monotonic()
                    if on_state is not None:
                        on_state(state)
                time.sleep(0.05)
            wrapped = result_future.result()
        finally:
            self._active_goal_handle = None

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            return self.get_state(), trace
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or wrapped.result.error_code != 0:
            raise RuntimeError(
                f"Trajectory failed: status={wrapped.status} "
                f"error_code={wrapped.result.error_code} "
                f"({wrapped.result.error_string})"
            )

        final = self.get_state()
        if max(abs(a - b) for a, b in zip(final.q_rad, target)) > tol_rad:
            raise TimeoutError(
                f"Controller reported success but joints are still "
                f"{max(abs(a - b) for a, b in zip(final.q_rad, target)):.4f} rad off target."
            )
        return final, trace

    # --- Emergency stop --------------------------------------------------- #
    def stop(self) -> RobotState:
        """Cancel the active trajectory goal, if any (best-effort: if
        nothing is moving, this is a no-op). Unlike the socket backend
        (a fresh program upload always preempts), a ROS2 goal can only be
        cancelled if move_joint/move_waypoints is still tracking it as
        active -- there's no separate out-of-band stop channel here."""
        handle = self._active_goal_handle
        if handle is not None:
            cancel_future = handle.cancel_goal_async()
            self._wait(cancel_future, time.monotonic() + 2.0)
        time.sleep(0.3)
        return self.get_state()

    # --- Tool IO -------------------------------------------------------- #
    def set_gripper(self, closed: bool) -> RobotState:
        """Set tool digital output 0 via the driver's SetIO service, ON to
        close, OFF to open, and report the resulting state."""
        state = self.get_state()
        if state.robot_mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(
                f"Robot is not powered on (mode {state.robot_mode}, need "
                f"{ROBOT_MODE_RUNNING}=RUNNING). Power it on + release brakes "
                "in the PolyScope X UI first."
            )
        if not self._set_io_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/io_and_status_controller/set_io service not available.")

        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = DIGITAL_OUT_GRIPPER_PIN
        req.state = float(SetIO.Request.STATE_ON if closed else SetIO.Request.STATE_OFF)

        deadline = time.monotonic() + 5.0
        future = self._set_io_client.call_async(req)
        self._wait(future, deadline)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("set_io service call failed or timed out.")

        time.sleep(0.5)  # let the signal land before reporting state
        return self.get_state()

    @staticmethod
    def _estimate_duration(
        q_start: list[float], q_target: list[float], speed: float, acceleration: float,
    ) -> float:
        """Trapezoidal-profile time estimate, max across joints (mirrors how
        movej times a coordinated move: every joint arrives together)."""
        speed = max(speed, 1e-3)
        acceleration = max(acceleration, 1e-3)
        t_accel = speed / acceleration
        d_accel = speed * speed / acceleration  # distance covered by accel+decel

        worst = 0.0
        for a, b in zip(q_start, q_target):
            d = abs(b - a)
            if d >= d_accel:
                t = d / speed + t_accel
            else:
                t = 2 * math.sqrt(d / acceleration) if d > 0 else 0.0
            worst = max(worst, t)
        return max(worst, 0.5)  # floor so a near-zero move still gets a valid goal

    @staticmethod
    def _wait(future, deadline: float) -> None:
        """Poll a future to completion; the executor thread runs its callback."""
        while not future.done():
            if time.monotonic() > deadline:
                return
            time.sleep(0.02)


if __name__ == "__main__":
    import os

    print(f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '(default)')}")
    client = ROS2URClient()
    client.connect()
    print(client.get_state())
    client.disconnect()  # idempotent; connect() also registered this via atexit
