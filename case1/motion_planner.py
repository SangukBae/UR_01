"""MoveIt-backed path checking and obstacle avoidance for case1/server.py.

Block-diagram gap this closes: "Path checking" / "Obstacle Avoidance" (see
../CLAUDE.md's block-diagram cross-check -- these were listed as genuinely
not built, needing a real motion planner). Talks to move_group's plain
ROS2 services (GetMotionPlan, GetStateValidity, ApplyPlanningScene,
GetPlanningScene) directly via rclpy -- not moveit_py (not packaged for
ROS2 Humble via apt, only iron/rolling) and not moveit_commander (ROS1
only). No motion execution here -- this only tells server.py's tools
whether a path is clear; robot.move_joint() (ur_client.py or
ros2_client.py, picked by UR_BACKEND) still does the actual moving.

This is a separate rclpy node/connection from ros2_ur_driver/ros2_client.py's
ROS2URClient, and works under UR_BACKEND=socket too -- move_group and the
driver are their own separate ROS2 processes regardless of which backend
server.py uses to command motion.

Requires (launched separately, not managed here -- see ../README.md and
../ros2_ur_driver/README.md):
    ros2 launch ur_robot_driver ur10.launch.py robot_ip:=127.0.0.1 \\
        reverse_ip:=host.docker.internal \\
        kinematics_params_file:=<repo>/ros2_ur_driver/config/ur10_calibration.yaml \\
        launch_rviz:=false headless_mode:=true
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 \\
        robot_ip:=127.0.0.1 \\
        kinematics_params_file:=<repo>/ros2_ur_driver/config/ur10_calibration.yaml \\
        launch_rviz:=false
"""
from __future__ import annotations

import atexit
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.msg import RobotState as MoveItRobotState
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.srv import GetStateValidity
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

GROUP_NAME = "ur_manipulator"
PLANNING_FRAME = "base_link"
# Driver's actual joint names -- matches ros2_ur_driver/ros2_client.py's
# ROS_JOINT_NAMES, same canonical base/shoulder/elbow/wrist1-3 order.
ROS_JOINT_NAMES = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)
JOINT_TOLERANCE_RAD = 0.01  # goal constraint tolerance, both directions
MOVEIT_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS


class MotionPlanner:
    """Connection to move_group's planning services."""

    def __init__(self) -> None:
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._stop_spinning = threading.Event()
        self._plan_client = None
        self._validity_client = None
        self._apply_scene_client = None
        self._get_scene_client = None
        self._joint_state: JointState | None = None

    # --- Lifecycle ---------------------------------------------------------#
    def connect(self, timeout_s: float = 10.0) -> None:
        """Bring up the node, subscribe to /joint_states, and wait for
        move_group's planning services.

        Raises:
            ConnectionError: move_group's services or /joint_states aren't
                up within timeout_s.
        """
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("case1_motion_planner")
        self._node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._plan_client = self._node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._validity_client = self._node.create_client(GetStateValidity, "/check_state_validity")
        self._apply_scene_client = self._node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._get_scene_client = self._node.create_client(GetPlanningScene, "/get_planning_scene")

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        deadline = time.monotonic() + timeout_s
        for client, name in (
            (self._plan_client, "/plan_kinematic_path"),
            (self._validity_client, "/check_state_validity"),
            (self._apply_scene_client, "/apply_planning_scene"),
            (self._get_scene_client, "/get_planning_scene"),
        ):
            remaining = max(0.1, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                self.disconnect()
                raise ConnectionError(
                    f"MoveIt service {name} not available within {timeout_s:.0f}s. "
                    "Is move_group launched (ros2 launch ur_moveit_config "
                    "ur_moveit.launch.py ur_type:=ur10 ...)? See ../CLAUDE.md's "
                    "Running things section."
                )

        while self._joint_state is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self._joint_state is None:
            self.disconnect()
            raise ConnectionError(
                f"No /joint_states within {timeout_s:.0f}s. Is ur_robot_driver "
                "launched? See ../ros2_ur_driver/README.md."
            )

        # Same reasoning as ros2_client.py's ROS2URClient.connect(): callers
        # written against the socket backend expect no cleanup call, but an
        # un-torn-down rclpy node/executor can abort the interpreter on exit
        # (reproduced live: "terminate called without an active exception" +
        # a core dump after test_server.py's own "ALL PASSED" had already
        # printed). Registering here makes this a true drop-in with no
        # cleanup call required from code that doesn't know it's ROS2.
        atexit.register(self.disconnect)

    def disconnect(self) -> None:
        """Stop spinning and tear down the node. Idempotent. Deliberately
        does NOT call rclpy.shutdown() -- server.py may also run
        ROS2URClient (UR_BACKEND=ros2) in the same process; only one rclpy
        user should own the global shutdown. Whatever owns the process
        lifecycle (a test script, or process exit) shuts rclpy down once,
        after every rclpy user is done."""
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

    def _spin_loop(self) -> None:
        while not self._stop_spinning.is_set() and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except RuntimeError:
                # Same interpreter-shutdown race as ros2_client.py's _spin_loop.
                break

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    @staticmethod
    def _wait(future, timeout_s: float = 10.0):
        """Poll a service future to completion; the executor thread runs its
        callback. Raises TimeoutError past timeout_s instead of returning
        None, since every caller here needs an actual result to proceed."""
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("MoveIt service call timed out.")
            time.sleep(0.02)
        return future.result()

    def _current_robot_state_msg(self) -> MoveItRobotState:
        js = self._joint_state
        state = MoveItRobotState()
        state.joint_state.name = list(js.name)
        state.joint_state.position = list(js.position)
        return state

    # --- Path checking -------------------------------------------------- #
    def check_path(self, goal_q_rad: dict[str, float]) -> dict:
        """Ask MoveIt whether a collision-free path exists from the robot's
        *current* joint state to ``goal_q_rad`` (ROS joint name -> radians;
        see ros_joint_dict below for building this from the tool-facing
        base/shoulder/elbow/wrist1-3 degree order).

        Two-step check so the reason is specific, not just yes/no:
        1. Is the goal state itself in collision (self-collision or a
           registered obstacle)? Cheap, and names the colliding link pair(s).
        2. If the goal is clear on its own, does OMPL actually find a
           collision-free path from here to there within its planning
           budget? A path can fail here even with the goal itself clear --
           an obstacle blocking the way, not just parked in the target spot.

        Returns:
            ``{"feasible": bool, ...}`` -- on infeasible, ``reason`` is
            either "goal state is in collision" (+ ``colliding_pairs``) or
            "no collision-free path found" (+ ``error_code``); on
            feasible, ``waypoints`` (planned trajectory point count) and
            ``planning_time_s``.

        Raises:
            ConnectionError: connect() wasn't called / no joint state yet.
            TimeoutError: A planning service call took too long.
        """
        if self._joint_state is None:
            raise ConnectionError("Not connected. Call connect() first.")

        goal_state = MoveItRobotState()
        goal_state.joint_state.name = list(goal_q_rad.keys())
        goal_state.joint_state.position = list(goal_q_rad.values())
        validity_req = GetStateValidity.Request()
        validity_req.robot_state = goal_state
        validity_req.group_name = GROUP_NAME
        validity_result = self._wait(self._validity_client.call_async(validity_req))
        if not validity_result.valid:
            pairs = [f"{c.contact_body_1}<->{c.contact_body_2}"
                     for c in validity_result.contacts]
            return {
                "feasible": False,
                "reason": "goal state is in collision",
                "colliding_pairs": pairs,
            }

        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.start_state = self._current_robot_state_msg()
        req.num_planning_attempts = 3
        req.allowed_planning_time = 3.0

        constraints = Constraints()
        for name, value in goal_q_rad.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = JOINT_TOLERANCE_RAD
            jc.tolerance_below = JOINT_TOLERANCE_RAD
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]

        plan_req = GetMotionPlan.Request()
        plan_req.motion_plan_request = req
        plan_result = self._wait(self._plan_client.call_async(plan_req), timeout_s=10.0)
        resp = plan_result.motion_plan_response
        feasible = resp.error_code.val == MOVEIT_SUCCESS
        if not feasible:
            return {
                "feasible": False,
                "reason": "no collision-free path found",
                "error_code": resp.error_code.val,
                "planning_time_s": round(resp.planning_time, 3),
            }
        return {
            "feasible": True,
            "waypoints": len(resp.trajectory.joint_trajectory.points),
            "planning_time_s": round(resp.planning_time, 3),
        }

    # --- Obstacle avoidance ---------------------------------------------- #
    def add_box_obstacle(
        self, obj_id: str, xyz_m: tuple[float, float, float],
        size_m: tuple[float, float, float],
    ) -> None:
        """Add (or replace, same obj_id) an axis-aligned box obstacle in the
        planning scene, ``base_link`` frame -- centre position ``xyz_m``,
        full extents ``size_m``. Every check_path() call after this routes
        around it (or reports collision if a requested goal is inside it).

        Raises:
            RuntimeError: move_group rejected the planning scene update.
        """
        obj = CollisionObject()
        obj.header.frame_id = PLANNING_FRAME
        obj.id = obj_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size_m)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz_m
        pose.orientation.w = 1.0
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._apply_scene(scene)

    def remove_obstacle(self, obj_id: str) -> None:
        """Remove a previously added obstacle by id. No error if it doesn't
        exist -- MoveIt's REMOVE operation is a no-op in that case, not a
        rejection."""
        obj = CollisionObject()
        obj.header.frame_id = PLANNING_FRAME
        obj.id = obj_id
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._apply_scene(scene)

    def list_obstacles(self) -> list[dict]:
        """Every box obstacle currently registered in the planning scene."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        result = self._wait(self._get_scene_client.call_async(req))
        objs = []
        for obj in result.scene.world.collision_objects:
            # move_group normalizes what add_box_obstacle() sent: the pose we
            # gave becomes obj.pose (the object's own origin), and
            # primitive_poses come back relative to THAT (identity here, not
            # the world position) -- found live, read obj.pose, not
            # primitive_poses[0], or every obstacle reports as (0, 0, 0).
            size = list(obj.primitives[0].dimensions) if obj.primitives else None
            objs.append({
                "id": obj.id,
                "xyz_m": [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z],
                "size_m": size,
            })
        return objs

    def _apply_scene(self, scene: PlanningScene) -> None:
        req = ApplyPlanningScene.Request()
        req.scene = scene
        result = self._wait(self._apply_scene_client.call_async(req))
        if not result.success:
            raise RuntimeError("move_group rejected the planning scene update.")


def ros_joint_dict(q_rad: list[float]) -> dict[str, float]:
    """[base, shoulder, elbow, wrist1, wrist2, wrist3] radians (server.py's
    canonical JOINT_NAMES order) -> {ROS joint name: radians} for
    check_path()'s goal_q_rad argument."""
    return dict(zip(ROS_JOINT_NAMES, q_rad))


if __name__ == "__main__":
    import math

    print(f"RMW_IMPLEMENTATION={__import__('os').environ.get('RMW_IMPLEMENTATION', '(default)')}")
    planner = MotionPlanner()
    planner.connect()
    print("Connected. Checking path to HOME...")
    home = ros_joint_dict([0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0])
    print(planner.check_path(home))
    planner.disconnect()
    rclpy.shutdown()
