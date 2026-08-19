"""Smoke test for the Case 1 MCP server, run in-process (no subprocess, no LLM).

Calls every tool through a FastMCP client against the live robot, so it checks
the tool logic, input validation, and real motion in one go.

    python test_server.py

Prereqs: the simulator is up (../simulation environment) and the robot is
powered on (RUNNING). The server connects to UR_HOST, or 127.0.0.1 by default.
"""
from __future__ import annotations

import asyncio
import logging
import time

from fastmcp import Client

from server import mcp, robot

# The validation check below triggers an expected error; keep the framework from
# logging its traceback so the test output stays clean.
logging.disable(logging.CRITICAL)

HOME_DEG = [0, -90, 0, -90, 0, 0]

# Populated by on_progress while move_through_waypoints streams live -- see
# the Gold section below.
progress_log: list[float] = []


async def on_progress(progress: float, total: float | None, message: str | None) -> None:
    progress_log.append(time.monotonic())


async def main() -> None:
    async with Client(mcp, progress_handler=on_progress) as client:
        names = [t.name for t in await client.list_tools()]
        assert set(names) == {
            "move_robot_to_position", "example",
            "get_robot_state", "move_robot_linear", "move_through_waypoints",
            "move_robot_to_position_safe", "set_gripper",
        }, names
        print("tools:", names)

        result = await client.call_tool("example", {})
        assert result.data == "this was an example"
        print("example: ok")

        # Move home (no arguments) and to an explicit pose.
        home = await client.call_tool("move_robot_to_position", {})
        assert home.data["status"] == "reached"
        print("move home:", home.data["joints_deg"])

        pose = await client.call_tool(
            "move_robot_to_position", {"joint_angles_deg": [10, -90, 0, -90, 0, 0]}
        )
        assert pose.data["status"] == "reached"
        print("move pose:", pose.data["joints_deg"])

        # Validation: the wrong number of angles must be rejected.
        try:
            await client.call_tool("move_robot_to_position", {"joint_angles_deg": [0, 0, 0]})
        except Exception as exc:
            print("bad input rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("bad input was accepted")

        await client.call_tool("move_robot_to_position", {})  # park home

        # Silver: read state without moving.
        state = await client.call_tool("get_robot_state", {})
        assert state.data["robot_mode"] == 7, state.data
        assert "safety_status" in state.data and "joint_speeds_deg_s" in state.data
        print("get_robot_state:", state.data["joints_deg"])

        # Silver: TCP-space linear move -- lift 5cm in z, then come straight
        # back, so the path is a straight line, not a curve. Not from home:
        # HOME_DEG sits at a wrist singularity (wrist2 = 0deg, wrist1/wrist3
        # axes align) where even a small movel spikes joint speed and trips
        # a protective stop on the real controller -- reproduced while
        # building this tool, along with a second singularity (shoulder)
        # nearby at [20, -90, 30, -90, 60, 0] that looked safe (wrist2 =
        # 60deg) but wasn't. This pose was verified clean over repeated
        # movel calls (round trip, 3x) before being used here.
        pre_linear = await client.call_tool(
            "move_robot_to_position", {"joint_angles_deg": [30, -100, 40, -80, 70, 10]})
        base_tcp = pre_linear.data["tcp_pose"]
        lifted_tcp = [base_tcp[0], base_tcp[1], base_tcp[2] + 0.05, *base_tcp[3:]]
        lin_up = await client.call_tool("move_robot_linear", {"tcp_pose": lifted_tcp})
        assert lin_up.data["status"] == "reached"
        print("move_robot_linear (up):", lin_up.data["tcp_pose"])

        lin_down = await client.call_tool("move_robot_linear", {"tcp_pose": base_tcp})
        assert lin_down.data["status"] == "reached"
        print("move_robot_linear (down):", lin_down.data["tcp_pose"])

        # Validation: the wrong number of pose values must be rejected.
        try:
            await client.call_tool("move_robot_linear", {"tcp_pose": [0, 0, 0]})
        except Exception as exc:
            print("bad linear pose rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("bad linear pose was accepted")

        # Gold: blended multi-waypoint move, streamed live via MCP progress
        # notifications as it moves -- not just a trace after the fact.
        # Slow and far enough (unlike a quick single-step hop) to produce
        # several updates spread over real time, so the test can verify
        # they actually arrive while the call is in flight, not all at once
        # when it returns.
        progress_log.clear()
        call_start = time.monotonic()
        path = await client.call_tool("move_through_waypoints", {
            "waypoints_deg": [[30, -70, 10, -100, 40, 20], [-30, -110, -10, -80, -40, -20], HOME_DEG],
            "speed": 0.3, "acceleration": 0.6,
        })
        call_duration = time.monotonic() - call_start
        assert path.data["status"] == "reached"
        assert len(path.data["trace"]) >= 1
        assert len(progress_log) == len(path.data["trace"]), (
            "expected one progress notification per trace point",
            len(progress_log), len(path.data["trace"]),
        )
        assert len(progress_log) >= 3, "expected several live updates over a multi-second move, not just one"
        first_at, last_at = progress_log[0] - call_start, progress_log[-1] - call_start
        assert last_at <= call_duration + 0.05, "a progress event arrived after the call returned -- not live"
        print(f"move_through_waypoints: {len(progress_log)} live progress events over "
              f"{call_duration:.1f}s (first +{first_at:.2f}s, last +{last_at:.2f}s of "
              f"{call_duration:.1f}s total) -- streamed during the move, not after it")

        # Diamond: the safety-gated move succeeds for a reasonable target...
        safe = await client.call_tool(
            "move_robot_to_position_safe", {"joint_angles_deg": HOME_DEG})
        assert safe.data["status"] == "reached"
        print("move_robot_to_position_safe:", safe.data["joints_deg"])

        # ...and rejects one before anything moves (speed past the safety cap).
        try:
            await client.call_tool("move_robot_to_position_safe", {
                "joint_angles_deg": HOME_DEG, "speed": 999.0,
            })
        except Exception as exc:
            print("unsafe speed rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("unsafe speed was accepted")

        # Diamond: gripper open/close.
        closed = await client.call_tool("set_gripper", {"state": "close"})
        assert closed.data["gripper"] == "CLOSE"
        opened = await client.call_tool("set_gripper", {"state": "OPEN"})
        assert opened.data["gripper"] == "OPEN"
        print("set_gripper: close/open ok")

        # Validation: an unknown gripper state must be rejected.
        try:
            await client.call_tool("set_gripper", {"state": "SIDEWAYS"})
        except Exception as exc:
            print("bad gripper state rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("bad gripper state was accepted")

        await client.call_tool("move_robot_to_position", {})  # park home
    print("ALL PASSED")


if __name__ == "__main__":
    robot.connect()  # fails fast if the simulator is down or the robot is off
    asyncio.run(main())
