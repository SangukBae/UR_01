"""Smoke test for the Case 1 MCP server, run in-process (no subprocess, no LLM).

Calls both tools through a FastMCP client against the live robot, so it checks
the tool logic, input validation, and real motion in one go.

    python test_server.py

Prereqs: the simulator is up (../simulation environment) and the robot is
powered on (RUNNING). The server connects to UR_HOST, or 127.0.0.1 by default.
"""
from __future__ import annotations

import asyncio
import logging

from fastmcp import Client

from server import mcp, robot

# The validation check below triggers an expected error; keep the framework from
# logging its traceback so the test output stays clean.
logging.disable(logging.CRITICAL)

HOME_DEG = [0, -90, 0, -90, 0, 0]


async def main() -> None:
    async with Client(mcp) as client:
        names = [t.name for t in await client.list_tools()]
        assert set(names) == {
            "move_robot_to_position", "example",
            "get_robot_state", "move_through_waypoints",
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

        # Gold: blended multi-waypoint move, with a trace of how it went.
        path = await client.call_tool("move_through_waypoints", {
            "waypoints_deg": [[20, -90, 0, -90, 0, 0], HOME_DEG],
        })
        assert path.data["status"] == "reached"
        assert len(path.data["trace"]) >= 1
        print("move_through_waypoints:", path.data["joints_deg"],
              f"({len(path.data['trace'])} trace points)")

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
