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
            "move_robot_to_position_safe", "set_gripper", "stop_robot",
            "sync_sim_to_real",
            "move_robot_queued", "get_queue",
            "save_waypoint", "list_waypoints", "delete_waypoint",
            "move_robot_to_waypoint", "free_drive",
            "move_robot_to_position_relative", "move_robot_linear_relative",
            "move_robot_linear_sequence",
            "program_new", "list_programs", "program_delete",
            "program_start", "program_stop",
            "get_vision", "get_environment", "get_environment_shadow",
            "track",
            "check_path", "add_obstacle", "remove_obstacle", "list_obstacles",
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

        # RELATIVE MOVES: delta-from-current variants, and a non-blended
        # TCP-space sequence.
        rel_joint = await client.call_tool(
            "move_robot_to_position_relative", {"delta_deg": [10, 0, 0, 0, 0, 0]})
        assert rel_joint.data["status"] == "reached"
        assert rel_joint.data["joints_deg"]["base"] == 40.0, rel_joint.data  # 30 + 10
        print("move_robot_to_position_relative: base 30 -> 40 deg")

        pre_rel_lin = await client.call_tool("get_robot_state", {})
        base_z = pre_rel_lin.data["tcp_pose"][2]
        rel_lin = await client.call_tool(
            "move_robot_linear_relative", {"delta": [0, 0, 0.03, 0, 0, 0]})
        assert rel_lin.data["status"] == "reached"
        assert abs(rel_lin.data["tcp_pose"][2] - (base_z + 0.03)) < 0.005, rel_lin.data
        print("move_robot_linear_relative: z +0.03m confirmed")

        cur_tcp = (await client.call_tool("get_robot_state", {})).data["tcp_pose"]
        up_tcp = [cur_tcp[0], cur_tcp[1], cur_tcp[2] + 0.03, *cur_tcp[3:]]
        seq = await client.call_tool("move_robot_linear_sequence", {
            "tcp_poses": [up_tcp, cur_tcp],
        })
        assert seq.data["status"] == "reached"
        assert len(seq.data["trace"]) == 2, seq.data
        print(f"move_robot_linear_sequence: {len(seq.data['trace'])} legs completed")

        try:
            await client.call_tool("move_robot_linear_sequence", {"tcp_poses": []})
        except Exception as exc:
            print("empty linear sequence rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("empty move_robot_linear_sequence was accepted")

        await client.call_tool("move_robot_to_position", {})  # park home

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

        # Diamond: gripper open/close -- "gripper" is a controller readback
        # (actual_digital_output_bits over RTDE), not an echo of what was
        # commanded, so this also proves the signal really toggled.
        closed = await client.call_tool("set_gripper", {"state": "close"})
        assert closed.data["gripper"] == "CLOSE"
        state_closed = await client.call_tool("get_robot_state", {})
        assert state_closed.data["gripper_closed"] is True, state_closed.data

        opened = await client.call_tool("set_gripper", {"state": "OPEN"})
        assert opened.data["gripper"] == "OPEN"
        state_open = await client.call_tool("get_robot_state", {})
        assert state_open.data["gripper_closed"] is False, state_open.data
        print("set_gripper: close/open ok, readback confirms the signal toggled")

        # Validation: an unknown gripper state must be rejected.
        try:
            await client.call_tool("set_gripper", {"state": "SIDEWAYS"})
        except Exception as exc:
            print("bad gripper state rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("bad gripper state was accepted")

        # WAYPOINT DB: save/list/delete/move-to, and free_drive.
        await client.call_tool("move_robot_to_position", {})  # known start: home
        saved = await client.call_tool("save_waypoint", {"name": "test_wp_1"})
        assert saved.data["joints_deg"]["base"] == 0.0, saved.data
        print("save_waypoint:", saved.data["name"])

        # Move away, then recall the saved waypoint by name.
        await client.call_tool("move_robot_to_position", {"joint_angles_deg": [20, -90, 0, -90, 0, 0]})
        back = await client.call_tool("move_robot_to_waypoint", {"name": "test_wp_1"})
        assert back.data["status"] == "reached"
        assert back.data["joints_deg"]["base"] == 0.0, back.data
        print("move_robot_to_waypoint: returned to test_wp_1's saved pose")

        listed = await client.call_tool("list_waypoints", {})
        assert "test_wp_1" in listed.data["waypoints"], listed.data
        print("list_waypoints:", list(listed.data["waypoints"].keys()))

        deleted = await client.call_tool("delete_waypoint", {"name": "test_wp_1"})
        assert deleted.data == {"status": "deleted", "name": "test_wp_1"}
        listed_after = await client.call_tool("list_waypoints", {})
        assert listed_after.data == {"waypoints": {}}, listed_after.data
        print("delete_waypoint: removed, confirmed gone from list_waypoints")

        # Validation: recalling or deleting an unknown name must be rejected.
        try:
            await client.call_tool("move_robot_to_waypoint", {"name": "no_such_waypoint"})
        except Exception as exc:
            print("unknown waypoint rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("move to an unknown waypoint was accepted")

        # free_drive: short duration to keep the test quick -- this is a
        # smoke test only (nothing physically pushes the simulated arm, so
        # it's expected to hold roughly still; the point is confirming the
        # call completes and control resumes afterward, not that it moved).
        freed = await client.call_tool("free_drive", {"duration_s": 2.0})
        assert "joints_deg" in freed.data and "tcp_pose" in freed.data
        print("free_drive: completed, resumed normal control at", freed.data["joints_deg"])
        # Confirm normal control really did resume: a move right after works.
        resumed = await client.call_tool("move_robot_to_position", {})
        assert resumed.data["status"] == "reached"
        print("move_robot_to_position after free_drive: ok, normal control confirmed resumed")

        # Validation: a non-positive duration must be rejected.
        try:
            await client.call_tool("free_drive", {"duration_s": 0})
        except Exception as exc:
            print("bad free_drive duration rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("non-positive free_drive duration was accepted")

        # PROGRAMS: a named, ordered sequence of saved waypoints, run
        # non-blocking via the same queue as move_robot_queued.
        await client.call_tool("move_robot_to_position", {})  # known start: home
        await client.call_tool("save_waypoint", {"name": "prog_wp_a"})
        await client.call_tool("move_robot_to_position", {"joint_angles_deg": [20, -90, 0, -90, 0, 0]})
        await client.call_tool("save_waypoint", {"name": "prog_wp_b"})

        prog = await client.call_tool(
            "program_new", {"name": "test_prog", "waypoint_names": ["prog_wp_a", "prog_wp_b"]})
        assert prog.data == {"name": "test_prog", "waypoint_names": ["prog_wp_a", "prog_wp_b"]}
        print("program_new: test_prog = [prog_wp_a, prog_wp_b]")

        listed_progs = await client.call_tool("list_programs", {})
        assert "test_prog" in listed_progs.data["programs"], listed_progs.data
        print("list_programs:", list(listed_progs.data["programs"].keys()))

        started = await client.call_tool("program_start", {"name": "test_prog"})
        assert started.data["status"] == "accepted", started.data
        print(f"program_start: accepted, ETA {started.data['estimated_duration_s']}s")

        for _ in range(100):
            qs = await client.call_tool("get_queue", {})
            if qs.data["current"] is None and not qs.data["pending"]:
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("program_start's queue entry never finished")
        after_prog = await client.call_tool("get_robot_state", {})
        assert after_prog.data["joints_deg"]["base"] == 20.0, after_prog.data  # ended at prog_wp_b
        print("program_start: ran both waypoints, ended at prog_wp_b's pose")

        deleted_prog = await client.call_tool("program_delete", {"name": "test_prog"})
        assert deleted_prog.data == {"status": "deleted", "name": "test_prog"}
        for wp in ("prog_wp_a", "prog_wp_b"):
            await client.call_tool("delete_waypoint", {"name": wp})
        print("program_delete: cleaned up test_prog and its waypoints")

        try:
            await client.call_tool("program_new", {"name": "x", "waypoint_names": ["no_such_wp"]})
        except Exception as exc:
            print("program with unknown waypoint rejected:", str(exc).splitlines()[-1].strip())
        else:
            raise AssertionError("program_new with an unknown waypoint was accepted")

        await client.call_tool("move_robot_to_position", {})  # park home

        # VISION PASSTHROUGH + TRACK: needs vision_human_track actually
        # running (docker compose up, or the local venv) with a camera --
        # optional dependency, so skip gracefully rather than failing the
        # whole suite if it's not up.
        try:
            probe = await client.call_tool("get_environment", {})
            vision_available = True
        except Exception as exc:
            vision_available = False
            print(f"vision service unreachable, skipping vision-passthrough tests: {exc}")

        if vision_available:
            print("get_environment:", probe.data)
            vis = await client.call_tool("get_vision", {})
            assert "humans" in vis.data, vis.data
            print(f"get_vision: {len(vis.data['humans'])} human(s), full skeleton/landmark detail present")

            shadow = await client.call_tool("get_environment_shadow", {})
            assert shadow.data["age_s"] is not None, shadow.data
            print(f"get_environment_shadow: age {shadow.data['age_s']}s, "
                  f"vision_error={shadow.data['vision_error']}")

            tracked = await client.call_tool("track", {"duration_s": 3.0, "poll_hz": 2.0})
            assert "updates" in tracked.data
            print(f"track: {tracked.data['updates']} re-aims over 3s, "
                  f"final base={tracked.data['joints_deg']['base']} deg")
            await client.call_tool("move_robot_to_position", {})  # park home

            try:
                await client.call_tool("track", {"duration_s": 0})
            except Exception as exc:
                print("bad track duration rejected:", str(exc).splitlines()[-1].strip())
            else:
                raise AssertionError("non-positive track duration was accepted")

        # PATH CHECKING / OBSTACLE AVOIDANCE: needs move_group (MoveIt)
        # actually launched -- optional dependency, same skip-gracefully
        # pattern as the vision passthrough block above.
        try:
            baseline = await client.call_tool("check_path", {"joint_angles_deg": HOME_DEG})
            moveit_available = True
        except Exception as exc:
            moveit_available = False
            print(f"MoveIt unreachable, skipping path-checking tests: {exc}")

        if moveit_available:
            assert baseline.data["feasible"], baseline.data
            print(f"check_path (home, no obstacle): feasible, {baseline.data['waypoints']} waypoints")

            # A box obstacle placed right where the target joint config's
            # links would be should make that goal infeasible -- proves
            # add_obstacle actually reaches the planner, not just storage.
            added = await client.call_tool("add_obstacle", {
                "obstacle_id": "test_wall", "xyz_m": [0.0, -0.5, 0.5], "size_m": [0.2, 1.5, 1.5],
            })
            assert any(o["id"] == "test_wall" for o in added.data["obstacles"]), added.data
            blocked = await client.call_tool("check_path", {"joint_angles_deg": [90, -90, 0, -90, 0, 0]})
            assert not blocked.data["feasible"], blocked.data
            print("check_path (with obstacle):", blocked.data["reason"])

            removed = await client.call_tool("remove_obstacle", {"obstacle_id": "test_wall"})
            assert not any(o["id"] == "test_wall" for o in removed.data["obstacles"]), removed.data
            cleared = await client.call_tool("check_path", {"joint_angles_deg": [90, -90, 0, -90, 0, 0]})
            assert cleared.data["feasible"], cleared.data
            print("check_path (after remove_obstacle): feasible again")

            try:
                await client.call_tool("check_path", {"joint_angles_deg": [0, 0, 0]})
            except Exception as exc:
                print("bad check_path angle count rejected:", str(exc).splitlines()[-1].strip())
            else:
                raise AssertionError("check_path with wrong angle count was accepted")

        # QUEUE: move_robot_queued / get_queue / stop_robot's queue-cancel
        # behavior -- the team's non-blocking architecture design. Deliberately
        # slow (speed=0.15) so each move takes a few real seconds, giving the
        # in-flight assertions (reject-while-busy, interrupt-while-moving) a
        # comfortable window instead of racing the robot's own motion time.
        SLOW_SPEED, SLOW_ACCEL = 0.15, 0.3

        # Returns near-instantly, well under the move's own ETA -- the
        # actual proof this tool is non-blocking, not just a status label.
        call_start = time.monotonic()
        q1 = await client.call_tool("move_robot_queued", {
            "joint_angles_deg": [15, -90, 0, -90, 0, 0],
            "speed": SLOW_SPEED, "acceleration": SLOW_ACCEL,
        })
        call_elapsed = time.monotonic() - call_start
        assert q1.data["status"] == "accepted", q1.data
        assert call_elapsed < 1.0, (
            f"move_robot_queued blocked for {call_elapsed:.2f}s -- should return immediately")
        assert q1.data["estimated_duration_s"] > call_elapsed
        print(f"move_robot_queued: returned in {call_elapsed:.2f}s "
              f"(estimated move duration {q1.data['estimated_duration_s']:.2f}s) -- non-blocking, confirmed")

        # Default mode="reject": a second request while q1 is still running
        # must be refused, not silently dropped or blocked-until-ready.
        q2 = await client.call_tool("move_robot_queued", {"joint_angles_deg": [0, -90, 0, -90, 0, 0]})
        assert q2.data["status"] == "rejected", q2.data
        print("move_robot_queued (busy, default mode): rejected as expected")

        # mode="queue": appends behind q1 instead of being refused.
        q3 = await client.call_tool("move_robot_queued", {
            "joint_angles_deg": [-15, -90, 0, -90, 0, 0], "mode": "queue",
        })
        assert q3.data["status"] == "queued", q3.data
        assert q3.data["queue_position"] == 1, q3.data
        print("move_robot_queued (mode=queue): queued behind the running command")

        for _ in range(100):
            qs = await client.call_tool("get_queue", {})
            if qs.data["current"] is None and not qs.data["pending"]:
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("q1/q3 queue never drained")
        after_q3 = await client.call_tool("get_robot_state", {})
        assert after_q3.data["joints_deg"]["base"] == -15.0, after_q3.data
        print("get_queue: drained; robot ended at q3's target as expected:", after_q3.data["joints_deg"])

        # mode="override": cancels both a running command and whatever's
        # still queued behind it, then runs the new one immediately.
        qA = await client.call_tool("move_robot_queued", {
            "joint_angles_deg": [15, -90, 0, -90, 0, 0],
            "speed": SLOW_SPEED, "acceleration": SLOW_ACCEL,
        })
        assert qA.data["status"] == "accepted", qA.data
        qB = await client.call_tool("move_robot_queued", {
            "joint_angles_deg": [0, -90, 0, -90, 0, 0], "mode": "queue",
        })
        assert qB.data["status"] == "queued", qB.data
        qC = await client.call_tool("move_robot_queued", {
            "joint_angles_deg": [0, -80, 0, -90, 0, 0], "mode": "override",
        })
        assert qC.data["status"] == "accepted_override", qC.data
        assert set(qC.data["cancelled_command_ids"]) == {qA.data["command_id"], qB.data["command_id"]}, qC.data
        print("move_robot_queued (mode=override): cancelled both the running and queued commands")

        for _ in range(100):
            qs = await client.call_tool("get_queue", {})
            if qs.data["current"] is None and not qs.data["pending"]:
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("qC never finished")
        after_qC = await client.call_tool("get_robot_state", {})
        assert after_qC.data["joints_deg"]["shoulder"] == -80.0, after_qC.data
        print("get_queue: drained after override; robot ended at qC's target:", after_qC.data["joints_deg"])

        # stop_robot genuinely interrupting an in-flight move: submit a slow
        # move, let it actually start, then stop -- unlike the plain
        # (blocking) move tools, this call didn't hold the tool-call loop
        # hostage, so stop_robot can run while the robot is still en route.
        far_target = [30, -90, 0, -90, 0, 0]
        qD = await client.call_tool("move_robot_queued", {
            "joint_angles_deg": far_target, "speed": SLOW_SPEED, "acceleration": SLOW_ACCEL,
        })
        assert qD.data["status"] == "accepted", qD.data
        await asyncio.sleep(0.3)  # let real motion actually begin
        stopped = await client.call_tool("stop_robot", {})
        assert stopped.data["status"] == "stopped"
        assert qD.data["command_id"] in stopped.data["cancelled_command_ids"], stopped.data
        mid_state = await client.call_tool("get_robot_state", {})
        assert mid_state.data["joints_deg"]["base"] != far_target[0], (
            "robot reached the target before stop_robot ran -- didn't actually catch it mid-motion",
            mid_state.data,
        )
        print(f"stop_robot: interrupted move_robot_queued mid-flight, base stopped at "
              f"{mid_state.data['joints_deg']['base']} deg (target was {far_target[0]})")

        # The worker thread's blocking robot.move_joint() call for qD takes a
        # moment to actually unwind after stop_requested was set (it's still
        # polling/reacting to the real stopj) -- wait for get_queue to show
        # idle before treating a further stop_robot call as a clean no-op.
        for _ in range(50):
            qs = await client.call_tool("get_queue", {})
            if qs.data["current"] is None and not qs.data["pending"]:
                break
            await asyncio.sleep(0.1)
        else:
            raise AssertionError("queue never settled after stop_robot interrupted qD")

        # stop_robot smoke test: harmless no-op with nothing moving.
        stopped_idle = await client.call_tool("stop_robot", {})
        assert stopped_idle.data["status"] == "stopped"
        assert stopped_idle.data["cancelled_command_ids"] == []
        print("stop_robot (idle): no-op as expected,", stopped_idle.data["joints_deg"])

        # SYNC_SIM_TO_REAL: only meaningful in shadow mode (UR_REAL_HOST set)
        # -- same skip-gracefully pattern as vision/path-checking above.
        try:
            synced = await client.call_tool("sync_sim_to_real", {})
        except Exception as exc:
            print(f"shadow mode not active, skipping sync_sim_to_real: "
                  f"{str(exc).splitlines()[-1].strip()}")
        else:
            print("sync_sim_to_real:", synced.data)

        await client.call_tool("move_robot_to_position", {})  # park home
    print("ALL PASSED")


if __name__ == "__main__":
    robot.connect()  # fails fast if the simulator is down or the robot is off
    asyncio.run(main())
