"""Unit test for ShadowClient's sim-first-then-real gating logic.

Deliberately network-free (fake stand-in clients, not real sockets) --
there's no real UR robot reachable to test shadow mode against live, but the
gating logic itself (call order, propagate-vs-swallow, get_state routing,
stop() best-effort-on-both) doesn't need one to verify.

    python3 test_shadow_client.py
"""
from __future__ import annotations

from dataclasses import dataclass, field

from shadow_client import _SYNC_ACCEL_RAD_S2, _SYNC_SPEED_RAD_S, ShadowClient
from ur_client import RobotState


def _state(tag: str) -> RobotState:
    # tcp_pose[0] is tagged so assertions can tell which client answered.
    return RobotState(
        q_rad=[0, 0, 0, 0, 0, 0], qd_rad=[0] * 6,
        tcp_pose=[hash(tag) % 100, 0, 0, 0, 0, 0],
        robot_mode=7, safety_status=1, digital_output_bits=0,
    )


@dataclass
class FakeClient:
    host: str
    fail: bool = False
    calls: list[tuple] = field(default_factory=list)

    def connect(self) -> None:
        self.calls.append(("connect",))

    def get_state(self) -> RobotState:
        return _state(self.host)

    def move_joint(self, *a, **kw) -> RobotState:
        self.calls.append(("move_joint", a, kw))
        if self.fail:
            raise TimeoutError(f"{self.host} did not reach target")
        return _state(self.host)

    def move_linear(self, *a, **kw) -> RobotState:
        self.calls.append(("move_linear", a, kw))
        if self.fail:
            raise TimeoutError(f"{self.host} did not reach target")
        return _state(self.host)

    def set_gripper(self, *a, **kw) -> RobotState:
        self.calls.append(("set_gripper", a, kw))
        if self.fail:
            raise RuntimeError(f"{self.host} not powered on")
        return _state(self.host)

    def move_waypoints(self, *a, **kw):
        self.calls.append(("move_waypoints", a, kw))
        if self.fail:
            raise TimeoutError(f"{self.host} did not reach last waypoint")
        return _state(self.host), [_state(self.host)]

    def free_drive(self, *a, **kw) -> RobotState:
        self.calls.append(("free_drive", a, kw))
        return _state(self.host)

    def stop(self, *a, **kw) -> RobotState:
        self.calls.append(("stop", a, kw))
        if self.fail:
            raise ConnectionError(f"{self.host} unreachable")
        return _state(self.host)


def main() -> None:
    # 1. real=None: behaves exactly like plain sim, no gating at all.
    sim = FakeClient("sim")
    shadow = ShadowClient(sim=sim, real=None)
    shadow.move_joint([0] * 6, 1.0, 1.0)
    assert [c[0] for c in sim.calls] == ["move_joint"]
    assert shadow.get_state().tcp_pose[0] == _state("sim").tcp_pose[0]
    print("real=None passthrough: ok")

    # 2. Sim succeeds -> real gets the identical command.
    sim = FakeClient("sim")
    real = FakeClient("real")
    shadow = ShadowClient(sim=sim, real=real)
    result = shadow.move_joint([1, 2, 3, 4, 5, 6], 0.5, 0.8)
    assert sim.calls == [("move_joint", ([1, 2, 3, 4, 5, 6], 0.5, 0.8), {})]
    assert real.calls == [("move_joint", ([1, 2, 3, 4, 5, 6], 0.5, 0.8), {})]
    assert result.tcp_pose[0] == _state("real").tcp_pose[0]  # real's state is authoritative
    print("sim success -> real replay: ok")

    # 3. Sim fails -> real is never touched.
    sim = FakeClient("sim", fail=True)
    real = FakeClient("real")
    shadow = ShadowClient(sim=sim, real=real)
    try:
        shadow.move_joint([1] * 6, 1.0, 1.0)
        assert False, "expected the sim's TimeoutError to propagate"
    except TimeoutError:
        pass
    assert real.calls == [], "real must never be called when sim fails"
    print("sim failure -> real never called: ok")

    # 4. Sim succeeds, real fails -> a clear combined error, not a silent pass.
    sim = FakeClient("sim")
    real = FakeClient("real", fail=True)
    shadow = ShadowClient(sim=sim, real=real)
    try:
        shadow.move_linear([0.1] * 6, 0.5, 0.5)
        assert False, "expected the real robot's failure to raise"
    except RuntimeError as exc:
        assert "simulator" in str(exc).lower() and "real robot" in str(exc).lower()
    assert len(sim.calls) == 1 and len(real.calls) == 1
    print("sim success, real failure -> raised with context: ok")

    # 5. get_state() routes to the real robot once one is configured.
    sim = FakeClient("sim")
    real = FakeClient("real")
    shadow = ShadowClient(sim=sim, real=real)
    assert shadow.get_state().tcp_pose[0] == _state("real").tcp_pose[0]
    print("get_state() prefers real: ok")

    # 6. stop() is best-effort on BOTH, independently -- one failing doesn't
    #    block the other, and both are always attempted.
    sim = FakeClient("sim", fail=True)
    real = FakeClient("real")
    shadow = ShadowClient(sim=sim, real=real)
    try:
        shadow.stop()
        assert False, "expected the sim-side stop failure to be surfaced"
    except RuntimeError as exc:
        assert "simulator" in str(exc)
    assert real.calls == [("stop", (), {})], "real stop must still have been attempted"
    print("stop() attempts both independently: ok")

    # 7. free_drive() goes straight to the real robot (no sim pre-flight --
    #    it has no target to verify) -- but the sim IS resynced afterward,
    #    since hand-guiding just moved the real robot out from under it.
    sim = FakeClient("sim")
    real = FakeClient("real")
    shadow = ShadowClient(sim=sim, real=real)
    shadow.free_drive(5.0)
    assert [c[0] for c in sim.calls] == ["move_joint"], (
        "sim must not run free_drive itself, only the resync move_joint after"
    )
    assert sim.calls == [
        ("move_joint", (_state("real").q_rad, _SYNC_SPEED_RAD_S, _SYNC_ACCEL_RAD_S2), {})
    ]
    assert real.calls == [("free_drive", (5.0,), {})]
    print("free_drive() -> real only, then sim resynced to real: ok")

    # 8. connect() syncs the sim to the real robot's pose up front (so the
    #    very first verified move already gates on the true starting pose,
    #    not the sim's own boot pose) -- and sync_sim_to_real() can be
    #    called again later on demand, with the same effect.
    sim = FakeClient("sim")
    real = FakeClient("real")
    shadow = ShadowClient(sim=sim, real=real)
    shadow.connect()
    assert real.calls[0] == ("connect",)
    assert sim.calls[-1] == (
        "move_joint", (_state("real").q_rad, _SYNC_SPEED_RAD_S, _SYNC_ACCEL_RAD_S2), {}
    )
    sim.calls.clear()
    result = shadow.sync_sim_to_real()
    assert sim.calls == [
        ("move_joint", (_state("real").q_rad, _SYNC_SPEED_RAD_S, _SYNC_ACCEL_RAD_S2), {})
    ]
    assert result.tcp_pose[0] == _state("sim").tcp_pose[0]
    print("connect() syncs sim to real up front, sync_sim_to_real() repeats it: ok")

    # 9. sync_sim_to_real() is a real=None no-op (just reports sim's state).
    sim = FakeClient("sim")
    shadow = ShadowClient(sim=sim, real=None)
    result = shadow.sync_sim_to_real()
    assert sim.calls == [], "no real robot configured, nothing to sync to"
    assert result.tcp_pose[0] == _state("sim").tcp_pose[0]
    print("sync_sim_to_real() with real=None: no-op: ok")

    print("\nAll ShadowClient gating tests passed.")


if __name__ == "__main__":
    main()
