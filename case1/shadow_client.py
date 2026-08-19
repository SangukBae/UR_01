"""Shadow-execution wrapper: verify every move in the simulator first, only
replay it on the real robot if the simulator move actually succeeded.

Opt-in only -- see server.py's backend selection. With no ``UR_REAL_HOST``
set, nothing in this file is ever touched and behavior is 100% unchanged
from a plain ``URClient``. Set ``UR_REAL_HOST`` to a real UR controller's IP
to enable shadow mode.

Design (chosen over mirroring to both at once): a target-reaching move
(``move_joint``/``move_linear``/``move_waypoints``/``set_gripper``) always
runs on the simulator FIRST and blocks until it either reaches its target or
raises. Only on success does the identical command go to the real robot.
This makes the simulator a genuine pre-flight gate -- a move that would trip
a protective stop, hit a joint limit, or time out gets caught in sim and
never reaches the physical arm. Mirroring both simultaneously was rejected:
if the simulator result arrives too late or differs, the real robot would
already be moving on an unverified command.

``stop()`` is the deliberate exception: it is sent to *both* robots
independently, each in its own try/except, regardless of whether the other
succeeded -- a stop must never be gated behind another command's outcome.

``get_state()`` (and therefore every read-only tool, e.g.
``get_robot_state``) reports the REAL robot once one is configured -- the
simulator here exists to gate motion, not to be what operators are told the
robot is doing. ``free_drive()`` also goes straight to the real robot
(falling back to sim only if no real robot is configured): it has no target
to verify against, so running it in sim first would just be a pointless
wait before the human ever gets to hand-guide the physical arm.
"""
from __future__ import annotations

from dataclasses import dataclass

from ur_client import RobotState, URClient


@dataclass
class ShadowClient:
    """Same call surface as ``URClient`` (``connect``, ``get_state``,
    ``move_joint``, ``move_linear``, ``move_waypoints``, ``set_gripper``,
    ``free_drive``, ``stop``) so ``server.py``'s tools need no changes.

    Args:
        sim: Client for the simulator -- always the pre-flight check.
        real: Client for the physical robot, or ``None`` to behave exactly
            like plain ``sim`` (no shadow behavior at all).
    """

    sim: URClient
    real: URClient | None

    def connect(self) -> None:
        self.sim.connect()
        if self.real is not None:
            self.real.connect()

    def get_state(self) -> RobotState:
        return (self.real or self.sim).get_state()

    def _verify_then_replay(self, method: str, *args, **kwargs) -> RobotState:
        sim_state = getattr(self.sim, method)(*args, **kwargs)
        if self.real is None:
            return sim_state
        try:
            return getattr(self.real, method)(*args, **kwargs)
        except Exception as exc:
            raise RuntimeError(
                f"Simulator {method}() succeeded, so the real robot at "
                f"{self.real.host} was sent the same command -- but IT "
                f"failed: {exc}. The simulator has already moved; the real "
                "robot may not have (check its state before retrying)."
            ) from exc

    def move_joint(self, *args, **kwargs) -> RobotState:
        return self._verify_then_replay("move_joint", *args, **kwargs)

    def move_linear(self, *args, **kwargs) -> RobotState:
        return self._verify_then_replay("move_linear", *args, **kwargs)

    def set_gripper(self, *args, **kwargs) -> RobotState:
        return self._verify_then_replay("set_gripper", *args, **kwargs)

    def move_waypoints(self, *args, **kwargs) -> tuple[RobotState, list[RobotState]]:
        # Live progress (on_state) streams off the simulator pass only --
        # that's the trial run; the real pass that follows is already
        # verified motion, not something to narrate a second time.
        sim_state, trace = self.sim.move_waypoints(*args, **kwargs)
        if self.real is None:
            return sim_state, trace
        real_kwargs = {k: v for k, v in kwargs.items() if k != "on_state"}
        try:
            real_state, _real_trace = self.real.move_waypoints(*args, **real_kwargs)
        except Exception as exc:
            raise RuntimeError(
                f"Simulator move_waypoints() succeeded, so the real robot "
                f"at {self.real.host} was sent the same path -- but IT "
                f"failed: {exc}. The simulator has already moved; the real "
                "robot may not have (check its state before retrying)."
            ) from exc
        return real_state, trace

    def free_drive(self, *args, **kwargs) -> RobotState:
        return (self.real or self.sim).free_drive(*args, **kwargs)

    def stop(self, *args, **kwargs) -> RobotState:
        """Best-effort stop on BOTH robots, independently -- never let one
        side's failure (or the other side not existing) block the other."""
        errors: list[str] = []
        sim_state: RobotState | None = None
        real_state: RobotState | None = None
        try:
            sim_state = self.sim.stop(*args, **kwargs)
        except Exception as exc:  # noqa: BLE001 - collect, don't let it block the real stop below
            errors.append(f"simulator: {exc}")
        if self.real is not None:
            try:
                real_state = self.real.stop(*args, **kwargs)
            except Exception as exc:  # noqa: BLE001 - report, but the sim stop above already ran
                errors.append(f"real robot ({self.real.host}): {exc}")
        if errors:
            raise RuntimeError("stop() failed on: " + "; ".join(errors))
        return real_state if real_state is not None else sim_state
