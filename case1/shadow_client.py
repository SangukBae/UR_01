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

Pose sync: the pre-flight gate above is only meaningful if the simulator's
CURRENT pose actually matches the real robot's -- otherwise "verified safe
in sim" describes a swept path (singularities, joint limits, self-collision
along the way) the real robot, starting somewhere else entirely, will never
actually take. There's no guarantee the two start aligned: the simulator
boots at its own home pose regardless of wherever the physical arm happens
to be sitting (powered on mid-lesson, jogged by hand, left from a previous
session, etc.). ``connect()`` therefore reads the real robot's actual joint
angles and drives the simulator to match BEFORE anything is verified --
never the other way around, since moving the simulator is free and moving
the real robot without a verified-safe target is exactly what shadow mode
exists to prevent. ``sync_sim_to_real()`` re-does this on demand, for any
later point where the two can drift apart again -- most notably after
``free_drive()``, which moves only the real robot by hand and has no target
to verify, so the sim is never told where the arm ended up.
"""
from __future__ import annotations

from dataclasses import dataclass

from ur_client import RobotState, URClient

# Fast but not instant -- this only ever moves the simulator (never the real
# robot), so there's no safety reason to go slow, just readability of the
# resulting demo/log output.
_SYNC_SPEED_RAD_S = 1.5
_SYNC_ACCEL_RAD_S2 = 2.0


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
            self.sync_sim_to_real()

    def sync_sim_to_real(
        self,
        speed: float = _SYNC_SPEED_RAD_S,
        acceleration: float = _SYNC_ACCEL_RAD_S2,
    ) -> RobotState:
        """Drive the simulator to the real robot's CURRENT joint angles.

        Call this any time the two may have drifted apart -- most notably
        after ``free_drive()`` (which only moves the real robot). Never the
        reverse: the real robot is never moved to match the simulator, since
        that would be exactly the unverified real-robot motion shadow mode
        exists to prevent.

        Returns the simulator's resulting state. A no-op (returns the plain
        sim state) if no real robot is configured.
        """
        if self.real is None:
            return self.sim.get_state()
        real_q = self.real.get_state().q_rad
        return self.sim.move_joint(real_q, speed, acceleration)

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
        state = (self.real or self.sim).free_drive(*args, **kwargs)
        if self.real is not None:
            # Hand-guiding only moved the real robot -- resync the simulator
            # so the next verified move gates on the pose the arm actually
            # ended up in, not wherever it was before free-drive started.
            self.sync_sim_to_real()
        return state

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
