"""Non-blocking command queue for MCP move tools.

From the team's first architecture meeting: a move request should return
immediately with accept/reject + an ETA, not block until the robot arrives.
The actual motion runs on a background worker thread here. That alone is
what makes a real mid-motion stop_robot() possible -- previously (see
CLAUDE.md's known gaps) a move tool call didn't return control until the
robot had already arrived, so nothing could interrupt it. With the move
itself non-blocking, the MCP server can accept a stop_robot() call while a
previous move is still in flight.

Queue semantics agreed in the meeting: a new command is rejected by default
while one is already running/queued; mode="queue" appends it; mode="override"
drops the whole queue (and cancels whatever's currently running) and runs
the new command immediately.
"""
from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class Command:
    id: str
    kind: str
    fn: Callable[[], dict]
    meta: dict
    status: str = "pending"  # pending -> running -> done/failed/cancelled
    error: str | None = None
    result: dict | None = None
    submitted_at: float = field(default_factory=time.time)
    started_at: float | None = None
    finished_at: float | None = None

    def describe(self) -> dict:
        return {
            "id": self.id,
            "kind": self.kind,
            "meta": self.meta,
            "status": self.status,
            "error": self.error,
        }


class CommandQueue:
    """One background worker thread executes commands one at a time, pulled
    from a plain list guarded by a lock. Deliberately not a fancier
    priority/multi-queue design -- the meeting explicitly agreed a single
    reject/queue/override queue is "already quite nice as a feature" and
    more complex queue management wasn't needed.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._pending: list[Command] = []
        self._current: Command | None = None
        self._wake = threading.Event()
        self.stop_requested = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def submit(self, kind: str, fn: Callable[[], dict], meta: dict, mode: str = "reject") -> dict:
        if mode not in ("reject", "queue", "override"):
            raise ValueError(f"mode must be 'reject', 'queue', or 'override', got {mode!r}.")

        with self._lock:
            busy = self._current is not None or bool(self._pending)

            if busy and mode == "reject":
                return {
                    "status": "rejected",
                    "reason": (
                        "A command is already running or queued. Pass "
                        "mode='queue' to run this after it, or mode='override' "
                        "to cancel everything queued/running and do this instead."
                    ),
                    "queue": self._describe_locked(),
                }

            cmd = Command(id=str(uuid.uuid4())[:8], kind=kind, fn=fn, meta=meta)

            if mode == "override":
                cancelled_ids = [c.id for c in self._pending]
                self._pending.clear()
                if self._current is not None:
                    cancelled_ids.append(self._current.id)
                    self.stop_requested.set()
                self._pending.append(cmd)
                response = {
                    "status": "accepted_override",
                    "command_id": cmd.id,
                    "cancelled_command_ids": cancelled_ids,
                }
            else:
                self._pending.append(cmd)
                response = {
                    "status": "queued" if busy else "accepted",
                    "command_id": cmd.id,
                    "queue_position": len(self._pending),
                }

            self._wake.set()
            return {**response, "queue": self._describe_locked()}

    def cancel_all(self) -> list[str]:
        """Drop every pending command and flag the running one cancelled.
        Does NOT itself tell the robot to stop moving -- stop_robot() in
        server.py still calls robot.stop() separately for that; this only
        stops the *queue* from continuing to the next command."""
        with self._lock:
            cancelled_ids = [c.id for c in self._pending]
            self._pending.clear()
            if self._current is not None:
                cancelled_ids.append(self._current.id)
                self.stop_requested.set()
            return cancelled_ids

    def describe(self) -> dict:
        with self._lock:
            return self._describe_locked()

    def _describe_locked(self) -> dict:
        return {
            "current": None if self._current is None else self._current.describe(),
            "pending": [c.describe() for c in self._pending],
            "queue_length": len(self._pending),
        }

    def _run(self) -> None:
        while True:
            self._wake.wait()
            with self._lock:
                if not self._pending:
                    self._wake.clear()
                    continue
                cmd = self._pending.pop(0)
                self._current = cmd
                self.stop_requested.clear()

            cmd.status = "running"
            cmd.started_at = time.time()

            # cmd.fn() (e.g. ur_client.move_joint) blocks until the robot
            # physically reaches its target -- or, if stopped short, until
            # ITS OWN ~20s poll timeout, since it has no cancellation hook.
            # Run it on a throwaway thread so a cancellation can free the
            # queue immediately instead of waiting out that timeout; the
            # stale thread is left to finish on its own in the background
            # (findings below just aren't surfaced as this command's result
            # anymore).
            done = threading.Event()

            def _call(cmd=cmd, done=done):
                try:
                    result = cmd.fn()
                    if cmd.status != "cancelled":
                        cmd.result = result
                        cmd.status = "done"
                except Exception as exc:  # noqa: BLE001 - surface any backend error into the queue record
                    if cmd.status != "cancelled":
                        cmd.status = "failed"
                        cmd.error = str(exc)
                finally:
                    cmd.finished_at = time.time()
                    done.set()

            threading.Thread(target=_call, daemon=True).start()

            while not done.is_set():
                if self.stop_requested.wait(timeout=0.1) and not done.is_set():
                    cmd.status = "cancelled"
                    cmd.finished_at = time.time()
                    break

            with self._lock:
                self._current = None
                if not self._pending:
                    self._wake.clear()
