"""Per-turn latency instrumentation for chat.py.

../CLAUDE.md's Known gaps flagged this as never measured: "no work has gone
into streaming responses, parallel tool calls, or measuring actual
round-trip time." This is the measuring part -- times each coarse-grained
stage of one chat turn (LLM call(s), each MCP tool call) against the team's
<1500ms voice-to-robot budget (see CLAUDE.md's "What the team is actually
building"), and prints a per-turn breakdown so the next optimization has
real numbers to aim at instead of a guess.

Doesn't include listening-for-speech wait time in the total (a human
deciding when to start talking isn't part of the team's latency budget --
CLAUDE.md's target is "voice command to robot reacting"). voice.py separately
exposes LAST_TIMING for the speech-capture/STT split within one listen()
call; chat.py's --voice path prints that alongside this module's turn
report.
"""
from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass, field


@dataclass
class _Span:
    label: str
    seconds: float


@dataclass
class TurnTimer:
    """One instance per chat.py process, reused across turns -- start_turn()
    resets it at the top of each loop iteration."""

    spans: list[_Span] = field(default_factory=list)
    _turn_start: float | None = field(default=None, repr=False)

    def start_turn(self) -> None:
        self.spans = []
        self._turn_start = time.perf_counter()

    @contextmanager
    def measure(self, label: str):
        """Time one stage (an LLM call, a tool call) and record it under
        ``label``. Records even if the stage raised -- a failed tool call's
        time is still real time spent in the turn."""
        t0 = time.perf_counter()
        try:
            yield
        finally:
            self.spans.append(_Span(label, time.perf_counter() - t0))

    def report(self) -> str:
        """Human-readable per-stage + total breakdown for this turn, or ''
        if start_turn() was never called."""
        if self._turn_start is None:
            return ""
        total_s = time.perf_counter() - self._turn_start
        lines = [f"    {s.label}: {s.seconds * 1000:.0f}ms" for s in self.spans]
        lines.append(f"    TOTAL (LLM + tools, this turn): {total_s * 1000:.0f}ms"
                      + ("  ** OVER the team's 1500ms budget **" if total_s > 1.5 else ""))
        return "  [latency]\n" + "\n".join(lines)
