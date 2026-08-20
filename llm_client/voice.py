"""Microphone input: record a spoken command, transcribe it, return text.

No wake word: call ``listen()``, it starts listening immediately, waits
(quietly) for you to start talking, records until you stop (or hits a time
cap), and returns the transcribed text -- an energy threshold is the
trigger, not a keyword. A real wake-word engine (Porcupine, openWakeWord)
would let this run continuously in the background instead of one
listen()-per-turn; that's its own model/dependency this doesn't pull in.

Capture: PulseAudio's ``parec`` (not ``parecord`` -- see below), streamed
into a small energy-based VAD (start on loud enough, stop after a hang of
quiet). Works wherever ``pactl info`` reports a real default source --
including WSLg, which bridges the Windows host's microphone in as PulseAudio
source ``RDPSource`` (the ``$PULSE_SERVER`` socket at
``/mnt/wslg/PulseServer`` -- already set by WSLg, nothing to configure).

Transcription: a local Whisper model via ``faster-whisper`` (CPU, no GPU
needed, no extra API key -- unlike the chat LLM, there's no speech
counterpart to the team's Cerebras key).

Why ``parec`` and not ``parecord --raw -``: reproduced in this sandbox --
``parecord`` with raw output to stdout (or any non-regular-file redirect)
never flushes, so a subprocess pipe reads 0 bytes no matter how long you
wait, even though the identical recording to an actual file on disk works
fine. ``parec`` (the lower-level tool, always raw, no file-format handling)
streams to a pipe correctly. If your ``listen()`` call hangs immediately
producing nothing, this is the first thing to suspect if you've swapped it
back to ``parecord``.
"""
from __future__ import annotations

import os
import subprocess
import time
from collections.abc import Iterator

import numpy as np

SAMPLE_RATE = 16000
CHUNK_MS = 20
CHUNK_SAMPLES = SAMPLE_RATE * CHUNK_MS // 1000
CHUNK_BYTES = CHUNK_SAMPLES * 2  # s16le = 2 bytes/sample

# Tune these if it cuts you off mid-sentence (raise SILENCE_HANG_S) or
# waits too long after you stop (lower it), or if it won't trigger on your
# mic (lower START_RMS) or triggers on room noise (raise it). Run with
# VOICE_DEBUG=1 to print live RMS values and calibrate against your mic.
START_RMS = float(os.environ.get("VOICE_START_RMS", "500"))
SILENCE_RMS = float(os.environ.get("VOICE_SILENCE_RMS", "300"))
SILENCE_HANG_S = float(os.environ.get("VOICE_SILENCE_HANG_S", "1.0"))
MAX_WAIT_S = float(os.environ.get("VOICE_MAX_WAIT_S", "8.0"))
MAX_UTTERANCE_S = float(os.environ.get("VOICE_MAX_UTTERANCE_S", "20.0"))
PREROLL_S = float(os.environ.get("VOICE_PREROLL_S", "0.3"))
DEBUG = os.environ.get("VOICE_DEBUG") == "1"

WHISPER_MODEL = os.environ.get("WHISPER_MODEL", "base.en")

_model = None

# Filled in by listen() every call -- read this right after listen() returns
# for a wait/record/transcribe breakdown of where the time went. wait_s
# (silent, waiting for you to start talking) is NOT part of the team's
# voice-to-robot latency budget (see ../CLAUDE.md); record_s + transcribe_s
# are the real STT contribution to it, reported alongside chat.py's own
# latency.py turn timing.
LAST_TIMING: dict[str, float | None] = {"wait_s": None, "record_s": None, "transcribe_s": None}


def _mic_chunks() -> Iterator[bytes]:
    """Yield raw s16le mono 16kHz chunks from the default PulseAudio source."""
    proc = subprocess.Popen(
        ["parec", "--channels=1", "--rate=16000", "--format=s16le"],
        stdout=subprocess.PIPE,
    )
    try:
        while True:
            data = proc.stdout.read(CHUNK_BYTES)
            if not data:
                return
            yield data
    finally:
        proc.terminate()
        proc.wait(timeout=2)


def _collect_utterance(
    chunks: Iterator[bytes], now_fn=time.monotonic,
) -> tuple[bytes | None, float | None]:
    """State machine: wait for speech to start, record until it stops (or a
    time cap hits). Pulled out of _mic_chunks so it's testable against a
    synthetic chunk sequence, without a real microphone -- ``now_fn`` is
    injectable for exactly that: a real mic paces chunks in real time (each
    ``parec`` read blocks until that much audio actually exists), so
    ``time.monotonic()`` is correct there, but a synthetic test generator
    yields chunks instantly, which would never trip a real-time-based check.
    A fake clock advanced by CHUNK_MS per chunk exercises the timers
    deterministically instead.

    Returns ``(audio, speech_started_at)`` -- ``audio`` is the recorded
    audio (concatenated raw s16le bytes), or None if nothing loud enough
    started within MAX_WAIT_S; ``speech_started_at`` is the ``now_fn()``
    timestamp speech was first detected (None alongside a None audio),
    for listen()'s wait-vs-record latency split.
    """
    collected: list[bytes] = []
    # A word's onset ramps up gradually, so waiting for RMS to cross
    # START_RMS clips the first syllable or two (reproduced: a live "move
    # the robot home" came back transcribed as "the robot home" -- "move"
    # eaten by exactly this). Keep a rolling pre-roll of recent chunks and
    # splice it in the moment speech is detected, so recording effectively
    # starts a bit before the threshold trip, not at it.
    preroll: list[bytes] = []
    preroll_chunks = int(PREROLL_S * 1000 / CHUNK_MS)
    speaking = False
    speech_started_at: float | None = None
    silence_start: float | None = None
    t_start = now_fn()

    for data in chunks:
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float64)
        rms = float(np.sqrt(np.mean(samples ** 2))) if len(samples) else 0.0
        now = now_fn()
        if DEBUG:
            print(f"  [voice] rms={rms:.0f} speaking={speaking}", end="\r")

        if not speaking:
            if rms >= START_RMS:
                speaking = True
                speech_started_at = now
                collected.extend(preroll)
                collected.append(data)
            else:
                preroll.append(data)
                if len(preroll) > preroll_chunks:
                    preroll.pop(0)
                if now - t_start >= MAX_WAIT_S:
                    return None, None
            continue

        collected.append(data)
        if rms < SILENCE_RMS:
            if silence_start is None:
                silence_start = now
            elif now - silence_start >= SILENCE_HANG_S:
                break
        else:
            silence_start = None

        if now - t_start >= MAX_UTTERANCE_S:
            break

    return (b"".join(collected) if collected else None), speech_started_at


def _get_model():
    global _model
    if _model is None:
        from faster_whisper import WhisperModel
        _model = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8")
    return _model


def transcribe(raw_audio: bytes) -> str | None:
    """Raw s16le mono 16kHz bytes -> transcribed text, or None if empty."""
    samples = np.frombuffer(raw_audio, dtype=np.int16).astype(np.float32) / 32768.0
    model = _get_model()
    segments, _ = model.transcribe(samples, vad_filter=True)
    text = " ".join(s.text for s in segments).strip()
    return text or None


# Bare interjections to drop before they ever reach the LLM (team
# requirement: ignore non-command chatter). Deliberately narrow -- this
# only fires when the ENTIRE utterance is exactly one of these words, so a
# real short command ("Stop", "Wait", "Go") is never at risk of being
# swallowed by it.
_FILLER_WORDS = {
    "uh", "um", "umm", "uhh", "hmm", "hm", "mm", "mhm", "ah", "eh", "huh", "oh",
}


def _is_filler(text: str) -> bool:
    words = text.strip().lower().rstrip(".!?,").split()
    return len(words) == 1 and words[0] in _FILLER_WORDS


def listen(prompt: str = "Listening... speak now.") -> str | None:
    """Record one spoken command from the microphone and transcribe it.
    Prints ``prompt``, then waits for you to start talking and records
    until you stop. Returns None if nothing was heard within MAX_WAIT_S,
    nothing intelligible was transcribed, or the whole utterance was a bare
    filler word ("um", "uh", ...) -- see ``_is_filler``.

    Sets LAST_TIMING (wait_s/record_s/transcribe_s) before returning, even
    on a None return, so a caller doing latency accounting always has
    something to read -- wait_s stays None only when speech never started
    at all (nothing to charge record/transcribe time against either)."""
    print(prompt)
    t_listen_start = time.monotonic()
    audio, speech_started_at = _collect_utterance(_mic_chunks())
    t_after_record = time.monotonic()
    if DEBUG:
        print()
    if audio is None:
        LAST_TIMING.update(wait_s=None, record_s=None, transcribe_s=None)
        return None
    LAST_TIMING["wait_s"] = speech_started_at - t_listen_start
    LAST_TIMING["record_s"] = t_after_record - speech_started_at
    t_transcribe_start = time.monotonic()
    text = transcribe(audio)
    LAST_TIMING["transcribe_s"] = time.monotonic() - t_transcribe_start
    if text is not None and _is_filler(text):
        return None
    return text
