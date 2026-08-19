"""Voice output: speak a short phrase through the default PulseAudio sink.

Team requirement: minimal spoken feedback ("Yes", "There was an issue",
"Cannot execute") instead of reading long text off a screen you're not
looking at. This module is just the mechanism (text -> audio out); keeping
the *text* itself terse is chat.py's job (its voice-mode system prompt).

Uses ``espeak-ng`` (already used for a TTS smoke test earlier in this
project) rendered to a temp WAV, then played with ``paplay`` -- not
espeak-ng's own direct playback, which guesses an audio backend (ALSA/OSS)
that doesn't exist in this sandbox (no ``/dev/snd``); routing through
``paplay`` explicitly targets the same PulseAudio bridge ``voice.py``
already captures from (WSLg's ``RDPSink``/``RDPSource``), so it's the
mechanism verified to actually reach the Windows host.
"""
from __future__ import annotations

import os
import subprocess
import tempfile

VOICE = os.environ.get("TTS_VOICE", "en")
SPEED_WPM = int(os.environ.get("TTS_SPEED_WPM", "175"))


def speak(text: str) -> None:
    """Synthesize ``text`` and play it. Best-effort: a TTS failure (missing
    espeak-ng/paplay, no audio sink) prints a warning instead of raising,
    since losing voice feedback shouldn't crash the chat loop."""
    text = text.strip()
    if not text:
        return
    wav_path = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            wav_path = f.name
        subprocess.run(
            ["espeak-ng", "-v", VOICE, "-s", str(SPEED_WPM), "-w", wav_path, text],
            check=True, capture_output=True, timeout=10,
        )
        subprocess.run(["paplay", wav_path], check=True, capture_output=True, timeout=10)
    except Exception as exc:  # noqa: BLE001 -- best-effort, never fatal
        print(f"  (TTS unavailable: {exc})")
    finally:
        if wav_path is not None:
            try:
                os.unlink(wav_path)
            except OSError:
                pass
