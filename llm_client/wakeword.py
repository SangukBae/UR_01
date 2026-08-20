"""Continuous wake-word listening, layered on top of voice.py's per-turn
capture. Closes ../CLAUDE.md's "No true wake-word / continuous background
listening" gap -- voice.py's listen() is armed per call (each chat turn),
not continuously; chat.py's --voice --wake-word mode uses this to wait
silently in the background until the wake word is heard, instead of
starting a real command capture (and printing a prompt) every single turn.

Uses openwakeword (pretrained ONNX models, no training/API key needed) --
default model is "hey_jarvis", one of openwakeword's own bundled pretrained
words. The team's earlier candidate wake word ("Ravel", see
../CLAUDE.md-adjacent meeting notes) would need training a custom
openwakeword model from scratch (its own multi-hour pipeline, synthetic
TTS-based data generation) -- out of scope here. Swap in any other
bundled word (alexa/hey_mycroft/hey_rhasspy/timer/weather) via the
WAKE_WORD env var without any code change; a real custom word is a
separate, later effort.

Reuses voice.py's exact mic transport (parec, 16kHz mono s16le, via
voice._mic_chunks()) so both share the same PulseAudio source -- no
second concurrent open of the microphone.
"""
from __future__ import annotations

import os

import numpy as np

import voice  # reuses _mic_chunks() -- same parec transport, no separate mic open

WAKE_WORD = os.environ.get("WAKE_WORD", "hey_jarvis")
WAKE_THRESHOLD = float(os.environ.get("WAKE_THRESHOLD", "0.5"))
# openwakeword's models expect 80ms (1280-sample) chunks at 16kHz -- 4x
# voice.py's own 20ms CHUNK_MS chunks, so this buffers and re-chunks
# voice.py's stream rather than opening a second parec process at a
# different chunk size.
_OWW_CHUNK_SAMPLES = 1280

_model = None


def _get_model():
    global _model
    if _model is None:
        from openwakeword.model import Model
        from openwakeword.utils import download_models

        download_models(model_names=[WAKE_WORD])
        _model = Model(wakeword_models=[WAKE_WORD], inference_framework="onnx")
    return _model


def wait_for_wake_word(prompt: str | None = None) -> None:
    """Block silently until WAKE_WORD is heard at or above WAKE_THRESHOLD,
    then return. Unlike voice.listen(), doesn't print per-chunk state (this
    can run for an arbitrarily long, unbounded time waiting for the wake
    word, unlike listen()'s bounded MAX_WAIT_S) -- just the one prompt line
    up front, then quiet until it triggers."""
    if prompt is None:
        prompt = f"\U0001F442 Waiting for wake word ({WAKE_WORD.replace('_', ' ')})..."
    print(prompt)
    model = _get_model()
    buffer = np.zeros(0, dtype=np.int16)
    for chunk_bytes in voice._mic_chunks():
        buffer = np.concatenate([buffer, np.frombuffer(chunk_bytes, dtype=np.int16)])
        while len(buffer) >= _OWW_CHUNK_SAMPLES:
            frame, buffer = buffer[:_OWW_CHUNK_SAMPLES], buffer[_OWW_CHUNK_SAMPLES:]
            scores = model.predict(frame)
            if scores[WAKE_WORD] >= WAKE_THRESHOLD:
                model.reset()  # clear internal state so the next wait starts fresh
                return


if __name__ == "__main__":
    print(f"RMW/model check: loading {WAKE_WORD!r}...")
    wait_for_wake_word()
    print("Wake word heard!")
