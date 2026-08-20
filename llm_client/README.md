# LLM Client

Standalone chat wrapper (no GUI app, unlike the course's `llm-client/`
Bionic/OpenClaw guides): natural language in, LLM picks a `case1/server.py`
tool, robot moves. Reimplements the team's Bronze "LLM wrapper" (Nina,
Cerebras) against this repo's own server (all four tiers, not just Bronze,
plus `stop_robot` -- say "Stop" and it halts in-progress motion).

## Setup

```bash
pip install -r requirements.txt
export CEREBRAS_API_KEY="your-key-here"   # never in a file in this repo
```

If using the team's shared key: env var only, never `git add`ed.

## Run

The simulator must be up and the robot powered on (see `../case1/README.md`
Setup section).

```bash
cd llm_client
python3 chat.py
```

```
Connected: 8 tools from ur-tools, model gpt-oss-120b @ https://api.cerebras.ai/v1
Type a message (e.g. 'move the robot home'). Ctrl-C to quit.

You: move the robot home
  -> move_robot_to_position({})
Robot: Done -- the robot is now at its home position...
```

## Voice input

Speak instead of typing:

```bash
python3 chat.py --voice
```

Starts listening immediately each turn, waits for you to start talking,
records until you stop, transcribes locally (no API key), prints what it
heard (STT can mishear), sends it same as typed input:

```
Connected: 8 tools from ur-tools, model gpt-oss-120b @ https://api.cerebras.ai/v1
Speaking mode -- each turn, just start talking. Ctrl-C to quit.

🎤 Listening -- speak your command...
You (heard): move the robot home
  -> move_robot_to_position({})
Robot: Done.
```

Voice mode replies are kept short -- a separate, terser system prompt
("Yes.", "Done.", "There was an issue.") -- and spoken aloud through
`tts.py` (`espeak-ng` -> `paplay`, the same PulseAudio bridge as the mic).
Bare interjections ("um", "uh"...) are dropped before ever reaching the
LLM (`voice._is_filler`); a real short command ("Stop") is never at risk,
only an utterance that's *exactly* one filler word gets swallowed.

**Verified live** over WSL2/WSLg's PulseAudio mic bridge (`RDPSource`,
`$PULSE_SERVER` already set, no config needed) -- "move the robot home"
spoken → captured → transcribed → `move_robot_to_position` called
correctly. Works on any Linux with a PulseAudio default source; never
hardcodes `RDPSource`.

```bash
sudo apt install -y pulseaudio-utils espeak-ng   # parec (input) + espeak-ng (output)
pactl list short sources                          # confirm a real default source
```

By default no wake word -- armed on each `listen()` call, triggers on
energy threshold. Env var tuning (see `voice.py`): `VOICE_START_RMS` (mic
sensitivity; `VOICE_DEBUG=1` to watch live RMS), `VOICE_SILENCE_HANG_S`
(cuts off mid-sentence? raise it), `VOICE_MAX_WAIT_S`/`VOICE_MAX_UTTERANCE_S`
(give-up caps), `WHISPER_MODEL` (`base.en` default; `small.en` more
accurate). Stray noise can hallucinate a phrase (Whisper quirk) -- raise
`VOICE_START_RMS` if that's frequent.

### Wake word (optional)

```bash
python3 chat.py --voice --wake-word
```

Instead of listening for a command immediately every turn, waits silently
in the background for a wake word first (`wakeword.py`, pretrained
[openwakeword](https://github.com/dscripka/openWakeWord) ONNX models, no
training or API key needed) -- default "hey jarvis" (openwakeword's own
bundled word; swap in `alexa`/`hey_mycroft`/`hey_rhasspy`/`timer`/`weather`
via `WAKE_WORD=...`, or tune sensitivity with `WAKE_THRESHOLD=...`, default
`0.5`). The team's earlier candidate wake word ("Ravel") isn't one of
openwakeword's bundled words -- would need training a custom model from
scratch, a separate effort, out of scope here.

```
Speaking mode -- say the wake word, then your command. Ctrl-C to quit.

👂 Waiting for wake word (hey jarvis)...
🎤 Listening -- speak your command...
You (heard): move the robot home
  -> move_robot_to_position({})
Robot: Done.
```

Shares `voice.py`'s exact mic transport (same `parec` process pattern, same
PulseAudio source) -- no second concurrent mic open. Verified live against
the real WSLg-bridged mic for a sustained wait with no crash; not verified
against an actual spoken "hey jarvis" in this sandbox (no way to speak into
the mic here) -- the pretrained model itself is openwakeword's own, not
retrained or otherwise modified by this repo.

**Known gap: not listening while a move is in progress.** Each turn blocks
until its tool call returns, so saying "Stop" *while* the robot is mid-motion
doesn't reach it -- the mic isn't even recording at that moment (`stop_robot`
exists and works, see `case1/server.py`, but only between turns or from a
second concurrent client). Fixing this needs running tool execution and
listening concurrently, not built here.

## Latency

Every turn prints a breakdown after the reply (`latency.py`'s `TurnTimer`):
each LLM call, each MCP tool call, and a total against the team's <1500ms
voice-to-robot budget (flagged over if it exceeds that). `--voice` adds a
line above it for the speech-capture/STT split (record + transcribe time;
silent wait-for-speech time is excluded from the budget -- deciding when to
talk isn't part of it):

```
You: move the robot home
  -> move_robot_to_position({})
Robot: Done.
  [stt] record: 1120ms, transcribe: 340ms  (wait-for-speech 890ms excluded from budget)
  [latency]
    llm_call_0: 610ms
    tool:move_robot_to_position: 2150ms
    TOTAL (LLM + tools, this turn): 2760ms  ** OVER the team's 1500ms budget **
```

Not optimized yet, just measured -- see `../CLAUDE.md`'s Known gaps for
what a real number from this points at next (streaming, parallel tool
calls, or a faster move for the actual robot-motion tools).

## Other endpoints

Default is Cerebras (`base_url=https://api.cerebras.ai/v1`,
`model=gpt-oss-120b`, matching what the team confirmed working). Point it at
any other OpenAI-compatible endpoint instead:

```bash
export LLM_BASE_URL="https://integrate.api.nvidia.com/v1"   # e.g. NVIDIA NIM
export LLM_MODEL="z-ai/glm-5.2"
export LLM_API_KEY="nvapi-..."                                # not CEREBRAS_API_KEY
python3 chat.py
```

(The API-key env var name follows the base URL: `CEREBRAS_API_KEY` when
`LLM_BASE_URL` contains "cerebras", `LLM_API_KEY` otherwise.)

## How it works

Connects to `case1/server.py` in-process (like `test_server.py`), converts
its tools to OpenAI's `tools` schema. Each turn: message + tool list → model;
tool call → executed against the real MCP server → JSON result fed back as
a `tool` message; repeats until plain text. Model-agnostic: any tool-calling
model behind an OpenAI-compatible endpoint works.
