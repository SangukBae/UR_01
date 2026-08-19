# LLM Client

Standalone chat wrapper (no GUI app, unlike the course's `llm-client/`
Bionic/OpenClaw guides): natural language in, LLM picks a `case1/server.py`
tool, robot moves. Reimplements the team's Bronze "LLM wrapper" (Nina,
Cerebras) against this repo's own server (all four tiers, not just Bronze).

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
Connected: 7 tools from ur-tools, model gpt-oss-120b @ https://api.cerebras.ai/v1
Type a message (e.g. 'move the robot home'), Ctrl-C to quit.

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
Connected: 7 tools from ur-tools, model gpt-oss-120b @ https://api.cerebras.ai/v1
🎤 Listening -- speak your command...
You (heard): move the robot home
  -> move_robot_to_position({})
Robot: Done -- the robot is now at its home position...
```

**Verified live** over WSL2/WSLg's PulseAudio mic bridge (`RDPSource`,
`$PULSE_SERVER` already set, no config needed) -- "move the robot home"
spoken → captured → transcribed → `move_robot_to_position` called
correctly. Works on any Linux with a PulseAudio default source; never
hardcodes `RDPSource`.

```bash
sudo apt install -y pulseaudio-utils   # parec, not parecord -- see voice.py
pactl list short sources               # confirm a real default source
```

No wake word -- armed on each `listen()` call, triggers on energy
threshold. Env var tuning (see `voice.py`): `VOICE_START_RMS` (mic
sensitivity; `VOICE_DEBUG=1` to watch live RMS), `VOICE_SILENCE_HANG_S`
(cuts off mid-sentence? raise it), `VOICE_MAX_WAIT_S`/`VOICE_MAX_UTTERANCE_S`
(give-up caps), `WHISPER_MODEL` (`base.en` default; `small.en` more
accurate). Stray noise can hallucinate a phrase (Whisper quirk) -- raise
`VOICE_START_RMS` if that's frequent.

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
