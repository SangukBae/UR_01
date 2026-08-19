# LLM Client

A minimal chat wrapper: type natural language, an LLM decides which
`case1/server.py` tool to call, the robot moves, the result goes back to the
LLM. This is the team's Bronze-tier "LLM wrapper" deliverable (originally
built by Nina Muller against the Cerebras API, confirmed working and fast on
2026-08-08), reimplemented here against this repo's own MCP server -- which
now has all four tiers of tools (Bronze through Diamond), not just the one
Bronze move Nina had at the time.

Unlike the course's `llm-client/` guides (which point you at a GUI app --
Bionic or OpenClaw -- to add as an MCP client), this is a standalone script:
useful in a sandbox/headless environment with no GUI, and it makes the
tool-calling loop visible instead of hidden behind an app.

## Setup

```bash
pip install -r requirements.txt
```

Get a Cerebras API key (or use another OpenAI-compatible endpoint, see
below) and set it as an environment variable -- **never put it in a file in
this repo**:

```bash
export CEREBRAS_API_KEY="your-key-here"
```

**About the key in the team chat:** the key Nina/the French teammate shared
in the team's chat was explicitly flagged "don't write it in clear when we
get the GitHub" -- it's a shared team testing key, not something to commit
or paste into code. This script only ever reads it from the environment
variable above; if you use that shared key, treat it the same way the team
asked (env var only, never in a file you `git add`), and if you're not sure
it's still meant to be shared, ask the team before using it.

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

Each turn it starts listening immediately, waits (quietly) for you to start
talking, records until you stop, transcribes locally (no API key needed for
this part), and sends that as your message -- same tool-calling loop either
way, only how the text arrives differs. It also prints what it heard, since
speech-to-text can mishear:

```
Connected: 7 tools from ur-tools, model gpt-oss-120b @ https://api.cerebras.ai/v1
🎤 Listening -- speak your command...
You (heard): move the robot home
  -> move_robot_to_position({})
Robot: Done -- the robot is now at its home position...
```

**Verified working end to end** against a real microphone: WSL2/WSLg
bridges the Windows host's mic in as a PulseAudio source (`RDPSource`) --
no extra setup, `$PULSE_SERVER` is already pointed at it. Confirmed live:
speaking "move the robot home" into the mic was captured, transcribed, and
correctly called `move_robot_to_position`.

Needs `pulseaudio-utils` for `parec` (the capture tool; see `voice.py`'s
docstring for why not `parecord`):

```bash
sudo apt install -y pulseaudio-utils
pactl info                      # confirms $PULSE_SERVER is reachable
pactl list short sources        # should list a real default source
```

If it's not WSL2/WSLg, this still works on any Linux with a working
PulseAudio default source (a real laptop mic, typically) -- the code never
hardcodes `RDPSource`, it just uses whatever `parec` picks up by default.

**Not a wake word** -- there's no "hey robot" trigger, no continuous
background listening. It's armed the moment `listen()` is called (each
chat turn) and triggers on an energy threshold once you start talking. A
real wake-word engine (Porcupine, openWakeWord) would run continuously in
the background instead of needing a fresh `listen()` call per turn; that's
its own model/dependency, not pulled in here.

**Tuning, if it doesn't fit your mic/room** (env vars, see `voice.py` for
defaults): `VOICE_START_RMS` (lower if it never triggers, raise if it
triggers on room noise -- run with `VOICE_DEBUG=1` to watch live RMS
values and pick a threshold between your room's noise floor and your
speaking volume), `VOICE_SILENCE_HANG_S` (how long a pause before it
decides you're done -- raise if it cuts you off mid-sentence),
`VOICE_MAX_WAIT_S` / `VOICE_MAX_UTTERANCE_S` (give-up caps),
`WHISPER_MODEL` (default `base.en`; `small.en` is more accurate and still
CPU-fast if `base.en` mishears too often). A stray loud noise can
occasionally get transcribed as a hallucinated phrase (a known Whisper
quirk on noise/near-silence, not something this VAD filters out) -- if
that happens a lot, raising `VOICE_START_RMS` past your room's noise floor
is the fix.

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

`chat.py` connects to `case1/server.py`'s MCP server in-process (the same
way `case1/test_server.py` does), reads its tool list, and converts each
tool's JSON schema into OpenAI's function-calling `tools` format. Each turn:
your message goes to the model with that tool list; if the model responds
with a tool call, the script executes it against the real MCP server (the
robot moves) and feeds the JSON result back to the model as a `tool` message;
this repeats until the model replies with plain text instead of another
call.

Model-agnostic by design (per the course's guidance): any tool-calling model
behind an OpenAI-compatible endpoint works, not just Cerebras's gpt-oss-120b.
