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
