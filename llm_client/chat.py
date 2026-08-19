"""Minimal LLM chat wrapper with MCP tool-calling.

Replicates the team's Bronze-tier "LLM wrapper" (Nina Muller, confirmed
working with the Cerebras API on 2026-08-08) against this repo's own
``case1/server.py`` -- which now has all four tiers of tools (Bronze
through Diamond), not just the one Bronze move Nina had at the time.

Talks to any OpenAI-compatible chat completions endpoint. Default is
Cerebras, the one the team confirmed fast and working (NVIDIA NIM needed a
credit card, this didn't). Connects to the MCP server in-process (the same
way ``case1/test_server.py`` does -- no subprocess, no stdio), converts its
tools to OpenAI's function-calling schema, and runs a plain chat loop: you
type a message, the model may call one or more robot tools, and the loop
feeds results back to it until it has a final answer.

Setup:
    pip install -r requirements.txt
    export CEREBRAS_API_KEY=...   # never hardcode this -- see README.md

Run (from this folder, with the simulator up and the robot powered on):
    python3 chat.py

Or speak instead of typing: each turn it listens, waits for you to start
talking, records until you stop, transcribes, and sends that -- see
voice.py for how, and its module docstring for the microphone/PulseAudio
setup this needs beyond pip:
    python3 chat.py --voice
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from pathlib import Path
from typing import Callable

from fastmcp import Client
from openai import OpenAI

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "case1"))
from server import mcp, robot  # noqa: E402

BASE_URL = os.environ.get("LLM_BASE_URL", "https://api.cerebras.ai/v1")
MODEL = os.environ.get("LLM_MODEL", "gpt-oss-120b")
API_KEY_ENV = "CEREBRAS_API_KEY" if "cerebras" in BASE_URL else "LLM_API_KEY"
MAX_TOOL_ROUNDS = 8

SYSTEM_PROMPT = (
    "You control a UR10 robot arm through the tools available to you. "
    "Read each tool's docstring for its units and defaults before calling it. "
    "If a tool call fails, read the error message -- it explains what was "
    "wrong -- and either fix the call or tell the user what went wrong."
)


def _to_openai_tools(mcp_tools) -> list[dict]:
    """MCP tool list -> OpenAI chat-completions ``tools`` schema."""
    return [
        {
            "type": "function",
            "function": {
                "name": t.name,
                "description": t.description or "",
                "parameters": t.inputSchema,
            },
        }
        for t in mcp_tools
    ]


async def main(get_input: Callable[[], str | None], hint: str) -> None:
    """``get_input`` returns the next user message (or None/empty to skip a
    turn, e.g. voice mode hearing nothing) -- ``input("You: ")`` for typed
    chat, ``voice.listen()`` for ``--voice``. Everything past that point
    (the LLM + tool-call loop) doesn't care which one produced the text.
    ``hint`` is just the startup line telling you which one is active.
    """
    api_key = os.environ.get(API_KEY_ENV)
    if not api_key:
        raise SystemExit(
            f"Set {API_KEY_ENV} first (export {API_KEY_ENV}=...). "
            "Never hardcode an API key in this file -- see README.md."
        )
    if not api_key.isascii():
        # Copying a key out of a chat app is a common way to pick up an
        # invisible character (smart quote, zero-width space) alongside it --
        # reproduced live: pasted this way, the key made it into the
        # Authorization header, and httpx2 (openai's HTTP client dep here)
        # failed trying to ascii-encode it, 8 stack frames deep with no hint
        # it was the key. Catch it here instead, pointing at the exact
        # character so it's obvious what to fix.
        bad = next((i, c) for i, c in enumerate(api_key) if not c.isascii())
        raise SystemExit(
            f"{API_KEY_ENV} has a non-ASCII character at position {bad[0]} "
            f"({bad[1]!r}) -- probably picked up copying the key from a chat "
            "app. Re-copy it from a plain-text source (or retype it) and "
            "export it again."
        )
    # Accept-Encoding: identity works around a brotli-decoder bug in this
    # sandbox's httpx2 (openai's HTTP client dep) -- decoder.decode() gets
    # called with an output_buffer_limit kwarg the installed `brotli`
    # package's Decompressor.process() doesn't accept, so a brotli response
    # body raises TypeError instead of decoding. Forcing uncompressed
    # responses sidesteps it; harmless outside this sandbox too.
    llm = OpenAI(base_url=BASE_URL, api_key=api_key,
                 default_headers={"Accept-Encoding": "identity"})

    robot.connect()  # fails fast if the simulator is down or the robot is off
    async with Client(mcp) as mcp_client:
        mcp_tools = await mcp_client.list_tools()
        tools = _to_openai_tools(mcp_tools)
        print(f"Connected: {len(tools)} tools from ur-tools, model {MODEL} @ {BASE_URL}")
        print(f"{hint} Ctrl-C to quit.\n")

        messages: list[dict] = [{"role": "system", "content": SYSTEM_PROMPT}]
        while True:
            try:
                user_text = get_input()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if not user_text:
                continue
            user_text = user_text.strip()
            if not user_text:
                continue
            messages.append({"role": "user", "content": user_text})

            # Tool-call loop: keep feeding results back until the model
            # replies with plain text instead of another tool call. Capped --
            # reproduced live: asked for a specific tool by description
            # rather than name, the model called a different tool 31 times
            # in a row (same args each time) instead of ever stopping or
            # switching, with no error to break the loop on its own.
            for _ in range(MAX_TOOL_ROUNDS):
                response = llm.chat.completions.create(
                    model=MODEL, messages=messages, tools=tools,
                )
                message = response.choices[0].message
                messages.append(message.model_dump(exclude_none=True))

                if not message.tool_calls:
                    print("Robot:", message.content)
                    break

                for call in message.tool_calls:
                    args = json.loads(call.function.arguments or "{}")
                    print(f"  -> {call.function.name}({args})")
                    try:
                        result = await mcp_client.call_tool(call.function.name, args)
                        content = json.dumps(result.data)
                    except Exception as exc:  # noqa: BLE001 -- surfaced to the LLM, not swallowed
                        content = json.dumps({"error": str(exc)})
                        print("     error:", exc)
                    messages.append({
                        "role": "tool", "tool_call_id": call.id, "content": content,
                    })
            else:
                print(f"Robot: (stopped after {MAX_TOOL_ROUNDS} tool-call rounds "
                      "without a final answer -- it may be stuck; try rephrasing)")


def _text_input() -> str | None:
    return input("You: ")


def _voice_input() -> str | None:
    import voice  # local import: numpy/faster-whisper are only needed here

    text = voice.listen("\U0001F3A4 Listening -- speak your command...")
    if text is None:
        print("(didn't catch anything, try again)")
        return None
    print(f"You (heard): {text}")
    return text


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--voice", action="store_true",
        help="Speak commands into the microphone instead of typing them (see voice.py).",
    )
    args = parser.parse_args()
    if args.voice:
        asyncio.run(main(_voice_input, "Speaking mode -- each turn, just start talking."))
    else:
        asyncio.run(main(_text_input, "Type a message (e.g. 'move the robot home')."))
