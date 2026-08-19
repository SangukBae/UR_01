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
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path

from fastmcp import Client
from openai import OpenAI

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "case1"))
from server import mcp, robot  # noqa: E402

BASE_URL = os.environ.get("LLM_BASE_URL", "https://api.cerebras.ai/v1")
MODEL = os.environ.get("LLM_MODEL", "gpt-oss-120b")
API_KEY_ENV = "CEREBRAS_API_KEY" if "cerebras" in BASE_URL else "LLM_API_KEY"

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


async def main() -> None:
    api_key = os.environ.get(API_KEY_ENV)
    if not api_key:
        raise SystemExit(
            f"Set {API_KEY_ENV} first (export {API_KEY_ENV}=...). "
            "Never hardcode an API key in this file -- see README.md."
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
        print("Type a message (e.g. 'move the robot home'), Ctrl-C to quit.\n")

        messages: list[dict] = [{"role": "system", "content": SYSTEM_PROMPT}]
        while True:
            try:
                user_text = input("You: ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if not user_text:
                continue
            messages.append({"role": "user", "content": user_text})

            # Tool-call loop: keep feeding results back until the model
            # replies with plain text instead of another tool call.
            while True:
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


if __name__ == "__main__":
    asyncio.run(main())
