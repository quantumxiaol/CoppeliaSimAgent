"""Interactive CLI chat client for MCP-backed CoppeliaSim agent."""

from __future__ import annotations

import argparse
import asyncio
import json
from typing import Any

from ..agent.mcp_backend import MCPAgentBackend, ToolTrace
from ..config import load_agent_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run CoppeliaSimAgent CLI")
    parser.add_argument(
        "--max-tool-rounds",
        type=int,
        default=8,
        help="Maximum tool-calling rounds per user turn",
    )
    return parser.parse_args()


def _format_tool_trace(trace: ToolTrace) -> str:
    status = "ERROR" if trace.is_error else "OK"
    args_json = json.dumps(trace.arguments, ensure_ascii=False)
    return f"[{status}] {trace.name} args={args_json} result={trace.result_preview}"


async def run_chat(max_tool_rounds: int) -> None:
    config = load_agent_config()
    history: list[dict[str, Any]] = []

    async with MCPAgentBackend(config=config, max_tool_rounds=max_tool_rounds) as backend:
        print("CoppeliaSimAgent CLI")
        print("Type '/exit' to quit.")

        while True:
            try:
                user_input = input("\nyou> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nexit")
                break

            if not user_input:
                continue

            if user_input.lower() in {"/exit", "exit", "quit", "/quit"}:
                break

            result = await backend.chat(user_input=user_input, history=history)
            history = result.history

            if result.tool_traces:
                print("tools>")
                for trace in result.tool_traces:
                    print("  " + _format_tool_trace(trace))

            print("agent> " + (result.assistant_text or "(empty response)"))


def main() -> None:
    args = parse_args()
    asyncio.run(run_chat(max_tool_rounds=args.max_tool_rounds))


if __name__ == "__main__":
    main()
