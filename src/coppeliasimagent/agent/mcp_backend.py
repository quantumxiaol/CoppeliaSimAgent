"""LLM agent backend that uses MCP tools for execution."""

from __future__ import annotations

import json
import os
import sys
from contextlib import AsyncExitStack
from dataclasses import dataclass
from datetime import timedelta
from pathlib import Path
from typing import Any

from mcp import types
from mcp.client.session import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
from openai import AsyncOpenAI

from ..config import AgentConfig
from ..prompts import load_agent_system_prompt


@dataclass
class ToolTrace:
    """One tool execution record for CLI display."""

    name: str
    arguments: dict[str, Any]
    is_error: bool
    result_preview: str


@dataclass
class AgentTurnResult:
    """Output of one agent turn."""

    assistant_text: str
    tool_traces: list[ToolTrace]
    history: list[dict[str, Any]]


class MCPToolClient:
    """Manage an MCP stdio client session to a local MCP server process."""

    def __init__(self, *, read_timeout_seconds: float = 60.0) -> None:
        self.read_timeout_seconds = read_timeout_seconds
        self._stack: AsyncExitStack | None = None
        self._session: ClientSession | None = None

    async def __aenter__(self) -> "MCPToolClient":
        await self.start()
        return self

    async def __aexit__(self, exc_type: object, exc: object, tb: object) -> None:
        await self.close()

    async def start(self) -> None:
        if self._stack is not None:
            return

        project_root = Path(__file__).resolve().parents[3]
        src_dir = project_root / "src"

        env = os.environ.copy()
        old_pythonpath = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = (
            f"{src_dir}{os.pathsep}{old_pythonpath}" if old_pythonpath else str(src_dir)
        )

        server_params = StdioServerParameters(
            command=sys.executable,
            args=[
                "-m",
                "coppeliasimagent.servers.mcp_server",
                "--transport",
                "stdio",
            ],
            env=env,
            cwd=project_root,
        )

        stack = AsyncExitStack()
        read_stream, write_stream = await stack.enter_async_context(stdio_client(server_params))
        session = await stack.enter_async_context(
            ClientSession(
                read_stream,
                write_stream,
                read_timeout_seconds=timedelta(seconds=self.read_timeout_seconds),
            )
        )
        await session.initialize()

        self._stack = stack
        self._session = session

    async def close(self) -> None:
        if self._stack is not None:
            await self._stack.aclose()
        self._stack = None
        self._session = None

    @property
    def session(self) -> ClientSession:
        if self._session is None:
            raise RuntimeError("MCPToolClient is not started")
        return self._session

    async def list_tools(self) -> list[types.Tool]:
        return (await self.session.list_tools()).tools

    async def call_tool(self, name: str, arguments: dict[str, Any]) -> types.CallToolResult:
        return await self.session.call_tool(name=name, arguments=arguments)


class MCPAgentBackend:
    """Agent backend that routes tool calls through MCP."""

    def __init__(
        self,
        config: AgentConfig,
        *,
        max_tool_rounds: int = 8,
        system_prompt: str | None = None,
    ) -> None:
        self.config = config
        self.max_tool_rounds = max_tool_rounds
        self.system_prompt = system_prompt or load_agent_system_prompt()
        self._openai = AsyncOpenAI(
            api_key=config.llm_model_api_key.get_secret_value(),
            base_url=config.llm_model_base_url,
        )
        self._tool_client = MCPToolClient()
        self._openai_tools_cache: list[dict[str, Any]] | None = None

    async def __aenter__(self) -> "MCPAgentBackend":
        await self.start()
        return self

    async def __aexit__(self, exc_type: object, exc: object, tb: object) -> None:
        await self.close()

    async def start(self) -> None:
        await self._tool_client.start()
        self._openai_tools_cache = await self._load_openai_tools()

    async def close(self) -> None:
        await self._tool_client.close()

    async def _load_openai_tools(self) -> list[dict[str, Any]]:
        tools = await self._tool_client.list_tools()
        openai_tools: list[dict[str, Any]] = []
        for tool in tools:
            params = tool.inputSchema if isinstance(tool.inputSchema, dict) else {
                "type": "object",
                "properties": {},
            }
            openai_tools.append(
                {
                    "type": "function",
                    "function": {
                        "name": tool.name,
                        "description": tool.description or "",
                        "parameters": params,
                    },
                }
            )
        return openai_tools

    @staticmethod
    def _safe_json_load(raw: str) -> dict[str, Any]:
        if not raw:
            return {}
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            return {"_raw": raw}
        if isinstance(value, dict):
            return value
        return {"value": value}

    @staticmethod
    def _call_result_to_payload(result: types.CallToolResult) -> dict[str, Any]:
        content_items: list[dict[str, Any]] = []
        for item in result.content:
            if hasattr(item, "model_dump"):
                content_items.append(item.model_dump())
            else:
                content_items.append({"value": str(item)})

        return {
            "is_error": bool(result.isError),
            "content": content_items,
            "structured_content": result.structuredContent,
        }

    async def chat(
        self,
        user_input: str,
        history: list[dict[str, Any]] | None = None,
    ) -> AgentTurnResult:
        if self._openai_tools_cache is None:
            self._openai_tools_cache = await self._load_openai_tools()

        prior_history = history[:] if history else []
        messages: list[dict[str, Any]] = [{"role": "system", "content": self.system_prompt}]
        messages.extend(prior_history)
        messages.append({"role": "user", "content": user_input})

        traces: list[ToolTrace] = []

        for _ in range(self.max_tool_rounds):
            completion = await self._openai.chat.completions.create(
                model=self.config.llm_model_name,
                messages=messages,
                tools=self._openai_tools_cache,
                tool_choice="auto",
                temperature=0.2,
            )
            message = completion.choices[0].message
            tool_calls = list(message.tool_calls or [])

            if not tool_calls:
                assistant_text = message.content or ""
                messages.append({"role": "assistant", "content": assistant_text})
                cleaned_history = [m for m in messages if m.get("role") != "system"]
                return AgentTurnResult(
                    assistant_text=assistant_text,
                    tool_traces=traces,
                    history=cleaned_history,
                )

            assistant_msg: dict[str, Any] = {
                "role": "assistant",
                "content": message.content or "",
                "tool_calls": [],
            }
            for call in tool_calls:
                assistant_msg["tool_calls"].append(
                    {
                        "id": call.id,
                        "type": "function",
                        "function": {
                            "name": call.function.name,
                            "arguments": call.function.arguments,
                        },
                    }
                )
            messages.append(assistant_msg)

            for call in tool_calls:
                tool_name = call.function.name
                arguments = self._safe_json_load(call.function.arguments)

                try:
                    result = await self._tool_client.call_tool(tool_name, arguments)
                    payload = self._call_result_to_payload(result)
                    is_error = bool(payload["is_error"])
                except Exception as exc:  # noqa: BLE001
                    payload = {
                        "is_error": True,
                        "content": [{"type": "text", "text": str(exc)}],
                        "structured_content": None,
                    }
                    is_error = True

                payload_text = json.dumps(payload, ensure_ascii=False)
                preview = payload_text if len(payload_text) <= 400 else payload_text[:400] + "..."

                traces.append(
                    ToolTrace(
                        name=tool_name,
                        arguments=arguments,
                        is_error=is_error,
                        result_preview=preview,
                    )
                )

                messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": call.id,
                        "content": payload_text,
                    }
                )

        fallback_text = "Tool-calling loop exceeded max rounds without a final response."
        cleaned_history = [m for m in messages if m.get("role") != "system"]
        return AgentTurnResult(
            assistant_text=fallback_text,
            tool_traces=traces,
            history=cleaned_history,
        )
