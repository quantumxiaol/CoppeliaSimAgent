"""Prompt templates and loaders for agent behavior."""

from __future__ import annotations

from pathlib import Path


PROMPT_DIR = Path(__file__).resolve().parent
DEFAULT_AGENT_SYSTEM_PROMPT_FILE = PROMPT_DIR / "agent_system_prompt.md"


def load_agent_system_prompt() -> str:
    """Load the default system prompt text for the MCP agent backend."""
    text = DEFAULT_AGENT_SYSTEM_PROMPT_FILE.read_text(encoding="utf-8").strip()
    if not text:
        raise RuntimeError(f"Prompt file is empty: {DEFAULT_AGENT_SYSTEM_PROMPT_FILE}")
    return text


__all__ = [
    "DEFAULT_AGENT_SYSTEM_PROMPT_FILE",
    "PROMPT_DIR",
    "load_agent_system_prompt",
]
