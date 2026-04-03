"""CoppeliaSimAgent package."""

from .core import (
    CollisionDetectedError,
    CoppeliaSimAgentError,
    PluginUnavailableError,
    SimConnection,
    SimConnectionError,
    ToolValidationError,
    get_connection,
    get_sim,
    get_simik,
    get_simompl,
)
from .config import AgentConfig, load_agent_config
from .prompts import load_agent_system_prompt

__all__ = [
    "AgentConfig",
    "CollisionDetectedError",
    "CoppeliaSimAgentError",
    "PluginUnavailableError",
    "SimConnection",
    "SimConnectionError",
    "ToolValidationError",
    "get_connection",
    "get_sim",
    "get_simik",
    "get_simompl",
    "load_agent_system_prompt",
    "load_agent_config",
]
