"""Core runtime utilities for CoppeliaSimAgent."""

from .connection import (
    SimConnection,
    get_connection,
    get_sim,
    get_simik,
    get_simompl,
    reset_default_connection,
    set_default_connection,
)
from .exceptions import (
    CollisionDetectedError,
    CoppeliaSimAgentError,
    PluginUnavailableError,
    SimConnectionError,
    ToolValidationError,
)

__all__ = [
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
    "reset_default_connection",
    "set_default_connection",
]
