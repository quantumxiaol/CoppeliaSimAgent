"""Custom exceptions for CoppeliaSimAgent."""

from __future__ import annotations


class CoppeliaSimAgentError(Exception):
    """Base exception for all project-level errors."""


class SimConnectionError(CoppeliaSimAgentError):
    """Raised when the remote API connection cannot be established or reused."""


class PluginUnavailableError(CoppeliaSimAgentError):
    """Raised when an optional CoppeliaSim plugin (e.g. simIK/simOMPL) is required but missing."""


class CollisionDetectedError(CoppeliaSimAgentError):
    """Raised when collision checks report a collision in the scene."""


class ToolValidationError(CoppeliaSimAgentError):
    """Raised when tool input validation fails before reaching CoppeliaSim."""
