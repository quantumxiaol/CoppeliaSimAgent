"""Connection management for CoppeliaSim ZMQ Remote API."""

from __future__ import annotations

import os
from threading import Lock
from typing import Any, Callable

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from .exceptions import PluginUnavailableError, SimConnectionError


class SimConnection:
    """Holds and reuses one CoppeliaSim Remote API client session.

    The class is intentionally stateful so every tool call can reuse the same
    network connection and already fetched plugin objects.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 23000,
        timeout_s: float = 3.0,
        preload_plugins: tuple[str, ...] = ("simIK", "simOMPL"),
        client_factory: Callable[..., Any] = RemoteAPIClient,
        auto_connect: bool = True,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.preload_plugins = preload_plugins
        self._client_factory = client_factory

        self._client: Any | None = None
        self._sim: Any | None = None
        self._plugins: dict[str, Any | None] = {}
        self._lock = Lock()

        if auto_connect:
            self.connect()

    def connect(self, force: bool = False) -> "SimConnection":
        """Connect to CoppeliaSim and preload frequently used plugins."""
        with self._lock:
            if self._client is not None and self._sim is not None and not force:
                return self

            if force:
                self._close_unlocked()

            try:
                client = self._client_factory(host=self.host, port=self.port)
                timeout_ms = max(1, int(self.timeout_s * 1000))
                if hasattr(client, "socket"):
                    client.socket.RCVTIMEO = timeout_ms
                    client.socket.SNDTIMEO = timeout_ms
                sim = client.getObject("sim")
            except Exception as exc:  # noqa: BLE001
                raise SimConnectionError(
                    f"Failed to connect to CoppeliaSim at {self.host}:{self.port}: {exc}"
                ) from exc

            self._client = client
            self._sim = sim
            self._plugins = {}

            for plugin_name in self.preload_plugins:
                try:
                    self._plugins[plugin_name] = client.getObject(plugin_name)
                except Exception:  # noqa: BLE001
                    # Optional plugin: keep None so callers can decide whether to fail.
                    self._plugins[plugin_name] = None

            return self

    def reconnect(self) -> "SimConnection":
        """Force a fresh client connection."""
        return self.connect(force=True)

    def ensure_connected(self) -> "SimConnection":
        """Ensure an active connection exists, creating it lazily when needed."""
        if self._client is None or self._sim is None:
            self.connect()
        return self

    @property
    def client(self) -> Any:
        return self.ensure_connected()._client

    @property
    def sim(self) -> Any:
        return self.ensure_connected()._sim

    def get_plugin(self, name: str, *, required: bool = False) -> Any | None:
        """Return a preloaded plugin object by name, optionally enforcing presence."""
        self.ensure_connected()
        if name not in self._plugins:
            try:
                self._plugins[name] = self.client.getObject(name)
            except Exception:  # noqa: BLE001
                self._plugins[name] = None

        plugin = self._plugins[name]
        if required and plugin is None:
            raise PluginUnavailableError(
                f"Plugin '{name}' is unavailable. Confirm it is enabled in CoppeliaSim."
            )
        return plugin

    @property
    def simIK(self) -> Any | None:  # noqa: N802
        return self.get_plugin("simIK", required=False)

    @property
    def simOMPL(self) -> Any | None:  # noqa: N802
        return self.get_plugin("simOMPL", required=False)

    def close(self) -> None:
        """Close underlying ZMQ resources if they exist."""
        with self._lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        if self._client is not None:
            try:
                if hasattr(self._client, "socket"):
                    self._client.socket.close(0)
                if hasattr(self._client, "context"):
                    self._client.context.term()
            except Exception:
                pass

        self._client = None
        self._sim = None
        self._plugins = {}


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        value = float(raw)
    except ValueError:
        return default
    return value if value > 0 else default


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        value = int(raw)
    except ValueError:
        return default
    return value if value > 0 else default


_default_connection: SimConnection | None = None
_default_lock = Lock()


def get_connection() -> SimConnection:
    """Return the process-wide default SimConnection singleton."""
    global _default_connection
    with _default_lock:
        if _default_connection is None:
            _default_connection = SimConnection(
                host=os.getenv("COPPELIASIM_HOST", "127.0.0.1"),
                port=_env_int("COPPELIASIM_ZMQ_PORT", 23000),
                timeout_s=_env_float("COPPELIASIM_TIMEOUT", 3.0),
            )
        return _default_connection


def set_default_connection(connection: SimConnection) -> None:
    """Override default singleton (mainly for tests or custom bootstrapping)."""
    global _default_connection
    with _default_lock:
        _default_connection = connection


def reset_default_connection() -> None:
    """Close and clear default singleton (mainly for tests)."""
    global _default_connection
    with _default_lock:
        if _default_connection is not None:
            _default_connection.close()
        _default_connection = None


def get_sim() -> Any:
    return get_connection().sim


def get_simik(*, required: bool = False) -> Any | None:
    return get_connection().get_plugin("simIK", required=required)


def get_simompl(*, required: bool = False) -> Any | None:
    return get_connection().get_plugin("simOMPL", required=required)
