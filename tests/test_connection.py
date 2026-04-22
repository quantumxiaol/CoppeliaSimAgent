from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from coppeliasimagent.core.connection import SimConnection
from coppeliasimagent.core.exceptions import PluginUnavailableError, SimConnectionError


class FakeSocket:
    def __init__(self) -> None:
        self.RCVTIMEO = None
        self.SNDTIMEO = None
        self.closed = False

    def close(self, linger: int) -> None:
        self.closed = True


class FakeContext:
    def __init__(self) -> None:
        self.terminated = False

    def term(self) -> None:
        self.terminated = True


class FakeRemoteAPIClient:
    instances: list["FakeRemoteAPIClient"] = []
    fail_on_sim = False
    missing_plugins: set[str] = set()
    require_only_plugins: set[str] = set()

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.socket = FakeSocket()
        self.context = FakeContext()
        self.objects: dict[str, object] = {"sim": object(), "simIK": object(), "simOMPL": object()}
        FakeRemoteAPIClient.instances.append(self)

    def getObject(self, name: str) -> object:  # noqa: N802
        if name == "sim" and FakeRemoteAPIClient.fail_on_sim:
            raise RuntimeError("sim unavailable")
        if name in FakeRemoteAPIClient.require_only_plugins:
            raise RuntimeError(f"plugin must be accessed via require: {name}")
        if name in FakeRemoteAPIClient.missing_plugins:
            raise RuntimeError(f"missing plugin: {name}")
        return self.objects.get(name, object())

    def require(self, name: str) -> object:
        if name in FakeRemoteAPIClient.missing_plugins:
            raise RuntimeError(f"missing plugin: {name}")
        return self.objects[name]


class TestSimConnection(unittest.TestCase):
    def setUp(self) -> None:
        FakeRemoteAPIClient.instances.clear()
        FakeRemoteAPIClient.fail_on_sim = False
        FakeRemoteAPIClient.missing_plugins = set()
        FakeRemoteAPIClient.require_only_plugins = set()

    def test_connect_and_preload_plugins(self) -> None:
        conn = SimConnection(
            host="127.0.0.1",
            port=23000,
            timeout_s=1.5,
            client_factory=FakeRemoteAPIClient,
            auto_connect=True,
        )

        self.assertIsNotNone(conn.client)
        self.assertIsNotNone(conn.sim)
        self.assertIsNotNone(conn.simIK)
        self.assertIsNotNone(conn.simOMPL)

        client = FakeRemoteAPIClient.instances[0]
        self.assertEqual(client.socket.RCVTIMEO, 1500)
        self.assertEqual(client.socket.SNDTIMEO, 1500)

    def test_required_plugin_raises(self) -> None:
        FakeRemoteAPIClient.missing_plugins = {"simIK"}
        conn = SimConnection(client_factory=FakeRemoteAPIClient, auto_connect=True)

        self.assertIsNone(conn.simIK)
        with self.assertRaises(PluginUnavailableError):
            conn.get_plugin("simIK", required=True)

    def test_plugin_resolution_prefers_require(self) -> None:
        FakeRemoteAPIClient.require_only_plugins = {"simIK", "simOMPL"}
        conn = SimConnection(client_factory=FakeRemoteAPIClient, auto_connect=True)

        self.assertIsNotNone(conn.simIK)
        self.assertIsNotNone(conn.simOMPL)

    def test_reconnect_creates_new_client(self) -> None:
        conn = SimConnection(client_factory=FakeRemoteAPIClient, auto_connect=True)
        first = conn.client
        conn.reconnect()
        second = conn.client

        self.assertIsNot(first, second)
        self.assertTrue(first.socket.closed)
        self.assertTrue(first.context.terminated)

    def test_connect_failure_raises_connection_error(self) -> None:
        FakeRemoteAPIClient.fail_on_sim = True

        with self.assertRaises(SimConnectionError):
            SimConnection(client_factory=FakeRemoteAPIClient, auto_connect=True)


if __name__ == "__main__":
    unittest.main()
