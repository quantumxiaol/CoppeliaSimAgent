from __future__ import annotations

import io
import sys
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from coppeliasimagent.cli.toolcli import main


class TestToolCli(unittest.TestCase):
    def test_help_exits_zero(self) -> None:
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = main([])
        self.assertEqual(code, 0)
        self.assertIn("coppelia-toolcli", stdout.getvalue())

    def test_list_contains_known_tool(self) -> None:
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = main(["list"])
        self.assertEqual(code, 0)
        self.assertIn("get_scene_graph", stdout.getvalue())
        self.assertIn("start_simulation", stdout.getvalue())

    def test_show_contains_parameters(self) -> None:
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = main(["show", "load_model"])
        self.assertEqual(code, 0)
        output = stdout.getvalue()
        self.assertIn("name: load_model", output)
        self.assertIn("parameters:", output)
        self.assertIn("model_path", output)

    def test_show_empty_schema_tool(self) -> None:
        stdout = io.StringIO()
        with redirect_stdout(stdout):
            code = main(["show", "start_simulation"])
        self.assertEqual(code, 0)
        output = stdout.getvalue()
        self.assertIn("name: start_simulation", output)
        self.assertIn('"properties": {}', output)

    def test_call_uses_registry(self) -> None:
        stdout = io.StringIO()
        with patch("coppeliasimagent.cli.toolcli.invoke_tool", return_value={"ok": True}) as mocked_invoke:
            with redirect_stdout(stdout):
                code = main(["call", "find_objects", "--payload", '{"name_query":"jar"}'])
        self.assertEqual(code, 0)
        mocked_invoke.assert_called_once_with("find_objects", {"name_query": "jar"})
        self.assertIn('"ok": true', stdout.getvalue())

    def test_call_rejects_multiple_payload_sources(self) -> None:
        stderr = io.StringIO()
        with redirect_stderr(stderr):
            code = main(["call", "find_objects", "--payload", "{}", "--stdin"])
        self.assertEqual(code, 4)
        self.assertIn("Use only one of", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
