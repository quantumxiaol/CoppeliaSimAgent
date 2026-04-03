from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from coppeliasimagent.prompts import DEFAULT_AGENT_SYSTEM_PROMPT_FILE, load_agent_system_prompt


class TestPrompts(unittest.TestCase):
    def test_default_prompt_file_exists(self) -> None:
        self.assertTrue(DEFAULT_AGENT_SYSTEM_PROMPT_FILE.exists())

    def test_load_prompt_non_empty(self) -> None:
        prompt = load_agent_system_prompt()
        self.assertTrue(prompt)
        self.assertIn("CoppeliaSim", prompt)
        self.assertIn("[x, y, z]", prompt)


if __name__ == "__main__":
    unittest.main()
