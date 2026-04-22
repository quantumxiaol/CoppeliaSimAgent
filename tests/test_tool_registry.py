from __future__ import annotations

import sys
import unittest
from pathlib import Path

from pydantic import ValidationError

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from coppeliasimagent.agent.tool_registry import get_openai_tools, get_tool, invoke_tool, list_tool_names


class TestToolRegistry(unittest.TestCase):
    def test_list_tool_names_contains_core_tools(self) -> None:
        names = list_tool_names()
        self.assertIn("spawn_cuboid", names)
        self.assertIn("get_scene_graph", names)
        self.assertIn("setup_ik_link", names)
        self.assertIn("setup_youbot_arm_ik", names)
        self.assertIn("drive_youbot_base", names)
        self.assertIn("set_youbot_base_locked", names)
        self.assertIn("find_objects", names)
        self.assertIn("duplicate_object", names)
        self.assertIn("rename_object", names)
        self.assertIn("set_object_color", names)
        self.assertIn("set_joint_target_velocity", names)

    def test_unknown_tool_raises(self) -> None:
        with self.assertRaises(KeyError):
            get_tool("not_exists")

    def test_validation_happens_before_execution(self) -> None:
        with self.assertRaises(ValidationError):
            invoke_tool("spawn_cuboid", {"size": [0.1, 0.2], "position": [0.0, 0.0, 0.0]})

    def test_openai_schema_export(self) -> None:
        tools = get_openai_tools()
        names = {tool["function"]["name"] for tool in tools}
        self.assertIn("spawn_primitive", names)
        self.assertIn("actuate_gripper", names)
        self.assertIn("actuate_youbot_gripper", names)
        self.assertIn("stop_youbot_base", names)


if __name__ == "__main__":
    unittest.main()
