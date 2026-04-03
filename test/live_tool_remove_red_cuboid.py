#!/usr/bin/env python3
"""Live test: spawn a red cuboid then remove it with remove_object."""

from __future__ import annotations

import argparse
import sys

from live_tool_common import print_fail, print_header, print_pass

from coppeliasimagent.tools.primitives import remove_object, spawn_cuboid
from coppeliasimagent.tools.scene import get_scene_graph


def _contains_handle(scene_graph: dict[str, dict[str, object]], handle: int) -> bool:
    return any(obj.get("handle") == handle for obj in scene_graph.values())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Remove one object by handle, or spawn-and-remove a test cuboid."
    )
    parser.add_argument(
        "--handle",
        type=int,
        default=None,
        help="Existing object handle to remove. If omitted, script spawns a temporary cuboid and removes it.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: remove_object")

    try:
        if args.handle is None:
            handle = spawn_cuboid(size=[0.1, 0.1, 0.1], position=[0.0, 0.0, 0.3], color=[1.0, 0.0, 0.0])
            print_pass(f"spawned handle={handle}")
        else:
            handle = args.handle
            print_pass(f"target existing handle={handle}")

        graph_before = get_scene_graph(include_types=["shape"])
        remove_object(handle)
        graph_after = get_scene_graph(include_types=["shape"])

        if not _contains_handle(graph_before, handle):
            print_fail("target handle not found before deletion")
            return 1

        if _contains_handle(graph_after, handle):
            print_fail("handle still exists after remove_object")
            return 1

        print_pass("remove_object verification passed")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"remove test failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
