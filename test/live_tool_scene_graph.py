#!/usr/bin/env python3
"""Live test: read scene graph using get_scene_graph."""

from __future__ import annotations

import argparse
import sys

from live_tool_common import print_fail, print_header, print_pass

from coppeliasimagent.tools.scene import get_scene_graph


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read scene graph from CoppeliaSim")
    parser.add_argument(
        "--types",
        default="shape,dummy",
        help="Comma-separated object types: shape,dummy,joint,camera,light,force_sensor",
    )
    parser.add_argument("--max-items", type=int, default=10, help="Max items to print")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: get_scene_graph")

    try:
        include_types = [t.strip() for t in args.types.split(",") if t.strip()]
        graph = get_scene_graph(include_types=include_types)

        print_pass(f"scene_graph items={len(graph)}")

        count = 0
        for name, data in graph.items():
            print(f"- {name}: handle={data['handle']}, pos={data['position']}")
            count += 1
            if count >= args.max_items:
                break

        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"scene graph test failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
