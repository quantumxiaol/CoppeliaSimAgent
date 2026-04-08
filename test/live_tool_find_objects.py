#!/usr/bin/env python3
"""Live test: find objects by name/type using find_objects."""

from __future__ import annotations

import argparse
import sys

from live_tool_common import print_fail, print_header, print_pass

from coppeliasimagent.tools.scene import find_objects


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Find objects in CoppeliaSim scene")
    parser.add_argument("--name", default="lid", help="Name query (substring by default)")
    parser.add_argument("--exact", action="store_true", help="Use exact-name match instead of substring")
    parser.add_argument(
        "--types",
        default="shape,dummy",
        help="Comma-separated object types: shape,dummy,joint,camera,light,force_sensor",
    )
    parser.add_argument("--limit", type=int, default=20, help="Maximum number of results")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: find_objects")

    try:
        include_types = [t.strip() for t in args.types.split(",") if t.strip()]
        items = find_objects(
            name_query=args.name,
            exact_name=args.exact,
            include_types=include_types,
            limit=args.limit,
        )

        print_pass(f"matched items={len(items)}")
        for item in items:
            print(f"- {item['name']}: handle={item['handle']}, pos={item['position']}")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"find_objects failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
