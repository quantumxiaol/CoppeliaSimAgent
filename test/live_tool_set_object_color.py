#!/usr/bin/env python3
"""Live test: set one shape object color by handle or name query."""

from __future__ import annotations

import argparse
import sys

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.tools.primitives import set_object_color
from coppeliasimagent.tools.scene import find_objects


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set one shape color in CoppeliaSim")
    parser.add_argument("--handle", type=int, default=None, help="Target shape handle")
    parser.add_argument("--name", default="lid", help="Name query used when --handle is not provided")
    parser.add_argument("--color", type=parse_vec3, default=[0.2, 0.4, 1.0], help="RGB in [0,1], format r,g,b")
    parser.add_argument(
        "--component",
        default="ambient_diffuse",
        help="Color component: ambient_diffuse,diffuse,specular,emission,transparency",
    )
    return parser.parse_args()


def _resolve_target_handle(handle: int | None, name: str) -> int:
    if handle is not None:
        return handle
    items = find_objects(name_query=name, exact_name=False, include_types=["shape"], limit=1)
    if not items:
        raise RuntimeError(f"no shape found for name query '{name}'")
    return int(items[0]["handle"])


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: set_object_color")

    try:
        handle = _resolve_target_handle(args.handle, args.name)
        out = set_object_color(
            handle=handle,
            color=args.color,
            color_component=args.component,
        )
        print_pass(f"set_object_color target handle={handle}")
        print_pass(f"output handle={out}")
        print_pass("color command sent successfully")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"set_object_color failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
