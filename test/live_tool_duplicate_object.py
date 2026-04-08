#!/usr/bin/env python3
"""Live test: duplicate one object by handle or name query."""

from __future__ import annotations

import argparse
import math
import sys

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.core.connection import get_sim
from coppeliasimagent.tools.primitives import duplicate_object
from coppeliasimagent.tools.scene import find_objects


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Duplicate one CoppeliaSim object")
    parser.add_argument("--handle", type=int, default=None, help="Source handle to duplicate")
    parser.add_argument("--name", default="lid", help="Name query used when --handle is not provided")
    parser.add_argument(
        "--offset",
        type=parse_vec3,
        default=[0.15, 0.0, 0.0],
        help="Offset [dx,dy,dz] for duplicated object when --target is not set",
    )
    parser.add_argument(
        "--target",
        type=parse_vec3,
        default=None,
        help="Optional absolute target position [x,y,z] for duplicated object",
    )
    return parser.parse_args()


def _almost_equal(a: list[float], b: list[float], tol: float = 1e-4) -> bool:
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b, strict=True))


def _resolve_source_handle(handle: int | None, name: str) -> int:
    if handle is not None:
        return handle

    items = find_objects(name_query=name, exact_name=False, include_types=["shape"], limit=1)
    if not items:
        raise RuntimeError(f"no shape found for name query '{name}'")
    return int(items[0]["handle"])


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: duplicate_object")

    try:
        sim = get_sim()
        src_handle = _resolve_source_handle(args.handle, args.name)
        src_pos = list(sim.getObjectPosition(src_handle, -1))
        print_pass(f"source handle={src_handle}, position={src_pos}")

        if args.target is not None:
            new_handle = duplicate_object(handle=src_handle, position=args.target)
            expected = args.target
        else:
            new_handle = duplicate_object(handle=src_handle, offset=args.offset)
            expected = [src_pos[i] + args.offset[i] for i in range(3)]

        actual = list(sim.getObjectPosition(new_handle, -1))
        print_pass(f"duplicated handle={new_handle}")
        print_pass(f"actual position={actual}")

        if not _almost_equal(actual, expected):
            print_fail(f"position mismatch: expected={expected}, actual={actual}")
            return 1

        print_pass("duplicate_object verification passed")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"duplicate test failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
