#!/usr/bin/env python3
"""Live test: rename one object alias by handle or name query."""

from __future__ import annotations

import argparse
import sys

from live_tool_common import print_fail, print_header, print_pass

from coppeliasimagent.core.connection import get_sim
from coppeliasimagent.tools.primitives import rename_object
from coppeliasimagent.tools.scene import find_objects


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Rename one object alias in CoppeliaSim")
    parser.add_argument("--handle", type=int, default=None, help="Target handle to rename")
    parser.add_argument("--name", default="lid", help="Name query used when --handle is not provided")
    parser.add_argument("--new-alias", default="jar_lid_copy", help="New alias")
    return parser.parse_args()


def _resolve_target_handle(handle: int | None, name: str) -> int:
    if handle is not None:
        return handle
    items = find_objects(name_query=name, exact_name=False, include_types=["shape", "dummy"], limit=1)
    if not items:
        raise RuntimeError(f"no object found for name query '{name}'")
    return int(items[0]["handle"])


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: rename_object")

    try:
        sim = get_sim()
        handle = _resolve_target_handle(args.handle, args.name)
        old_name = sim.getObjectAlias(handle) if hasattr(sim, "getObjectAlias") else sim.getObjectName(handle)
        print_pass(f"target handle={handle}, old name={old_name}")

        out_alias = rename_object(handle=handle, new_alias=args.new_alias)
        now_name = sim.getObjectAlias(handle) if hasattr(sim, "getObjectAlias") else sim.getObjectName(handle)

        print_pass(f"renamed to {out_alias}")
        print_pass(f"current name={now_name}")
        if str(now_name) != out_alias:
            print_fail("rename verification failed")
            return 1
        print_pass("rename verification passed")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"rename_object failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
