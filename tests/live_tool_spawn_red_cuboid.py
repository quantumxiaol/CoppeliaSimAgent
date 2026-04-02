#!/usr/bin/env python3
"""Live test: spawn one red cuboid in the current CoppeliaSim scene."""

from __future__ import annotations

import argparse
import sys

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.core.connection import get_sim
from coppeliasimagent.tools.primitives import spawn_cuboid


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Spawn a red cuboid in CoppeliaSim")
    parser.add_argument(
        "--size",
        type=parse_vec3,
        default=[0.1, 0.1, 0.1],
        help="Cuboid size in meters, format x,y,z (default: 0.1,0.1,0.1)",
    )
    parser.add_argument(
        "--position",
        type=parse_vec3,
        default=[0.0, 0.0, 0.3],
        help="Spawn position, format x,y,z (default: 0,0,0.3)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: spawn_cuboid")

    try:
        handle = spawn_cuboid(
            size=args.size,
            position=args.position,
            color=[1.0, 0.0, 0.0],
            dynamic=True,
        )
        sim = get_sim()
        actual_pos = sim.getObjectPosition(handle, -1)

        print_pass(f"spawn_cuboid created handle={handle}")
        print_pass(f"actual position={actual_pos}")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"spawn_cuboid failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
