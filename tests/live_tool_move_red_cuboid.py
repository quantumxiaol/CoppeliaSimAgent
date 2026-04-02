#!/usr/bin/env python3
"""Live test: spawn a red cuboid then move it with set_object_pose."""

from __future__ import annotations

import argparse
import math
import sys

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.core.connection import get_sim
from coppeliasimagent.tools.primitives import set_object_pose, spawn_cuboid


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Spawn and move a red cuboid")
    parser.add_argument("--start", type=parse_vec3, default=[0.0, 0.0, 0.3], help="Start position x,y,z")
    parser.add_argument("--target", type=parse_vec3, default=[0.3, 0.0, 0.3], help="Target position x,y,z")
    return parser.parse_args()


def _almost_equal(a: list[float], b: list[float], tol: float = 1e-4) -> bool:
    return all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b, strict=True))


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: set_object_pose")

    try:
        handle = spawn_cuboid(size=[0.1, 0.1, 0.1], position=args.start, color=[1.0, 0.0, 0.0])
        set_object_pose(handle=handle, position=args.target)

        sim = get_sim()
        actual_pos = list(sim.getObjectPosition(handle, -1))

        print_pass(f"spawned handle={handle}, moved to target={args.target}")
        print_pass(f"actual position={actual_pos}")

        if not _almost_equal(actual_pos, args.target):
            print_fail("actual position does not match target")
            return 1

        print_pass("position verification passed")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"move test failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
