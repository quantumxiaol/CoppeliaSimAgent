#!/usr/bin/env python3
"""Live test: check_collision with overlapping and separated cuboids."""

from __future__ import annotations

import sys

from live_tool_common import print_fail, print_header, print_pass

from coppeliasimagent.tools.primitives import set_object_pose, spawn_cuboid
from coppeliasimagent.tools.scene import check_collision


def main() -> int:
    print_header("Live Tool Test: check_collision")

    try:
        h1 = spawn_cuboid(size=[0.1, 0.1, 0.1], position=[0.0, 0.0, 0.3], color=[1.0, 0.0, 0.0])
        h2 = spawn_cuboid(size=[0.1, 0.1, 0.1], position=[0.03, 0.0, 0.3], color=[0.0, 1.0, 0.0])

        colliding = check_collision(h1, h2)
        print_pass(f"overlap collision result={colliding}")

        set_object_pose(handle=h2, position=[0.6, 0.0, 0.3])
        separated = check_collision(h1, h2)
        print_pass(f"separated collision result={separated}")

        if not colliding:
            print_fail("expected overlap case to collide, but got False")
            return 1

        if separated:
            print_fail("expected separated case to be non-colliding, but got True")
            return 1

        print_pass("check_collision verification passed")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"collision test failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
