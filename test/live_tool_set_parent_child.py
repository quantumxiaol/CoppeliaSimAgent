#!/usr/bin/env python3
"""Live test: set_parent_child by linking two cuboids."""

from __future__ import annotations

import math
import sys

from live_tool_common import print_fail, print_header, print_pass

from coppeliasimagent.core.connection import get_sim
from coppeliasimagent.tools.models import set_parent_child
from coppeliasimagent.tools.primitives import set_object_pose, spawn_cuboid


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b, strict=True)))


def main() -> int:
    print_header("Live Tool Test: set_parent_child")

    try:
        parent = spawn_cuboid(size=[0.1, 0.1, 0.1], position=[0.0, 0.0, 0.4], color=[0.0, 0.0, 1.0])
        child = spawn_cuboid(size=[0.05, 0.05, 0.05], position=[0.2, 0.0, 0.4], color=[1.0, 1.0, 0.0])

        sim = get_sim()
        child_before = list(sim.getObjectPosition(child, -1))

        set_parent_child(child_handle=child, parent_handle=parent, keep_in_place=True)
        set_object_pose(handle=parent, position=[0.1, 0.2, 0.4])

        child_after = list(sim.getObjectPosition(child, -1))
        moved_distance = _distance(child_before, child_after)

        print_pass(f"parent={parent}, child={child}")
        print_pass(f"child before={child_before}, after={child_after}")

        if moved_distance < 1e-4:
            print_fail("child did not move after parent motion")
            return 1

        print_pass("set_parent_child verification passed")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"set_parent_child test failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
