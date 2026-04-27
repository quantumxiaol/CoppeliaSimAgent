#!/usr/bin/env python3
"""Live task: create a point-cloud surface and remove points with a tool proxy."""

from __future__ import annotations

import argparse
import sys

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.tools.point_cloud import (
    create_point_cloud_surface_from_shape,
    get_point_cloud_stats,
    simulate_polishing_step,
)
from coppeliasimagent.tools.primitives import spawn_primitive


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exercise point-cloud polishing tools")
    parser.add_argument("--surface-position", type=parse_vec3, default=[0.0, 0.0, 0.2])
    parser.add_argument("--tool-position", type=parse_vec3, default=[0.0, 0.0, 0.25])
    parser.add_argument("--grid-size", type=float, default=0.02)
    parser.add_argument("--point-size", type=float, default=0.01)
    parser.add_argument("--contact-radius", type=float, default=0.04)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_header("Live Task: point-cloud polishing")
    try:
        surface = spawn_primitive(
            primitive="cuboid",
            size=[0.2, 0.2, 0.04],
            position=args.surface_position,
            color=[0.7, 0.7, 0.7],
            dynamic=False,
        )
        tool = spawn_primitive(
            primitive="sphere",
            size=[0.03, 0.03, 0.03],
            position=args.tool_position,
            color=[1.0, 0.2, 0.1],
            dynamic=False,
        )
        cloud = create_point_cloud_surface_from_shape(
            shape_handle=surface,
            grid_size=args.grid_size,
            point_size=args.point_size,
            color=[0.1, 0.6, 1.0],
        )
        before = get_point_cloud_stats(cloud["point_cloud_handle"])
        polish = simulate_polishing_step(
            tool_handle=tool,
            surface_cloud_handle=cloud["point_cloud_handle"],
            contact_radius=args.contact_radius,
        )
        after = get_point_cloud_stats(cloud["point_cloud_handle"])

        print_pass(f"surface handle={surface}, tool handle={tool}")
        print_pass(f"cloud created: {cloud}")
        print_pass(f"stats before: {before}")
        print_pass(f"polish result: {polish}")
        print_pass(f"stats after: {after}")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"point-cloud polishing task failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
