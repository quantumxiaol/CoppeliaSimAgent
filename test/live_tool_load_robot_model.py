#!/usr/bin/env python3
"""Live test: load a robot model (.ttm) into CoppeliaSim."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.core.connection import get_sim
from coppeliasimagent.tools.models import load_model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Load one robot model into CoppeliaSim")
    parser.add_argument("--model-path", required=True, help="Path to .ttm model file")
    parser.add_argument(
        "--position",
        type=parse_vec3,
        default=[0.0, 0.0, 0.0],
        help="Base position x,y,z (z is height)",
    )
    parser.add_argument(
        "--orientation-deg",
        type=parse_vec3,
        default=[0.0, 0.0, 0.0],
        help="Base orientation in degrees rx,ry,rz",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_header("Live Tool Test: load_model")

    model_path = Path(args.model_path).expanduser()
    if not model_path.exists():
        print_fail(f"model file not found: {model_path}")
        return 1

    try:
        handle = load_model(
            model_path=str(model_path),
            position=args.position,
            orientation_deg=args.orientation_deg,
        )

        sim = get_sim()
        obj_name = sim.getObjectName(handle)
        actual_pos = sim.getObjectPosition(handle, -1)

        print_pass(f"load_model created handle={handle}, name={obj_name}")
        print_pass(f"actual position={actual_pos}")
        return 0
    except Exception as exc:  # noqa: BLE001
        print_fail(f"load_model failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
