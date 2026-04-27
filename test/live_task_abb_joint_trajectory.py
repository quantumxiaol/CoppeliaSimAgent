#!/usr/bin/env python3
"""Live task: load ABB IRB4600 and execute a small joint trajectory."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.tools.kinematics import configure_abb_arm_drive
from coppeliasimagent.tools.models import load_model
from coppeliasimagent.tools.runtime import wait_seconds, wait_until_state
from coppeliasimagent.tools.scene import find_objects
from coppeliasimagent.tools.simulation import start_simulation
from coppeliasimagent.tools.trajectory import execute_joint_trajectory
from coppeliasimagent.tools.verification import verify_joint_positions_reached


def parse_joint_vec(raw: str) -> list[float]:
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("joint vector must contain 6 comma-separated radians")
    try:
        return [float(p) for p in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid float in joint vector: {raw}") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Execute a live ABB joint trajectory")
    parser.add_argument("--model-path", default="robot_ttm/ABB IRB 4600-40-255.ttm")
    parser.add_argument("--position", type=parse_vec3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--orientation-deg", type=parse_vec3, default=[0.0, 0.0, 0.0])
    parser.add_argument("--pose-a", type=parse_joint_vec, default=[0.0, -0.4, 0.6, 0.0, -0.2, 0.0])
    parser.add_argument("--pose-b", type=parse_joint_vec, default=[0.15, -0.25, 0.75, 0.0, -0.35, 0.0])
    parser.add_argument("--dwell-seconds", type=float, default=0.5)
    parser.add_argument("--tolerance", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_header("Live Task: ABB joint trajectory")
    model_path = Path(args.model_path).expanduser()
    if not model_path.exists():
        print_fail(f"model file not found: {model_path}")
        return 1

    try:
        robot_handle = load_model(str(model_path), args.position, args.orientation_deg)
        print_pass(f"ABB loaded: handle={robot_handle}")

        drive = configure_abb_arm_drive(robot_path="/IRB4600", max_force_or_torque=2000.0)
        joint_handles = drive["joint_handles"]
        print_pass(f"ABB drive configured: joints={joint_handles}")

        start_simulation()
        wait_until_state("simulation_advancing_running", timeout_s=3.0)
        out = execute_joint_trajectory(
            joint_handles=joint_handles,
            waypoints=[args.pose_a, args.pose_b],
            mode="target_position",
            dwell_seconds=args.dwell_seconds,
        )
        wait_seconds(args.dwell_seconds)
        check = verify_joint_positions_reached(joint_handles, args.pose_b, tolerance=args.tolerance)

        print_pass(f"trajectory executed: {out}")
        print_pass(f"verification: {check}")
        matches = find_objects(name_query="IRB4600", include_types=["joint", "shape", "dummy"], limit=20)
        print_pass(f"IRB4600 scene objects found={len(matches)}")
        return 0 if check["reached"] else 2
    except Exception as exc:  # noqa: BLE001
        print_fail(f"ABB trajectory task failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
