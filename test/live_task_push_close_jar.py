#!/usr/bin/env python3
"""Live task runner: load close_jar and push the jar with an ABB IRB4600 arm.

This script is intended to run against a real CoppeliaSim instance with the
simulation already started.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from pathlib import Path

from live_tool_common import parse_vec3, print_fail, print_header, print_pass

from coppeliasimagent.core.connection import get_sim
from coppeliasimagent.tools.kinematics import (
    configure_abb_arm_drive,
    set_joint_position,
    set_joint_target_position,
)
from coppeliasimagent.tools.models import load_model
from coppeliasimagent.tools.scene import find_objects

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE_MODEL = ROOT / "robot_ttm" / "close_jar.ttm"
DEFAULT_ABB_MODEL = ROOT / "robot_ttm" / "ABB IRB 4600-40-255.ttm"
DEFAULT_PRE_POSE_DEG = [0.0, -28.0, 42.0, 0.0, -18.0, 0.0]
DEFAULT_PUSH_POSE_DEG = [0.0, -16.0, 58.0, 0.0, -26.0, 0.0]


def parse_joint_vec(raw: str) -> list[float]:
    parts = [item.strip() for item in raw.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("joint vector must be in format j1,j2,j3,j4,j5,j6")
    try:
        return [float(item) for item in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid float in joint vector: {raw}") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Push the close_jar jar with an ABB IRB4600")
    parser.add_argument(
        "--scene-model-path",
        default=str(DEFAULT_SCENE_MODEL),
        help="Path to close_jar.ttm",
    )
    parser.add_argument(
        "--abb-model-path",
        default=str(DEFAULT_ABB_MODEL),
        help="Path to ABB IRB 4600-40-255.ttm",
    )
    parser.add_argument(
        "--scene-position",
        type=parse_vec3,
        default=[0.0, 0.0, 0.0],
        help="close_jar base position x,y,z",
    )
    parser.add_argument(
        "--scene-orientation-deg",
        type=parse_vec3,
        default=[0.0, 0.0, 0.0],
        help="close_jar base orientation rx,ry,rz in degrees",
    )
    parser.add_argument(
        "--abb-position",
        type=parse_vec3,
        default=None,
        help="ABB base position x,y,z. If omitted, computed from jar position and --abb-offset-from-jar.",
    )
    parser.add_argument(
        "--abb-offset-from-jar",
        type=parse_vec3,
        default=[-0.85, 0.0, 0.0],
        help="ABB base offset relative to jar x,y,z when --abb-position is omitted",
    )
    parser.add_argument(
        "--abb-orientation-deg",
        type=parse_vec3,
        default=[0.0, 0.0, 0.0],
        help="ABB base orientation rx,ry,rz in degrees",
    )
    parser.add_argument(
        "--jar-name",
        default="jar1",
        help="Exact shape alias to use as push target",
    )
    parser.add_argument(
        "--pre-pose-deg",
        type=parse_joint_vec,
        default=DEFAULT_PRE_POSE_DEG,
        help="ABB contact-approach pose in degrees",
    )
    parser.add_argument(
        "--push-pose-deg",
        type=parse_joint_vec,
        default=DEFAULT_PUSH_POSE_DEG,
        help="ABB pushing pose in degrees",
    )
    parser.add_argument(
        "--max-force",
        type=float,
        default=2000.0,
        help="Per-joint target force/torque used by configure_abb_arm_drive",
    )
    parser.add_argument(
        "--push-wait-seconds",
        type=float,
        default=2.5,
        help="Time to wait after target positions are sent",
    )
    parser.add_argument(
        "--success-threshold",
        type=float,
        default=0.01,
        help="Minimum jar displacement in meters to count as success",
    )
    return parser.parse_args()


def _resolve_existing_file(raw_path: str) -> Path:
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def _safe_object_alias(sim: object, handle: int) -> str:
    if hasattr(sim, "getObjectAlias"):
        try:
            return str(sim.getObjectAlias(handle))
        except Exception:
            pass
    return str(sim.getObjectName(handle))


def _object_type(sim: object, handle: int) -> int:
    if hasattr(sim, "getObjectType"):
        try:
            return int(sim.getObjectType(handle))
        except Exception:
            pass
    if hasattr(sim, "objintparam_type"):
        return int(sim.getObjectInt32Param(handle, sim.objintparam_type))
    raise RuntimeError("Cannot resolve object type for current CoppeliaSim API")


def _abb_joint_handles(sim: object, robot_handle: int) -> list[int]:
    pattern = re.compile(r"IRB4600_joint(\d+)$")
    joint_type = int(getattr(sim, "object_joint_type"))
    indexed_handles: list[tuple[int, int]] = []
    for handle in sim.getObjectsInTree(robot_handle):
        if _object_type(sim, handle) != joint_type:
            continue
        alias = _safe_object_alias(sim, handle)
        match = pattern.search(alias)
        if match is None:
            continue
        indexed_handles.append((int(match.group(1)), int(handle)))
    indexed_handles.sort(key=lambda item: item[0])
    return [handle for _, handle in indexed_handles]


def _deg_to_rad(values: list[float]) -> list[float]:
    return [math.radians(value) for value in values]


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def main() -> int:
    args = parse_args()
    print_header("Live Task: close_jar push with ABB IRB4600")

    scene_model = _resolve_existing_file(args.scene_model_path)
    abb_model = _resolve_existing_file(args.abb_model_path)
    if not scene_model.exists():
        print_fail(f"scene model not found: {scene_model}")
        return 1
    if not abb_model.exists():
        print_fail(f"ABB model not found: {abb_model}")
        return 1

    try:
        scene_handle = load_model(
            model_path=str(scene_model),
            position=args.scene_position,
            orientation_deg=args.scene_orientation_deg,
        )
        print_pass(f"loaded close_jar handle={scene_handle}")

        jar_matches = find_objects(
            name_query=args.jar_name,
            exact_name=True,
            include_types=["shape"],
            limit=5,
        )
        if not jar_matches:
            print_fail(f"target jar shape not found by exact alias: {args.jar_name}")
            return 1

        jar_info = jar_matches[0]
        jar_handle = int(jar_info["handle"])
        jar_before = [float(value) for value in jar_info["position"]]
        print_pass(f"target jar handle={jar_handle}, position={jar_before}")

        abb_position = (
            args.abb_position
            if args.abb_position is not None
            else [
                float(jar_before[0]) + float(args.abb_offset_from_jar[0]),
                float(jar_before[1]) + float(args.abb_offset_from_jar[1]),
                float(jar_before[2]) + float(args.abb_offset_from_jar[2]),
            ]
        )
        robot_handle = load_model(
            model_path=str(abb_model),
            position=abb_position,
            orientation_deg=args.abb_orientation_deg,
        )

        sim = get_sim()
        robot_alias = _safe_object_alias(sim, robot_handle)
        print_pass(f"loaded ABB handle={robot_handle}, alias={robot_alias}, position={abb_position}")

        joint_handles = _abb_joint_handles(sim, robot_handle)
        if len(joint_handles) < 6:
            print_fail(f"expected 6 ABB joints, found {len(joint_handles)}")
            return 1
        print_pass(f"resolved ABB joints={joint_handles}")

        pre_pose_rad = _deg_to_rad(args.pre_pose_deg)
        push_pose_rad = _deg_to_rad(args.push_pose_deg)

        for handle, position in zip(joint_handles[:6], pre_pose_rad, strict=True):
            set_joint_position(handle, position)
        print_pass(f"applied pre-pose degrees={args.pre_pose_deg}")

        drive_result = configure_abb_arm_drive(
            robot_path=f"/{robot_alias}",
            joint_mode="dynamic",
            dyn_ctrl_mode="position",
            max_force_or_torque=args.max_force,
            signed_value=True,
            include_aux_joint=False,
            reset_dynamics=True,
        )
        print_pass(f"configured ABB dynamic drive={drive_result}")

        for handle, target in zip(joint_handles[:6], push_pose_rad, strict=True):
            set_joint_target_position(handle, target)
        print_pass(f"sent push pose degrees={args.push_pose_deg}")

        time.sleep(max(0.0, args.push_wait_seconds))

        jar_after = [float(value) for value in sim.getObjectPosition(jar_handle, -1)]
        displacement = _distance(jar_before, jar_after)
        if displacement >= args.success_threshold:
            print_pass(
                f"jar moved from {jar_before} to {jar_after}, displacement={displacement:.4f} m"
            )
            return 0

        print_fail(
            "jar displacement below threshold: "
            f"before={jar_before}, after={jar_after}, displacement={displacement:.4f} m"
        )
        return 1
    except Exception as exc:  # noqa: BLE001
        print_fail(f"task failed: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
