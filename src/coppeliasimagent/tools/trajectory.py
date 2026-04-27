"""Trajectory execution helpers."""

from __future__ import annotations

import time

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .kinematics import move_ik_target, set_joint_position, set_joint_target_position
from .schemas import ExecuteCartesianWaypointsInput, ExecuteJointTrajectoryInput


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def execute_joint_trajectory(
    joint_handles: list[int],
    waypoints: list[list[float]],
    mode: str = "target_position",
    dwell_seconds: float = 0.0,
    motion_params: list[float] | None = None,
) -> dict[str, object]:
    """Execute a sequence of joint-space waypoints."""
    try:
        payload = ExecuteJointTrajectoryInput.model_validate(
            {
                "joint_handles": joint_handles,
                "waypoints": waypoints,
                "mode": mode,
                "dwell_seconds": dwell_seconds,
                "motion_params": motion_params,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    for waypoint in payload.waypoints:
        for handle, value in zip(payload.joint_handles, waypoint, strict=True):
            if payload.mode.value == "position":
                set_joint_position(handle=handle, position=value)
            else:
                set_joint_target_position(
                    handle=handle,
                    target_position=value,
                    motion_params=payload.motion_params,
                )
        if payload.dwell_seconds > 0.0:
            time.sleep(payload.dwell_seconds)

    sim = get_sim()
    final_positions = [float(sim.getJointPosition(handle)) for handle in payload.joint_handles]
    return {
        "joint_handles": payload.joint_handles,
        "waypoint_count": len(payload.waypoints),
        "mode": payload.mode.value,
        "final_positions": final_positions,
    }


def execute_cartesian_waypoints(
    environment_handle: int,
    group_handle: int,
    target_handle: int,
    waypoints: list[list[float]],
    relative_to: int = -1,
    steps_per_waypoint: int = 1,
    dwell_seconds: float = 0.0,
) -> dict[str, object]:
    """Move an IK target through Cartesian waypoints and solve each segment."""
    try:
        payload = ExecuteCartesianWaypointsInput.model_validate(
            {
                "environment_handle": environment_handle,
                "group_handle": group_handle,
                "target_handle": target_handle,
                "waypoints": waypoints,
                "relative_to": relative_to,
                "steps_per_waypoint": steps_per_waypoint,
                "dwell_seconds": dwell_seconds,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    for waypoint in payload.waypoints:
        move_ik_target(
            environment_handle=payload.environment_handle,
            group_handle=payload.group_handle,
            target_handle=payload.target_handle,
            position=waypoint,
            relative_to=payload.relative_to,
            steps=payload.steps_per_waypoint,
        )
        if payload.dwell_seconds > 0.0:
            time.sleep(payload.dwell_seconds)

    sim = get_sim()
    return {
        "environment_handle": payload.environment_handle,
        "group_handle": payload.group_handle,
        "target_handle": payload.target_handle,
        "waypoint_count": len(payload.waypoints),
        "final_position": [float(v) for v in sim.getObjectPosition(payload.target_handle, payload.relative_to)],
    }
