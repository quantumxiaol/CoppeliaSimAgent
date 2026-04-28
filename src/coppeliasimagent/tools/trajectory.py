"""Trajectory execution helpers."""

from __future__ import annotations

import time

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .kinematics import move_ik_target, set_joint_position, set_joint_target_position
from .runtime import step_simulation
from .schemas import ExecuteCartesianWaypointsInput, ExecuteJointTrajectoryInput, ExecuteSteppedIKPathInput


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


def execute_stepped_ik_path(
    environment_handle: int,
    group_handle: int,
    target_handle: int,
    waypoints: list[list[float]],
    relative_to: int = -1,
    ik_steps_per_waypoint: int = 1,
    simulation_steps_per_waypoint: int = 1,
    start_simulation: bool = True,
    keep_stepping_enabled: bool = True,
    record_handles: list[int] | None = None,
    record_every: int = 1,
) -> dict[str, object]:
    """Move an IK target and advance simulation in one Remote API session."""
    try:
        payload = ExecuteSteppedIKPathInput.model_validate(
            {
                "environment_handle": environment_handle,
                "group_handle": group_handle,
                "target_handle": target_handle,
                "waypoints": waypoints,
                "relative_to": relative_to,
                "ik_steps_per_waypoint": ik_steps_per_waypoint,
                "simulation_steps_per_waypoint": simulation_steps_per_waypoint,
                "start_simulation": start_simulation,
                "keep_stepping_enabled": keep_stepping_enabled,
                "record_handles": record_handles or [],
                "record_every": record_every,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    records: list[dict[str, object]] = []
    last_step_result: dict[str, object] | None = None
    for index, waypoint in enumerate(payload.waypoints, start=1):
        move_ik_target(
            environment_handle=payload.environment_handle,
            group_handle=payload.group_handle,
            target_handle=payload.target_handle,
            position=waypoint,
            relative_to=payload.relative_to,
            steps=payload.ik_steps_per_waypoint,
        )
        last_step_result = step_simulation(
            steps=payload.simulation_steps_per_waypoint,
            start_if_stopped=payload.start_simulation,
            keep_stepping_enabled=payload.keep_stepping_enabled,
        )
        if index == 1 or index == len(payload.waypoints) or index % payload.record_every == 0:
            sim = get_sim()
            records.append(
                {
                    "index": index,
                    "waypoint": waypoint,
                    "target_position": [
                        float(v) for v in sim.getObjectPosition(payload.target_handle, payload.relative_to)
                    ],
                    "objects": {
                        str(handle): [float(v) for v in sim.getObjectPosition(handle, payload.relative_to)]
                        for handle in payload.record_handles
                    },
                }
            )

    sim = get_sim()
    return {
        "environment_handle": payload.environment_handle,
        "group_handle": payload.group_handle,
        "target_handle": payload.target_handle,
        "waypoint_count": len(payload.waypoints),
        "final_position": [float(v) for v in sim.getObjectPosition(payload.target_handle, payload.relative_to)],
        "records": records,
        "last_step_result": last_step_result,
    }
