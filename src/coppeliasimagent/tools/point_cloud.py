"""Point-cloud surface and polishing helpers."""

from __future__ import annotations

import time

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .kinematics import move_ik_target
from .schemas import (
    CreatePointCloudSurfaceFromShapeInput,
    ExecutePolishingPathInput,
    GetPointCloudStatsInput,
    InsertPointsIntoPointCloudInput,
    RemovePointsNearToolInput,
    SimulatePolishingStepInput,
)

_POINT_CLOUD_COUNTS: dict[int, int] = {}


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _rgb255(color: list[float] | None) -> list[int] | None:
    if color is None:
        return None
    return [max(0, min(255, int(round(float(channel) * 255)))) for channel in color]


def _flat_points(points: list[list[float]]) -> list[float]:
    return [float(value) for point in points for value in point]


def create_point_cloud_surface_from_shape(
    shape_handle: int,
    grid_size: float = 0.02,
    point_size: float = 0.01,
    color: list[float] | None = None,
) -> dict[str, object]:
    """Create an OC-tree backed point cloud by sampling a shape."""
    try:
        payload = CreatePointCloudSurfaceFromShapeInput.model_validate(
            {
                "shape_handle": shape_handle,
                "grid_size": grid_size,
                "point_size": point_size,
                "color": color if color is not None else [0.8, 0.8, 0.8],
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "createPointCloud") or not hasattr(sim, "insertObjectIntoPointCloud"):
        raise RuntimeError("Current CoppeliaSim API does not expose point cloud creation/insertion")

    pc_handle = int(sim.createPointCloud(payload.grid_size, 1, 0, payload.point_size))
    total = int(
        sim.insertObjectIntoPointCloud(
            pc_handle,
            payload.shape_handle,
            0,
            payload.grid_size,
            _rgb255(payload.color),
        )
    )
    _POINT_CLOUD_COUNTS[pc_handle] = total
    return {
        "point_cloud_handle": pc_handle,
        "shape_handle": payload.shape_handle,
        "grid_size": payload.grid_size,
        "point_size": payload.point_size,
        "point_count": total,
    }


def insert_points_into_point_cloud(
    point_cloud_handle: int,
    points: list[list[float]],
    color: list[float] | None = None,
) -> dict[str, object]:
    """Insert explicit world-frame points into a point cloud."""
    try:
        payload = InsertPointsIntoPointCloudInput.model_validate(
            {
                "point_cloud_handle": point_cloud_handle,
                "points": points,
                "color": color,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "insertPointsIntoPointCloud"):
        raise RuntimeError("Current CoppeliaSim API does not expose insertPointsIntoPointCloud")
    total = int(
        sim.insertPointsIntoPointCloud(
            payload.point_cloud_handle,
            0,
            _flat_points(payload.points),
            _rgb255(payload.color),
        )
    )
    _POINT_CLOUD_COUNTS[payload.point_cloud_handle] = total
    return {
        "point_cloud_handle": payload.point_cloud_handle,
        "inserted_points": len(payload.points),
        "point_count": total,
    }


def remove_points_near_tool(
    point_cloud_handle: int,
    tool_handle: int,
    radius: float,
    tolerance: float = 0.0,
) -> dict[str, object]:
    """Remove point-cloud points around a tool object's current position."""
    try:
        payload = RemovePointsNearToolInput.model_validate(
            {
                "point_cloud_handle": point_cloud_handle,
                "tool_handle": tool_handle,
                "radius": radius,
                "tolerance": tolerance,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "removePointsFromPointCloud"):
        raise RuntimeError("Current CoppeliaSim API does not expose removePointsFromPointCloud")
    tool_position = [float(v) for v in sim.getObjectPosition(payload.tool_handle, -1)]
    total = int(
        sim.removePointsFromPointCloud(
            payload.point_cloud_handle,
            0,
            tool_position,
            payload.radius + payload.tolerance,
        )
    )
    previous = _POINT_CLOUD_COUNTS.get(payload.point_cloud_handle)
    _POINT_CLOUD_COUNTS[payload.point_cloud_handle] = total
    return {
        "point_cloud_handle": payload.point_cloud_handle,
        "tool_handle": payload.tool_handle,
        "tool_position": tool_position,
        "radius": payload.radius,
        "tolerance": payload.tolerance,
        "point_count": total,
        "removed_estimate": None if previous is None else max(0, previous - total),
    }


def get_point_cloud_stats(point_cloud_handle: int) -> dict[str, object]:
    """Return known point-cloud count and options when the simulator supports them."""
    try:
        payload = GetPointCloudStatsInput.model_validate({"point_cloud_handle": point_cloud_handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    options = None
    if hasattr(sim, "getPointCloudOptions"):
        options = sim.getPointCloudOptions(payload.point_cloud_handle)
    return {
        "point_cloud_handle": payload.point_cloud_handle,
        "known_point_count": _POINT_CLOUD_COUNTS.get(payload.point_cloud_handle),
        "options": options,
    }


def simulate_polishing_step(
    tool_handle: int,
    surface_cloud_handle: int,
    contact_radius: float,
    removal_depth: float = 0.0,
) -> dict[str, object]:
    """Remove surface points near the polishing tool."""
    try:
        payload = SimulatePolishingStepInput.model_validate(
            {
                "tool_handle": tool_handle,
                "surface_cloud_handle": surface_cloud_handle,
                "contact_radius": contact_radius,
                "removal_depth": removal_depth,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    return remove_points_near_tool(
        point_cloud_handle=payload.surface_cloud_handle,
        tool_handle=payload.tool_handle,
        radius=payload.contact_radius,
        tolerance=payload.removal_depth,
    )


def execute_polishing_path(
    environment_handle: int,
    group_handle: int,
    target_handle: int,
    tool_handle: int,
    surface_cloud_handle: int,
    waypoints: list[list[float]],
    contact_radius: float,
    removal_depth: float = 0.0,
    relative_to: int = -1,
    steps_per_waypoint: int = 1,
    dwell_seconds: float = 0.0,
) -> dict[str, object]:
    """Move an IK target through a path and polish after each waypoint."""
    try:
        payload = ExecutePolishingPathInput.model_validate(
            {
                "environment_handle": environment_handle,
                "group_handle": group_handle,
                "target_handle": target_handle,
                "tool_handle": tool_handle,
                "surface_cloud_handle": surface_cloud_handle,
                "waypoints": waypoints,
                "contact_radius": contact_radius,
                "removal_depth": removal_depth,
                "relative_to": relative_to,
                "steps_per_waypoint": steps_per_waypoint,
                "dwell_seconds": dwell_seconds,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    polish_results: list[dict[str, object]] = []
    for waypoint in payload.waypoints:
        move_ik_target(
            environment_handle=payload.environment_handle,
            group_handle=payload.group_handle,
            target_handle=payload.target_handle,
            position=waypoint,
            relative_to=payload.relative_to,
            steps=payload.steps_per_waypoint,
        )
        polish_results.append(
            simulate_polishing_step(
                tool_handle=payload.tool_handle,
                surface_cloud_handle=payload.surface_cloud_handle,
                contact_radius=payload.contact_radius,
                removal_depth=payload.removal_depth,
            )
        )
        if payload.dwell_seconds > 0.0:
            time.sleep(payload.dwell_seconds)

    return {
        "surface_cloud_handle": payload.surface_cloud_handle,
        "tool_handle": payload.tool_handle,
        "waypoint_count": len(payload.waypoints),
        "final_point_count": _POINT_CLOUD_COUNTS.get(payload.surface_cloud_handle),
        "polish_results": polish_results,
    }
