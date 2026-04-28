"""Point-cloud surface and polishing helpers."""

from __future__ import annotations

import time
import math

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .kinematics import move_ik_target
from .schemas import (
    CreatePointCloudSurfaceFromShapeInput,
    CreatePointCloudPotteryCylinderInput,
    ExecutePolishingPathInput,
    ExecutePolishingGrooveInput,
    GetPointCloudStatsInput,
    InsertPointsIntoPointCloudInput,
    RemovePointsNearToolInput,
    SimulatePolishingContactInput,
    SimulatePolishingStepInput,
)
from .primitives import remove_object, rename_object, set_object_visibility, spawn_primitive

_POINT_CLOUD_COUNTS: dict[int, int] = {}


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _rgb255(color: list[float] | None) -> list[int] | None:
    if color is None:
        return None
    return [max(0, min(255, int(round(float(channel) * 255)))) for channel in color]


def _flat_points(points: list[list[float]]) -> list[float]:
    return [float(value) for point in points for value in point]


def _generate_cylinder_shell_points(
    *,
    radius: float,
    height: float,
    center: list[float],
    grid_size: float,
    layers: int,
    wall_thickness: float,
    angular_step_deg: float | None,
    include_caps: bool,
) -> list[list[float]]:
    angular_step = math.radians(angular_step_deg if angular_step_deg is not None else max(2.0, math.degrees(grid_size / radius)))
    z_steps = max(1, int(round(height / grid_size)))
    z_values = [center[2] - height / 2.0 + i * height / z_steps for i in range(z_steps + 1)]
    if layers == 1:
        radii = [radius]
    else:
        radii = [radius - wall_thickness * i / (layers - 1) for i in range(layers)]
        radii = [r for r in radii if r > 0.0]

    points: list[list[float]] = []
    angle_count = max(8, int(math.ceil((2.0 * math.pi) / angular_step)))
    for r in radii:
        for zi, z in enumerate(z_values):
            for ai in range(angle_count):
                theta = (2.0 * math.pi * ai) / angle_count
                points.append([center[0] + r * math.cos(theta), center[1] + r * math.sin(theta), z])
            if include_caps and zi in (0, len(z_values) - 1) and len(radii) > 1:
                radial_steps = max(1, int(round(r / grid_size)))
                for ri in range(1, radial_steps):
                    cap_r = r * ri / radial_steps
                    cap_angles = max(8, int(math.ceil((2.0 * math.pi * cap_r) / grid_size)))
                    for ai in range(cap_angles):
                        theta = (2.0 * math.pi * ai) / cap_angles
                        points.append([center[0] + cap_r * math.cos(theta), center[1] + cap_r * math.sin(theta), z])
    return points


def create_point_cloud_surface_from_shape(
    shape_handle: int,
    grid_size: float = 0.02,
    point_size: float = 0.01,
    color: list[float] | None = None,
    hide_source_shape: bool = False,
    remove_source_shape: bool = False,
) -> dict[str, object]:
    """Create an OC-tree backed point cloud by sampling a shape."""
    try:
        payload = CreatePointCloudSurfaceFromShapeInput.model_validate(
            {
                "shape_handle": shape_handle,
                "grid_size": grid_size,
                "point_size": point_size,
                "color": color if color is not None else [0.8, 0.8, 0.8],
                "hide_source_shape": hide_source_shape,
                "remove_source_shape": remove_source_shape,
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
    source_action = "kept"
    if payload.remove_source_shape:
        remove_object(payload.shape_handle)
        source_action = "removed"
    elif payload.hide_source_shape:
        set_object_visibility(payload.shape_handle, visible=False, include_descendants=False)
        source_action = "hidden"

    return {
        "point_cloud_handle": pc_handle,
        "shape_handle": payload.shape_handle,
        "grid_size": payload.grid_size,
        "point_size": payload.point_size,
        "point_count": total,
        "source_shape_action": source_action,
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


def create_point_cloud_pottery_cylinder(
    radius: float = 0.11,
    height: float = 0.45,
    center: list[float] | None = None,
    grid_size: float = 0.015,
    point_size: float = 0.008,
    color: list[float] | None = None,
    alias: str = "point_cloud_pottery_cylinder",
    keep_source_shape: bool = False,
    layers: int = 1,
    wall_thickness: float = 0.0,
    angular_step_deg: float | None = None,
    include_caps: bool = True,
    use_explicit_points: bool = False,
) -> dict[str, object]:
    """Create a point-cloud cylinder and hide/remove its source mesh by default."""
    try:
        payload = CreatePointCloudPotteryCylinderInput.model_validate(
            {
                "radius": radius,
                "height": height,
                "center": center if center is not None else [0.55, 0.0, 0.225],
                "grid_size": grid_size,
                "point_size": point_size,
                "color": color if color is not None else [0.9, 0.65, 0.35],
                "alias": alias,
                "keep_source_shape": keep_source_shape,
                "layers": layers,
                "wall_thickness": wall_thickness,
                "angular_step_deg": angular_step_deg,
                "include_caps": include_caps,
                "use_explicit_points": use_explicit_points,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    if payload.use_explicit_points or payload.layers > 1:
        sim = get_sim()
        if not hasattr(sim, "createPointCloud") or not hasattr(sim, "insertPointsIntoPointCloud"):
            raise RuntimeError("Current CoppeliaSim API does not expose explicit point-cloud insertion")
        points = _generate_cylinder_shell_points(
            radius=payload.radius,
            height=payload.height,
            center=payload.center,
            grid_size=payload.grid_size,
            layers=payload.layers,
            wall_thickness=payload.wall_thickness,
            angular_step_deg=payload.angular_step_deg,
            include_caps=payload.include_caps,
        )
        pc_handle = int(sim.createPointCloud(payload.grid_size, 1, 0, payload.point_size))
        total = int(sim.insertPointsIntoPointCloud(pc_handle, 0, _flat_points(points), _rgb255(payload.color)))
        _POINT_CLOUD_COUNTS[pc_handle] = total
        return {
            "point_cloud_handle": pc_handle,
            "alias": payload.alias,
            "radius": payload.radius,
            "height": payload.height,
            "center": payload.center,
            "grid_size": payload.grid_size,
            "point_size": payload.point_size,
            "point_count": total,
            "generated_points": len(points),
            "layers": payload.layers,
            "wall_thickness": payload.wall_thickness,
            "source_shape_handle": None,
            "source_shape_action": "not_created",
            "source_shape_kept": False,
        }

    source_handle = spawn_primitive(
        primitive="cylinder",
        size=[payload.radius * 2.0, payload.radius * 2.0, payload.height],
        position=payload.center,
        color=payload.color,
        dynamic=False,
    )
    rename_object(source_handle, f"{payload.alias}_source")
    cloud = create_point_cloud_surface_from_shape(
        shape_handle=source_handle,
        grid_size=payload.grid_size,
        point_size=payload.point_size,
        color=payload.color,
        remove_source_shape=not payload.keep_source_shape,
    )
    cloud.update(
        {
            "alias": payload.alias,
            "radius": payload.radius,
            "height": payload.height,
            "center": payload.center,
            "source_shape_handle": source_handle,
            "source_shape_kept": payload.keep_source_shape,
        }
    )
    return cloud


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


def simulate_polishing_contact(
    surface_cloud_handle: int,
    tool_position: list[float],
    contact_radius: float,
    removal_depth: float = 0.0,
) -> dict[str, object]:
    """Remove surface points around an explicit contact position."""
    try:
        payload = SimulatePolishingContactInput.model_validate(
            {
                "surface_cloud_handle": surface_cloud_handle,
                "tool_position": tool_position,
                "contact_radius": contact_radius,
                "removal_depth": removal_depth,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "removePointsFromPointCloud"):
        raise RuntimeError("Current CoppeliaSim API does not expose removePointsFromPointCloud")

    total = int(
        sim.removePointsFromPointCloud(
            payload.surface_cloud_handle,
            0,
            payload.tool_position,
            payload.contact_radius + payload.removal_depth,
        )
    )
    previous = _POINT_CLOUD_COUNTS.get(payload.surface_cloud_handle)
    _POINT_CLOUD_COUNTS[payload.surface_cloud_handle] = total
    return {
        "point_cloud_handle": payload.surface_cloud_handle,
        "tool_position": payload.tool_position,
        "radius": payload.contact_radius,
        "tolerance": payload.removal_depth,
        "point_count": total,
        "removed_estimate": None if previous is None else max(0, previous - total),
    }


def execute_polishing_groove(
    surface_cloud_handle: int,
    start_position: list[float],
    end_position: list[float],
    contact_radius: float,
    removal_depth: float = 0.0,
    steps: int = 12,
) -> dict[str, object]:
    """Cut a visible groove by applying contact removal along a straight segment."""
    try:
        payload = ExecutePolishingGrooveInput.model_validate(
            {
                "surface_cloud_handle": surface_cloud_handle,
                "start_position": start_position,
                "end_position": end_position,
                "contact_radius": contact_radius,
                "removal_depth": removal_depth,
                "steps": steps,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    results: list[dict[str, object]] = []
    for i in range(payload.steps):
        ratio = i / (payload.steps - 1)
        position = [
            payload.start_position[j] + (payload.end_position[j] - payload.start_position[j]) * ratio
            for j in range(3)
        ]
        results.append(
            simulate_polishing_contact(
                surface_cloud_handle=payload.surface_cloud_handle,
                tool_position=position,
                contact_radius=payload.contact_radius,
                removal_depth=payload.removal_depth,
            )
        )
    return {
        "surface_cloud_handle": payload.surface_cloud_handle,
        "steps": payload.steps,
        "start_position": payload.start_position,
        "end_position": payload.end_position,
        "final_point_count": _POINT_CLOUD_COUNTS.get(payload.surface_cloud_handle),
        "polish_results": results,
    }


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
