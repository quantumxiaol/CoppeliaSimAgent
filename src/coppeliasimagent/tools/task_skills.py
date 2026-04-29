"""Task-level robot skills built from lower-level simulator tools."""

from __future__ import annotations

import math

from pydantic import ValidationError

from ..core.connection import get_connection, get_sim
from ..core.exceptions import ToolValidationError
from .dynamics import get_object_velocity, set_shape_dynamics
from .kinematics import (
    _resolve_object_handle,
    configure_abb_arm_drive,
    find_robot_joints,
    move_ik_target_checked,
    setup_abb_arm_ik,
)
from .primitives import _set_object_alias, set_object_visibility, spawn_composite_object, spawn_cuboid, spawn_primitive
from .runtime import step_simulation
from .scene import check_collision
from .schemas import CreatePusherToolForAbbInput, CreateTabletopPushSceneInput, PushObjectWithAbbInput


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _norm(values: list[float]) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in values))


def _unit_horizontal(values: list[float]) -> list[float]:
    horizontal = [float(values[0]), float(values[1]), 0.0]
    length = _norm(horizontal)
    if length <= 1e-9:
        length = _norm([float(values[0]), float(values[1]), float(values[2])])
        if length <= 1e-9:
            raise RuntimeError("push_direction cannot be zero")
        return [float(values[i]) / length for i in range(3)]
    return [horizontal[i] / length for i in range(3)]


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _dot(a: list[float], b: list[float]) -> float:
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def _find_alias_in_tree(sim: object, root_handle: int, alias: str) -> int:
    if not hasattr(sim, "getObjectsInTree"):
        return -1
    for handle in sim.getObjectsInTree(root_handle):
        try:
            name = str(sim.getObjectAlias(handle) if hasattr(sim, "getObjectAlias") else sim.getObjectName(handle))
        except Exception:
            continue
        if name == alias:
            return int(handle)
    return -1


def _bbox(sim: object, handle: int) -> dict[str, object]:
    attrs = (
        "objfloatparam_objbbox_min_x",
        "objfloatparam_objbbox_min_y",
        "objfloatparam_objbbox_min_z",
        "objfloatparam_objbbox_max_x",
        "objfloatparam_objbbox_max_y",
        "objfloatparam_objbbox_max_z",
    )
    if hasattr(sim, "getObjectFloatParam") and all(hasattr(sim, attr) for attr in attrs):
        values = [float(sim.getObjectFloatParam(handle, getattr(sim, attr))) for attr in attrs]
        mins = values[:3]
        maxs = values[3:]
    else:
        mins = [-0.05, -0.05, -0.05]
        maxs = [0.05, 0.05, 0.05]
    size = [maxs[i] - mins[i] for i in range(3)]
    center = [(mins[i] + maxs[i]) / 2.0 for i in range(3)]
    return {"min": mins, "max": maxs, "size": size, "center": center}


def _extent_along_horizontal(bbox: dict[str, object], direction: list[float]) -> float:
    size = bbox["size"]
    if not isinstance(size, list):
        return 0.05
    half_x = max(0.0, float(size[0]) / 2.0)
    half_y = max(0.0, float(size[1]) / 2.0)
    return max(0.01, abs(direction[0]) * half_x + abs(direction[1]) * half_y)


def _safe_collision(entity1: int, entity2: int) -> bool:
    try:
        return bool(check_collision(entity1, entity2))
    except Exception:
        return False


def _handle_exists(sim: object, handle: int | None) -> bool:
    if handle is None:
        return False
    try:
        if hasattr(sim, "getObjectType"):
            sim.getObjectType(int(handle))
        else:
            sim.getObjectPosition(int(handle), -1)
        return True
    except Exception:
        return False


def _release_remote_stepping() -> None:
    try:
        connection = get_connection()
        client = connection.client
        if hasattr(client, "setStepping"):
            client.setStepping(False)
    except Exception:
        pass


def _step_simulation_checked(*, steps: int, release_on_failure: bool) -> dict[str, object]:
    try:
        return step_simulation(
            steps=steps,
            start_if_stopped=True,
            keep_stepping_enabled=True,
        )
    except Exception as exc:  # noqa: BLE001
        if release_on_failure:
            _release_remote_stepping()
        return {
            "ok": False,
            "failure_reason": "REMOTE_API_STEP_TIMEOUT",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "steps": steps,
        }


def _push_geometry(
    *,
    object_center: list[float],
    object_size: list[float],
    direction: list[float],
    contact_height_ratio: float,
    pre_contact_clearance: float,
    contact_margin: float,
    push_distance: float,
) -> dict[str, object]:
    half_x = float(object_size[0]) / 2.0
    half_y = float(object_size[1]) / 2.0
    extent = max(0.01, abs(direction[0]) * half_x + abs(direction[1]) * half_y)
    contact_z = float(object_center[2]) - float(object_size[2]) / 2.0 + float(object_size[2]) * contact_height_ratio
    side_center = [float(object_center[0]), float(object_center[1]), contact_z]
    contact_point = [
        side_center[i] - direction[i] * (extent + contact_margin)
        for i in range(3)
    ]
    pre_contact_point = [
        side_center[i] - direction[i] * (extent + contact_margin + pre_contact_clearance)
        for i in range(3)
    ]
    post_contact_point = [
        contact_point[i] + direction[i] * push_distance
        for i in range(3)
    ]
    return {
        "extent": extent,
        "contact_point": contact_point,
        "pre_contact_point": pre_contact_point,
        "post_contact_point": post_contact_point,
        "pusher_waypoints": [pre_contact_point, contact_point, post_contact_point],
    }


def _reachability_suggestion(
    *,
    tip_position: list[float],
    direction: list[float],
    object_size: list[float],
    contact_height_ratio: float,
    pre_contact_clearance: float,
    contact_margin: float,
) -> dict[str, object]:
    center_z = tip_position[2] - (-object_size[2] / 2.0 + object_size[2] * contact_height_ratio)
    extent = abs(direction[0]) * object_size[0] / 2.0 + abs(direction[1]) * object_size[1] / 2.0
    suggested_center = [
        tip_position[i] + direction[i] * (extent + contact_margin + pre_contact_clearance)
        for i in range(3)
    ]
    suggested_center[2] = center_z
    return {
        "reason": "Place the object so the pre-contact point lies near the current reachable tip pose.",
        "suggested_object_center": suggested_center,
        "suggested_table_top_z": center_z - object_size[2] / 2.0,
        "suggested_push_direction": direction,
    }


def create_pusher_tool_for_abb(
    robot_path: str = "/IRB4600",
    parent_path: str | None = None,
    alias: str = "pusher_tip",
    shape: str = "cuboid",
    size: list[float] | None = None,
    radius: float = 0.02,
    offset: list[float] | None = None,
    color: list[float] | None = None,
    static: bool = True,
    respondable: bool = True,
    visible: bool = True,
    reuse_existing: bool = True,
) -> dict[str, object]:
    """Create or reuse a visible respondable pusher tool attached near the ABB IK tip."""
    try:
        payload = CreatePusherToolForAbbInput.model_validate(
            {
                "robot_path": robot_path,
                "parent_path": parent_path,
                "alias": alias,
                "shape": shape,
                "size": size if size is not None else [0.04, 0.04, 0.04],
                "radius": radius,
                "offset": offset if offset is not None else [0.0, 0.0, 0.0],
                "color": color if color is not None else [1.0, 0.25, 0.05],
                "static": static,
                "respondable": respondable,
                "visible": visible,
                "reuse_existing": reuse_existing,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    robot_handle = _resolve_object_handle(sim, payload.robot_path)
    default_parent_path = f"{payload.robot_path}/IkTip"
    parent_handle = _resolve_object_handle(
        sim,
        payload.parent_path if payload.parent_path is not None else default_parent_path,
        no_error=True,
    )
    if parent_handle == -1:
        parent_handle = robot_handle

    handle = _find_alias_in_tree(sim, robot_handle, payload.alias) if payload.reuse_existing else -1
    created = False
    if handle == -1:
        primitive_size = payload.size or [payload.radius * 2.0, payload.radius * 2.0, payload.radius * 2.0]
        handle = spawn_primitive(
            primitive=payload.shape.value,
            size=primitive_size,
            position=payload.offset,
            color=payload.color,
            dynamic=not payload.static,
            relative_to=parent_handle,
        )
        created = True
    else:
        sim.setObjectPosition(handle, parent_handle, payload.offset)

    _set_object_alias(sim, handle, payload.alias)
    if hasattr(sim, "setObjectParent"):
        sim.setObjectParent(handle, parent_handle, True)
    set_shape_dynamics(handle, static=payload.static, respondable=payload.respondable)
    set_object_visibility(handle, visible=payload.visible, include_descendants=False)

    return {
        "handle": int(handle),
        "robot_path": payload.robot_path,
        "robot_handle": robot_handle,
        "parent_handle": int(parent_handle),
        "alias": payload.alias,
        "shape": payload.shape.value,
        "offset": payload.offset,
        "static": payload.static,
        "respondable": payload.respondable,
        "visible": payload.visible,
        "created": created,
    }


def create_tabletop_push_scene(
    robot_path: str = "/IRB4600",
    table_size: list[float] | None = None,
    table_height: float | None = None,
    object_radius: float = 0.05,
    object_height: float = 0.12,
    object_mass: float = 0.08,
    object_friction: float | None = 0.45,
    preferred_workspace: list[float] | None = None,
    push_direction: list[float] | None = None,
    contact_height_ratio: float = 0.55,
    pre_contact_clearance: float = 0.05,
    contact_margin: float = 0.005,
    push_distance: float = 0.08,
    max_tip_error: float = 0.02,
    ik_steps_per_waypoint: int = 12,
    alias_prefix: str = "push_test",
    constraint_policy: str = "position_only",
) -> dict[str, object]:
    """Create a tabletop push scene by placing the object in a checked reachable region."""
    try:
        payload = CreateTabletopPushSceneInput.model_validate(
            {
                "robot_path": robot_path,
                "table_size": table_size if table_size is not None else [0.8, 0.6, 0.05],
                "table_height": table_height,
                "object_radius": object_radius,
                "object_height": object_height,
                "object_mass": object_mass,
                "object_friction": object_friction,
                "preferred_workspace": preferred_workspace,
                "push_direction": push_direction,
                "contact_height_ratio": contact_height_ratio,
                "pre_contact_clearance": pre_contact_clearance,
                "contact_margin": contact_margin,
                "push_distance": push_distance,
                "max_tip_error": max_tip_error,
                "ik_steps_per_waypoint": ik_steps_per_waypoint,
                "alias_prefix": alias_prefix,
                "constraint_policy": constraint_policy,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    ik = setup_abb_arm_ik(
        robot_path=payload.robot_path,
        verify_motion=False,
        constraint_policy=payload.constraint_policy.value,
    )
    tip_handle = int(ik["tip_handle"])
    tip_position = [float(v) for v in sim.getObjectPosition(tip_handle, -1)]
    robot_handle = int(ik["robot_handle"])
    robot_position = [float(v) for v in sim.getObjectPosition(robot_handle, -1)]

    if payload.push_direction is None:
        direction = _unit_horizontal([
            tip_position[0] - robot_position[0],
            tip_position[1] - robot_position[1],
            0.0,
        ])
    else:
        direction = _unit_horizontal(payload.push_direction)
    lateral = [-direction[1], direction[0], 0.0]
    object_size = [payload.object_radius * 2.0, payload.object_radius * 2.0, payload.object_height]

    seed_center = payload.preferred_workspace
    if seed_center is None:
        suggestion = _reachability_suggestion(
            tip_position=tip_position,
            direction=direction,
            object_size=object_size,
            contact_height_ratio=payload.contact_height_ratio,
            pre_contact_clearance=payload.pre_contact_clearance,
            contact_margin=payload.contact_margin,
        )
        seed_center = suggestion["suggested_object_center"]
        if not isinstance(seed_center, list):
            seed_center = list(tip_position)
    if payload.table_height is not None:
        seed_center = [float(seed_center[0]), float(seed_center[1]), payload.table_height + payload.object_height / 2.0]

    offsets = (
        [0.0, 0.0, 0.0],
        [payload.pre_contact_clearance, 0.0, 0.0],
        [-payload.pre_contact_clearance, 0.0, 0.0],
        [0.0, payload.object_radius * 1.5, 0.0],
        [0.0, -payload.object_radius * 1.5, 0.0],
        [payload.pre_contact_clearance, payload.object_radius * 1.5, 0.0],
        [payload.pre_contact_clearance, -payload.object_radius * 1.5, 0.0],
    )
    candidates: list[dict[str, object]] = []
    selected: dict[str, object] | None = None
    joint_handles = [int(handle) for handle in find_robot_joints(payload.robot_path)["joint_handles"]]
    for offset in offsets:
        center = [
            float(seed_center[i]) + direction[i] * offset[0] + lateral[i] * offset[1] + offset[2]
            for i in range(3)
        ]
        geometry = _push_geometry(
            object_center=center,
            object_size=object_size,
            direction=direction,
            contact_height_ratio=payload.contact_height_ratio,
            pre_contact_clearance=payload.pre_contact_clearance,
            contact_margin=payload.contact_margin,
            push_distance=payload.push_distance,
        )
        reachability: list[dict[str, object]] = []
        ok = True
        max_error = 0.0
        for waypoint in geometry["pusher_waypoints"]:
            if not isinstance(waypoint, list):
                ok = False
                continue
            result = move_ik_target_checked(
                environment_handle=int(ik["environment_handle"]),
                group_handle=int(ik["group_handle"]),
                target_handle=int(ik["target_handle"]),
                tip_handle=tip_handle,
                position=waypoint,
                relative_to=-1,
                steps=payload.ik_steps_per_waypoint,
                max_position_error=payload.max_tip_error,
                record_joint_handles=joint_handles,
            )
            reachability.append(
                {
                    "ok": bool(result["ok"]),
                    "failure_reason": result["failure_reason"],
                    "position_error": result["position_error"],
                    "target_position": result["target_position"],
                    "tip_position": result["tip_position"],
                }
            )
            max_error = max(max_error, float(result["position_error"]))
            ok = ok and bool(result["ok"])

        candidate = {
            "object_center": center,
            "table_top_z": center[2] - payload.object_height / 2.0,
            "push_direction": direction,
            "reachable": ok,
            "max_position_error": max_error,
            "reachability": reachability,
            **geometry,
        }
        candidates.append(candidate)
        if ok and selected is None:
            selected = candidate

    if selected is None:
        return {
            "ok": False,
            "failure_reason": "IK_TARGET_UNREACHABLE",
            "robot_path": payload.robot_path,
            "tip_position": tip_position,
            "suggestion": _reachability_suggestion(
                tip_position=tip_position,
                direction=direction,
                object_size=object_size,
                contact_height_ratio=payload.contact_height_ratio,
                pre_contact_clearance=payload.pre_contact_clearance,
                contact_margin=payload.contact_margin,
            ),
            "candidates": candidates,
        }

    object_center = selected["object_center"]
    table_top_z = float(selected["table_top_z"])
    if not isinstance(object_center, list):
        raise RuntimeError("invalid selected object center")
    table_center = [
        object_center[0] + direction[0] * payload.push_distance * 0.5,
        object_center[1] + direction[1] * payload.push_distance * 0.5,
        table_top_z - payload.table_size[2] / 2.0,
    ]
    table_handle = spawn_cuboid(
        size=payload.table_size,
        position=table_center,
        color=[0.55, 0.42, 0.28],
        dynamic=False,
    )
    set_shape_dynamics(table_handle, static=True, respondable=True, friction=0.7)
    table_alias = f"{payload.alias_prefix}_table"
    _set_object_alias(sim, table_handle, table_alias)

    obj = spawn_composite_object(
        visual_primitive="cylinder",
        proxy_type="cylinder_proxy",
        size=object_size,
        position=object_center,
        visual_color=[0.1, 0.45, 0.9],
        proxy_color=[0.1, 0.9, 0.2],
        dynamic=True,
        visible_proxy=False,
        alias=f"{payload.alias_prefix}_can",
        mass=payload.object_mass,
        friction=payload.object_friction,
    )
    pusher = create_pusher_tool_for_abb(robot_path=payload.robot_path)
    return {
        "ok": True,
        "table_handle": int(table_handle),
        "object_handle": int(obj["proxy_handle"]),
        "object_visual_handle": int(obj["visual_handle"]),
        "object_proxy_handle": int(obj["proxy_handle"]),
        "pusher_tool_handle": int(pusher["handle"]),
        "recommended_push_direction": direction,
        "recommended_contact_pose": {
            "pre_contact_point": selected["pre_contact_point"],
            "contact_point": selected["contact_point"],
            "post_contact_point": selected["post_contact_point"],
        },
        "reachability_score": max(0.0, 1.0 - float(selected["max_position_error"]) / payload.max_tip_error),
        "selected": selected,
        "candidates": candidates,
        "table_top_z": table_top_z,
        "table_center": table_center,
        "object_center": object_center,
        "pusher": pusher,
    }


def push_object_with_abb(
    robot_path: str = "/IRB4600",
    object_handle: int | None = None,
    push_direction: list[float] | None = None,
    push_distance: float = 0.10,
    contact_height_ratio: float = 0.5,
    pre_contact_clearance: float = 0.04,
    contact_margin: float = 0.005,
    table_handle: int | None = None,
    pusher_tool_handle: int | None = None,
    max_tip_error: float = 0.015,
    simulation_steps_per_waypoint: int = 5,
    ik_steps_per_waypoint: int = 10,
    object_mass: float = 0.1,
    object_friction: float | None = None,
    min_moved_distance: float = 0.01,
    settle_steps: int = 20,
    constraint_policy: str = "position_only",
    preflight_only: bool = False,
    auto_create_pusher: bool = True,
    release_stepping_on_finish: bool = True,
) -> dict[str, object]:
    """Push an object with ABB using checked IK, stepping, and numeric verification."""
    try:
        payload = PushObjectWithAbbInput.model_validate(
            {
                "robot_path": robot_path,
                "object_handle": object_handle,
                "push_direction": push_direction if push_direction is not None else [1.0, 0.0, 0.0],
                "push_distance": push_distance,
                "contact_height_ratio": contact_height_ratio,
                "pre_contact_clearance": pre_contact_clearance,
                "contact_margin": contact_margin,
                "table_handle": table_handle,
                "pusher_tool_handle": pusher_tool_handle,
                "max_tip_error": max_tip_error,
                "simulation_steps_per_waypoint": simulation_steps_per_waypoint,
                "ik_steps_per_waypoint": ik_steps_per_waypoint,
                "object_mass": object_mass,
                "object_friction": object_friction,
                "min_moved_distance": min_moved_distance,
                "settle_steps": settle_steps,
                "constraint_policy": constraint_policy,
                "preflight_only": preflight_only,
                "auto_create_pusher": auto_create_pusher,
                "release_stepping_on_finish": release_stepping_on_finish,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    direction = _unit_horizontal(payload.push_direction)
    object_start = [float(v) for v in sim.getObjectPosition(payload.object_handle, -1)]
    bbox = _bbox(sim, payload.object_handle)
    bbox_min = bbox["min"]
    bbox_size = bbox["size"]
    if not isinstance(bbox_min, list) or not isinstance(bbox_size, list):
        raise RuntimeError("invalid bounding box result")
    geometry = _push_geometry(
        object_center=object_start,
        object_size=[float(v) for v in bbox_size],
        direction=direction,
        contact_height_ratio=payload.contact_height_ratio,
        pre_contact_clearance=payload.pre_contact_clearance,
        contact_margin=payload.contact_margin,
        push_distance=payload.push_distance,
    )
    pre_contact_point = geometry["pre_contact_point"]
    contact_point = geometry["contact_point"]
    post_contact_point = geometry["post_contact_point"]
    pusher_waypoints = geometry["pusher_waypoints"]
    if not isinstance(pre_contact_point, list) or not isinstance(contact_point, list) or not isinstance(post_contact_point, list):
        raise RuntimeError("invalid push geometry")
    if not isinstance(pusher_waypoints, list):
        raise RuntimeError("invalid pusher waypoint list")

    ik = setup_abb_arm_ik(
        robot_path=payload.robot_path,
        verify_motion=False,
        constraint_policy=payload.constraint_policy.value,
    )
    tip_handle = int(ik["tip_handle"])
    target_handle = int(ik["target_handle"])

    pusher_info: dict[str, object]
    if payload.pusher_tool_handle is None or not _handle_exists(sim, payload.pusher_tool_handle):
        if not payload.auto_create_pusher:
            return {
                "ok": False,
                "failure_reason": "PUSHER_TOOL_UNAVAILABLE",
                "robot_path": payload.robot_path,
                "object_handle": payload.object_handle,
                "pusher_tool_handle": payload.pusher_tool_handle,
                "suggestion": "Call create_pusher_tool_for_abb or set auto_create_pusher=true.",
            }
        pusher_info = create_pusher_tool_for_abb(robot_path=payload.robot_path)
        pusher_handle = int(pusher_info["handle"])
    else:
        pusher_handle = int(payload.pusher_tool_handle)
        pusher_info = {"handle": pusher_handle, "created": False}
        set_shape_dynamics(pusher_handle, static=True, respondable=True)

    drive = configure_abb_arm_drive(
        robot_path=payload.robot_path,
        joint_mode="kinematic",
        dyn_ctrl_mode="position",
        max_force_or_torque=2000.0,
        reset_dynamics=False,
    )
    joint_handles = [int(handle) for handle in drive.get("joint_handles", [])]
    if not joint_handles:
        joint_handles = [int(handle) for handle in find_robot_joints(payload.robot_path)["joint_handles"]]

    object_dynamics = set_shape_dynamics(
        payload.object_handle,
        static=False,
        respondable=True,
        mass=payload.object_mass,
        friction=payload.object_friction,
    )
    table_dynamics = None
    if payload.table_handle is not None:
        table_dynamics = set_shape_dynamics(payload.table_handle, static=True, respondable=True)

    tip_position = [float(v) for v in sim.getObjectPosition(tip_handle, -1)]
    pusher_position = [float(v) for v in sim.getObjectPosition(pusher_handle, -1)]
    tip_offset_from_pusher = [tip_position[i] - pusher_position[i] for i in range(3)]
    tip_waypoints = [
        [point[i] + tip_offset_from_pusher[i] for i in range(3)]
        for point in pusher_waypoints
    ]

    preflight_results: list[dict[str, object]] = []
    preflight_failure: dict[str, object] | None = None
    for index, waypoint in enumerate(tip_waypoints, start=1):
        result = move_ik_target_checked(
            environment_handle=int(ik["environment_handle"]),
            group_handle=int(ik["group_handle"]),
            target_handle=target_handle,
            tip_handle=tip_handle,
            position=waypoint,
            relative_to=-1,
            steps=payload.ik_steps_per_waypoint,
            max_position_error=payload.max_tip_error,
            record_joint_handles=joint_handles,
        )
        phase = ["pre_contact", "contact", "post_contact"][index - 1]
        item = {
            "index": index,
            "phase": phase,
            "waypoint": waypoint,
            "ok": bool(result["ok"]),
            "failure_reason": result["failure_reason"],
            "position_error": result["position_error"],
            "tip_position": result["tip_position"],
            "target_position": result["target_position"],
            "joint_delta_norm": result["joint_delta_norm"],
        }
        preflight_results.append(item)
        if not bool(result["ok"]) and preflight_failure is None:
            preflight_failure = item

    if preflight_failure is not None or payload.preflight_only:
        current_tip = [float(v) for v in sim.getObjectPosition(tip_handle, -1)]
        reason = "IK_TARGET_UNREACHABLE" if preflight_failure is not None else None
        return {
            "ok": preflight_failure is None,
            "failure_reason": reason,
            "preflight_only": payload.preflight_only,
            "robot_path": payload.robot_path,
            "object_handle": payload.object_handle,
            "pusher_tool_handle": pusher_handle,
            "tip_handle": tip_handle,
            "target_handle": target_handle,
            "environment_handle": int(ik["environment_handle"]),
            "group_handle": int(ik["group_handle"]),
            "push_direction": direction,
            "object_start_position": object_start,
            "pre_contact_point": pre_contact_point,
            "contact_point": contact_point,
            "post_contact_point": post_contact_point,
            "tip_waypoints": tip_waypoints,
            "preflight_results": preflight_results,
            "failed_preflight": preflight_failure,
            "suggestion": _reachability_suggestion(
                tip_position=current_tip,
                direction=direction,
                object_size=[float(v) for v in bbox_size],
                contact_height_ratio=payload.contact_height_ratio,
                pre_contact_clearance=payload.pre_contact_clearance,
                contact_margin=payload.contact_margin,
            ) if preflight_failure is not None else None,
            "pusher": pusher_info,
            "drive": drive,
        }

    records: list[dict[str, object]] = []
    ik_results: list[dict[str, object]] = []
    contact_happened = False
    max_abs_joint_force = 0.0
    failure_reason: str | None = None
    try:
        for index, waypoint in enumerate(tip_waypoints, start=1):
            ik_result = move_ik_target_checked(
                environment_handle=int(ik["environment_handle"]),
                group_handle=int(ik["group_handle"]),
                target_handle=target_handle,
                tip_handle=tip_handle,
                position=waypoint,
                relative_to=-1,
                steps=payload.ik_steps_per_waypoint,
                max_position_error=payload.max_tip_error,
                record_joint_handles=joint_handles,
            )
            ik_results.append(ik_result)
            collided_before_step = _safe_collision(pusher_handle, payload.object_handle)
            step = _step_simulation_checked(
                steps=payload.simulation_steps_per_waypoint,
                release_on_failure=payload.release_stepping_on_finish,
            )
            if not bool(step.get("ok", True)):
                failure_reason = "REMOTE_API_STEP_TIMEOUT"
            collided_after_step = _safe_collision(pusher_handle, payload.object_handle)
            collided = collided_before_step or collided_after_step
            contact_happened = contact_happened or collided
            object_position = [float(v) for v in sim.getObjectPosition(payload.object_handle, -1)]
            pusher_position_now = [float(v) for v in sim.getObjectPosition(pusher_handle, -1)]
            forces = [float(sim.getJointForce(handle)) for handle in joint_handles if hasattr(sim, "getJointForce")]
            if forces:
                max_abs_joint_force = max(max_abs_joint_force, max(abs(force) for force in forces))
            records.append(
                {
                    "index": index,
                    "phase": ["pre_contact", "contact", "post_contact"][index - 1],
                    "tip_waypoint": waypoint,
                    "pusher_waypoint": pusher_waypoints[index - 1],
                    "ik_ok": bool(ik_result["ok"]),
                    "ik_failure_reason": ik_result["failure_reason"],
                    "tip_position": ik_result["tip_position"],
                    "position_error": ik_result["position_error"],
                    "pusher_position": pusher_position_now,
                    "object_position": object_position,
                    "pusher_object_collision": collided,
                    "pusher_object_collision_before_step": collided_before_step,
                    "pusher_object_collision_after_step": collided_after_step,
                    "joint_forces": forces,
                    "step_result": step,
                }
            )
            if failure_reason is not None:
                break
            if not bool(ik_result["ok"]):
                failure_reason = str(ik_result.get("failure_reason"))
                break

        if failure_reason is None and payload.settle_steps > 0:
            step = _step_simulation_checked(
                steps=payload.settle_steps,
                release_on_failure=payload.release_stepping_on_finish,
            )
            if not bool(step.get("ok", True)):
                failure_reason = "REMOTE_API_STEP_TIMEOUT"
                records.append({"index": len(records) + 1, "phase": "settle", "step_result": step})
    finally:
        if payload.release_stepping_on_finish:
            _release_remote_stepping()

    object_final = [float(v) for v in sim.getObjectPosition(payload.object_handle, -1)]
    displacement = [object_final[i] - object_start[i] for i in range(3)]
    moved_distance = _distance(object_final, object_start)
    directional_progress = _dot(displacement, direction)
    velocity = get_object_velocity(payload.object_handle)

    if failure_reason is None and not contact_happened:
        failure_reason = "NO_CONTACT"
    if failure_reason is None and moved_distance < payload.min_moved_distance:
        failure_reason = "OBJECT_DID_NOT_MOVE"
    if failure_reason is None and directional_progress < payload.min_moved_distance:
        failure_reason = "INSUFFICIENT_DIRECTIONAL_PROGRESS"

    return {
        "ok": failure_reason is None,
        "failure_reason": failure_reason,
        "robot_path": payload.robot_path,
        "object_handle": payload.object_handle,
        "pusher_tool_handle": pusher_handle,
        "tip_handle": tip_handle,
        "target_handle": target_handle,
        "environment_handle": int(ik["environment_handle"]),
        "group_handle": int(ik["group_handle"]),
        "push_direction": direction,
        "push_distance": payload.push_distance,
        "object_start_position": object_start,
        "object_final_position": object_final,
        "displacement": displacement,
        "moved_distance": moved_distance,
        "directional_progress": directional_progress,
        "contact_happened": contact_happened,
        "force_happened": max_abs_joint_force > 0.0,
        "max_abs_joint_force": max_abs_joint_force,
        "object_velocity": velocity,
        "pre_contact_point": pre_contact_point,
        "contact_point": contact_point,
        "post_contact_point": post_contact_point,
        "tip_waypoints": tip_waypoints,
        "bbox": bbox,
        "drive": drive,
        "preflight_results": preflight_results,
        "object_dynamics": object_dynamics,
        "table_dynamics": table_dynamics,
        "pusher": pusher_info,
        "ik_results": ik_results,
        "records": records,
    }
