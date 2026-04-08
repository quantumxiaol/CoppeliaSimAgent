"""Scene perception and collision query tools."""

from __future__ import annotations

import math

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import CollisionDetectedError, ToolValidationError
from .schemas import CheckCollisionInput, FindObjectsInput, GetSceneGraphInput

_OBJECT_TYPE_ATTRS = {
    "shape": "object_shape_type",
    "dummy": "object_dummy_type",
    "joint": "object_joint_type",
    "camera": "object_camera_type",
    "light": "object_light_type",
    "force_sensor": "object_forcesensor_type",
}


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _parse_collision_result(result: object) -> bool:
    if isinstance(result, bool):
        return result
    if isinstance(result, int):
        return result not in (0, -1)
    if isinstance(result, (list, tuple)) and result:
        head = result[0]
        if isinstance(head, int):
            return head not in (0, -1)
        if isinstance(head, bool):
            return head
    return bool(result)


def _safe_get_object_name(sim: object, handle: int) -> str:
    if hasattr(sim, "getObjectAlias"):
        try:
            return str(sim.getObjectAlias(handle))
        except Exception:
            pass
    return str(sim.getObjectName(handle))


def _get_object_type(sim: object, handle: int) -> int:
    """Read object type with API compatibility across simulator versions."""
    if hasattr(sim, "getObjectType"):
        try:
            return int(sim.getObjectType(handle))
        except Exception:
            pass

    if hasattr(sim, "objintparam_type"):
        return int(sim.getObjectInt32Param(handle, sim.objintparam_type))

    if hasattr(sim, "objintparam_object_type"):
        return int(sim.getObjectInt32Param(handle, sim.objintparam_object_type))

    raise RuntimeError("Cannot resolve object type from current CoppeliaSim API")


def get_scene_graph(
    include_types: list[str] | None = None,
    round_digits: int = 3,
) -> dict[str, dict[str, object]]:
    """提取场景图信息，返回对象名称、句柄、类型与位姿。

    参数:
        include_types: 需要提取的对象类型列表，默认 `shape` 与 `dummy`。
        round_digits: 数值保留小数位。

    返回:
        `{object_name: {handle, type, position, orientation_deg}}`
    """
    try:
        payload = GetSceneGraphInput.model_validate(
            {
                "include_types": include_types if include_types is not None else ["shape", "dummy"],
                "round_digits": round_digits,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    handles = sim.getObjectsInTree(sim.handle_scene)

    allowed_types: set[int] = set()
    for t in payload.include_types:
        attr = _OBJECT_TYPE_ATTRS[t.value]
        if hasattr(sim, attr):
            allowed_types.add(int(getattr(sim, attr)))

    scene_state: dict[str, dict[str, object]] = {}
    for handle in handles:
        obj_type = _get_object_type(sim, handle)
        if allowed_types and obj_type not in allowed_types:
            continue

        name = _safe_get_object_name(sim, handle)
        position = [round(float(v), payload.round_digits) for v in sim.getObjectPosition(handle, -1)]
        orientation_rad = sim.getObjectOrientation(handle, -1)
        orientation_deg = [round(math.degrees(float(v)), payload.round_digits) for v in orientation_rad]

        key = name if name not in scene_state else f"{name}#{handle}"
        scene_state[key] = {
            "handle": int(handle),
            "type": obj_type,
            "position": position,
            "orientation_deg": orientation_deg,
        }

    return scene_state


def find_objects(
    name_query: str | None = None,
    exact_name: bool = False,
    include_types: list[str] | None = None,
    round_digits: int = 3,
    limit: int = 20,
) -> list[dict[str, object]]:
    """按名称与类型过滤场景对象，返回匹配结果列表。"""
    try:
        payload = FindObjectsInput.model_validate(
            {
                "name_query": name_query,
                "exact_name": exact_name,
                "include_types": include_types if include_types is not None else ["shape", "dummy"],
                "round_digits": round_digits,
                "limit": limit,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    handles = sim.getObjectsInTree(sim.handle_scene)
    query = payload.name_query.lower() if payload.name_query is not None else None

    allowed_types: set[int] = set()
    for t in payload.include_types:
        attr = _OBJECT_TYPE_ATTRS[t.value]
        if hasattr(sim, attr):
            allowed_types.add(int(getattr(sim, attr)))

    matched: list[dict[str, object]] = []
    for handle in handles:
        obj_type = _get_object_type(sim, handle)
        if allowed_types and obj_type not in allowed_types:
            continue

        name = _safe_get_object_name(sim, handle)
        if query is not None:
            name_lower = name.lower()
            if payload.exact_name:
                if name_lower != query:
                    continue
            elif query not in name_lower:
                continue

        position = [round(float(v), payload.round_digits) for v in sim.getObjectPosition(handle, -1)]
        orientation_rad = sim.getObjectOrientation(handle, -1)
        orientation_deg = [round(math.degrees(float(v)), payload.round_digits) for v in orientation_rad]

        matched.append(
            {
                "name": name,
                "handle": int(handle),
                "type": obj_type,
                "position": position,
                "orientation_deg": orientation_deg,
            }
        )
        if len(matched) >= payload.limit:
            break

    return matched


def check_collision(entity1: int, entity2: int) -> bool:
    """执行碰撞检测，返回是否发生碰撞。"""
    try:
        payload = CheckCollisionInput.model_validate({"entity1": entity1, "entity2": entity2})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    result = sim.checkCollision(payload.entity1, payload.entity2)
    return _parse_collision_result(result)


def assert_no_collision(entity1: int, entity2: int) -> None:
    """碰撞拦截器：若碰撞则抛出异常，便于 Agent 决策回退。"""
    if check_collision(entity1, entity2):
        raise CollisionDetectedError(f"Collision detected between {entity1} and {entity2}")
