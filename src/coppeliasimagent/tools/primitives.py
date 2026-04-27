"""Primitive geometry tools exposed to the LLM layer."""

from __future__ import annotations

import math

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .schemas import (
    ColorComponent,
    DuplicateObjectInput,
    PrimitiveType,
    RenameObjectInput,
    RemoveObjectInput,
    SetObjectVisibilityInput,
    SetObjectColorInput,
    SetObjectPoseInput,
    SpawnCuboidInput,
    SpawnPrimitiveInput,
)

_PRIMITIVE_CONST_MAP = {
    PrimitiveType.CUBOID: "primitiveshape_cuboid",
    PrimitiveType.SPHERE: "primitiveshape_sphere",
    PrimitiveType.CYLINDER: "primitiveshape_cylinder",
}


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _resolve_primitive_constant(sim: object, primitive: PrimitiveType) -> int:
    attr = _PRIMITIVE_CONST_MAP[primitive]
    if not hasattr(sim, attr):
        raise RuntimeError(f"CoppeliaSim API constant missing: {attr}")
    return getattr(sim, attr)


def spawn_primitive(
    primitive: str,
    size: list[float],
    position: list[float],
    color: list[float] | None = None,
    dynamic: bool = True,
    relative_to: int = -1,
) -> int:
    """在场景中生成基础几何体并完成初始位姿与颜色设置。

    参数:
        primitive: 几何体类型，支持 `cuboid` / `sphere` / `cylinder`。
        size: 尺寸 `[x, y, z]`，单位米 (m)。每个值必须 > 0。
        position: 目标坐标 `[x, y, z]`。约定 `z` 为高度（up 轴）。
        color: RGB，范围 `[0, 1]`，默认 `[0.8, 0.8, 0.8]`。
        dynamic: `True` 时设置为可响应物理。
        relative_to: 参考坐标系句柄，`-1` 表示世界坐标。

    返回:
        生成对象的句柄 (handle)。
    """
    try:
        payload = SpawnPrimitiveInput.model_validate(
            {
                "primitive": primitive,
                "size": size,
                "position": position,
                "color": color if color is not None else [0.8, 0.8, 0.8],
                "dynamic": dynamic,
                "relative_to": relative_to,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    primitive_const = _resolve_primitive_constant(sim, payload.primitive)

    handle = sim.createPrimitiveShape(primitive_const, payload.size)
    sim.setObjectPosition(handle, payload.relative_to, payload.position)
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, payload.color)

    if payload.dynamic:
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 0)
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)

    return int(handle)


def spawn_cuboid(
    size: list[float],
    position: list[float],
    color: list[float] | None = None,
    dynamic: bool = True,
    relative_to: int = -1,
) -> int:
    """在仿真场景中生成一个长方体。

    参数:
        size: 长方体尺寸 `[x, y, z]`，单位米 (m)。例如 `[0.1, 0.1, 0.1]`。
        position: 生成的空间坐标 `[x, y, z]`。约定 `z` 为高度（up 轴）。
        color: RGB 颜色值，范围 `[0, 1]`，默认浅灰色。
        dynamic: 是否启用动力学属性（受重力与碰撞响应）。
        relative_to: 参考坐标系句柄，`-1` 为世界坐标。

    返回:
        长方体的句柄 (Handle ID)。
    """
    try:
        payload = SpawnCuboidInput.model_validate(
            {
                "size": size,
                "position": position,
                "color": color if color is not None else [0.8, 0.8, 0.8],
                "dynamic": dynamic,
                "relative_to": relative_to,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    return spawn_primitive(
        primitive=PrimitiveType.CUBOID.value,
        size=payload.size,
        position=payload.position,
        color=payload.color,
        dynamic=payload.dynamic,
        relative_to=payload.relative_to,
    )


def set_object_pose(
    handle: int,
    position: list[float] | None = None,
    orientation_deg: list[float] | None = None,
    relative_to: int = -1,
) -> int:
    """设置物体位姿。

    角度约定:
        - 输入 `orientation_deg` 使用角度制 `[rx, ry, rz]`
        - 内部自动转换为 CoppeliaSim 所需弧度制

    参数:
        handle: 目标物体句柄。
        position: 目标位置 `[x, y, z]`。约定 `z` 为高度（up 轴）。
        orientation_deg: 欧拉角（角度制）。
        relative_to: 参考坐标系句柄，`-1` 为世界坐标。

    返回:
        原始句柄，便于流水线调用。
    """
    try:
        payload = SetObjectPoseInput.model_validate(
            {
                "handle": handle,
                "position": position,
                "orientation_deg": orientation_deg,
                "relative_to": relative_to,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if payload.position is not None:
        sim.setObjectPosition(payload.handle, payload.relative_to, payload.position)

    if payload.orientation_deg is not None:
        orientation_rad = [math.radians(v) for v in payload.orientation_deg]
        sim.setObjectOrientation(payload.handle, payload.relative_to, orientation_rad)

    return payload.handle


def remove_object(handle: int) -> None:
    """删除场景中的对象。"""
    try:
        payload = RemoveObjectInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if hasattr(sim, "removeObjects"):
        sim.removeObjects([payload.handle])
        return

    # Backward compatibility with old APIs (deprecated in recent versions).
    sim.removeObject(payload.handle)


def set_object_visibility(
    handle: int,
    visible: bool,
    include_descendants: bool = False,
) -> dict[str, object]:
    """Set object visibility for one object or a whole subtree."""
    try:
        payload = SetObjectVisibilityInput.model_validate(
            {
                "handle": handle,
                "visible": visible,
                "include_descendants": include_descendants,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    targets = [payload.handle]
    if payload.include_descendants:
        targets = [int(h) for h in sim.getObjectsInTree(payload.handle)]

    visibility_layer = 1 if payload.visible else 0
    for target in targets:
        if hasattr(sim, "setObjectInt32Param") and hasattr(sim, "objintparam_visibility_layer"):
            sim.setObjectInt32Param(target, sim.objintparam_visibility_layer, visibility_layer)
        elif hasattr(sim, "setObjectProperty") and hasattr(sim, "objectproperty_hidden"):
            prop = int(sim.getObjectProperty(target)) if hasattr(sim, "getObjectProperty") else 0
            hidden = int(sim.objectproperty_hidden)
            prop = prop & ~hidden if payload.visible else prop | hidden
            sim.setObjectProperty(target, prop)
        else:
            raise RuntimeError("Current CoppeliaSim API does not expose object visibility controls")

    return {"handle": payload.handle, "visible": payload.visible, "applied_handles": targets}


def _extract_first_handle(copy_result: object) -> int:
    if isinstance(copy_result, int):
        return int(copy_result)
    if isinstance(copy_result, (list, tuple)):
        if not copy_result:
            raise RuntimeError("copyPasteObjects returned no handles")
        head = copy_result[0]
        if isinstance(head, (list, tuple)):
            if not head:
                raise RuntimeError("copyPasteObjects returned empty nested handles")
            head = head[0]
        return int(head)
    raise RuntimeError(f"Unexpected copyPasteObjects result type: {type(copy_result)!r}")


def duplicate_object(
    handle: int,
    position: list[float] | None = None,
    offset: list[float] | None = None,
    relative_to: int = -1,
) -> int:
    """复制对象（按句柄）并可选移动新副本。

    参数:
        handle: 要复制的源对象句柄。
        position: 若提供，复制后把新对象移动到该绝对/相对位置 `[x, y, z]`。
        offset: 若提供，复制后在当前位置基础上偏移 `[dx, dy, dz]`。
        relative_to: 参考坐标系句柄，`-1` 表示世界坐标。

    返回:
        新复制对象的句柄。
    """
    try:
        payload = DuplicateObjectInput.model_validate(
            {
                "handle": handle,
                "position": position,
                "offset": offset,
                "relative_to": relative_to,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "copyPasteObjects"):
        raise RuntimeError("Current CoppeliaSim API does not expose copyPasteObjects")

    new_handle = _extract_first_handle(sim.copyPasteObjects([payload.handle], 0))

    if payload.position is not None:
        sim.setObjectPosition(new_handle, payload.relative_to, payload.position)
    elif payload.offset is not None:
        current = sim.getObjectPosition(new_handle, payload.relative_to)
        target = [float(current[i]) + payload.offset[i] for i in range(3)]
        sim.setObjectPosition(new_handle, payload.relative_to, target)

    return int(new_handle)


def rename_object(handle: int, new_alias: str) -> str:
    """重命名对象别名（Alias）。

    优先使用 `setObjectAlias`，若 API 不可用则回退到 `setObjectName`。
    """
    try:
        payload = RenameObjectInput.model_validate({"handle": handle, "new_alias": new_alias})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if hasattr(sim, "setObjectAlias"):
        sim.setObjectAlias(payload.handle, payload.new_alias)
    elif hasattr(sim, "setObjectName"):
        sim.setObjectName(payload.handle, payload.new_alias)
    else:
        raise RuntimeError("Current CoppeliaSim API does not expose setObjectAlias/setObjectName")

    return payload.new_alias


_COLOR_COMPONENT_ATTRS: dict[ColorComponent, str] = {
    ColorComponent.AMBIENT_DIFFUSE: "colorcomponent_ambient_diffuse",
    ColorComponent.DIFFUSE: "colorcomponent_diffuse",
    ColorComponent.SPECULAR: "colorcomponent_specular",
    ColorComponent.EMISSION: "colorcomponent_emission",
    ColorComponent.TRANSPARENCY: "colorcomponent_transparency",
}


def _get_object_type(sim: object, handle: int) -> int:
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


def _resolve_shape_targets(sim: object, handle: int) -> list[int]:
    if not hasattr(sim, "object_shape_type"):
        raise RuntimeError("CoppeliaSim API constant missing: object_shape_type")

    shape_type = int(getattr(sim, "object_shape_type"))
    if _get_object_type(sim, handle) == shape_type:
        return [int(handle)]

    if hasattr(sim, "getObjectsInTree"):
        tree_handles = sim.getObjectsInTree(int(handle))
        targets = [int(h) for h in tree_handles if _get_object_type(sim, int(h)) == shape_type]
        if targets:
            return targets

    raise RuntimeError("target object is not a shape and has no descendant shape")


def set_object_color(
    handle: int,
    color: list[float],
    color_name: str | None = None,
    color_component: str = ColorComponent.AMBIENT_DIFFUSE.value,
) -> list[int]:
    """设置 shape 对象颜色。

    参数:
        handle: 目标 shape 句柄。
        color: RGB 颜色，范围 `[0, 1]`。
        color_name: 颜色通道名，`None` 表示主颜色。
        color_component: 颜色分量，默认 `ambient_diffuse`。

    返回:
        原始句柄，便于链式调用。
    """
    try:
        payload = SetObjectColorInput.model_validate(
            {
                "handle": handle,
                "color": color,
                "color_name": color_name,
                "color_component": color_component,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "setShapeColor"):
        raise RuntimeError("Current CoppeliaSim API does not expose setShapeColor")

    attr = _COLOR_COMPONENT_ATTRS[payload.color_component]
    if not hasattr(sim, attr):
        raise RuntimeError(f"CoppeliaSim API constant missing: {attr}")

    component = getattr(sim, attr)
    target_handles = _resolve_shape_targets(sim, payload.handle)
    for target_handle in target_handles:
        sim.setShapeColor(target_handle, payload.color_name, component, payload.color)
    return target_handles
