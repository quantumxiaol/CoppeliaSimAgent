"""Model loading and parent-child assembly tools."""

from __future__ import annotations

import math

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .schemas import LoadModelInput, SetParentChildInput


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def load_model(
    model_path: str,
    position: list[float],
    orientation_deg: list[float] | None = None,
    relative_to: int = -1,
) -> int:
    """加载一个 `.ttm` 模型并放置到指定坐标。

    参数:
        model_path: 模型路径（绝对/相对路径均可）。
        position: 目标位置 `[x, y, z]`。约定 `z` 为高度（up 轴）。
        orientation_deg: 欧拉角（角度制），可选。
        relative_to: 参考坐标系句柄，`-1` 表示世界坐标。

    返回:
        模型基座句柄 (Base Handle)。
    """
    try:
        payload = LoadModelInput.model_validate(
            {
                "model_path": model_path,
                "position": position,
                "orientation_deg": orientation_deg,
                "relative_to": relative_to,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    base_handle = sim.loadModel(payload.model_path)
    sim.setObjectPosition(base_handle, payload.relative_to, payload.position)

    if payload.orientation_deg is not None:
        orientation_rad = [math.radians(v) for v in payload.orientation_deg]
        sim.setObjectOrientation(base_handle, payload.relative_to, orientation_rad)

    return int(base_handle)


def load_robot_model(model_path: str, position: list[float], orientation_deg: list[float] | None = None) -> int:
    """兼容别名：等同于 `load_model`。"""
    return load_model(model_path=model_path, position=position, orientation_deg=orientation_deg)


def set_parent_child(child_handle: int, parent_handle: int, keep_in_place: bool = True) -> None:
    """建立父子装配关系，例如把夹爪装到机械臂法兰上。"""
    try:
        payload = SetParentChildInput.model_validate(
            {
                "child_handle": child_handle,
                "parent_handle": parent_handle,
                "keep_in_place": keep_in_place,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.setObjectParent(payload.child_handle, payload.parent_handle, payload.keep_in_place)
