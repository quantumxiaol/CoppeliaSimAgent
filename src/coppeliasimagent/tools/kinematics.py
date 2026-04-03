"""Kinematics and end-effector control helpers."""

from __future__ import annotations

from pydantic import ValidationError

from ..core.connection import get_sim, get_simik
from ..core.exceptions import ToolValidationError
from .schemas import (
    ActuateGripperInput,
    MoveIKTargetInput,
    SetupIKLinkInput,
    SpawnWaypointInput,
)


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _default_constraint_mask(simik: object) -> int:
    mask = 0
    for attr in (
        "constraint_x",
        "constraint_y",
        "constraint_z",
        "constraint_alpha_beta",
        "constraint_gamma",
    ):
        mask |= int(getattr(simik, attr, 0))
    return mask


def spawn_waypoint(position: list[float], size: float = 0.02, relative_to: int = -1) -> int:
    """生成一个 Dummy 作为 IK Target 航点。

    `position` 采用 `[x, y, z]`，其中 `z` 为高度（up 轴）。
    """
    try:
        payload = SpawnWaypointInput.model_validate(
            {"position": position, "size": size, "relative_to": relative_to}
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    target_handle = sim.createDummy(payload.size)
    sim.setObjectPosition(target_handle, payload.relative_to, payload.position)
    return int(target_handle)


def setup_ik_link(
    base_handle: int,
    tip_handle: int,
    target_handle: int,
    constraints_mask: int | None = None,
) -> dict[str, int | dict[int, int]]:
    """建立 `base -> tip -> target` 的 IK 追踪链路。"""
    try:
        payload = SetupIKLinkInput.model_validate(
            {
                "base_handle": base_handle,
                "tip_handle": tip_handle,
                "target_handle": target_handle,
                "constraints_mask": constraints_mask,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    simik = get_simik(required=True)

    env_handle = int(simik.createEnvironment())
    group_handle = int(simik.createGroup(env_handle))

    constraint_mask = payload.constraints_mask
    if constraint_mask is None:
        constraint_mask = _default_constraint_mask(simik)

    result = simik.addElementFromScene(
        env_handle,
        group_handle,
        payload.base_handle,
        payload.tip_handle,
        payload.target_handle,
        int(constraint_mask),
    )

    response: dict[str, int | dict[int, int]] = {
        "environment_handle": env_handle,
        "group_handle": group_handle,
    }

    if isinstance(result, (list, tuple)) and len(result) >= 3:
        response["ik_element_handle"] = int(result[0])
        response["sim_to_ik_map"] = result[1]
        response["ik_to_sim_map"] = result[2]
    elif isinstance(result, int):
        response["ik_element_handle"] = int(result)

    return response


def move_ik_target(
    environment_handle: int,
    group_handle: int,
    target_handle: int,
    position: list[float],
    relative_to: int = -1,
    steps: int = 1,
) -> None:
    """移动 IK target，并循环执行 IK 求解。

    `position` 采用 `[x, y, z]`，其中 `z` 为高度（up 轴）。
    """
    try:
        payload = MoveIKTargetInput.model_validate(
            {
                "environment_handle": environment_handle,
                "group_handle": group_handle,
                "target_handle": target_handle,
                "position": position,
                "relative_to": relative_to,
                "steps": steps,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    simik = get_simik(required=True)

    sim.setObjectPosition(payload.target_handle, payload.relative_to, payload.position)

    for _ in range(payload.steps):
        simik.handleGroup(payload.environment_handle, payload.group_handle)


def actuate_gripper(signal_name: str, closed: bool) -> int:
    """通过信号控制夹爪开合。

    约定:
        - `closed=True` 发送 `1`
        - `closed=False` 发送 `0`
    """
    try:
        payload = ActuateGripperInput.model_validate(
            {"signal_name": signal_name, "closed": closed}
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    signal_value = 1 if payload.closed else 0
    sim.setInt32Signal(payload.signal_name, signal_value)
    return signal_value
