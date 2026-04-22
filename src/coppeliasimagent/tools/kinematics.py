"""Kinematics and end-effector control helpers."""

from __future__ import annotations

from pydantic import ValidationError

from ..core.connection import get_sim, get_simik
from ..core.exceptions import ToolValidationError
from .schemas import (
    ActuateGripperInput,
    ActuateYouBotGripperInput,
    DriveYouBotBaseInput,
    GetJointPositionInput,
    MoveIKTargetInput,
    SetJointPositionInput,
    SetJointTargetPositionInput,
    SetJointTargetVelocityInput,
    SetYouBotBaseLockedInput,
    SetYouBotWheelVelocitiesInput,
    SetupIKLinkInput,
    SetupYouBotArmIKInput,
    SpawnWaypointInput,
)

_YOUBOT_WHEEL_JOINT_NAMES = (
    "rollingJoint_rr",
    "rollingJoint_rl",
    "rollingJoint_fr",
    "rollingJoint_fl",
)

_YOUBOT_BASE_SHAPE_NAMES = (
    "ME_Platfo2_sub1",
    "Rectangle0",
    "Rectangle",
    "Rectangle13",
    "intermediateLink_rr",
    "intermediateLink_rl",
    "intermediateLink_fr",
    "intermediateLink_fl",
    "swedishWheel_rr",
    "swedishWheel_rl",
    "swedishWheel_fr",
    "swedishWheel_fl",
    "wheel_respondable_rr",
    "wheel_respondable_rl",
    "wheel_respondable_fr",
    "wheel_respondable_fl",
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


def _resolve_object_handle(sim: object, object_path: str, *, no_error: bool = False) -> int:
    options = {"noError": True} if no_error else {}
    try:
        handle = sim.getObject(object_path, options)
    except TypeError:
        try:
            handle = sim.getObject(object_path)
        except Exception:
            if no_error:
                return -1
            raise
    except Exception:
        if no_error:
            return -1
        raise

    if handle in (-1, None):
        if no_error:
            return -1
        raise RuntimeError(f"Object not found: {object_path}")
    return int(handle)


def _set_object_alias(sim: object, handle: int, alias: str) -> None:
    if hasattr(sim, "setObjectAlias"):
        sim.setObjectAlias(handle, alias)
        return
    if hasattr(sim, "setObjectName"):
        sim.setObjectName(handle, alias)
        return
    raise RuntimeError("Current CoppeliaSim API does not expose setObjectAlias/setObjectName")


def _normalize_child_path(robot_path: str, object_path: str) -> str:
    return object_path if object_path.startswith("/") else f"{robot_path}/{object_path}"


def _resolve_existing_handles(sim: object, object_paths: list[str]) -> tuple[list[int], list[str]]:
    handles: list[int] = []
    missing: list[str] = []
    for object_path in object_paths:
        handle = _resolve_object_handle(sim, object_path, no_error=True)
        if handle == -1:
            missing.append(object_path)
            continue
        handles.append(handle)
    return handles, missing


def _reset_dynamic_object(sim: object, handle: int, *, include_model: bool = True) -> int:
    object_handle = int(handle)
    if include_model and hasattr(sim, "handleflag_model"):
        object_handle |= int(getattr(sim, "handleflag_model"))
    if hasattr(sim, "resetDynamicObject"):
        return int(sim.resetDynamicObject(object_handle))
    return 0


def _youbot_wheel_handles(sim: object, robot_path: str) -> list[int]:
    return [
        _resolve_object_handle(sim, f"{robot_path}/{joint_name}")
        for joint_name in _YOUBOT_WHEEL_JOINT_NAMES
    ]


def _ensure_dummy(
    *,
    sim: object,
    object_path: str,
    alias: str,
    position_relative_to: int,
    position: list[float],
    orientation_relative_to: int,
    orientation_rad: list[float],
    parent_handle: int,
    size: float = 0.02,
    reuse_existing: bool = True,
) -> int:
    handle = _resolve_object_handle(sim, object_path, no_error=True) if reuse_existing else -1
    if handle == -1:
        handle = int(sim.createDummy(size))

    sim.setObjectPosition(handle, position_relative_to, position)
    sim.setObjectOrientation(handle, orientation_relative_to, orientation_rad)
    sim.setObjectParent(handle, parent_handle, True)
    _set_object_alias(sim, handle, alias)
    return handle


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
        if hasattr(simik, "handleGroup"):
            simik.handleGroup(payload.environment_handle, payload.group_handle, {"syncWorlds": True})
        else:
            if hasattr(simik, "syncFromSim"):
                simik.syncFromSim(payload.environment_handle, [payload.group_handle])
            simik.handleGroup(payload.environment_handle, payload.group_handle)
            if hasattr(simik, "syncToSim"):
                simik.syncToSim(payload.environment_handle, [payload.group_handle])


def get_joint_position(handle: int) -> float:
    """读取一个非球形关节的当前位置。"""
    try:
        payload = GetJointPositionInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    return float(sim.getJointPosition(payload.handle))


def set_joint_position(handle: int, position: float) -> float:
    """直接设置关节当前位置。"""
    try:
        payload = SetJointPositionInput.model_validate({"handle": handle, "position": position})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.setJointPosition(payload.handle, payload.position)
    return float(payload.position)


def set_joint_target_position(
    handle: int,
    target_position: float,
    motion_params: list[float] | None = None,
) -> float:
    """设置关节目标位置。

    `motion_params` 对应 CoppeliaSim 的最大速度/加速度/jerk。
    """
    try:
        payload = SetJointTargetPositionInput.model_validate(
            {
                "handle": handle,
                "target_position": target_position,
                "motion_params": motion_params,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if payload.motion_params:
        sim.setJointTargetPosition(payload.handle, payload.target_position, payload.motion_params)
    else:
        sim.setJointTargetPosition(payload.handle, payload.target_position)
    return float(payload.target_position)


def set_joint_target_velocity(
    handle: int,
    target_velocity: float,
    motion_params: list[float] | None = None,
) -> float:
    """设置关节目标速度。

    `motion_params` 对应 CoppeliaSim 的最大加速度/jerk。
    """
    try:
        payload = SetJointTargetVelocityInput.model_validate(
            {
                "handle": handle,
                "target_velocity": target_velocity,
                "motion_params": motion_params,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if payload.motion_params:
        sim.setJointTargetVelocity(payload.handle, payload.target_velocity, payload.motion_params)
    else:
        sim.setJointTargetVelocity(payload.handle, payload.target_velocity)
    return float(payload.target_velocity)


def set_youbot_wheel_velocities(
    robot_path: str = "/youBot",
    wheel_velocities: list[float] | None = None,
    motion_params: list[float] | None = None,
) -> dict[str, object]:
    """直接设置 youBot 四个 rolling joints 的目标速度。

    轮子顺序固定为: rr, rl, fr, fl。
    """
    try:
        payload = SetYouBotWheelVelocitiesInput.model_validate(
            {
                "robot_path": robot_path,
                "wheel_velocities": wheel_velocities if wheel_velocities is not None else [0.0, 0.0, 0.0, 0.0],
                "motion_params": motion_params,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    wheel_handles = _youbot_wheel_handles(sim, payload.robot_path)
    for handle, velocity in zip(wheel_handles, payload.wheel_velocities, strict=True):
        set_joint_target_velocity(handle, float(velocity), motion_params=payload.motion_params)

    return {
        "robot_path": payload.robot_path,
        "wheel_joint_handles": wheel_handles,
        "wheel_joint_names": list(_YOUBOT_WHEEL_JOINT_NAMES),
        "wheel_velocities": [float(v) for v in payload.wheel_velocities],
    }


def drive_youbot_base(
    robot_path: str = "/youBot",
    forward_velocity: float = 0.0,
    lateral_velocity: float = 0.0,
    yaw_velocity: float = 0.0,
    motion_params: list[float] | None = None,
) -> dict[str, object]:
    """按 youBot 全向底盘约定把底盘指令映射到四个轮子的目标速度。

    输出轮子顺序为 rr, rl, fr, fl。
    """
    try:
        payload = DriveYouBotBaseInput.model_validate(
            {
                "robot_path": robot_path,
                "forward_velocity": forward_velocity,
                "lateral_velocity": lateral_velocity,
                "yaw_velocity": yaw_velocity,
                "motion_params": motion_params,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    rr = -payload.forward_velocity - payload.lateral_velocity + payload.yaw_velocity
    rl = -payload.forward_velocity + payload.lateral_velocity - payload.yaw_velocity
    fr = -payload.forward_velocity + payload.lateral_velocity + payload.yaw_velocity
    fl = -payload.forward_velocity - payload.lateral_velocity - payload.yaw_velocity

    out = set_youbot_wheel_velocities(
        robot_path=payload.robot_path,
        wheel_velocities=[rr, rl, fr, fl],
        motion_params=payload.motion_params,
    )
    out.update(
        {
            "forward_velocity": float(payload.forward_velocity),
            "lateral_velocity": float(payload.lateral_velocity),
            "yaw_velocity": float(payload.yaw_velocity),
        }
    )
    return out


def stop_youbot_base(robot_path: str = "/youBot", motion_params: list[float] | None = None) -> dict[str, object]:
    """显式把 youBot 四个底盘轮子的目标速度设为 0。"""
    return set_youbot_wheel_velocities(
        robot_path=robot_path,
        wheel_velocities=[0.0, 0.0, 0.0, 0.0],
        motion_params=motion_params,
    )


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


def set_youbot_base_locked(
    robot_path: str = "/youBot",
    locked: bool = True,
    base_shape_paths: list[str] | None = None,
    zero_wheels: bool = True,
    reset_dynamics: bool = True,
    motion_params: list[float] | None = None,
) -> dict[str, object]:
    """切换 youBot 底盘固定模式，便于只测试机械臂动作。"""
    try:
        payload = SetYouBotBaseLockedInput.model_validate(
            {
                "robot_path": robot_path,
                "locked": locked,
                "base_shape_paths": base_shape_paths,
                "zero_wheels": zero_wheels,
                "reset_dynamics": reset_dynamics,
                "motion_params": motion_params,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    robot_handle = _resolve_object_handle(sim, payload.robot_path)
    if payload.base_shape_paths is None:
        shape_handles = [
            int(handle)
            for handle in sim.getObjectsInTree(robot_handle)
            if int(sim.getObjectType(handle)) == int(getattr(sim, "object_shape_type"))
        ]
        missing_paths: list[str] = []
    else:
        shape_paths = [
            _normalize_child_path(payload.robot_path, item)
            for item in payload.base_shape_paths
        ]
        shape_handles, missing_paths = _resolve_existing_handles(sim, shape_paths)

    if payload.zero_wheels:
        stop_youbot_base(robot_path=payload.robot_path, motion_params=payload.motion_params)

    static_value = 1 if payload.locked else 0
    for handle in shape_handles:
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, static_value)

    reset_result = None
    if payload.reset_dynamics:
        reset_result = _reset_dynamic_object(sim, robot_handle, include_model=True)

    return {
        "robot_path": payload.robot_path,
        "robot_handle": robot_handle,
        "locked": payload.locked,
        "zero_wheels": payload.zero_wheels,
        "modified_shape_handles": shape_handles,
        "missing_shape_paths": missing_paths,
        "reset_result": reset_result,
    }


def setup_youbot_arm_ik(
    robot_path: str = "/youBot",
    base_path: str | None = None,
    tip_parent_path: str | None = None,
    tip_dummy_name: str = "youBotArmTip",
    target_dummy_name: str = "youBotArmTarget",
    tip_offset: list[float] | None = None,
    target_offset: list[float] | None = None,
    constraints_mask: int | None = None,
    reuse_existing: bool = True,
) -> dict[str, int | dict[int, int] | str]:
    """为 youBot 机械臂创建/复用 tip 与 target dummy，并建立 IK 链。"""
    try:
        payload = SetupYouBotArmIKInput.model_validate(
            {
                "robot_path": robot_path,
                "base_path": base_path,
                "tip_parent_path": tip_parent_path,
                "tip_dummy_name": tip_dummy_name,
                "target_dummy_name": target_dummy_name,
                "tip_offset": tip_offset if tip_offset is not None else [0.0, 0.0, 0.0],
                "target_offset": target_offset if target_offset is not None else [0.0, 0.0, 0.0],
                "constraints_mask": constraints_mask,
                "reuse_existing": reuse_existing,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    robot_handle = _resolve_object_handle(sim, payload.robot_path)
    base_handle = _resolve_object_handle(sim, payload.base_path) if payload.base_path else robot_handle
    tip_parent_handle = _resolve_object_handle(
        sim,
        payload.tip_parent_path if payload.tip_parent_path else f"{payload.robot_path}/Rectangle7",
    )

    tip_path = f"{payload.robot_path}/{payload.tip_dummy_name}"
    target_path = f"/{payload.target_dummy_name}"

    tip_handle = _ensure_dummy(
        sim=sim,
        object_path=tip_path,
        alias=payload.tip_dummy_name,
        position_relative_to=tip_parent_handle,
        position=payload.tip_offset,
        orientation_relative_to=tip_parent_handle,
        orientation_rad=[0.0, 0.0, 0.0],
        parent_handle=tip_parent_handle,
        reuse_existing=payload.reuse_existing,
    )

    tip_world_position = [float(v) for v in sim.getObjectPosition(tip_handle, -1)]
    tip_world_orientation = [float(v) for v in sim.getObjectOrientation(tip_handle, -1)]
    target_world_position = [
        float(tip_world_position[i]) + float(payload.target_offset[i]) for i in range(3)
    ]

    target_handle = _ensure_dummy(
        sim=sim,
        object_path=target_path,
        alias=payload.target_dummy_name,
        position_relative_to=-1,
        position=target_world_position,
        orientation_relative_to=-1,
        orientation_rad=tip_world_orientation,
        parent_handle=-1,
        reuse_existing=payload.reuse_existing,
    )

    if hasattr(sim, "setLinkDummy"):
        sim.setLinkDummy(tip_handle, target_handle)

    ik_info = setup_ik_link(
        base_handle=base_handle,
        tip_handle=tip_handle,
        target_handle=target_handle,
        constraints_mask=payload.constraints_mask,
    )
    return {
        "robot_handle": robot_handle,
        "base_handle": base_handle,
        "tip_parent_handle": tip_parent_handle,
        "tip_handle": tip_handle,
        "target_handle": target_handle,
        "robot_path": payload.robot_path,
        "tip_path": tip_path,
        "target_path": target_path,
        **ik_info,
    }


def actuate_youbot_gripper(
    robot_path: str = "/youBot",
    closed: bool = True,
    command_mode: str = "target_position",
    joint1_open: float = 0.025,
    joint1_closed: float = 0.0,
    joint2_open: float = -0.05,
    joint2_closed: float = 0.0,
    motion_params: list[float] | None = None,
) -> dict[str, int | float | str]:
    """按 youBot 夹爪模型的两指关节约定执行开合。"""
    try:
        payload = ActuateYouBotGripperInput.model_validate(
            {
                "robot_path": robot_path,
                "closed": closed,
                "command_mode": command_mode,
                "joint1_open": joint1_open,
                "joint1_closed": joint1_closed,
                "joint2_open": joint2_open,
                "joint2_closed": joint2_closed,
                "motion_params": motion_params,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    joint1_handle = _resolve_object_handle(sim, f"{payload.robot_path}/youBotGripperJoint1")
    joint2_handle = _resolve_object_handle(sim, f"{payload.robot_path}/youBotGripperJoint2")

    target1 = payload.joint1_closed if payload.closed else payload.joint1_open
    target2 = payload.joint2_closed if payload.closed else payload.joint2_open

    if payload.command_mode.value == "position":
        set_joint_position(joint1_handle, target1)
        set_joint_position(joint2_handle, target2)
    else:
        set_joint_target_position(joint1_handle, target1, motion_params=payload.motion_params)
        set_joint_target_position(joint2_handle, target2, motion_params=payload.motion_params)

    return {
        "robot_path": payload.robot_path,
        "command_mode": payload.command_mode.value,
        "joint1_handle": joint1_handle,
        "joint2_handle": joint2_handle,
        "joint1_target": float(target1),
        "joint2_target": float(target2),
        "closed": payload.closed,
        "joint1_position": float(sim.getJointPosition(joint1_handle)),
        "joint2_position": float(sim.getJointPosition(joint2_handle)),
    }
