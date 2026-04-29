"""Kinematics and end-effector control helpers."""

from __future__ import annotations

import math

from pydantic import ValidationError

from ..core.connection import get_sim, get_simik
from ..core.exceptions import PluginUnavailableError, ToolValidationError
from .schemas import (
    ActuateGripperInput,
    ActuateYouBotGripperInput,
    ConfigureAbbArmDriveInput,
    ConstraintPolicy,
    DriveYouBotBaseInput,
    GetJointDynCtrlModeInput,
    GetJointForceInput,
    GetJointModeInput,
    GetJointPositionInput,
    GetJointTargetForceInput,
    FindRobotJointsInput,
    MoveIKTargetCheckedInput,
    MoveIKTargetInput,
    SetJointDynCtrlModeInput,
    SetJointModeInput,
    SetJointPositionInput,
    SetJointTargetForceInput,
    SetJointTargetPositionInput,
    SetJointTargetVelocityInput,
    SetYouBotBaseLockedInput,
    SetYouBotWheelVelocitiesInput,
    SetupAbbArmIKInput,
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

_JOINT_MODE_CONST_MAP = {
    "kinematic": "jointmode_kinematic",
    "dependent": "jointmode_dependent",
    "dynamic": "jointmode_dynamic",
}

_JOINT_DYN_CTRL_CONST_MAP = {
    "free": "jointdynctrl_free",
    "force": "jointdynctrl_force",
    "velocity": "jointdynctrl_velocity",
    "position": "jointdynctrl_position",
    "spring": "jointdynctrl_spring",
    "callback": "jointdynctrl_callback",
}


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


def _constraint_mask_from_policy(simik: object, policy: ConstraintPolicy | str) -> int:
    policy_value = policy.value if isinstance(policy, ConstraintPolicy) else str(policy)
    position_mask = 0
    for attr in ("constraint_x", "constraint_y", "constraint_z"):
        position_mask |= int(getattr(simik, attr, 0))
    if policy_value == ConstraintPolicy.POSITION_ONLY.value:
        return position_mask
    if policy_value == ConstraintPolicy.POSITION_YAW.value:
        return position_mask | int(getattr(simik, "constraint_gamma", 0))
    if policy_value == ConstraintPolicy.FULL_POSE.value:
        return _default_constraint_mask(simik)
    raise RuntimeError(f"Unsupported IK constraint policy: {policy_value}")


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _orientation_error_deg(a_rad: list[float], b_rad: list[float]) -> float:
    deltas = []
    for i in range(3):
        delta = float(a_rad[i]) - float(b_rad[i])
        delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
        deltas.append(delta)
    return math.degrees(math.sqrt(sum(delta * delta for delta in deltas)))


def _joint_positions(sim: object, handles: list[int]) -> list[float]:
    return [float(sim.getJointPosition(handle)) for handle in handles]


def _joint_delta_norm(before: list[float], after: list[float]) -> float:
    if not before or not after:
        return 0.0
    return math.sqrt(sum((after[i] - before[i]) ** 2 for i in range(len(before))))


def _run_ik_group(simik: object, environment_handle: int, group_handle: int) -> object:
    if hasattr(simik, "handleGroup"):
        return simik.handleGroup(environment_handle, group_handle, {"syncWorlds": True})
    if hasattr(simik, "syncFromSim"):
        simik.syncFromSim(environment_handle, [group_handle])
    result = simik.handleGroup(environment_handle, group_handle)
    if hasattr(simik, "syncToSim"):
        simik.syncToSim(environment_handle, [group_handle])
    return result


def _primary_ik_code(result: object) -> int | None:
    if isinstance(result, bool):
        return int(result)
    if isinstance(result, int):
        return result
    if isinstance(result, float):
        return int(result)
    if isinstance(result, (list, tuple)) and result:
        return _primary_ik_code(result[0])
    return None


def _explicit_ik_failure(simik: object, codes: list[object]) -> bool:
    success_code = getattr(simik, "result_success", None)
    if success_code is None:
        return False
    for code in codes:
        primary = _primary_ik_code(code)
        if primary is not None and int(primary) != int(success_code):
            return True
    return False


def _collision_results(sim: object, collision_pairs: list[list[int]]) -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    if not collision_pairs:
        return results
    for entity1, entity2 in collision_pairs:
        raw = sim.checkCollision(entity1, entity2)
        if isinstance(raw, bool):
            collides = raw
        elif isinstance(raw, int):
            collides = raw not in (0, -1)
        elif isinstance(raw, (list, tuple)) and raw:
            head = raw[0]
            collides = bool(head) if isinstance(head, bool) else int(head) not in (0, -1)
        else:
            collides = bool(raw)
        results.append({"entity1": int(entity1), "entity2": int(entity2), "collides": collides, "raw": raw})
    return results


def _ik_failure_reason(
    *,
    simik: object,
    ik_return_codes: list[object],
    position_error: float,
    max_position_error: float,
    orientation_error_deg: float | None,
    max_orientation_error_deg: float,
    target_moved_distance: float,
    tip_moved_distance: float,
    joint_delta_norm: float,
    record_joint_handles: list[int],
    collision_results: list[dict[str, object]],
) -> str | None:
    if any(bool(item["collides"]) for item in collision_results):
        return "COLLISION_BLOCKED"
    if _explicit_ik_failure(simik, ik_return_codes):
        return "IK_TARGET_UNREACHABLE"
    if position_error > max_position_error:
        if target_moved_distance > max_position_error and tip_moved_distance <= 1e-5:
            return "TARGET_MOVED_BUT_TIP_NOT_MOVED"
        if record_joint_handles and joint_delta_norm <= 1e-6:
            return "JOINTS_NOT_MOVING"
        if tip_moved_distance <= 1e-5:
            return "IK_SOLVER_NO_PROGRESS"
        if position_error > max(0.05, max_position_error * 5.0):
            return "IK_TARGET_UNREACHABLE"
        return "IK_TIP_TARGET_RESIDUAL_TOO_LARGE"
    if orientation_error_deg is not None and orientation_error_deg > max_orientation_error_deg:
        return "IK_TIP_TARGET_RESIDUAL_TOO_LARGE"
    return None


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


def _resolve_sim_constant(sim: object, attr_name: str) -> int:
    if not hasattr(sim, attr_name):
        raise RuntimeError(f"CoppeliaSim API constant missing: {attr_name}")
    return int(getattr(sim, attr_name))


def _resolve_joint_mode_value(sim: object, joint_mode: str) -> int:
    return _resolve_sim_constant(sim, _JOINT_MODE_CONST_MAP[joint_mode])


def _resolve_joint_dyn_ctrl_mode_value(sim: object, dyn_ctrl_mode: str) -> int:
    return _resolve_sim_constant(sim, _JOINT_DYN_CTRL_CONST_MAP[dyn_ctrl_mode])


def _normalize_scalar_result(value: object, *, field_name: str) -> int | float:
    if isinstance(value, (list, tuple)):
        if not value:
            raise RuntimeError(f"CoppeliaSim API returned an empty result for {field_name}")
        value = value[0]
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value
    raise RuntimeError(f"CoppeliaSim API returned an unsupported result for {field_name}: {value!r}")


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


def _resolve_ik_target_handle(sim: object, target_path: str) -> tuple[int, str]:
    handle = _resolve_object_handle(sim, target_path, no_error=True)
    if handle != -1:
        return handle, target_path

    target_name = target_path.rstrip("/").rsplit("/", 1)[-1]
    if target_name:
        world_path = f"/{target_name}"
        handle = _resolve_object_handle(sim, world_path, no_error=True)
        if handle != -1:
            return handle, world_path

    raise RuntimeError(f"Object not found: {target_path}")


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


def _abb_arm_joint_handles(sim: object, robot_path: str, *, include_aux_joint: bool = False) -> list[int]:
    joint_handles: list[int] = []
    for handle in sim.getObjectsInTree(_resolve_object_handle(sim, robot_path)):
        if int(sim.getObjectType(handle)) != int(getattr(sim, "object_joint_type")):
            continue
        name = str(sim.getObjectAlias(handle) if hasattr(sim, "getObjectAlias") else sim.getObjectName(handle))
        if not include_aux_joint and name == "auxJoint":
            continue
        joint_handles.append(int(handle))
    return joint_handles


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
    constraint_policy: str | None = None,
) -> dict[str, int | dict[int, int]]:
    """建立 `base -> tip -> target` 的 IK 追踪链路。"""
    try:
        payload = SetupIKLinkInput.model_validate(
            {
                "base_handle": base_handle,
                "tip_handle": tip_handle,
                "target_handle": target_handle,
                "constraints_mask": constraints_mask,
                "constraint_policy": constraint_policy,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    simik = get_simik(required=True)

    env_handle = int(simik.createEnvironment())
    group_handle = int(simik.createGroup(env_handle))

    constraint_mask = payload.constraints_mask
    if constraint_mask is None:
        constraint_mask = (
            _constraint_mask_from_policy(simik, payload.constraint_policy)
            if payload.constraint_policy is not None
            else _default_constraint_mask(simik)
        )

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
        _run_ik_group(simik, payload.environment_handle, payload.group_handle)


def move_ik_target_checked(
    environment_handle: int,
    group_handle: int,
    target_handle: int,
    tip_handle: int,
    position: list[float],
    orientation_deg: list[float] | None = None,
    relative_to: int = -1,
    steps: int = 10,
    max_position_error: float = 0.01,
    max_orientation_error_deg: float = 5.0,
    record_joint_handles: list[int] | None = None,
    collision_pairs: list[list[int]] | None = None,
) -> dict[str, object]:
    """Move an IK target and return explicit solver, residual and joint diagnostics."""
    try:
        payload = MoveIKTargetCheckedInput.model_validate(
            {
                "environment_handle": environment_handle,
                "group_handle": group_handle,
                "target_handle": target_handle,
                "tip_handle": tip_handle,
                "position": position,
                "orientation_deg": orientation_deg,
                "relative_to": relative_to,
                "steps": steps,
                "max_position_error": max_position_error,
                "max_orientation_error_deg": max_orientation_error_deg,
                "record_joint_handles": record_joint_handles or [],
                "collision_pairs": collision_pairs or [],
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    try:
        simik = get_simik(required=True)
    except PluginUnavailableError as exc:
        return {
            "ok": False,
            "environment_handle": payload.environment_handle,
            "group_handle": payload.group_handle,
            "target_handle": payload.target_handle,
            "tip_handle": payload.tip_handle,
            "target_requested_position": payload.position,
            "failure_reason": "PLUGIN_UNAVAILABLE",
            "error": str(exc),
        }

    joint_positions_before = _joint_positions(sim, payload.record_joint_handles)
    target_position_before = [float(v) for v in sim.getObjectPosition(payload.target_handle, -1)]
    tip_position_before = [float(v) for v in sim.getObjectPosition(payload.tip_handle, -1)]

    sim.setObjectPosition(payload.target_handle, payload.relative_to, payload.position)
    if payload.orientation_deg is not None:
        sim.setObjectOrientation(
            payload.target_handle,
            payload.relative_to,
            [math.radians(v) for v in payload.orientation_deg],
        )

    ik_return_codes: list[object] = []
    for _ in range(payload.steps):
        ik_return_codes.append(_run_ik_group(simik, payload.environment_handle, payload.group_handle))

    joint_positions_after = _joint_positions(sim, payload.record_joint_handles)
    target_position = [float(v) for v in sim.getObjectPosition(payload.target_handle, -1)]
    tip_position = [float(v) for v in sim.getObjectPosition(payload.tip_handle, -1)]
    target_orientation = [float(v) for v in sim.getObjectOrientation(payload.target_handle, -1)]
    tip_orientation = [float(v) for v in sim.getObjectOrientation(payload.tip_handle, -1)]
    position_error = _distance(tip_position, target_position)
    orientation_error = (
        _orientation_error_deg(tip_orientation, target_orientation)
        if payload.orientation_deg is not None
        else None
    )
    joint_delta = _joint_delta_norm(joint_positions_before, joint_positions_after)
    target_moved_distance = _distance(target_position, target_position_before)
    tip_moved_distance = _distance(tip_position, tip_position_before)
    collisions = _collision_results(sim, payload.collision_pairs)
    failure_reason = _ik_failure_reason(
        simik=simik,
        ik_return_codes=ik_return_codes,
        position_error=position_error,
        max_position_error=payload.max_position_error,
        orientation_error_deg=orientation_error,
        max_orientation_error_deg=payload.max_orientation_error_deg,
        target_moved_distance=target_moved_distance,
        tip_moved_distance=tip_moved_distance,
        joint_delta_norm=joint_delta,
        record_joint_handles=payload.record_joint_handles,
        collision_results=collisions,
    )

    return {
        "ok": failure_reason is None,
        "environment_handle": payload.environment_handle,
        "group_handle": payload.group_handle,
        "target_handle": payload.target_handle,
        "tip_handle": payload.tip_handle,
        "ik_return_codes": ik_return_codes,
        "target_requested_position": payload.position,
        "target_position": target_position,
        "tip_position": tip_position,
        "position_error": position_error,
        "target_orientation_deg": [math.degrees(v) for v in target_orientation],
        "tip_orientation_deg": [math.degrees(v) for v in tip_orientation],
        "orientation_error_deg": orientation_error,
        "joint_handles": payload.record_joint_handles,
        "joint_positions_before": joint_positions_before,
        "joint_positions_after": joint_positions_after,
        "joint_delta_norm": joint_delta,
        "target_moved_distance": target_moved_distance,
        "tip_moved_distance": tip_moved_distance,
        "collision_results": collisions,
        "failure_reason": failure_reason,
    }


def get_joint_position(handle: int) -> float:
    """读取一个非球形关节的当前位置。"""
    try:
        payload = GetJointPositionInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    return float(sim.getJointPosition(payload.handle))


def get_joint_mode(handle: int) -> int:
    """读取关节工作模式。"""
    try:
        payload = GetJointModeInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    return int(_normalize_scalar_result(sim.getJointMode(payload.handle), field_name="getJointMode"))


def set_joint_mode(handle: int, joint_mode: str) -> int:
    """设置关节工作模式。"""
    try:
        payload = SetJointModeInput.model_validate({"handle": handle, "joint_mode": joint_mode})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    mode_value = _resolve_joint_mode_value(sim, payload.joint_mode.value)
    sim.setJointMode(payload.handle, mode_value)
    return mode_value


def get_joint_dyn_ctrl_mode(handle: int) -> int:
    """读取关节动态控制模式。"""
    try:
        payload = GetJointDynCtrlModeInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    return int(sim.getObjectInt32Param(payload.handle, sim.jointintparam_dynctrlmode))


def set_joint_dyn_ctrl_mode(handle: int, dyn_ctrl_mode: str) -> int:
    """设置关节动态控制模式。"""
    try:
        payload = SetJointDynCtrlModeInput.model_validate({"handle": handle, "dyn_ctrl_mode": dyn_ctrl_mode})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    mode_value = _resolve_joint_dyn_ctrl_mode_value(sim, payload.dyn_ctrl_mode.value)
    sim.setObjectInt32Param(payload.handle, sim.jointintparam_dynctrlmode, mode_value)
    return mode_value


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


def get_joint_target_force(handle: int) -> float:
    """读取关节可施加的最大力/力矩。"""
    try:
        payload = GetJointTargetForceInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    return float(sim.getJointTargetForce(payload.handle))


def get_joint_force(handle: int) -> float:
    """读取关节当前实际力/力矩。"""
    try:
        payload = GetJointForceInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    return float(sim.getJointForce(payload.handle))


def set_joint_target_force(handle: int, force_or_torque: float, signed_value: bool = True) -> float:
    """设置关节可施加的最大力/力矩。"""
    try:
        payload = SetJointTargetForceInput.model_validate(
            {
                "handle": handle,
                "force_or_torque": force_or_torque,
                "signed_value": signed_value,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.setJointTargetForce(payload.handle, payload.force_or_torque, payload.signed_value)
    return float(payload.force_or_torque)


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


def configure_abb_arm_drive(
    robot_path: str = "/IRB4600",
    joint_mode: str = "dynamic",
    dyn_ctrl_mode: str = "position",
    max_force_or_torque: float = 2000.0,
    signed_value: bool = True,
    include_aux_joint: bool = False,
    reset_dynamics: bool = True,
) -> dict[str, object]:
    """批量配置 ABB IRB4600 机械臂关节的工作模式、动态控制模式和最大驱动力矩。"""
    try:
        payload = ConfigureAbbArmDriveInput.model_validate(
            {
                "robot_path": robot_path,
                "joint_mode": joint_mode,
                "dyn_ctrl_mode": dyn_ctrl_mode,
                "max_force_or_torque": max_force_or_torque,
                "signed_value": signed_value,
                "include_aux_joint": include_aux_joint,
                "reset_dynamics": reset_dynamics,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    robot_handle = _resolve_object_handle(sim, payload.robot_path)
    joint_handles = _abb_arm_joint_handles(sim, payload.robot_path, include_aux_joint=payload.include_aux_joint)
    joint_mode_value = _resolve_joint_mode_value(sim, payload.joint_mode.value)
    dyn_ctrl_mode_value = _resolve_joint_dyn_ctrl_mode_value(sim, payload.dyn_ctrl_mode.value)

    for handle in joint_handles:
        sim.setJointMode(handle, joint_mode_value)
        sim.setObjectInt32Param(handle, sim.jointintparam_dynctrlmode, dyn_ctrl_mode_value)
        sim.setJointTargetForce(handle, payload.max_force_or_torque, payload.signed_value)

    reset_result = None
    if payload.reset_dynamics:
        reset_result = _reset_dynamic_object(sim, robot_handle, include_model=True)

    return {
        "robot_path": payload.robot_path,
        "robot_handle": robot_handle,
        "joint_handles": joint_handles,
        "joint_mode": payload.joint_mode.value,
        "joint_mode_value": joint_mode_value,
        "dyn_ctrl_mode": payload.dyn_ctrl_mode.value,
        "dyn_ctrl_mode_value": dyn_ctrl_mode_value,
        "max_force_or_torque": float(payload.max_force_or_torque),
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


def find_robot_joints(
    robot_path: str = "/IRB4600",
    include_aux_joint: bool = False,
) -> dict[str, object]:
    """Find joint handles below a robot model path in tree order."""
    try:
        payload = FindRobotJointsInput.model_validate(
            {"robot_path": robot_path, "include_aux_joint": include_aux_joint}
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    robot_handle = _resolve_object_handle(sim, payload.robot_path)
    joint_handles = _abb_arm_joint_handles(
        sim,
        payload.robot_path,
        include_aux_joint=payload.include_aux_joint,
    )
    joint_names = [
        str(sim.getObjectAlias(handle) if hasattr(sim, "getObjectAlias") else sim.getObjectName(handle))
        for handle in joint_handles
    ]
    return {
        "robot_path": payload.robot_path,
        "robot_handle": robot_handle,
        "joint_handles": joint_handles,
        "joint_names": joint_names,
        "count": len(joint_handles),
    }


def setup_abb_arm_ik(
    robot_path: str = "/IRB4600",
    base_path: str | None = None,
    tip_path: str = "/IRB4600/IkTip",
    target_path: str = "/IRB4600/IkTarget",
    constraints_mask: int | None = None,
    constraint_policy: str | None = None,
    verify_motion: bool = True,
    test_offset: list[float] | None = None,
    restore_target: bool = True,
    detach_target_to_world: bool = True,
) -> dict[str, object]:
    """Set up ABB IRB4600 IK using the model's existing IkTip/IkTarget dummies."""
    try:
        payload = SetupAbbArmIKInput.model_validate(
            {
                "robot_path": robot_path,
                "base_path": base_path,
                "tip_path": tip_path,
                "target_path": target_path,
                "constraints_mask": constraints_mask,
                "constraint_policy": constraint_policy,
                "verify_motion": verify_motion,
                "test_offset": test_offset if test_offset is not None else [0.0, 0.0, 0.02],
                "restore_target": restore_target,
                "detach_target_to_world": detach_target_to_world,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    robot_handle = _resolve_object_handle(sim, payload.robot_path)
    base_handle = _resolve_object_handle(sim, payload.base_path) if payload.base_path else robot_handle
    tip_handle = _resolve_object_handle(sim, payload.tip_path)
    target_handle, resolved_target_path = _resolve_ik_target_handle(sim, payload.target_path)

    target_parent_before = int(sim.getObjectParent(target_handle)) if hasattr(sim, "getObjectParent") else None
    target_detached_to_world = False
    if payload.detach_target_to_world and target_parent_before not in (None, -1):
        sim.setObjectParent(target_handle, -1, True)
        target_detached_to_world = True
    target_parent_after = int(sim.getObjectParent(target_handle)) if hasattr(sim, "getObjectParent") else None

    target_before = [float(v) for v in sim.getObjectPosition(target_handle, -1)]
    tip_before = [float(v) for v in sim.getObjectPosition(tip_handle, -1)]
    ik_info = setup_ik_link(
        base_handle=base_handle,
        tip_handle=tip_handle,
        target_handle=target_handle,
        constraints_mask=payload.constraints_mask,
        constraint_policy=payload.constraint_policy.value if payload.constraint_policy is not None else None,
    )

    verification: dict[str, object] | None = None
    if payload.verify_motion:
        target_test = [target_before[i] + payload.test_offset[i] for i in range(3)]
        move_ik_target(
            environment_handle=int(ik_info["environment_handle"]),
            group_handle=int(ik_info["group_handle"]),
            target_handle=target_handle,
            position=target_test,
            relative_to=-1,
            steps=5,
        )
        tip_after = [float(v) for v in sim.getObjectPosition(tip_handle, -1)]
        moved_distance = sum((tip_after[i] - tip_before[i]) ** 2 for i in range(3)) ** 0.5
        verification = {
            "target_test_position": target_test,
            "tip_before": tip_before,
            "tip_after": tip_after,
            "tip_moved_distance": moved_distance,
            "tip_moved": moved_distance > 1e-5,
        }
        if payload.restore_target:
            sim.setObjectPosition(target_handle, -1, target_before)
            move_ik_target(
                environment_handle=int(ik_info["environment_handle"]),
                group_handle=int(ik_info["group_handle"]),
                target_handle=target_handle,
                position=target_before,
                relative_to=-1,
                steps=5,
            )

    return {
        "robot_path": payload.robot_path,
        "robot_handle": robot_handle,
        "base_handle": base_handle,
        "tip_handle": tip_handle,
        "target_handle": target_handle,
        "tip_path": payload.tip_path,
        "target_path": payload.target_path,
        "constraint_policy": payload.constraint_policy.value if payload.constraint_policy is not None else None,
        "resolved_target_path": resolved_target_path,
        "target_parent_before": target_parent_before,
        "target_parent_after": target_parent_after,
        "target_detached_to_world": target_detached_to_world,
        "verification": verification,
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
