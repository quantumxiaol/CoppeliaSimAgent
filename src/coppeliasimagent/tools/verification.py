"""Action result verification tools."""

from __future__ import annotations

import math

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .dynamics import get_object_velocity
from .schemas import (
    VerifyForceThresholdInput,
    VerifyJointPositionsReachedInput,
    VerifyObjectMovedInput,
    VerifyObjectVelocityBelowInput,
)


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def verify_joint_positions_reached(
    joint_handles: list[int],
    target_positions: list[float],
    tolerance: float = 0.01,
) -> dict[str, object]:
    """Check whether joints are within tolerance of requested positions."""
    try:
        payload = VerifyJointPositionsReachedInput.model_validate(
            {
                "joint_handles": joint_handles,
                "target_positions": target_positions,
                "tolerance": tolerance,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    current = [float(sim.getJointPosition(handle)) for handle in payload.joint_handles]
    errors = [abs(current[i] - payload.target_positions[i]) for i in range(len(current))]
    return {
        "reached": all(error <= payload.tolerance for error in errors),
        "joint_handles": payload.joint_handles,
        "target_positions": payload.target_positions,
        "current_positions": current,
        "errors": errors,
        "tolerance": payload.tolerance,
    }


def verify_object_moved(
    handle: int,
    start_position: list[float],
    min_distance: float = 0.01,
    relative_to: int = -1,
) -> dict[str, object]:
    """Check whether an object has moved at least a minimum distance."""
    try:
        payload = VerifyObjectMovedInput.model_validate(
            {
                "handle": handle,
                "start_position": start_position,
                "min_distance": min_distance,
                "relative_to": relative_to,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    current = [float(v) for v in sim.getObjectPosition(payload.handle, payload.relative_to)]
    moved_distance = _distance(current, payload.start_position)
    return {
        "moved": moved_distance >= payload.min_distance,
        "handle": payload.handle,
        "start_position": payload.start_position,
        "current_position": current,
        "moved_distance": moved_distance,
        "min_distance": payload.min_distance,
    }


def verify_object_velocity_below(
    handle: int,
    max_linear_speed: float = 0.01,
    max_angular_speed: float = 0.01,
) -> dict[str, object]:
    """Check whether object linear/angular speed is below thresholds."""
    try:
        payload = VerifyObjectVelocityBelowInput.model_validate(
            {
                "handle": handle,
                "max_linear_speed": max_linear_speed,
                "max_angular_speed": max_angular_speed,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    velocity = get_object_velocity(payload.handle)
    below = (
        float(velocity["linear_speed"]) <= payload.max_linear_speed
        and float(velocity["angular_speed"]) <= payload.max_angular_speed
    )
    return {
        "below_threshold": below,
        "handle": payload.handle,
        "max_linear_speed": payload.max_linear_speed,
        "max_angular_speed": payload.max_angular_speed,
        **velocity,
    }


def verify_force_threshold(
    joint_handles: list[int],
    min_abs_force: float = 0.1,
) -> dict[str, object]:
    """Check whether any listed joint reports at least the requested force."""
    try:
        payload = VerifyForceThresholdInput.model_validate(
            {
                "joint_handles": joint_handles,
                "min_abs_force": min_abs_force,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    forces = [float(sim.getJointForce(handle)) for handle in payload.joint_handles]
    return {
        "threshold_met": any(abs(force) >= payload.min_abs_force for force in forces),
        "joint_handles": payload.joint_handles,
        "forces": forces,
        "min_abs_force": payload.min_abs_force,
    }
