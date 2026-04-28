"""Dynamics and physical property tools."""

from __future__ import annotations

import math

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .schemas import GetObjectVelocityInput, ResetDynamicObjectInput, SetShapeDynamicsInput


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _vector_norm(values: list[float]) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in values))


def _reset_handle(sim: object, handle: int, include_model: bool) -> int:
    object_handle = int(handle)
    if include_model and hasattr(sim, "handleflag_model"):
        object_handle |= int(getattr(sim, "handleflag_model"))
    if not hasattr(sim, "resetDynamicObject"):
        raise RuntimeError("Current CoppeliaSim API does not expose resetDynamicObject")
    return int(sim.resetDynamicObject(object_handle))


def get_object_velocity(handle: int) -> dict[str, object]:
    """Read linear and angular object velocity."""
    try:
        payload = GetObjectVelocityInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "getObjectVelocity"):
        raise RuntimeError("Current CoppeliaSim API does not expose getObjectVelocity")
    result = sim.getObjectVelocity(payload.handle)
    if not isinstance(result, (list, tuple)) or len(result) < 2:
        raise RuntimeError(f"Unexpected getObjectVelocity result: {result!r}")
    linear = [float(v) for v in result[0]]
    angular = [float(v) for v in result[1]]
    return {
        "handle": payload.handle,
        "linear_velocity": linear,
        "angular_velocity": angular,
        "linear_speed": _vector_norm(linear),
        "angular_speed": _vector_norm(angular),
    }


def reset_dynamic_object(handle: int, include_model: bool = True) -> dict[str, object]:
    """Reset dynamic state for an object or model."""
    try:
        payload = ResetDynamicObjectInput.model_validate({"handle": handle, "include_model": include_model})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    result = _reset_handle(sim, payload.handle, payload.include_model)
    return {"handle": payload.handle, "include_model": payload.include_model, "result": result}


def set_shape_dynamics(
    handle: int,
    static: bool | None = None,
    respondable: bool | None = None,
    mass: float | None = None,
    friction: float | None = None,
) -> dict[str, object]:
    """Set common shape dynamic flags and optional mass/friction when supported."""
    try:
        payload = SetShapeDynamicsInput.model_validate(
            {
                "handle": handle,
                "static": static,
                "respondable": respondable,
                "mass": mass,
                "friction": friction,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    applied: dict[str, object] = {}
    if payload.static is not None:
        if not hasattr(sim, "shapeintparam_static"):
            raise RuntimeError("CoppeliaSim API constant missing: shapeintparam_static")
        sim.setObjectInt32Param(payload.handle, sim.shapeintparam_static, 1 if payload.static else 0)
        applied["static"] = payload.static
    if payload.respondable is not None:
        if not hasattr(sim, "shapeintparam_respondable"):
            raise RuntimeError("CoppeliaSim API constant missing: shapeintparam_respondable")
        sim.setObjectInt32Param(payload.handle, sim.shapeintparam_respondable, 1 if payload.respondable else 0)
        applied["respondable"] = payload.respondable
    if payload.mass is not None:
        if not hasattr(sim, "setShapeMass"):
            raise RuntimeError("Current CoppeliaSim API does not expose setShapeMass")
        sim.setShapeMass(payload.handle, payload.mass)
        applied["mass"] = payload.mass
    if payload.friction is not None:
        param = getattr(sim, "shapefloatparam_friction", None)
        if param is None or not hasattr(sim, "setObjectFloatParam"):
            applied["friction"] = {
                "requested": payload.friction,
                "applied": False,
                "reason": "Current CoppeliaSim API does not expose shape friction parameter",
            }
        else:
            sim.setObjectFloatParam(payload.handle, param, payload.friction)
            applied["friction"] = payload.friction

    if applied:
        _reset_handle(sim, payload.handle, include_model=False)
    return {"handle": payload.handle, "applied": applied}
