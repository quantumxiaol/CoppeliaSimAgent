"""Sensor and contact-monitor tools."""

from __future__ import annotations

import time

from pydantic import ValidationError

from ..core.connection import get_sim
from ..core.exceptions import ToolValidationError
from .scene import check_collision
from .schemas import (
    CheckCollisionMonitorInput,
    GetVisionSensorImageInput,
    ReadForceSensorInput,
    ReadProximitySensorInput,
)


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def read_proximity_sensor(handle: int) -> dict[str, object]:
    """Read one proximity sensor."""
    try:
        payload = ReadProximitySensorInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "readProximitySensor"):
        raise RuntimeError("Current CoppeliaSim API does not expose readProximitySensor")
    result = sim.readProximitySensor(payload.handle)
    detected = bool(result[0]) if isinstance(result, (list, tuple)) and result else bool(result)
    return {"handle": payload.handle, "detected": detected, "raw": result}


def read_force_sensor(handle: int) -> dict[str, object]:
    """Read one force sensor."""
    try:
        payload = ReadForceSensorInput.model_validate({"handle": handle})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "readForceSensor"):
        raise RuntimeError("Current CoppeliaSim API does not expose readForceSensor")
    result = sim.readForceSensor(payload.handle)
    force = list(result[1]) if isinstance(result, (list, tuple)) and len(result) > 1 else []
    torque = list(result[2]) if isinstance(result, (list, tuple)) and len(result) > 2 else []
    return {"handle": payload.handle, "raw": result, "force": force, "torque": torque}


def get_vision_sensor_image(
    handle: int,
    grayscale: bool = False,
    metadata_only: bool = True,
) -> dict[str, object]:
    """Read a vision sensor image or metadata."""
    try:
        payload = GetVisionSensorImageInput.model_validate(
            {"handle": handle, "grayscale": grayscale, "metadata_only": metadata_only}
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    if not hasattr(sim, "getVisionSensorImg"):
        raise RuntimeError("Current CoppeliaSim API does not expose getVisionSensorImg")
    result = sim.getVisionSensorImg(payload.handle, 1 if payload.grayscale else 0)
    image = result[0] if isinstance(result, (list, tuple)) and result else result
    resolution = result[1] if isinstance(result, (list, tuple)) and len(result) > 1 else None
    out: dict[str, object] = {
        "handle": payload.handle,
        "grayscale": payload.grayscale,
        "resolution": resolution,
        "byte_length": len(image) if hasattr(image, "__len__") else None,
    }
    if not payload.metadata_only:
        out["image"] = image
    return out


def check_collision_monitor(
    entity1: int,
    entity2: int,
    duration_s: float = 0.0,
    poll_interval_s: float = 0.05,
) -> dict[str, object]:
    """Monitor a collision pair for a duration."""
    try:
        payload = CheckCollisionMonitorInput.model_validate(
            {
                "entity1": entity1,
                "entity2": entity2,
                "duration_s": duration_s,
                "poll_interval_s": poll_interval_s,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    deadline = time.monotonic() + payload.duration_s
    checks = 0
    ever_collided = False
    while True:
        checks += 1
        collides = check_collision(payload.entity1, payload.entity2)
        ever_collided = ever_collided or collides
        if payload.duration_s <= 0.0 or time.monotonic() >= deadline:
            return {
                "entity1": payload.entity1,
                "entity2": payload.entity2,
                "checks": checks,
                "collides": collides,
                "ever_collided": ever_collided,
            }
        time.sleep(payload.poll_interval_s)
