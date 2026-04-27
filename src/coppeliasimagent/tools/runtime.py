"""Runtime stepping and wait tools."""

from __future__ import annotations

import math
import time

from pydantic import ValidationError

from ..core.connection import get_connection, get_sim
from ..core.exceptions import ToolValidationError
from .schemas import (
    StepSimulationInput,
    WaitSecondsInput,
    WaitUntilObjectPoseStableInput,
    WaitUntilStateInput,
)
from .simulation import _state_payload


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _orientation_distance_deg(a: list[float], b: list[float]) -> float:
    return math.degrees(_distance(a, b))


def step_simulation(
    steps: int = 1,
    start_if_stopped: bool = True,
    keep_stepping_enabled: bool = True,
) -> dict[str, object]:
    """Advance the simulator by a fixed number of remote stepping ticks."""
    try:
        payload = StepSimulationInput.model_validate(
            {
                "steps": steps,
                "start_if_stopped": start_if_stopped,
                "keep_stepping_enabled": keep_stepping_enabled,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    connection = get_connection()
    sim = connection.sim
    client = connection.client
    before = _state_payload(sim)

    if bool(before["is_running"]) is False and payload.start_if_stopped:
        if hasattr(client, "setStepping"):
            client.setStepping(True)
        sim.startSimulation()

    if hasattr(client, "setStepping"):
        client.setStepping(True)

    if hasattr(client, "step"):
        for _ in range(payload.steps):
            client.step()
    elif hasattr(sim, "step"):
        for _ in range(payload.steps):
            sim.step()
    else:
        # Compatibility fallback for fake/older clients. It does not provide
        # deterministic stepping, but still gives physics time to progress.
        time.sleep(0.01 * payload.steps)

    if hasattr(client, "setStepping") and not payload.keep_stepping_enabled:
        client.setStepping(False)

    after = _state_payload(sim)
    return {
        "steps": payload.steps,
        "state_before": before,
        "state_after": after,
        "used_remote_step": hasattr(client, "step"),
    }


def wait_seconds(seconds: float) -> dict[str, float]:
    """Sleep in the external agent process."""
    try:
        payload = WaitSecondsInput.model_validate({"seconds": seconds})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    started = time.monotonic()
    time.sleep(payload.seconds)
    return {"requested_seconds": payload.seconds, "elapsed_seconds": time.monotonic() - started}


def wait_until_state(
    target_state: str,
    timeout_s: float = 5.0,
    poll_interval_s: float = 0.05,
) -> dict[str, object]:
    """Poll simulation lifecycle state until the requested label or value appears."""
    try:
        payload = WaitUntilStateInput.model_validate(
            {
                "target_state": target_state,
                "timeout_s": timeout_s,
                "poll_interval_s": poll_interval_s,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    deadline = time.monotonic() + payload.timeout_s
    last = _state_payload(sim)
    while time.monotonic() <= deadline:
        last = _state_payload(sim)
        if str(last["state"]) == payload.target_state or str(last["state_value"]) == payload.target_state:
            return {"reached": True, "state": last}
        time.sleep(payload.poll_interval_s)

    return {"reached": False, "state": last, "timeout_s": payload.timeout_s}


def wait_until_object_pose_stable(
    handle: int,
    position_tolerance: float = 0.001,
    orientation_tolerance_deg: float = 0.1,
    stable_duration_s: float = 0.25,
    timeout_s: float = 5.0,
    poll_interval_s: float = 0.05,
    relative_to: int = -1,
) -> dict[str, object]:
    """Poll an object's pose until it remains within tolerances for a duration."""
    try:
        payload = WaitUntilObjectPoseStableInput.model_validate(
            {
                "handle": handle,
                "position_tolerance": position_tolerance,
                "orientation_tolerance_deg": orientation_tolerance_deg,
                "stable_duration_s": stable_duration_s,
                "timeout_s": timeout_s,
                "poll_interval_s": poll_interval_s,
                "relative_to": relative_to,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    deadline = time.monotonic() + payload.timeout_s
    stable_since: float | None = None
    previous_position = [float(v) for v in sim.getObjectPosition(payload.handle, payload.relative_to)]
    previous_orientation = [float(v) for v in sim.getObjectOrientation(payload.handle, payload.relative_to)]
    last_delta_position = 0.0
    last_delta_orientation_deg = 0.0

    while time.monotonic() <= deadline:
        time.sleep(payload.poll_interval_s)
        position = [float(v) for v in sim.getObjectPosition(payload.handle, payload.relative_to)]
        orientation = [float(v) for v in sim.getObjectOrientation(payload.handle, payload.relative_to)]
        last_delta_position = _distance(position, previous_position)
        last_delta_orientation_deg = _orientation_distance_deg(orientation, previous_orientation)

        if (
            last_delta_position <= payload.position_tolerance
            and last_delta_orientation_deg <= payload.orientation_tolerance_deg
        ):
            if stable_since is None:
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= payload.stable_duration_s:
                return {
                    "stable": True,
                    "handle": payload.handle,
                    "position": position,
                    "orientation": orientation,
                    "last_delta_position": last_delta_position,
                    "last_delta_orientation_deg": last_delta_orientation_deg,
                }
        else:
            stable_since = None

        previous_position = position
        previous_orientation = orientation

    return {
        "stable": False,
        "handle": payload.handle,
        "last_delta_position": last_delta_position,
        "last_delta_orientation_deg": last_delta_orientation_deg,
        "timeout_s": payload.timeout_s,
    }
