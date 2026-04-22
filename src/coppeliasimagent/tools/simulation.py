"""Simulation lifecycle tools."""

from __future__ import annotations

from pydantic import ValidationError

from ..core.connection import get_connection, get_sim
from ..core.exceptions import ToolValidationError
from .schemas import (
    GetPluginStatusInput,
    GetSimulationStateInput,
    PauseSimulationInput,
    StartSimulationInput,
    StopSimulationInput,
)

_SIM_STATE_NAMES = (
    "simulation_stopped",
    "simulation_paused",
    "simulation_advancing_firstafterstop",
    "simulation_advancing_running",
    "simulation_advancing_lastbeforepause",
    "simulation_advancing_firstafterpause",
    "simulation_advancing_abouttostop",
    "simulation_advancing_lastbeforestop",
)


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _simulation_state_label(sim: object, state_value: int) -> str:
    for name in _SIM_STATE_NAMES:
        if hasattr(sim, name) and int(getattr(sim, name)) == int(state_value):
            return name
    return f"unknown({state_value})"


def _state_payload(sim: object) -> dict[str, int | str | bool]:
    state_value = int(sim.getSimulationState())
    state_label = _simulation_state_label(sim, state_value)
    return {
        "state": state_label,
        "state_value": state_value,
        "is_running": state_label not in {"simulation_stopped", "simulation_paused"},
    }


def get_simulation_state() -> dict[str, int | str | bool]:
    """Read current CoppeliaSim simulation lifecycle state."""
    try:
        GetSimulationStateInput.model_validate({})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    return _state_payload(sim)


def get_plugin_status(
    plugin_names: list[str] | None = None,
    refresh: bool = False,
) -> dict[str, object]:
    """Read availability of simulator-side plugin namespaces such as simIK."""
    try:
        payload = GetPluginStatusInput.model_validate(
            {
                "plugin_names": plugin_names if plugin_names is not None else ["simIK", "simOMPL"],
                "refresh": refresh,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    connection = get_connection()
    if payload.refresh:
        connection.reconnect()

    plugins: dict[str, dict[str, object]] = {}
    for name in payload.plugin_names:
        plugin = connection.get_plugin(name, required=False)
        plugins[name] = {
            "available": plugin is not None,
            "kind": "simulator_plugin",
        }

    return {
        "host": connection.host,
        "port": connection.port,
        "plugins": plugins,
        "notes": {
            "simulator_side": "plugin availability depends on the running CoppeliaSim instance, not the project venv",
            "python_wrapper_error": "pythonWrapperV2.lua/pyzmq/cbor2 errors refer to simulator-side Python script execution, not simIK",
        },
    }


def start_simulation() -> dict[str, int | str | bool]:
    """Start or resume simulation execution."""
    try:
        StartSimulationInput.model_validate({})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.startSimulation()
    return _state_payload(sim)


def pause_simulation() -> dict[str, int | str | bool]:
    """Pause simulation execution."""
    try:
        PauseSimulationInput.model_validate({})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.pauseSimulation()
    return _state_payload(sim)


def stop_simulation() -> dict[str, int | str | bool]:
    """Stop simulation execution."""
    try:
        StopSimulationInput.model_validate({})
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    sim = get_sim()
    sim.stopSimulation()
    return _state_payload(sim)
