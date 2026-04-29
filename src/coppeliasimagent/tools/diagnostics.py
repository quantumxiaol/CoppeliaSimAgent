"""Diagnostics helpers for CoppeliaSim Remote API failures."""

from __future__ import annotations

import os
import socket
import subprocess
import time
import traceback
from typing import Any

from pydantic import ValidationError

from ..core.connection import SimConnection
from ..core.exceptions import ToolValidationError
from .schemas import CollectRemoteApiDiagnosticsInput
from .simulation import _state_payload


def _validation_error(exc: ValidationError) -> ToolValidationError:
    return ToolValidationError(str(exc))


def _env_int(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        value = int(raw)
    except ValueError:
        return default
    return value if value > 0 else default


def _exception_payload(exc: BaseException) -> dict[str, object]:
    return {
        "type": type(exc).__name__,
        "message": str(exc),
        "traceback_tail": traceback.format_exception(type(exc), exc, exc.__traceback__)[-8:],
    }


def _socket_probe(host: str, port: int, timeout_s: float) -> dict[str, object]:
    started = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return {"ok": True, "elapsed_s": time.monotonic() - started}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "elapsed_s": time.monotonic() - started, "error": _exception_payload(exc)}


def _process_probe() -> dict[str, object]:
    commands = (
        ["pgrep", "-fl", "coppeliaSim"],
        ["pgrep", "-fl", "pythonLauncher.py"],
        ["pgrep", "-fl", "coppelia-toolcli|skills/toolcli.py"],
    )
    results: dict[str, object] = {}
    for command in commands:
        key = " ".join(command)
        try:
            proc = subprocess.run(command, capture_output=True, text=True, timeout=2.0, check=False)
            lines = [line for line in proc.stdout.splitlines() if line.strip()]
            results[key] = {
                "returncode": proc.returncode,
                "matches": lines[:20],
                "truncated": len(lines) > 20,
                "stderr": proc.stderr.strip() or None,
            }
        except Exception as exc:  # noqa: BLE001
            results[key] = {"error": _exception_payload(exc)}
    return results


def _safe_object_name(sim: object, handle: int) -> str:
    if hasattr(sim, "getObjectAlias"):
        try:
            return str(sim.getObjectAlias(handle))
        except Exception:
            pass
    return str(sim.getObjectName(handle))


def _safe_object_type(sim: object, handle: int) -> int | None:
    try:
        if hasattr(sim, "getObjectType"):
            return int(sim.getObjectType(handle))
        if hasattr(sim, "objintparam_type"):
            return int(sim.getObjectInt32Param(handle, sim.objintparam_type))
    except Exception:
        return None
    return None


def _safe_pose(sim: object, handle: int) -> dict[str, object]:
    pose: dict[str, object] = {}
    try:
        pose["position"] = [float(v) for v in sim.getObjectPosition(handle, -1)]
    except Exception as exc:  # noqa: BLE001
        pose["position_error"] = _exception_payload(exc)
    try:
        pose["orientation"] = [float(v) for v in sim.getObjectOrientation(handle, -1)]
    except Exception as exc:  # noqa: BLE001
        pose["orientation_error"] = _exception_payload(exc)
    return pose


def _scene_sample(
    sim: object,
    *,
    object_name_queries: list[str],
    scene_sample_limit: int,
) -> dict[str, object]:
    try:
        handles = [int(handle) for handle in sim.getObjectsInTree(sim.handle_scene)]
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": _exception_payload(exc)}

    sample: list[dict[str, Any]] = []
    matches: dict[str, list[dict[str, Any]]] = {query: [] for query in object_name_queries}
    type_counts: dict[str, int] = {}
    query_lower = {query: query.lower() for query in object_name_queries}

    for handle in handles:
        try:
            name = _safe_object_name(sim, handle)
        except Exception:
            name = f"<unreadable:{handle}>"
        obj_type = _safe_object_type(sim, handle)
        type_key = "unknown" if obj_type is None else str(obj_type)
        type_counts[type_key] = type_counts.get(type_key, 0) + 1

        item: dict[str, Any] = {
            "handle": handle,
            "name": name,
            "type": obj_type,
        }
        if len(sample) < scene_sample_limit:
            item.update(_safe_pose(sim, handle))
            sample.append(item)

        name_lower = name.lower()
        for query, lowered in query_lower.items():
            if lowered in name_lower and len(matches[query]) < scene_sample_limit:
                matched_item = dict(item)
                matched_item.update(_safe_pose(sim, handle))
                matches[query].append(matched_item)

    return {
        "ok": True,
        "object_count": len(handles),
        "type_counts": type_counts,
        "sample": sample,
        "matches": matches,
    }


def collect_remote_api_diagnostics(
    host: str | None = None,
    port: int | None = None,
    timeout_s: float = 3.0,
    plugin_names: list[str] | None = None,
    include_scene_sample: bool = True,
    object_name_queries: list[str] | None = None,
    scene_sample_limit: int = 40,
    include_process_probe: bool = True,
    probe_step: bool = False,
) -> dict[str, object]:
    """Collect connection, plugin, scene and exception context without raising on Remote API failures."""
    try:
        payload = CollectRemoteApiDiagnosticsInput.model_validate(
            {
                "host": host,
                "port": port,
                "timeout_s": timeout_s,
                "plugin_names": plugin_names if plugin_names is not None else ["simIK", "simOMPL"],
                "include_scene_sample": include_scene_sample,
                "object_name_queries": object_name_queries
                if object_name_queries is not None
                else ["IRB4600", "Ik", "push_test", "jar"],
                "scene_sample_limit": scene_sample_limit,
                "include_process_probe": include_process_probe,
                "probe_step": probe_step,
            }
        )
    except ValidationError as exc:
        raise _validation_error(exc) from exc

    resolved_host = payload.host or os.getenv("COPPELIASIM_HOST", "127.0.0.1")
    resolved_port = payload.port or _env_int("COPPELIASIM_ZMQ_PORT", 23000)
    started = time.monotonic()
    diagnostics: dict[str, object] = {
        "ok": False,
        "host": resolved_host,
        "port": resolved_port,
        "timeout_s": payload.timeout_s,
        "socket_probe": _socket_probe(resolved_host, resolved_port, payload.timeout_s),
    }
    if payload.include_process_probe:
        diagnostics["process_probe"] = _process_probe()

    connection: SimConnection | None = None
    try:
        connection = SimConnection(
            host=resolved_host,
            port=resolved_port,
            timeout_s=payload.timeout_s,
            preload_plugins=tuple(payload.plugin_names),
            auto_connect=True,
        )
        sim = connection.sim
        client = connection.client
        diagnostics["remote_api"] = {"connected": True}
        diagnostics["simulation_state"] = _state_payload(sim)
        diagnostics["plugins"] = {
            name: {"available": connection.get_plugin(name, required=False) is not None}
            for name in payload.plugin_names
        }
        if hasattr(sim, "getSimulationTime"):
            try:
                diagnostics["simulation_time"] = float(sim.getSimulationTime())
            except Exception as exc:  # noqa: BLE001
                diagnostics["simulation_time_error"] = _exception_payload(exc)
        if payload.include_scene_sample:
            diagnostics["scene"] = _scene_sample(
                sim,
                object_name_queries=payload.object_name_queries,
                scene_sample_limit=payload.scene_sample_limit,
            )
        if payload.probe_step:
            try:
                if hasattr(client, "setStepping"):
                    client.setStepping(True)
                if hasattr(client, "step"):
                    client.step()
                elif hasattr(sim, "step"):
                    sim.step()
                diagnostics["step_probe"] = {"ok": True}
            except Exception as exc:  # noqa: BLE001
                diagnostics["step_probe"] = {"ok": False, "error": _exception_payload(exc)}

        diagnostics["ok"] = True
    except Exception as exc:  # noqa: BLE001
        diagnostics["remote_api"] = {
            "connected": False,
            "error": _exception_payload(exc),
        }
    finally:
        diagnostics["elapsed_s"] = time.monotonic() - started
        if connection is not None:
            connection.close()

    return diagnostics
