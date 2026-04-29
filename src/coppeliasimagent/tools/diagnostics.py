"""Diagnostics helpers for CoppeliaSim Remote API failures."""

from __future__ import annotations

import os
import json
import signal
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


def _toolcli_process_entries(stale_min_age_s: float) -> list[dict[str, object]]:
    try:
        proc = subprocess.run(
            ["pgrep", "-fl", "coppelia-toolcli|skills/toolcli.py"],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except Exception:
        return []

    current_pid = os.getpid()
    parent_pid = os.getppid()
    entries: list[dict[str, object]] = []
    seen: set[int] = set()
    for line in proc.stdout.splitlines():
        text = line.strip()
        if not text:
            continue
        parts = text.split(maxsplit=1)
        try:
            pid = int(parts[0])
        except (ValueError, IndexError):
            continue
        if pid in seen or pid in (current_pid, parent_pid):
            continue
        seen.add(pid)
        command = parts[1] if len(parts) > 1 else ""
        age_s: float | None = None
        try:
            ps = subprocess.run(
                ["ps", "-p", str(pid), "-o", "etimes=", "-o", "command="],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=False,
            )
            ps_line = ps.stdout.strip()
            if ps_line:
                ps_parts = ps_line.split(maxsplit=1)
                age_s = float(ps_parts[0])
                if len(ps_parts) > 1:
                    command = ps_parts[1]
        except Exception:
            pass
        stale = age_s is not None and age_s >= stale_min_age_s
        entries.append(
            {
                "pid": pid,
                "age_s": age_s,
                "stale": stale,
                "command": command,
            }
        )
    return entries


def _cleanup_stale_toolcli(entries: list[dict[str, object]]) -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    for entry in entries:
        if not bool(entry.get("stale")):
            continue
        pid = int(entry["pid"])
        try:
            os.kill(pid, signal.SIGTERM)
            results.append({"pid": pid, "ok": True, "signal": "SIGTERM"})
        except Exception as exc:  # noqa: BLE001
            results.append({"pid": pid, "ok": False, "error": _exception_payload(exc)})
    return results


def _process_probe(*, stale_toolcli_min_age_s: float, cleanup_stale_toolcli: bool) -> dict[str, object]:
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
    toolcli_entries = _toolcli_process_entries(stale_toolcli_min_age_s)
    results["stale_toolcli_processes"] = {
        "min_age_s": stale_toolcli_min_age_s,
        "matches": toolcli_entries,
        "stale_count": sum(1 for entry in toolcli_entries if bool(entry.get("stale"))),
    }
    if cleanup_stale_toolcli:
        results["stale_toolcli_processes"]["cleanup"] = _cleanup_stale_toolcli(toolcli_entries)
    return results


def _zmq_recovery_status(diagnostics: dict[str, object]) -> dict[str, object]:
    socket_probe = diagnostics.get("socket_probe")
    socket_ok = isinstance(socket_probe, dict) and bool(socket_probe.get("ok"))
    remote_api = diagnostics.get("remote_api")
    connected = isinstance(remote_api, dict) and bool(remote_api.get("connected"))
    step_probe = diagnostics.get("step_probe")
    if isinstance(step_probe, dict) and not bool(step_probe.get("ok", False)):
        return {
            "status": "step_failed_possible_addon_abort",
            "socket_open": socket_ok,
            "remote_api_connected": connected,
            "recoverable": False,
        }
    if connected:
        return {
            "status": "remote_api_responsive",
            "socket_open": socket_ok,
            "remote_api_connected": True,
            "recoverable": True,
        }
    if socket_ok:
        return {
            "status": "socket_open_remote_api_failed",
            "socket_open": True,
            "remote_api_connected": False,
            "recoverable": False,
        }
    return {
        "status": "socket_unreachable",
        "socket_open": False,
        "remote_api_connected": False,
        "recoverable": False,
    }


def _append_diagnostics_log(path: str, diagnostics: dict[str, object], exception: dict[str, object] | None) -> dict[str, object]:
    entry = {
        "timestamp_unix": time.time(),
        "ok": diagnostics.get("ok"),
        "host": diagnostics.get("host"),
        "port": diagnostics.get("port"),
        "remote_api": diagnostics.get("remote_api"),
        "step_probe": diagnostics.get("step_probe"),
        "zmq_server_status": diagnostics.get("zmq_server_status"),
        "recent_exception": exception,
    }
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "a", encoding="utf-8") as file:
            file.write(json.dumps(entry, ensure_ascii=True, default=str) + "\n")
        return {"ok": True, "path": path}
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "path": path, "error": _exception_payload(exc)}


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
    stale_toolcli_min_age_s: float = 30.0,
    cleanup_stale_toolcli: bool = False,
    record_log: bool = True,
    log_path: str | None = "/tmp/coppeliasimagent_remote_api_diagnostics.jsonl",
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
                "stale_toolcli_min_age_s": stale_toolcli_min_age_s,
                "cleanup_stale_toolcli": cleanup_stale_toolcli,
                "record_log": record_log,
                "log_path": log_path,
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
        diagnostics["process_probe"] = _process_probe(
            stale_toolcli_min_age_s=payload.stale_toolcli_min_age_s,
            cleanup_stale_toolcli=payload.cleanup_stale_toolcli,
        )

    connection: SimConnection | None = None
    recent_exception: dict[str, object] | None = None
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
                recent_exception = _exception_payload(exc)
                diagnostics["step_probe"] = {"ok": False, "error": recent_exception}
            finally:
                try:
                    if hasattr(client, "setStepping"):
                        client.setStepping(False)
                except Exception:
                    pass

        diagnostics["ok"] = True
    except Exception as exc:  # noqa: BLE001
        recent_exception = _exception_payload(exc)
        diagnostics["remote_api"] = {
            "connected": False,
            "error": recent_exception,
        }
    finally:
        diagnostics["elapsed_s"] = time.monotonic() - started
        diagnostics["zmq_server_status"] = _zmq_recovery_status(diagnostics)
        if recent_exception is not None:
            diagnostics["recent_exception"] = recent_exception
        if payload.record_log and payload.log_path is not None:
            diagnostics["log_record"] = _append_diagnostics_log(payload.log_path, diagnostics, recent_exception)
        if connection is not None:
            connection.close()

    return diagnostics
