#!/usr/bin/env python3
"""Connectivity check for CoppeliaSim Remote APIs.

Checks:
1) TCP reachability for ZMQ port (default: 23000)
2) TCP reachability for WebSocket port (default: 23050)
3) A real ZMQ RPC call via coppeliasim-zmqremoteapi-client
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
from typing import Tuple

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="CoppeliaSim connectivity test")
    parser.add_argument(
        "--host",
        default=os.getenv("COPPELIASIM_HOST", "127.0.0.1"),
        help="CoppeliaSim host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--zmq-port",
        type=int,
        default=int(os.getenv("COPPELIASIM_ZMQ_PORT", "23000")),
        help="ZMQ Remote API port (default: 23000)",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=int(os.getenv("COPPELIASIM_WS_PORT", "23050")),
        help="WebSocket Remote API port (default: 23050)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=float(os.getenv("COPPELIASIM_TIMEOUT", "3.0")),
        help="Socket timeout seconds (default: 3.0)",
    )
    return parser.parse_args()


def check_tcp(host: str, port: int, timeout: float) -> Tuple[bool, str]:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, f"TCP reachable: {host}:{port}"
    except OSError as exc:
        return False, f"TCP connect failed: {host}:{port} ({exc})"


def check_zmq_rpc(host: str, port: int, timeout: float) -> Tuple[bool, str]:
    client = None
    try:
        client = RemoteAPIClient(host=host, port=port)
        timeout_ms = int(timeout * 1000)
        client.socket.RCVTIMEO = timeout_ms
        client.socket.SNDTIMEO = timeout_ms

        sim = client.getObject("sim")
        time_step = sim.getFloatParam(sim.floatparam_simulation_time_step)
        return True, f"ZMQ RPC OK: simulation timestep = {time_step} s"
    except Exception as exc:  # noqa: BLE001
        return False, f"ZMQ RPC failed: {exc}"
    finally:
        if client is not None:
            try:
                client.socket.close(0)
                client.context.term()
            except Exception:
                pass


def main() -> int:
    args = parse_args()

    print("CoppeliaSim connectivity test")
    print(f"host={args.host}, zmq={args.zmq_port}, websocket={args.ws_port}, timeout={args.timeout}s")

    ok = True

    zmq_tcp_ok, zmq_tcp_msg = check_tcp(args.host, args.zmq_port, args.timeout)
    print(("[PASS] " if zmq_tcp_ok else "[FAIL] ") + zmq_tcp_msg)
    ok = ok and zmq_tcp_ok

    ws_tcp_ok, ws_tcp_msg = check_tcp(args.host, args.ws_port, args.timeout)
    print(("[PASS] " if ws_tcp_ok else "[FAIL] ") + ws_tcp_msg)
    ok = ok and ws_tcp_ok

    zmq_rpc_ok, zmq_rpc_msg = check_zmq_rpc(args.host, args.zmq_port, args.timeout)
    print(("[PASS] " if zmq_rpc_ok else "[FAIL] ") + zmq_rpc_msg)
    ok = ok and zmq_rpc_ok

    if ok:
        print("All checks passed.")
        return 0

    print("One or more checks failed.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
