"""Direct tool CLI without LLM orchestration."""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, Sequence

from pydantic import ValidationError

from ..agent.tool_registry import get_tool, invoke_tool, list_tool_names


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="coppelia-toolcli",
        description="Inspect and call coppeliasimagent tools directly without an LLM.",
    )
    subparsers = parser.add_subparsers(dest="command")

    list_parser = subparsers.add_parser("list", help="List all available tools.")
    list_parser.add_argument(
        "--json",
        action="store_true",
        help="Print the tool list as JSON.",
    )

    show_parser = subparsers.add_parser("show", help="Show one tool's description and input schema.")
    show_parser.add_argument("tool_name", help="Tool name to inspect.")
    show_parser.add_argument(
        "--json",
        action="store_true",
        help="Print the full tool metadata as JSON.",
    )

    call_parser = subparsers.add_parser("call", help="Call one tool with a JSON payload.")
    call_parser.add_argument("tool_name", help="Tool name to invoke.")
    call_parser.add_argument(
        "--payload",
        help="JSON payload for the tool call. Defaults to '{}'.",
    )
    call_parser.add_argument(
        "--payload-file",
        help="Read the JSON payload from a file path.",
    )
    call_parser.add_argument(
        "--stdin",
        action="store_true",
        help="Read the JSON payload from stdin.",
    )

    return parser


def _load_payload(args: argparse.Namespace) -> dict[str, Any]:
    sources = [
        args.payload is not None,
        bool(args.payload_file),
        bool(args.stdin),
    ]
    if sum(sources) > 1:
        raise ValueError("Use only one of --payload, --payload-file, or --stdin.")

    if args.payload_file:
        with open(args.payload_file, "r", encoding="utf-8") as fh:
            raw = fh.read()
    elif args.stdin:
        raw = sys.stdin.read()
    else:
        raw = args.payload if args.payload is not None else "{}"

    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON payload: {exc.msg}") from exc

    if not isinstance(parsed, dict):
        raise ValueError("Payload must decode to a JSON object.")
    return parsed


def _print_json(value: Any) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))


def _run_list(*, as_json: bool) -> int:
    names = list_tool_names()
    if as_json:
        payload = [
            {
                "name": name,
                "description": get_tool(name).description,
            }
            for name in names
        ]
        _print_json(payload)
        return 0

    for name in names:
        print(f"{name}: {get_tool(name).description}")
    return 0


def _run_show(tool_name: str, *, as_json: bool) -> int:
    tool = get_tool(tool_name)
    payload = {
        "name": tool.name,
        "description": tool.description,
        "parameters": tool.input_model.model_json_schema(),
    }
    if as_json:
        _print_json(payload)
        return 0

    print(f"name: {tool.name}")
    print(f"description: {tool.description}")
    print("parameters:")
    _print_json(payload["parameters"])
    return 0


def _run_call(args: argparse.Namespace) -> int:
    payload = _load_payload(args)
    result = invoke_tool(args.tool_name, payload)
    _print_json(
        {
            "tool": args.tool_name,
            "payload": payload,
            "result": result,
        }
    )
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.command is None:
        parser.print_help()
        return 0

    try:
        if args.command == "list":
            return _run_list(as_json=args.json)
        if args.command == "show":
            return _run_show(args.tool_name, as_json=args.json)
        if args.command == "call":
            return _run_call(args)
    except KeyError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    except ValidationError as exc:
        print(exc, file=sys.stderr)
        return 3
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 4
    except Exception as exc:  # noqa: BLE001
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 5

    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
