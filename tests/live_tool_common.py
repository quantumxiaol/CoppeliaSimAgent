#!/usr/bin/env python3
"""Shared helpers for live tool scripts.

These scripts are intended to run against a real CoppeliaSim instance.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def parse_vec3(raw: str) -> list[float]:
    """Parse 'x,y,z' into a 3-float vector."""
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("vector must be in format x,y,z")

    try:
        vec = [float(p) for p in parts]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid float in vector: {raw}") from exc

    return vec


def print_header(title: str) -> None:
    print(title)
    print("-" * len(title))


def print_pass(message: str) -> None:
    print(f"[PASS] {message}")


def print_fail(message: str) -> None:
    print(f"[FAIL] {message}")
