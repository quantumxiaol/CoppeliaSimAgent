"""Thin wrapper to run the packaged tool CLI from the skills directory."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _bootstrap() -> None:
    repo_root = _repo_root()
    src_dir = repo_root / "src"
    if str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))


def _maybe_reexec_into_repo_venv() -> None:
    repo_python = _repo_root() / ".venv" / "bin" / "python"
    if not repo_python.exists():
        return
    try:
        same_python = Path(sys.executable).resolve() == repo_python.resolve()
    except OSError:
        same_python = False
    if same_python:
        return
    os.execv(str(repo_python), [str(repo_python), str(Path(__file__).resolve()), *sys.argv[1:]])


def main() -> int:
    _bootstrap()
    try:
        from coppeliasimagent.cli.toolcli import main as toolcli_main
    except ModuleNotFoundError:
        _maybe_reexec_into_repo_venv()
        from coppeliasimagent.cli.toolcli import main as toolcli_main

    return toolcli_main()


if __name__ == "__main__":
    raise SystemExit(main())
