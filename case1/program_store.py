"""Simple JSON-file-backed "program" database: a named, ordered list of
saved waypoint names, distinct from waypoint_store.py's single poses.
Same "plain JSON file, not a real database" call as waypoint_store.py.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

_STORE_PATH = Path(__file__).resolve().parent / "programs.json"


def _load() -> dict:
    if not _STORE_PATH.exists():
        return {}
    return json.loads(_STORE_PATH.read_text())


def _save(data: dict) -> None:
    _STORE_PATH.write_text(json.dumps(data, indent=2))


def save_program(name: str, waypoint_names: list[str]) -> dict:
    data = _load()
    entry = {"waypoint_names": list(waypoint_names), "saved_at": time.time()}
    data[name] = entry
    _save(data)
    return entry


def get_program(name: str) -> dict:
    data = _load()
    if name not in data:
        raise KeyError(f"No program named {name!r}. Known: {sorted(data)}")
    return data[name]


def list_programs() -> dict:
    return _load()


def delete_program(name: str) -> dict:
    data = _load()
    if name not in data:
        raise KeyError(f"No program named {name!r}. Known: {sorted(data)}")
    removed = data.pop(name)
    _save(data)
    return removed
