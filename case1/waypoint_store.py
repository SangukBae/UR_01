"""Simple JSON-file-backed waypoint database.

Meeting design: "we have a database that saves the waypoints" -- kept as a
plain JSON file rather than a real database engine. A handful of named
poses doesn't need more than that (same "don't build more than needed"
call the team already made about queue management).
"""
from __future__ import annotations

import json
import time
from pathlib import Path

_STORE_PATH = Path(__file__).resolve().parent / "waypoints.json"


def _load() -> dict:
    if not _STORE_PATH.exists():
        return {}
    return json.loads(_STORE_PATH.read_text())


def _save(data: dict) -> None:
    _STORE_PATH.write_text(json.dumps(data, indent=2))


def save_waypoint(name: str, q_rad: list[float], tcp_pose: list[float]) -> dict:
    data = _load()
    entry = {"q_rad": q_rad, "tcp_pose": tcp_pose, "saved_at": time.time()}
    data[name] = entry
    _save(data)
    return entry


def get_waypoint(name: str) -> dict:
    data = _load()
    if name not in data:
        raise KeyError(f"No waypoint named {name!r}. Known: {sorted(data)}")
    return data[name]


def list_waypoints() -> dict:
    return _load()


def delete_waypoint(name: str) -> dict:
    data = _load()
    if name not in data:
        raise KeyError(f"No waypoint named {name!r}. Known: {sorted(data)}")
    removed = data.pop(name)
    _save(data)
    return removed
