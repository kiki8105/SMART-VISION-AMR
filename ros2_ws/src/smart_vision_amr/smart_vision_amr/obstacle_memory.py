"""Persistent, observation-count-based memory for changed obstacle cells."""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

from .grid import Cell, Grid

CellState = Literal["candidate", "confirmed"]


@dataclass
class CellMemory:
    occupied_observations: int = 0
    free_observations: int = 0
    state: CellState = "candidate"
    first_occupied_at: float | None = None
    last_occupied_at: float | None = None
    first_free_at: float | None = None
    last_free_at: float | None = None


class ObstacleMemory:
    """Tracks repeated occupied/free evidence without modifying the base map."""

    FORMAT_VERSION = 1

    def __init__(self, width: int, height: int, confirm_observations: int = 3, clear_observations: int = 3,
                 map_signature: str | None = None, confirm_min_duration_sec: float = 0.0,
                 clear_min_duration_sec: float = 0.0, min_observation_interval_sec: float = 0.0):
        if width <= 0 or height <= 0:
            raise ValueError("memory dimensions must be positive")
        if confirm_observations < 1 or clear_observations < 1:
            raise ValueError("observation thresholds must be at least 1")
        if min(confirm_min_duration_sec, clear_min_duration_sec, min_observation_interval_sec) < 0:
            raise ValueError("observation durations and intervals cannot be negative")
        self.width = width
        self.height = height
        self.confirm_observations = confirm_observations
        self.clear_observations = clear_observations
        self.map_signature = map_signature
        self.confirm_min_duration_sec = confirm_min_duration_sec
        self.clear_min_duration_sec = clear_min_duration_sec
        self.min_observation_interval_sec = min_observation_interval_sec
        self.cells: dict[Cell, CellMemory] = {}

    def observe_occupied(self, cell: Cell, observed_at: float | None = None) -> bool:
        """Record occupied evidence; return True when the confirmed set changes."""
        self._validate_cell(cell)
        memory = self.cells.setdefault(cell, CellMemory())
        was_confirmed = memory.state == "confirmed"
        if observed_at is not None and memory.last_occupied_at is not None:
            if observed_at - memory.last_occupied_at < self.min_observation_interval_sec:
                return False
        if memory.occupied_observations == 0:
            memory.first_occupied_at = observed_at
        memory.occupied_observations += 1
        memory.last_occupied_at = observed_at
        memory.free_observations = 0
        memory.first_free_at = memory.last_free_at = None
        duration = (observed_at - memory.first_occupied_at
                    if observed_at is not None and memory.first_occupied_at is not None else 0.0)
        if (memory.occupied_observations >= self.confirm_observations
                and duration >= self.confirm_min_duration_sec):
            memory.state = "confirmed"
        return not was_confirmed and memory.state == "confirmed"

    def observe_free(self, cell: Cell, observed_at: float | None = None) -> bool:
        """Record visible-free evidence; return True when a confirmed cell is cleared."""
        self._validate_cell(cell)
        memory = self.cells.get(cell)
        if memory is None:
            return False
        was_confirmed = memory.state == "confirmed"
        if observed_at is not None and memory.last_free_at is not None:
            if observed_at - memory.last_free_at < self.min_observation_interval_sec:
                return False
        if memory.free_observations == 0:
            memory.first_free_at = observed_at
        memory.free_observations += 1
        memory.last_free_at = observed_at
        memory.occupied_observations = 0
        memory.first_occupied_at = memory.last_occupied_at = None
        duration = (observed_at - memory.first_free_at
                    if observed_at is not None and memory.first_free_at is not None else 0.0)
        if (memory.free_observations >= self.clear_observations
                and duration >= self.clear_min_duration_sec):
            del self.cells[cell]
            return was_confirmed
        return False

    @property
    def confirmed_cells(self) -> set[Cell]:
        return {cell for cell, memory in self.cells.items() if memory.state == "confirmed"}

    def save(self, path: str | Path) -> None:
        """Atomically persist evidence and thresholds as JSON."""
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "format_version": self.FORMAT_VERSION,
            "width": self.width,
            "height": self.height,
            "confirm_observations": self.confirm_observations,
            "clear_observations": self.clear_observations,
            "map_signature": self.map_signature,
            "confirm_min_duration_sec": self.confirm_min_duration_sec,
            "clear_min_duration_sec": self.clear_min_duration_sec,
            "min_observation_interval_sec": self.min_observation_interval_sec,
            "cells": [
                {"x": cell[0], "y": cell[1], **asdict(memory)}
                for cell, memory in sorted(self.cells.items())
            ],
        }
        fd, temporary = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent, text=True)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, target)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def load(cls, path: str | Path, expected_grid: Grid,
             expected_map_signature: str | None = None) -> "ObstacleMemory":
        """Load memory only when its grid dimensions match the supplied base map."""
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("format_version") != cls.FORMAT_VERSION:
            raise ValueError("unsupported obstacle-memory format version")
        if payload.get("width") != expected_grid.width or payload.get("height") != expected_grid.height:
            raise ValueError("saved memory dimensions do not match the base map")
        if expected_map_signature is not None and payload.get("map_signature") != expected_map_signature:
            raise ValueError("saved memory signature does not match the base map")
        memory = cls(
            expected_grid.width,
            expected_grid.height,
            payload["confirm_observations"],
            payload["clear_observations"],
            payload.get("map_signature"),
            payload.get("confirm_min_duration_sec", 0.0),
            payload.get("clear_min_duration_sec", 0.0),
            payload.get("min_observation_interval_sec", 0.0),
        )
        for item in payload.get("cells", []):
            cell = (int(item["x"]), int(item["y"]))
            memory._validate_cell(cell)
            state = item["state"]
            if state not in ("candidate", "confirmed"):
                raise ValueError(f"invalid saved cell state: {state}")
            memory.cells[cell] = CellMemory(
                occupied_observations=int(item["occupied_observations"]),
                free_observations=int(item["free_observations"]),
                state=state,
                first_occupied_at=item.get("first_occupied_at"),
                last_occupied_at=item.get("last_occupied_at"),
                first_free_at=item.get("first_free_at"),
                last_free_at=item.get("last_free_at"),
            )
        return memory

    def _validate_cell(self, cell: Cell) -> None:
        x, y = cell
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise ValueError(f"cell {cell} is outside the remembered map")
