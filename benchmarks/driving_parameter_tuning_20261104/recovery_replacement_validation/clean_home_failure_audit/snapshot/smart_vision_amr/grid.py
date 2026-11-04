"""Small occupancy-grid model used before connecting ROS 2 messages."""

from __future__ import annotations

from dataclasses import dataclass

Cell = tuple[int, int]


@dataclass(frozen=True)
class Grid:
    """Immutable rectangular grid; ``True`` cells are static obstacles."""

    rows: tuple[tuple[bool, ...], ...]

    @classmethod
    def from_ascii(cls, text: str) -> "Grid":
        lines = [line.strip() for line in text.strip().splitlines() if line.strip()]
        if not lines or not lines[0]:
            raise ValueError("grid must contain at least one cell")
        width = len(lines[0])
        if any(len(line) != width for line in lines):
            raise ValueError("all grid rows must have the same width")
        if any(char not in ".#" for line in lines for char in line):
            raise ValueError("grid supports only '.' (free) and '#' (occupied)")
        return cls(tuple(tuple(char == "#" for char in line) for line in lines))

    @property
    def height(self) -> int:
        return len(self.rows)

    @property
    def width(self) -> int:
        return len(self.rows[0])

    def contains(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_static_obstacle(self, cell: Cell) -> bool:
        x, y = cell
        return not self.contains(cell) or self.rows[y][x]

    def render(self, path: list[Cell] | None = None, extra: set[Cell] | None = None) -> str:
        path_cells = set(path or [])
        extra_cells = extra or set()
        lines: list[str] = []
        for y, row in enumerate(self.rows):
            chars = []
            for x, blocked in enumerate(row):
                cell = (x, y)
                chars.append("#" if blocked else "*" if cell in path_cells else "X" if cell in extra_cells else ".")
            lines.append("".join(chars))
        return "\n".join(lines)
