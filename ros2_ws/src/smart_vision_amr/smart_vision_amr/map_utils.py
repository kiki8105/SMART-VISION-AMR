"""OccupancyGrid conversion helpers shared by the ROS nodes."""

from math import cos, sin, atan2, floor

from smart_vision_amr.grid import Grid


def yaw_from_quaternion(q):
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def world_to_cell(info, x, y):
    yaw = yaw_from_quaternion(info.origin.orientation)
    dx, dy = x - info.origin.position.x, y - info.origin.position.y
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    return floor(local_x / info.resolution), floor(local_y / info.resolution)


def cell_to_world(info, cell):
    x, y = cell
    lx = (x + 0.5) * info.resolution
    ly = (y + 0.5) * info.resolution
    yaw = yaw_from_quaternion(info.origin.orientation)
    return (info.origin.position.x + cos(yaw) * lx - sin(yaw) * ly,
            info.origin.position.y + sin(yaw) * lx + cos(yaw) * ly)


def occupancy_to_grid(message, occupied_threshold=65, inflation_cells=0):
    width, height = message.info.width, message.info.height
    rows = []
    for y in range(height):
        row = []
        for x in range(width):
            value = message.data[y * width + x]
            # Unknown is conservatively blocked for global planning.
            row.append(value < 0 or value >= occupied_threshold)
        rows.append(tuple(row))
    if inflation_cells > 0:
        # Expand only from source rows. Blocked/unknown cells are already true,
        # so this avoids a large per-cell inflation set for mostly-unknown maps.
        source_rows = [tuple(x for x, blocked in enumerate(row) if blocked) for row in rows]
        for y in range(height):
            expanded = set()
            for source_y in range(max(0, y - inflation_cells), min(height, y + inflation_cells + 1)):
                for x in source_rows[source_y]:
                    expanded.update(range(max(0, x - inflation_cells), min(width, x + inflation_cells + 1)))
            if expanded:
                rows[y] = tuple(value or x in expanded for x, value in enumerate(rows[y]))
    return Grid(tuple(rows))


def inflate_cells(cells, width, height, radius):
    if radius <= 0:
        return set(cells)
    result = set()
    for x, y in cells:
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    result.add((nx, ny))
    return result


def bresenham(start, end):
    """Yield integer cells from start through end, inclusive."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx - dy
    while True:
        yield x0, y0
        if (x0, y0) == (x1, y1):
            break
        twice = 2 * error
        if twice > -dy:
            error -= dy
            x0 += sx
        if twice < dx:
            error += dx
            y0 += sy
