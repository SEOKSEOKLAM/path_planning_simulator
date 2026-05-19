"""
Obstacle generators for creating test scenarios.
"""

import math
import random
from typing import List, Tuple

from .grid_map import GridMap


def generate_boundary_obstacles(
    grid_map: GridMap,
) -> List[Tuple[float, float]]:
    """Generate boundary walls around the map edges."""
    ox, oy = [], []
    margin = grid_map.resolution

    x_min = grid_map.origin_x - margin
    x_max = grid_map.origin_x + grid_map.width + margin
    y_min = grid_map.origin_y - margin
    y_max = grid_map.origin_y + grid_map.height + margin

    for x in range(int(x_min), int(x_max) + 1):
        ox.append(float(x))
        oy.append(y_min)
    for x in range(int(x_min), int(x_max) + 1):
        ox.append(float(x))
        oy.append(y_max)
    for y in range(int(y_min), int(y_max) + 1):
        ox.append(x_min)
        oy.append(float(y))
    for y in range(int(y_min), int(y_max) + 1):
        ox.append(x_max)
        oy.append(float(y))

    for x, y in zip(ox, oy):
        grid_map.set_obstacle(x, y)

    return list(zip(ox, oy))


def generate_random_obstacles(
    grid_map: GridMap,
    num_obstacles: int = 20,
    min_size: float = 1.0,
    max_size: float = 3.0,
    seed: int = 42,
) -> List[Tuple[float, float, float]]:
    """Generate random circular obstacles. Returns [(x, y, radius), ...]."""
    random.seed(seed)
    obstacles = []
    for _ in range(num_obstacles):
        radius = random.uniform(min_size, max_size)
        x = random.uniform(
            grid_map.origin_x + radius,
            grid_map.origin_x + grid_map.width - radius,
        )
        y = random.uniform(
            grid_map.origin_y + radius,
            grid_map.origin_y + grid_map.height - radius,
        )
        obstacles.append((x, y, radius))
        _mark_circular_obstacle(grid_map, x, y, radius)
    return obstacles


def generate_room_obstacles(
    grid_map: GridMap,
) -> List[Tuple[float, float]]:
    """Generate wall-like obstacles typical in indoor navigation."""
    ox, oy = [], []
    walls = [
        (15, 0, 15, 15),
        (15, 25, 15, 45),
        (35, 15, 35, 35),
        (45, 0, 45, 25),
        (45, 35, 45, 45),
    ]
    for x1, y1, x2, y2 in walls:
        if x1 == x2:
            for y in range(min(y1, y2), max(y1, y2) + 1):
                ox.append(float(x1))
                oy.append(float(y))
        else:
            for x in range(min(x1, x2), max(x1, x2) + 1):
                ox.append(float(x))
                oy.append(float(y1))

    for x, y in zip(ox, oy):
        grid_map.set_obstacle(x, y)

    return list(zip(ox, oy))


def _mark_circular_obstacle(grid_map: GridMap, cx: float, cy: float, radius: float):
    """Fill circular obstacle into the grid map."""
    steps = 360
    for i in range(steps):
        angle = 2 * math.pi * i / steps
        for r in range(int(radius / grid_map.resolution) + 1):
            r_step = r * grid_map.resolution
            x = cx + r_step * math.cos(angle)
            y = cy + r_step * math.sin(angle)
            grid_map.set_obstacle(x, y)
