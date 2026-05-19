"""
Collision detection for both grid-based and continuous-space planning.
"""

import math
from typing import List, Tuple

from maps.grid_map import GridMap


def check_collision_grid(gx: int, gy: int, grid_map: GridMap) -> bool:
    """Check if a grid cell is occupied by an obstacle."""
    return grid_map.is_obstacle(gx, gy)


def check_collision_continuous(
    x: float,
    y: float,
    obstacle_list: List[Tuple[float, float, float]],
    robot_radius: float = 0.0,
) -> bool:
    """Check if a continuous point (x,y) collides with circular obstacles."""
    for ox, oy, size in obstacle_list:
        d = math.hypot(ox - x, oy - y)
        if d <= size + robot_radius:
            return True
    return False


def check_collision_path(
    path_x: List[float],
    path_y: List[float],
    obstacle_list: List[Tuple[float, float, float]],
    robot_radius: float = 0.0,
) -> bool:
    """Check if any point along a path segment collides."""
    for x, y in zip(path_x, path_y):
        if check_collision_continuous(x, y, obstacle_list, robot_radius):
            return True
    return False


def is_outside_bounds(
    x: float,
    y: float,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> bool:
    """Check if (x,y) is outside the allowed area."""
    return x < x_min or x > x_max or y < y_min or y > y_max
