"""
Path evaluation metrics for benchmarking planners.
"""

import math
from typing import List, Tuple


def path_length(path: List[Tuple[float, float]]) -> float:
    """Compute total Euclidean length of a path."""
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        total += math.hypot(dx, dy)
    return total


def path_smoothness(path: List[Tuple[float, float]]) -> float:
    """
    Compute path smoothness as the average absolute turning angle.
    Lower values = smoother path. Returns value in radians.
    """
    if len(path) < 3:
        return 0.0
    angles = []
    for i in range(1, len(path) - 1):
        dx1 = path[i][0] - path[i - 1][0]
        dy1 = path[i][1] - path[i - 1][1]
        dx2 = path[i + 1][0] - path[i][0]
        dy2 = path[i + 1][1] - path[i][1]
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        diff = abs(angle2 - angle1)
        diff = min(diff, 2 * math.pi - diff)
        angles.append(diff)
    return sum(angles) / len(angles)


def clearance(
    path: List[Tuple[float, float]],
    obstacle_list: List[Tuple[float, float, float]],
) -> float:
    """Compute minimum distance from path to any obstacle. Higher = safer."""
    if not path or not obstacle_list:
        return float("inf")
    min_dist = float("inf")
    for px, py in path:
        for ox, oy, size in obstacle_list:
            d = math.hypot(ox - px, oy - py) - size
            if d < min_dist:
                min_dist = d
    return max(0.0, min_dist)


def success_rate(success_count: int, total_count: int) -> float:
    """Compute planning success rate."""
    if total_count == 0:
        return 0.0
    return success_count / total_count


def compute_all_metrics(
    path: List[Tuple[float, float]],
    obstacle_list: List[Tuple[float, float, float]],
    nodes_explored: int,
    planning_time_ms: float,
    success: bool,
) -> dict:
    """Compute all metrics for a planning run."""
    return {
        "success": success,
        "path_length": path_length(path) if success else float("inf"),
        "smoothness_rad": path_smoothness(path) if success else float("inf"),
        "clearance_m": clearance(path, obstacle_list) if success else 0.0,
        "nodes_explored": nodes_explored,
        "planning_time_ms": planning_time_ms,
        "path_points": len(path),
    }
