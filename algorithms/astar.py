"""
A* path planning on a grid map.

Implements the classic A* search algorithm with:
- 8-directional motion model
- Euclidean distance heuristic
- Configurable grid resolution and robot radius
"""

import math
from typing import List, Tuple

from maps.grid_map import GridMap


class AStarPlanner:
    """A* grid-based path planner."""

    def __init__(self, grid_map: GridMap, robot_radius: float = 0.5):
        self.grid_map = grid_map
        self.robot_radius = robot_radius
        self.motions = self._build_motion_model()
        self.nodes_explored = 0

    @staticmethod
    def _build_motion_model() -> List[Tuple[int, int, float]]:
        """8-directional motion model with costs. (dx, dy, cost)."""
        sqrt2 = math.sqrt(2)
        return [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (-1, -1, sqrt2),
            (-1, 1, sqrt2),
            (1, -1, sqrt2),
            (1, 1, sqrt2),
        ]

    def plan(
        self, start: Tuple[float, float], goal: Tuple[float, float]
    ) -> Tuple[List[float], List[float]]:
        """
        Run A* search.

        Returns:
            (rx, ry): path x and y coordinates, or empty lists if no path found.
        """
        sx, sy = self.grid_map.world_to_grid(*start)
        gx, gy = self.grid_map.world_to_grid(*goal)
        self.nodes_explored = 0

        # Pre-inflate obstacles for robot radius
        self.grid_map.inflate_obstacles(self.robot_radius)

        start_key = self._key(sx, sy)
        goal_key = self._key(gx, gy)

        open_set = {start_key: (sx, sy, 0.0, -1)}
        closed_set = {}

        while open_set:
            current_key = min(
                open_set,
                key=lambda k: open_set[k][2]
                + self._heuristic(open_set[k][0], open_set[k][1], gx, gy),
            )
            cx, cy, cost, parent = open_set.pop(current_key)
            closed_set[current_key] = (cx, cy, cost, parent)
            self.nodes_explored += 1

            if cx == gx and cy == gy:
                return self._reconstruct_path(closed_set, current_key)

            for dx, dy, step_cost in self.motions:
                nx, ny = cx + dx, cy + dy
                n_key = self._key(nx, ny)

                if n_key in closed_set:
                    continue
                if self.grid_map.is_obstacle(nx, ny):
                    continue

                new_cost = cost + step_cost
                if n_key not in open_set or new_cost < open_set[n_key][2]:
                    open_set[n_key] = (nx, ny, new_cost, current_key)

        return [], []

    def _reconstruct_path(self, closed_set, goal_key):
        """Backtrack from goal to start."""
        rx, ry = [], []
        key = goal_key
        while key != -1:
            cx, cy, _, parent = closed_set[key]
            wx, wy = self.grid_map.grid_to_world(cx, cy)
            rx.append(wx)
            ry.append(wy)
            key = parent
        rx.reverse()
        ry.reverse()
        return rx, ry

    @staticmethod
    def _heuristic(x1: int, y1: int, x2: int, y2: int) -> float:
        return math.hypot(x1 - x2, y1 - y2)

    def _key(self, x: int, y: int) -> int:
        return y * self.grid_map.grid_width + x
