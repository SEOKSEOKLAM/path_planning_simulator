"""
Grid map representation for path planning.

Supports both grid-based (A*) and continuous-space (RRT) planners
by providing obstacle maps in both discrete and continuous forms.
"""

import math
from typing import List, Tuple


class GridMap:
    """2D occupancy grid map for path planning."""

    def __init__(
        self,
        width: float,
        height: float,
        resolution: float = 1.0,
        origin: Tuple[float, float] = (0.0, 0.0),
    ):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x, self.origin_y = origin

        self.grid_width = int(width / resolution) + 1
        self.grid_height = int(height / resolution) + 1
        self.grid = [[False] * self.grid_height for _ in range(self.grid_width)]

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = gx * self.resolution + self.origin_x
        wy = gy * self.resolution + self.origin_y
        return wx, wy

    def set_obstacle(self, wx: float, wy: float):
        gx, gy = self.world_to_grid(wx, wy)
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            self.grid[gx][gy] = True

    def is_obstacle(self, gx: int, gy: int) -> bool:
        if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
            return self.grid[gx][gy]
        return True

    def is_free(self, gx: int, gy: int) -> bool:
        return not self.is_obstacle(gx, gy)

    def inflate_obstacles(self, radius: float):
        """Inflate obstacles by robot radius (in grid cells)."""
        cell_radius = int(math.ceil(radius / self.resolution))
        if cell_radius <= 0:
            return
        new_grid = [row[:] for row in self.grid]
        for gx in range(self.grid_width):
            for gy in range(self.grid_height):
                if self.grid[gx][gy]:
                    for dx in range(-cell_radius, cell_radius + 1):
                        for dy in range(-cell_radius, cell_radius + 1):
                            nx, ny = gx + dx, gy + dy
                            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                                if dx * dx + dy * dy <= cell_radius * cell_radius:
                                    new_grid[nx][ny] = True
        self.grid = new_grid

    def get_obstacle_points(self) -> List[Tuple[float, float]]:
        points = []
        for gx in range(self.grid_width):
            for gy in range(self.grid_height):
                if self.grid[gx][gy]:
                    wx, wy = self.grid_to_world(gx, gy)
                    points.append((wx, wy))
        return points
