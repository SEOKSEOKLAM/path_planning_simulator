"""
RRT (Rapidly-exploring Random Tree) path planning.

Implements the classic RRT algorithm in continuous space:
- Random sampling with goal bias
- Nearest-neighbor selection
- Step-size-limited extension
- Circular-obstacle collision detection
"""

import math
import random
from typing import List, Optional, Tuple

from algorithms.collision import check_collision_continuous, is_outside_bounds


class RRTNode:
    """A node in the RRT tree."""

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None


class RRTPlanner:
    """RRT path planner in continuous space."""

    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        obstacle_list: List[Tuple[float, float, float]],
        bounds: Tuple[float, float, float, float],
        expand_dis: float = 3.0,
        goal_sample_rate: int = 5,
        max_iter: int = 500,
        robot_radius: float = 0.5,
        seed: int = 42,
    ):
        self.start = RRTNode(*start)
        self.goal = RRTNode(*goal)
        self.obstacle_list = obstacle_list
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot_radius = robot_radius
        self.nodes_explored = 0
        random.seed(seed)

        self.node_list: List[RRTNode] = []

    def plan(self) -> Optional[List[Tuple[float, float]]]:
        """Run RRT planning. Returns path or None."""
        self.node_list = [self.start]
        self.nodes_explored = 0

        for _ in range(self.max_iter):
            rnd = self._get_random_node()
            nearest = self._get_nearest(rnd)
            new_node = self._steer(nearest, rnd)

            if self._is_collision_free(new_node):
                self.node_list.append(new_node)
                self.nodes_explored += 1

            if self._distance(self.node_list[-1], self.goal) <= self.expand_dis:
                final = self._steer(self.node_list[-1], self.goal)
                if self._is_collision_free(final):
                    return self._extract_path(final)

        # Try to reach goal from closest node
        closest = min(self.node_list, key=lambda n: self._distance(n, self.goal))
        final = self._steer(closest, self.goal)
        if self._is_collision_free(final):
            return self._extract_path(final)

        return None

    def _get_random_node(self) -> RRTNode:
        if random.randint(0, 100) > self.goal_sample_rate:
            return RRTNode(
                random.uniform(self.x_min, self.x_max),
                random.uniform(self.y_min, self.y_max),
            )
        return RRTNode(self.goal.x, self.goal.y)

    def _get_nearest(self, target: RRTNode) -> RRTNode:
        return min(
            self.node_list, key=lambda n: self._distance(n, target)
        )

    def _steer(self, from_node: RRTNode, to_node: RRTNode) -> RRTNode:
        dist, theta = self._distance_and_angle(from_node, to_node)
        step = min(self.expand_dis, dist)
        new_node = RRTNode(
            from_node.x + step * math.cos(theta),
            from_node.y + step * math.sin(theta),
        )
        new_node.parent = from_node
        return new_node

    def _is_collision_free(self, node: RRTNode) -> bool:
        if is_outside_bounds(
            node.x, node.y, self.x_min, self.x_max, self.y_min, self.y_max
        ):
            return False
        if node.parent is None:
            return True
        # Interpolate and check intermediate points
        dist, theta = self._distance_and_angle(node.parent, node)
        steps = max(1, int(dist / 0.1))
        for i in range(steps + 1):
            t = i / steps
            px = node.parent.x + t * (node.x - node.parent.x)
            py = node.parent.y + t * (node.y - node.parent.y)
            if check_collision_continuous(px, py, self.obstacle_list, self.robot_radius):
                return False
        return True

    def _extract_path(self, node: RRTNode) -> List[Tuple[float, float]]:
        path = [(node.x, node.y)]
        while node.parent is not None:
            node = node.parent
            path.append((node.x, node.y))
        path.reverse()
        return path

    @staticmethod
    def _distance(a: RRTNode, b: RRTNode) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def _distance_and_angle(a: RRTNode, b: RRTNode) -> Tuple[float, float]:
        dx = b.x - a.x
        dy = b.y - a.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
