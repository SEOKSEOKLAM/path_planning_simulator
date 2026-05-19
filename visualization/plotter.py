"""
Matplotlib-based visualization for path planning results.
"""

import os
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from maps.grid_map import GridMap


class PathPlotter:
    """Visualize grid maps, obstacles, paths, and algorithm search trees."""

    def __init__(self, figsize: Tuple[int, int] = (10, 8)):
        self.figsize = figsize

    def plot_grid_map(
        self,
        grid_map: GridMap,
        title: str = "Grid Map",
    ):
        """Plot the occupancy grid map."""
        fig, ax = plt.subplots(figsize=self.figsize)
        data = np.array(grid_map.grid, dtype=int).T
        ax.imshow(
            data,
            origin="lower",
            cmap="gray_r",
            extent=[
                grid_map.origin_x,
                grid_map.origin_x + grid_map.width,
                grid_map.origin_y,
                grid_map.origin_y + grid_map.height,
            ],
        )
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal")
        return fig, ax

    def plot_astar_result(
        self,
        grid_map: GridMap,
        path_x: List[float],
        path_y: List[float],
        start: Tuple[float, float],
        goal: Tuple[float, float],
        nodes_explored: int = 0,
        title: str = "A* Path Planning",
        save_path: str = "",
    ):
        """Plot A* planning result."""
        fig, ax = self.plot_grid_map(grid_map, title)

        # Obstacles
        obs_points = grid_map.get_obstacle_points()
        if obs_points:
            ox, oy = zip(*obs_points)
            ax.scatter(ox, oy, s=1, c="black", alpha=0.5)

        # Path
        if path_x and path_y:
            ax.plot(path_x, path_y, "-r", linewidth=2, label="Path")

        # Start and goal
        ax.plot(start[0], start[1], "go", markersize=10, label="Start")
        ax.plot(goal[0], goal[1], "bo", markersize=10, label="Goal")

        path_len = sum(
            np.hypot(path_x[i] - path_x[i - 1], path_y[i] - path_y[i - 1])
            for i in range(1, len(path_x))
        ) if len(path_x) > 1 else 0.0
        ax.legend()
        ax.set_title(
            f"{title}\nNodes explored: {nodes_explored} | Path length: {path_len:.1f}m"
        )

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            fig.savefig(save_path, dpi=150, bbox_inches="tight")

        return fig, ax

    def plot_rrt_result(
        self,
        node_list: List,
        path: List[Tuple[float, float]],
        obstacle_list: List[Tuple[float, float, float]],
        start: Tuple[float, float],
        goal: Tuple[float, float],
        bounds: Tuple[float, float, float, float],
        nodes_explored: int = 0,
        title: str = "RRT Path Planning",
        save_path: str = "",
    ):
        """Plot RRT planning result with search tree."""
        fig, ax = plt.subplots(figsize=self.figsize)

        x_min, x_max, y_min, y_max = bounds

        # Obstacles
        for ox, oy, size in obstacle_list:
            circle = plt.Circle((ox, oy), size, color="black", alpha=0.5)
            ax.add_patch(circle)

        # RRT tree
        for node in node_list:
            if node.parent is not None:
                ax.plot(
                    [node.x, node.parent.x],
                    [node.y, node.parent.y],
                    "-g",
                    alpha=0.3,
                    linewidth=0.5,
                )

        # Path
        if path:
            px, py = zip(*path)
            ax.plot(px, py, "-r", linewidth=2, label="Path")

        # Start and goal
        ax.plot(start[0], start[1], "go", markersize=10, label="Start")
        ax.plot(goal[0], goal[1], "bo", markersize=10, label="Goal")

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title(f"{title}\nNodes explored: {nodes_explored} | Tree size: {len(node_list)}")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend()

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            fig.savefig(save_path, dpi=150, bbox_inches="tight")

        return fig, ax

    def plot_comparison(
        self,
        grid_map: GridMap,
        astar_path_x: List[float],
        astar_path_y: List[float],
        rrt_path: List[Tuple[float, float]],
        obstacle_list: List[Tuple[float, float, float]],
        start: Tuple[float, float],
        goal: Tuple[float, float],
        bounds: Tuple[float, float, float, float],
        rrt_tree: List,
        astar_nodes: int = 0,
        rrt_nodes: int = 0,
        astar_time_ms: float = 0.0,
        rrt_time_ms: float = 0.0,
        save_path: str = "",
    ):
        """Side-by-side comparison of A* and RRT on the same map."""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

        x_min, x_max, y_min, y_max = bounds

        # --- Left: A* ---
        data = np.array(grid_map.grid, dtype=int).T
        ax1.imshow(
            data, origin="lower", cmap="gray_r",
            extent=[grid_map.origin_x, grid_map.origin_x + grid_map.width,
                    grid_map.origin_y, grid_map.origin_y + grid_map.height],
        )
        if astar_path_x and astar_path_y:
            ax1.plot(astar_path_x, astar_path_y, "-r", linewidth=2, label="A* Path")
        ax1.plot(start[0], start[1], "go", markersize=10, label="Start")
        ax1.plot(goal[0], goal[1], "bo", markersize=10, label="Goal")
        astar_len = sum(
            ((astar_path_x[i]-astar_path_x[i-1])**2 + (astar_path_y[i]-astar_path_y[i-1])**2)**0.5
            for i in range(1, len(astar_path_x))
        ) if len(astar_path_x) > 1 else 0
        ax1.set_title(f"A* Path Planning\nPath: {astar_len:.1f}m | Nodes: {astar_nodes} | Time: {astar_time_ms:.1f}ms")
        ax1.set_xlabel("X [m]"); ax1.set_ylabel("Y [m]")
        ax1.set_aspect("equal"); ax1.grid(True, alpha=0.3); ax1.legend()

        # --- Right: RRT ---
        for ox, oy, size in obstacle_list:
            circle = plt.Circle((ox, oy), size, color="black", alpha=0.5)
            ax2.add_patch(circle)
        for node in rrt_tree:
            if node.parent is not None:
                ax2.plot([node.x, node.parent.x], [node.y, node.parent.y],
                         "-g", alpha=0.3, linewidth=0.5)
        if rrt_path:
            px, py = zip(*rrt_path)
            rrt_len = sum(
                ((px[i]-px[i-1])**2 + (py[i]-py[i-1])**2)**0.5
                for i in range(1, len(px))
            )
            ax2.plot(px, py, "-r", linewidth=2, label=f"RRT Path ({rrt_len:.1f}m)")
        ax2.plot(start[0], start[1], "go", markersize=10, label="Start")
        ax2.plot(goal[0], goal[1], "bo", markersize=10, label="Goal")
        ax2.set_xlim(x_min, x_max); ax2.set_ylim(y_min, y_max)
        ax2.set_title(f"RRT Path Planning\nTree: {len(rrt_tree)} | Nodes explored: {rrt_nodes} | Time: {rrt_time_ms:.1f}ms")
        ax2.set_xlabel("X [m]"); ax2.set_ylabel("Y [m]")
        ax2.set_aspect("equal"); ax2.grid(True, alpha=0.3); ax2.legend()

        fig.suptitle("A* vs RRT Comparison on Same Map", fontsize=14, fontweight="bold")
        plt.tight_layout()

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            fig.savefig(save_path, dpi=150, bbox_inches="tight")

        return fig, (ax1, ax2)

    @staticmethod
    def plot_batch_summary(csv_path: str, save_path: str = ""):
        """Generate batch metrics summary chart from CSV results."""
        import csv

        algorithms = []
        path_lengths = []
        times = []
        successes = []
        nodes_list = []
        labels = []

        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                algorithms.append(row["algorithm"])
                labels.append(f"{row['algorithm']}-T{row['test_id']}")
                if row["success"] == "True":
                    path_lengths.append(float(row["path_length"]))
                else:
                    path_lengths.append(0)
                times.append(float(row.get("runtime_ms", row.get("planning_time_ms", 0))))
                successes.append(1 if row["success"] == "True" else 0)
                nodes_list.append(int(row["nodes_explored"]))

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # Path length comparison
        colors = ["#2196F3" if a == "A*" else "#FF9800" for a in algorithms]
        ax1 = axes[0, 0]
        bars = ax1.bar(range(len(labels)), path_lengths, color=colors)
        ax1.set_xticks(range(len(labels)))
        ax1.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
        ax1.set_ylabel("Path Length [m]")
        ax1.set_title("Path Length Comparison (0 = failed)")
        ax1.legend([plt.Rectangle((0,0),1,1,color="#2196F3"), plt.Rectangle((0,0),1,1,color="#FF9800")],
                   ["A*", "RRT"])
        for bar, val in zip(bars, path_lengths):
            if val > 0:
                ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                         f"{val:.1f}", ha="center", va="bottom", fontsize=7)

        # Runtime comparison
        ax2 = axes[0, 1]
        bars2 = ax2.bar(range(len(labels)), times, color=colors)
        ax2.set_xticks(range(len(labels)))
        ax2.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
        ax2.set_ylabel("Time [ms]")
        ax2.set_title("Planning Time Comparison")
        ax2.legend([plt.Rectangle((0,0),1,1,color="#2196F3"), plt.Rectangle((0,0),1,1,color="#FF9800")],
                   ["A*", "RRT"])
        for bar, val in zip(bars2, times):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                     f"{val:.1f}", ha="center", va="bottom", fontsize=7)

        # Success rate
        ax3 = axes[1, 0]
        astar_success = sum(1 for a, s in zip(algorithms, successes) if a == "A*" and s == 1)
        rrt_success = sum(1 for a, s in zip(algorithms, successes) if a == "RRT" and s == 1)
        astar_total = sum(1 for a in algorithms if a == "A*")
        rrt_total = sum(1 for a in algorithms if a == "RRT")
        ax3.bar(["A*", "RRT"],
                [astar_success/astar_total*100 if astar_total else 0,
                 rrt_success/rrt_total*100 if rrt_total else 0],
                color=["#2196F3", "#FF9800"])
        ax3.set_ylabel("Success Rate [%]")
        ax3.set_title("Planning Success Rate")
        ax3.set_ylim(0, 110)

        # Nodes explored
        ax4 = axes[1, 1]
        bars4 = ax4.bar(range(len(labels)), nodes_list, color=colors)
        ax4.set_xticks(range(len(labels)))
        ax4.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
        ax4.set_ylabel("Nodes Explored")
        ax4.set_title("Search Nodes Explored")
        ax4.legend([plt.Rectangle((0,0),1,1,color="#2196F3"), plt.Rectangle((0,0),1,1,color="#FF9800")],
                   ["A*", "RRT"])

        fig.suptitle("Batch Benchmark Summary", fontsize=14, fontweight="bold")
        plt.tight_layout()

        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            fig.savefig(save_path, dpi=150, bbox_inches="tight")

        return fig, axes

    @staticmethod
    def show():
        plt.show()

    @staticmethod
    def close_all():
        plt.close("all")
