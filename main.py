"""
Path Planning Simulator - Main Entry Point

A* and RRT-based mobile robot path planning simulation system.

Usage:
    python main.py                    # Run interactive demo
    python main.py --batch            # Run batch tests and export CSV
    python main.py --algo astar       # Run A* only
    python main.py --algo rrt         # Run RRT only
    python main.py --no-show          # Save figures without displaying
"""

import argparse
import os
import sys
import time

from algorithms.astar import AStarPlanner
from algorithms.rrt import RRTPlanner
from evaluation.metrics import compute_all_metrics
from maps.grid_map import GridMap
from maps.obstacle_generator import (
    generate_boundary_obstacles,
    generate_random_obstacles,
    generate_room_obstacles,
)
from visualization.plotter import PathPlotter


PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(PROJECT_ROOT, "outputs")


def add_obstacles(grid_map, map_type, num_obstacles, seed):
    """Populate the map and return circular obstacles for continuous planners."""
    generate_boundary_obstacles(grid_map)
    if map_type == "room":
        wall_points = generate_room_obstacles(grid_map)
        radius = max(grid_map.resolution * 0.5, 0.1)
        return [(x, y, radius) for x, y in wall_points]
    return generate_random_obstacles(
        grid_map, num_obstacles=num_obstacles, seed=seed
    )


def demo_astar(grid_map, start, goal, robot_radius, show=True, save=False):
    """Run A* demo."""
    print("\n" + "=" * 50)
    print("Running A* Path Planning...")
    print(f"  Start: {start}, Goal: {goal}, Robot Radius: {robot_radius}")

    planner = AStarPlanner(grid_map, robot_radius=robot_radius)
    t0 = time.time()
    rx, ry = planner.plan(start, goal)
    elapsed = (time.time() - t0) * 1000

    path = list(zip(rx, ry))
    success = len(rx) > 0

    metrics = compute_all_metrics(
        path,
        [],
        planner.nodes_explored,
        elapsed,
        success,
    )

    print(f"  Result: {'SUCCESS' if success else 'FAILED'}")
    print(f"  Path length: {metrics['path_length']:.2f} m")
    print(f"  Nodes explored: {planner.nodes_explored}")
    print(f"  Time: {elapsed:.1f} ms")

    plotter = PathPlotter()
    save_path = os.path.join(OUTPUT_DIR, "figures", "astar_demo.png") if save else ""
    plotter.plot_astar_result(
        grid_map, rx, ry, start, goal,
        nodes_explored=planner.nodes_explored,
        save_path=save_path,
    )

    if show:
        plotter.show()
    else:
        plotter.close_all()

    return path, metrics, elapsed, planner.nodes_explored


def demo_rrt(grid_map, start, goal, obstacle_list, robot_radius, show=True, save=False):
    """Run RRT demo."""
    print("\n" + "=" * 50)
    print("Running RRT Path Planning...")
    print(f"  Start: {start}, Goal: {goal}, Robot Radius: {robot_radius}")

    bounds = (0, grid_map.width, 0, grid_map.height)
    planner = RRTPlanner(
        start=start,
        goal=goal,
        obstacle_list=obstacle_list,
        bounds=bounds,
        expand_dis=2.0,
        max_iter=1000,
        robot_radius=robot_radius,
        seed=42,
    )

    t0 = time.time()
    path = planner.plan()
    elapsed = (time.time() - t0) * 1000
    success = path is not None

    metrics = compute_all_metrics(
        path if success else [],
        obstacle_list,
        planner.nodes_explored,
        elapsed,
        success,
    )

    print(f"  Result: {'SUCCESS' if success else 'FAILED'}")
    if success:
        print(f"  Path length: {metrics['path_length']:.2f} m")
    print(f"  Nodes explored: {planner.nodes_explored}")
    print(f"  Tree size: {len(planner.node_list)}")
    print(f"  Time: {elapsed:.1f} ms")

    plotter = PathPlotter()
    save_path = os.path.join(OUTPUT_DIR, "figures", "rrt_demo.png") if save else ""
    plotter.plot_rrt_result(
        planner.node_list,
        path if success else [],
        obstacle_list,
        start,
        goal,
        bounds,
        nodes_explored=planner.nodes_explored,
        save_path=save_path,
    )

    if show:
        plotter.show()
    else:
        plotter.close_all()

    return path, metrics, elapsed, planner.nodes_explored, planner.node_list


def main():
    parser = argparse.ArgumentParser(description="Path Planning Simulator")
    parser.add_argument(
        "--algo", choices=["astar", "rrt", "both"], default="both",
        help="Algorithm to run (default: both)",
    )
    parser.add_argument(
        "--map", choices=["random", "room"], default="random",
        help="Map type (default: random)",
    )
    parser.add_argument(
        "--map-size", type=int, default=50,
        help="Map size in meters (default: 50)",
    )
    parser.add_argument(
        "--resolution", type=float, default=1.0,
        help="Grid resolution for A* (default: 1.0)",
    )
    parser.add_argument(
        "--num-obstacles", type=int, default=15,
        help="Number of random obstacles (default: 15)",
    )
    parser.add_argument(
        "--robot-radius", type=float, default=0.5,
        help="Robot radius in meters (default: 0.5)",
    )
    parser.add_argument(
        "--start-x", type=float, default=5.0, help="Start X coordinate",
    )
    parser.add_argument(
        "--start-y", type=float, default=5.0, help="Start Y coordinate",
    )
    parser.add_argument(
        "--goal-x", type=float, default=45.0, help="Goal X coordinate",
    )
    parser.add_argument(
        "--goal-y", type=float, default=45.0, help="Goal Y coordinate",
    )
    parser.add_argument(
        "--no-show", action="store_true",
        help="Do not display figures (save to files instead)",
    )
    parser.add_argument(
        "--batch", action="store_true",
        help="Run batch tests and export CSV",
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="Random seed for reproducibility (default: 42)",
    )

    args = parser.parse_args()

    if args.batch:
        from evaluation.batch_test import run_batch_tests
        run_batch_tests(os.path.join(OUTPUT_DIR, "results"))
        return

    show = not args.no_show
    save = args.no_show

    start = (args.start_x, args.start_y)
    goal = (args.goal_x, args.goal_y)
    map_size = args.map_size

    print("=" * 50)
    print("Path Planning Simulator")
    print("=" * 50)
    print(f"  Map: {args.map}, Size: {map_size}x{map_size}")
    print(f"  Obstacles: {args.num_obstacles}")
    print(f"  Start: {start}, Goal: {goal}")
    print(f"  Robot radius: {args.robot_radius}")
    print(f"  Algorithm: {args.algo}")

    # Create shared grid map for fair comparison
    grid_map = GridMap(map_size, map_size, args.resolution)
    obstacle_list = add_obstacles(grid_map, args.map, args.num_obstacles, args.seed)

    astar_result = None
    rrt_result = None
    save_individual = save and args.algo != "both"

    if args.algo in ("astar", "both"):
        # Use a copy of grid_map since A* inflates obstacles
        grid_map_astar = GridMap(map_size, map_size, args.resolution)
        _ = add_obstacles(grid_map_astar, args.map, args.num_obstacles, args.seed)
        astar_result = demo_astar(
            grid_map_astar, start, goal, args.robot_radius, show, save_individual
        )

    if args.algo in ("rrt", "both"):
        grid_map_rrt = GridMap(map_size, map_size, args.resolution)
        obstacle_list_rrt = add_obstacles(
            grid_map_rrt, args.map, args.num_obstacles, args.seed
        )
        rrt_result = demo_rrt(
            grid_map_rrt, start, goal, obstacle_list_rrt,
            args.robot_radius, show, save_individual
        )

    # Generate comparison plot when both algorithms run
    if args.algo == "both" and astar_result and rrt_result:
        astar_path, astar_metrics, astar_time, astar_nodes_explored = astar_result
        rrt_path, rrt_metrics, rrt_time, rrt_nodes_explored, rrt_tree = rrt_result

        plotter = PathPlotter()
        bounds = (0, map_size, 0, map_size)
        astar_rx, astar_ry = zip(*astar_path) if astar_path else ([], [])

        save_path = os.path.join(OUTPUT_DIR, "figures", "astar_rrt_comparison.png") if save else ""
        plotter.plot_comparison(
            grid_map=grid_map,
            astar_path_x=list(astar_rx),
            astar_path_y=list(astar_ry),
            rrt_path=rrt_path if rrt_path else [],
            obstacle_list=obstacle_list,
            start=start,
            goal=goal,
            bounds=bounds,
            rrt_tree=rrt_tree,
            astar_nodes=astar_nodes_explored,
            rrt_nodes=rrt_nodes_explored,
            astar_time_ms=astar_time,
            rrt_time_ms=rrt_time,
            save_path=save_path,
        )

        if show:
            plotter.show()
        else:
            plotter.close_all()

    print("\nDone.")


if __name__ == "__main__":
    main()
