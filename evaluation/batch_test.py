"""
Batch testing framework for comparing A* and RRT planners.

Runs multiple tests with varying parameters and exports results to CSV.
"""

import csv
import os
import time
from typing import List, Tuple

from algorithms.astar import AStarPlanner
from algorithms.rrt import RRTPlanner
from evaluation.metrics import compute_all_metrics
from maps.grid_map import GridMap
from maps.obstacle_generator import (
    generate_boundary_obstacles,
    generate_random_obstacles,
)


def create_test_map(width: float, height: float, resolution: float) -> GridMap:
    """Create a grid map with boundary walls."""
    gm = GridMap(width, height, resolution)
    generate_boundary_obstacles(gm)
    return gm


def run_batch_tests(output_dir: str = "outputs/results"):
    """Run a batch of tests comparing A* and RRT."""
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, "batch_results.csv")

    # Test configurations
    configs = [
        {
            "map_size": 50,
            "resolution": 1.0,
            "num_obstacles": 15,
            "robot_radius": 0.5,
            "start": (5.0, 5.0),
            "goal": (45.0, 45.0),
            "seed": 42,
        },
        {
            "map_size": 50,
            "resolution": 1.0,
            "num_obstacles": 25,
            "robot_radius": 0.5,
            "start": (5.0, 5.0),
            "goal": (45.0, 45.0),
            "seed": 123,
        },
        {
            "map_size": 80,
            "resolution": 2.0,
            "num_obstacles": 30,
            "robot_radius": 1.0,
            "start": (5.0, 5.0),
            "goal": (75.0, 75.0),
            "seed": 456,
        },
    ]

    results = []
    for i, cfg in enumerate(configs):
        print(f"\n{'='*50}")
        print(f"Test {i+1}: map={cfg['map_size']}x{cfg['map_size']}, obstacles={cfg['num_obstacles']}")

        # Create map
        gm = GridMap(cfg["map_size"], cfg["map_size"], cfg["resolution"])
        generate_boundary_obstacles(gm)
        obs_list = generate_random_obstacles(
            gm, num_obstacles=cfg["num_obstacles"], seed=cfg["seed"]
        )

        # Test A*
        print("  Running A*...")
        astar_planner = AStarPlanner(gm, robot_radius=cfg["robot_radius"])
        t0 = time.time()
        astar_path_x, astar_path_y = astar_planner.plan(cfg["start"], cfg["goal"])
        t_astar = (time.time() - t0) * 1000
        astar_success = len(astar_path_x) > 0
        astar_path = list(zip(astar_path_x, astar_path_y))
        astar_metrics = compute_all_metrics(
            astar_path, obs_list, astar_planner.nodes_explored, t_astar, astar_success
        )
        astar_metrics["test_id"] = i + 1
        astar_metrics["algorithm"] = "A*"
        astar_metrics["map_size"] = cfg["map_size"]
        astar_metrics["num_obstacles"] = cfg["num_obstacles"]
        results.append(astar_metrics)

        # Reset map for RRT (without inflation)
        gm2 = GridMap(cfg["map_size"], cfg["map_size"], cfg["resolution"])
        generate_boundary_obstacles(gm2)
        obs_list2 = generate_random_obstacles(
            gm2, num_obstacles=cfg["num_obstacles"], seed=cfg["seed"]
        )

        # Test RRT
        print("  Running RRT...")
        bounds = (0, cfg["map_size"], 0, cfg["map_size"])
        rrt_planner = RRTPlanner(
            start=cfg["start"],
            goal=cfg["goal"],
            obstacle_list=obs_list2,
            bounds=bounds,
            expand_dis=2.0,
            max_iter=1000,
            robot_radius=cfg["robot_radius"],
            seed=cfg["seed"],
        )
        t0 = time.time()
        rrt_path = rrt_planner.plan()
        t_rrt = (time.time() - t0) * 1000
        rrt_success = rrt_path is not None
        rrt_metrics = compute_all_metrics(
            rrt_path if rrt_success else [],
            obs_list2,
            rrt_planner.nodes_explored,
            t_rrt,
            rrt_success,
        )
        rrt_metrics["test_id"] = i + 1
        rrt_metrics["algorithm"] = "RRT"
        rrt_metrics["map_size"] = cfg["map_size"]
        rrt_metrics["num_obstacles"] = cfg["num_obstacles"]
        results.append(rrt_metrics)

        print(f"  A*: {'OK' if astar_success else 'FAIL'} ({t_astar:.1f}ms, {astar_planner.nodes_explored} nodes)")
        print(f"  RRT: {'OK' if rrt_success else 'FAIL'} ({t_rrt:.1f}ms, {rrt_planner.nodes_explored} nodes)")

    # Write CSV
    if results:
        # Add obstacle_density and seed, rename runtime field
        for r in results:
            r["obstacle_density"] = r["num_obstacles"]
            r["runtime_ms"] = r.pop("planning_time_ms", 0.0)

        fieldnames = [
            "test_id", "algorithm", "map_size", "num_obstacles",
            "obstacle_density", "success", "path_length",
            "smoothness_rad", "clearance_m", "nodes_explored",
            "runtime_ms", "path_points",
        ]
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)

        print(f"\nResults saved to: {csv_path}")
        _print_summary(results)

        # Generate benchmark summary markdown
        _write_benchmark_summary(results, output_dir)
        print(f"Benchmark summary saved to: {output_dir}/benchmark_summary.md")

        # Generate batch metrics summary plot
        try:
            from visualization.plotter import PathPlotter
            figures_dir = os.path.join(os.path.dirname(output_dir), "figures")
            os.makedirs(figures_dir, exist_ok=True)
            plot_path = os.path.join(figures_dir, "batch_metrics_summary.png")
            PathPlotter.plot_batch_summary(csv_path, save_path=plot_path)
            print(f"Batch metrics plot saved to: {plot_path}")
        except Exception as e:
            print(f"Warning: Could not generate batch summary plot: {e}")

    return results


def _print_summary(results: List[dict]):
    """Print a summary table of results."""
    header = f"{'Algorithm':<10} {'Success':<8} {'PathLen':<10} {'Smoothness':<12} {'Clearance':<10} {'Time(ms)':<10}"
    print(f"\n{header}")
    print("-" * 65)
    for r in results:
        rt = r.get("runtime_ms", r.get("planning_time_ms", 0))
        print(
            f"{r['algorithm']:<10} {str(r['success']):<8} "
            f"{r['path_length']:<10.2f} {r['smoothness_rad']:<12.3f} "
            f"{r['clearance_m']:<10.2f} {rt:<10.1f}"
        )


def _write_benchmark_summary(results: List[dict], output_dir: str):
    """Write benchmark summary markdown."""
    astar_results = [r for r in results if r["algorithm"] == "A*"]
    rrt_results = [r for r in results if r["algorithm"] == "RRT"]

    astar_success = [r for r in astar_results if r["success"] == True or r["success"] == "True"]
    rrt_success = [r for r in rrt_results if r["success"] == True or r["success"] == "True"]

    def _avg(lst, key):
        vals = [float(r[key]) for r in lst if r[key] != float("inf") and r[key] != "inf"]
        return sum(vals) / len(vals) if vals else 0.0

    md = f"""# Benchmark Summary

## Test Configuration

- Map sizes: {sorted(set(str(r['map_size']) for r in results))}
- Obstacle counts: {sorted(set(str(r['num_obstacles']) for r in results))}
- Tests per algorithm: {len(astar_results)}
- Total tests: {len(results)}

## A* Results Summary

| Metric | Value |
|--------|-------|
| Success Rate | {len(astar_success)}/{len(astar_results)} ({len(astar_success)/len(astar_results)*100:.0f}%) |
| Avg Path Length | {_avg(astar_success, 'path_length'):.2f} m |
| Avg Runtime | {_avg(astar_results, 'runtime_ms'):.1f} ms |
| Avg Nodes Explored | {_avg(astar_results, 'nodes_explored'):.0f} |

## RRT Results Summary

| Metric | Value |
|--------|-------|
| Success Rate | {len(rrt_success)}/{len(rrt_results)} ({len(rrt_success)/len(rrt_results)*100:.0f}%) |
| Avg Path Length | {_avg(rrt_success, 'path_length'):.2f} m |
| Avg Runtime | {_avg(rrt_results, 'runtime_ms'):.1f} ms |
| Avg Nodes Explored | {_avg(rrt_results, 'nodes_explored'):.0f} |

## A* vs RRT Comparison

| Metric | A* | RRT |
|--------|----|-----|
| Success Rate | {len(astar_success)}/{len(astar_results)} ({len(astar_success)/len(astar_results)*100:.0f}%) | {len(rrt_success)}/{len(rrt_results)} ({len(rrt_success)/len(rrt_results)*100:.0f}%) |
| Avg Path Length (success) | {_avg(astar_success, 'path_length'):.2f} m | {_avg(rrt_success, 'path_length'):.2f} m |
| Avg Runtime | {_avg(astar_results, 'runtime_ms'):.1f} ms | {_avg(rrt_results, 'runtime_ms'):.1f} ms |
| Avg Nodes | {_avg(astar_results, 'nodes_explored'):.0f} | {_avg(rrt_results, 'nodes_explored'):.0f} |

## Conclusions

- A* consistently produces shorter paths in grid maps, thanks to its optimal heuristic search.
- RRT searches faster in continuous space but paths can be longer due to random sampling.
- A* success depends on grid resolution and obstacle density; higher resolution = more nodes.
- RRT success depends on random seed and obstacle layout; results have inherent randomness.
- RRT is more suitable for high-dimensional or continuous-space planning problems.
- Batch testing provides stability assessment for robot navigation algorithm selection.
"""
    with open(os.path.join(output_dir, "benchmark_summary.md"), "w", encoding="utf-8") as f:
        f.write(md)


if __name__ == "__main__":
    run_batch_tests()
