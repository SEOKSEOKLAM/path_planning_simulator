# Run Audit Log

## Audit Date
2026-05-19

## Project Path
`path_planning_simulator/`

## Initial State

### Files Present
- `main.py` — main entry point with CLI
- `requirements.txt` — numpy, matplotlib
- `algorithms/` — astar.py, rrt.py, collision.py
- `maps/` — grid_map.py, obstacle_generator.py
- `evaluation/` — metrics.py, batch_test.py
- `visualization/` — plotter.py
- `outputs/` — figures/, results/

### CLI Parameters Verified
```
--algo {astar,rrt,both}
--map {random,room}
--map-size MAP_SIZE
--resolution RESOLUTION
--num-obstacles NUM_OBSTACLES
--robot-radius ROBOT_RADIUS
--start-x / --start-y / --goal-x / --goal-y
--no-show
--batch
--seed SEED
```

### Python Environment
- Python 3.10.11
- numpy, matplotlib installed
- .venv present and functional

## Issues Found and Fixed

1. **Save filenames**: Changed from `astar_result.png`/`rrt_result.png` to `astar_demo.png`/`rrt_demo.png`
2. **Missing comparison plot**: Added `plot_comparison()` to `plotter.py` and integrated into `main.py`
3. **Missing batch summary plot**: Added `plot_batch_summary()` to `plotter.py` and called from `batch_test.py`
4. **CSV field names**: Unified to `runtime_ms`, added `obstacle_density` column
5. **Missing benchmark_summary.md**: Added auto-generation in `batch_test.py`
6. **Missing docs/ directory**: Created with 6 documentation files
7. **Missing .gitignore**: Created with standard Python exclusions
8. **Missing LICENSE / NOTICE**: Created with MIT license and attribution

## Final State

All 4 demo images generated:
- `outputs/figures/astar_demo.png`
- `outputs/figures/rrt_demo.png`
- `outputs/figures/astar_rrt_comparison.png`
- `outputs/figures/batch_metrics_summary.png`

All results generated:
- `outputs/results/batch_results.csv`
- `outputs/results/benchmark_summary.md`

Copies in `assets/` for README display.

## Verified Commands
```powershell
python main.py --algo astar --no-show
python main.py --algo rrt --no-show
python main.py --algo both --map-size 80 --num-obstacles 20 --seed 123 --no-show
python main.py --batch
```

All commands completed successfully.
