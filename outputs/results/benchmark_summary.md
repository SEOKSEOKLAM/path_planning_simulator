# Benchmark Summary

## Test Configuration

- Map sizes: ['50', '80']
- Obstacle counts: ['15', '25', '30']
- Tests per algorithm: 3
- Total tests: 6

## A* Results Summary

| Metric | Value |
|--------|-------|
| Success Rate | 2/3 (67%) |
| Avg Path Length | 82.18 m |
| Avg Runtime | 16.0 ms |
| Avg Nodes Explored | 825 |

## RRT Results Summary

| Metric | Value |
|--------|-------|
| Success Rate | 1/3 (33%) |
| Avg Path Length | 66.72 m |
| Avg Runtime | 31.6 ms |
| Avg Nodes Explored | 312 |

## A* vs RRT Comparison

| Metric | A* | RRT |
|--------|----|-----|
| Success Rate | 2/3 (67%) | 1/3 (33%) |
| Avg Path Length (success) | 82.18 m | 66.72 m |
| Avg Runtime | 16.0 ms | 31.6 ms |
| Avg Nodes | 825 | 312 |

## Conclusions

- A* consistently produces shorter paths in grid maps, thanks to its optimal heuristic search.
- RRT searches faster in continuous space but paths can be longer due to random sampling.
- A* success depends on grid resolution and obstacle density; higher resolution = more nodes.
- RRT success depends on random seed and obstacle layout; results have inherent randomness.
- RRT is more suitable for high-dimensional or continuous-space planning problems.
- Batch testing provides stability assessment for robot navigation algorithm selection.
