# Resume Description

## Version 1: Standard (3 bullet points)

- Built a 2D grid map simulation environment in Python, implementing obstacle generation, start/goal configuration, and path visualization with Matplotlib.
- Implemented A* and RRT path planning algorithms, including shortest-path search with heuristic, random tree expansion with step-size control, and collision detection.
- Designed a batch testing pipeline to evaluate path length, runtime, nodes explored, and success rate across varying map sizes and obstacle densities, with CSV result export.

## Version 2: Algorithm-Focused

- Developed a modular path planning simulator in Python, implementing A* (grid-based optimal search with 8-direction motion model) and RRT (sampling-based planning with goal bias and step-size control).
- Built configurable 2D grid maps with random obstacle generation, robot-radius inflation, and continuous-space collision checking.
- Created a benchmark framework comparing A* vs RRT across multiple scenarios, measuring path optimality, computational efficiency, and planning success rate.
- Implemented evaluation metrics including path length, smoothness (turning angle), and obstacle clearance.

## Version 3: Robotics Test-Focused

- Designed and implemented a simulation testbed for mobile robot path planning algorithms (A* and RRT) in 2D grid environments with configurable obstacle layouts.
- Built automated batch testing workflows to systematically evaluate planning performance across different map configurations (size, obstacle density, random seeds).
- Developed quantitative evaluation metrics (path length, runtime, success rate, nodes explored) with CSV export for data-driven algorithm comparison and failure analysis.
- Generated publication-quality visualization outputs showing planned paths, search trees, and comparative algorithm performance.

## Project Link
Add the repository URL after publishing, for example:
`https://github.com/<your-github-username>/path_planning_simulator`

## Tech Stack
Python, NumPy, Matplotlib, A*, RRT, Path Planning, Collision Detection
