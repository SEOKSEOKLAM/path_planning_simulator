# GitHub Upload Checklist

- [x] `main.py` can run (A*, RRT, both modes verified)
- [x] `requirements.txt` is complete (numpy, matplotlib)
- [x] `README.md` is complete with all required sections
- [x] Demo images are available in `assets/` (4 images)
- [x] Batch CSV exists in `outputs/results/`
- [x] Benchmark summary exists in `outputs/results/`
- [x] `.gitignore` excludes `.venv/`, `__pycache__/`, IDE files
- [x] No absolute local paths appear in README or source files
- [x] No private files are included
- [x] Project can be run with Quick Start commands (verified)
- [x] `LICENSE` file present (MIT)
- [x] `NOTICE` file present with attribution
- [x] `docs/` directory complete (6 files)

## Before `git push`, verify:

1. `.venv/` is NOT in the commit:
   ```bash
   git status  # should not show .venv/
   ```

2. All images are rendered correctly by checking the PNG files.

3. Quick Start commands from README work on a fresh clone:
   ```powershell
   pip install -r requirements.txt
   python main.py --algo astar --no-show
   python main.py --algo rrt --no-show
   python main.py --batch
   ```

4. No local absolute paths visible:
   ```bash
   grep -r "absolute local path pattern" --include="*.md" --include="*.py" . || echo "clean"
   ```
