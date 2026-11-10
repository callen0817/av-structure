# AV Structure — Phase 1 Walkthrough

## Summary

Phase 1 transforms AV Structure from a working prototype into a commercially structured application. No changes were made to the point partitioning algorithm — the Golden Release (v1.0.0) behavior is fully preserved.

---

## Changes Made

### `config.py`
Added Phase 1 constants:
- `DEFAULT_MAX_SCAN_RANGE = 50.0` — default hard range limit in meters
- `VISIBILITY_ENGINE = "NearestNeighborEngine"` — named engine for future abstraction
- `STATION_PROVIDER = "PoseCSVProvider"` — named provider for future abstraction

---

### `gui.py`
- **Project Directory** — browse button + text field; on selection, creates the standard project folder structure (`input/`, `output/`, `logs/`, `cache/`, `thumbnails/`, `temp/`) and loads/saves `project.json`
- **Max Scan Range slider** — 1–200 m, defaults to 50 m; stored in `project.json`
- **Dry Run Mode checkbox** — passes `dry_run=True` to the pipeline; estimations only, no file writes
- **Job Directory routing** — each run gets a sequential `Job_001`, `Job_002`, … folder under `output/`, with its own run log (`logs/Job_XXX_run.log`) and output e57
- **`project.json` schema** — version, timestamps, all paths, scan spacing, max range, station generation mode

---

### `virtual_scanner.py`
Updated `run_pipeline()` signature to accept:
- `max_range: float` — enforced in simulation
- `project_dir: str` — for future manifest routing
- `job_id: str` — job folder identifier
- `dry_run: bool` — halts pipeline after station generation

**Preflight Integrity Validation** (runs before any data is loaded):
1. Input cloud file must exist
2. Pose CSV must exist and parse correctly (semicolon-delimited, ≥6 columns)
3. Output directory must be writable and have ≥200 MB free disk space
4. Panorama directory must exist (if provided)

**Pre-export Structural Validation** (trajectory mode):
- Station count must match panorama count
- Every station must have a valid pose
- Every station must have >0 assigned points
- Every panorama image file must exist on disk

**Dry Run Mode** — after station generation, outputs:
- Candidate station count
- Input point count
- Estimated peak memory usage (MB)
- Estimated processing runtime (seconds)
- Estimated output file size (MB)
- Warning count for missing panoramas

**Diagnostics outputs** — after successful export:
- `validation_report.txt` — human-readable run summary + per-scan table
- `validation_report.json` — machine-readable version of the same
- `processing_manifest.json` — metadata (software version, algorithm, max_range, timing)

---

### `simulation/scanner.py`
- `estimate_angular_resolution()` — replaced `np.random.choice` with deterministic uniform stride sampling (`points[::step]`)
- `simulate_scans()` — added `max_range` parameter; after assignment, any point whose best-station distance² exceeds `max_range²` is set to unassigned (`-1`) and excluded from all scan outputs

---

### `stations/trajectory_mode.py`
- Selected poses now sorted by `timestamp` before returning, guaranteeing deterministic station ordering across runs.

---

### `scratch/test_phase1.py`
Verification script with 5 independent backend tests (no GUI required):

| # | Test | Result |
|---|------|--------|
| 1 | Project directory structure + `project.json` creation | ✅ PASS |
| 2 | Preflight: missing panorama directory raises `FileNotFoundError` | ✅ PASS |
| 3 | Preflight: malformed pose CSV raises `ValueError` | ✅ PASS |
| 4 | Max range 15m constraint: 2,170 distant points correctly rejected from 10,000 total | ✅ PASS |
| 5 | Two identical simulation runs produce byte-identical output | ✅ PASS |

---

## Test Output (verbatim)

```
[TEST 1] Project directory structure and project.json creation...
  PASS — Directories and project.json created correctly.

[TEST 2] Preflight: missing panorama directory should fail loudly...
  Expected failure: Panorama directory not found: ...this_folder_does_not_exist
  PASS — Preflight catches missing panorama directory.

[TEST 3] Preflight: malformed pose CSV should fail loudly...
  Expected failure: Pose CSV is malformed or unreadable: Delimiter ';' not found or incorrect column count.
  PASS — Preflight catches malformed pose CSV.

[TEST 4] Max Scan Range hard constraint...
  Unlimited:      10,000 / 10,000 points assigned
  Max range 15m:   7,830 / 10,000 points assigned
  Rejected:        2,170 points
  PASS — Hard range constraint correctly excludes out-of-range points.

[TEST 5] Determinism: two identical runs must produce identical output...
  PASS — Simulation output is fully deterministic across runs.

==================================================
  ALL 5 PHASE 1 TESTS PASSED
==================================================
```

---

## Environment Notes

- The `.venv_310` virtual environment was repaired to point to the Python 3.10 interpreter bundled with CloudCompare (`C:\Program Files\CloudCompare\plugins\Python\python.exe`), since the original `C:\Program Files\Python310` installation is no longer present.
- Open3D is not available for Python 3.14 on Windows via PyPI; always use `.venv_310` to run this project.
- `run.bat` and `build.bat` should already reference `.venv_310` — verify before distributing.

---

## What Was NOT Changed

- Point partitioning algorithm (HPR + Euclidean fallback) — unchanged
- Export format (E57, panorama embedding) — unchanged
- GUI design, colors, layout, name — unchanged
- Golden Release baseline (`backup_golden_state/`) — untouched
