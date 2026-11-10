# AV Structure Phase 1 Checklist

## Project Infrastructure & Folder Management
- [x] Implement standardized Project/Job directory structure creation in `gui.py`.
- [x] Implement `project.json` load/save logic including schema versioning and status variables.
- [x] Redirect GUI and pipeline logs to Job-specific log files under `logs/`.
- [x] Add Project Directory browse button and text field to GUI.

## Max Scan Range Hard Constraint
- [x] Add `Max Scan Range (m)` slider to GUI settings frame.
- [x] Add global default `MAX_SCAN_RANGE = 50.0` in `config.py`.
- [x] Pass `max_range` parameter from GUI/CLI down to simulation.
- [x] Implement the hard physical constraint in `ScannerSimulator.simulate_scans` (filter out points beyond `max_range` in world space).

## Preflight Integrity Checks & Validation
- [x] Implement pre-flight checks (input cloud readability, pose CSV formatting, panorama path presence, disk space, writable output, coordinate consistency).
- [x] Implement strict panorama/scan validation rule prior to export (non-empty points, valid pose, linked panorama exists). Abort export loudly on failure.

## Diagnostics, Manifest & Reports
- [x] Implement `validation_report.txt` generation in the Job folder.
- [x] Implement `processing_manifest.json` generation containing run metadata.
- [x] Calculate scanner diagnostics per scan (points, avg/max distance, overlap, rejection breakdown: out of range).

## Determinism
- [x] Replace `np.random.choice` inside resolution estimation with deterministic uniform slice sampling.
- [x] Sort candidate stations and point indices deterministically before export.

## Dry Run Mode
- [x] Implement Dry Run mode in the GUI and command-line interfaces.
- [x] Run preflight, load data, generate stations, and output estimates (scan count, memory, runtime, warnings) without performing simulation or export.

## Verification
- [x] Write a verification script `scratch/test_phase1.py` to test preflight validation, max range filtering, and determinism.
- [x] Run verification tests and document results in `walkthrough.md`.
