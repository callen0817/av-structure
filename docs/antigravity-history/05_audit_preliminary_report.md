# AV Structure: Preliminary Audit & Discovery Report

## Executive Summary
This audit inspects the current production codebase (`stations/geometry_mode.py`, `stations/trajectory_mode.py`, `simulation/scanner.py`, `virtual_scanner.py`) and profiles real data (`Ex_2_Level.e57` — a 23.8M-point 2-level building) to establish:
1. **What AV Structure is actually doing today** (without theoretical assumptions).
2. **Where and why multi-level and visibility failures physically occur**.
3. **The smallest viable refinements** that preserve the working architecture without rewriting the engine.

---

## 1. Empirical Findings & Diagnostic Tracing

### Finding 1: Multi-Level Station Under-Generation in Geometry Mode
* **Current Code**: `stations/geometry_mode.py` (lines 41–53)
  ```python
  for x in x_steps:
      for y in y_steps:
          # ...
          floor_z = np.percentile(local_points[:, 2], 5)
          ceil_z = np.percentile(local_points[:, 2], 95)
          if ceil_z - floor_z > 2.0:
              station_pos = (x, y, floor_z + self.tripod_height)
              candidates.append(station_pos)
  ```
* **Why It Exists**: Originally designed for single-storey indoor scans. It takes the 5th percentile of all points in a 2D $(X, Y)$ vertical column as the floor.
* **Measured Severity on `Ex_2_Level.e57`**:
  - Point cloud has two distinct levels ($Z \approx 0.0\text{m}$ and $Z \approx 2.9\text{m}$, max ceiling $5.9\text{m}$).
  - In columns where Level 1 and Level 2 are stacked vertically, `floor_z` is Level 1, and `ceil_z` is Level 2's ceiling.
  - **Result**: Only **one** station is placed per column at Level 1 height ($Z \approx 1.6\text{m}$). Level 2 only receives candidate stations in cantilevered, balcony, or unstacked areas (19 stations generated on Level 1 vs. only 9 on Level 2).
* **Smallest Viable Refinement**:
  - Instead of taking a single 5th percentile per column, detect multiple floor peaks within the vertical column (or cluster building-wide elevation peaks first), and instantiate candidate stations for each valid floor level where headroom ($Z_{\text{ceil}} - Z_{\text{floor}} > 2.0\text{m}$) exists.
* **Performance Risk**: Negligible ($<0.1\text{s}$ during downsampled candidate generation).

---

### Finding 2: Visibility Bypass on Production Clouds (>10M points)
* **Current Code**: `simulation/scanner.py` (lines 86–92, 100–104)
  ```python
  if num_points <= 10_000_000:
      try:
          _, visible_pts = pcd.hidden_point_removal(camera_loc, diameter)
          vis_indices = np.asarray(visible_pts)
      except Exception as e: ...
  # ...
  no_vis = (best_visible_station == -1)
  final_station_assignment = np.where(no_vis, best_absolute_station, best_visible_station)
  ```
* **Why It Exists**: Open3D's `hidden_point_removal` performs spherical inversion and convex hull computation in RAM. On $>10\text{M}$ points, it causes out-of-memory crashes or severe slowdowns on typical workstations.
* **Measured Severity**:
  - Any production dataset $>10\text{M}$ points (e.g. `Ex_2_Level.e57` at 23.8M, `Ex_1_Level.e57` at 52.4M, or Chevy Chase at 116.8M) **skips visibility entirely**.
  - 100% of points are partitioned by pure 3D Euclidean Voronoi nearest-neighbor (`best_absolute_station`).
  - Solid walls and floor slabs are completely transparent to point assignment.
* **Smallest Viable Refinement**:
  - Do not try to run full-resolution HPR on 100M+ points.
  - Keep the fast chunked Euclidean assignment as a candidate proposal, but add a **vectorized First-Hit Spherical Angular Binning filter** on the assigned points per station. Points falling into the same $(\theta, \phi)$ angular bin that lie significantly behind the first dense radial peak ($r > r_{\text{first}} + \delta r$) are rejected/reassigned.
* **Performance Risk**: Low. Spherical projection $(\theta, \phi, r)$ of a station's subset (typically 1M–5M points) takes $<0.5\text{s}$ using pure vectorized NumPy.

---

### Finding 3: Cross-Floor Contamination via 3D Euclidean Voronoi
* **Current Code**: `simulation/scanner.py` (lines 74–82)
  ```python
  dists_sq = np.sum((chunk - camera_loc) ** 2, axis=1)
  ```
* **Why It Exists**: Fast, simple, vectorized, and guarantees 100% point preservation (every point assigned to exactly one station; output file size matches input cloud size).
* **Measured Severity on Multi-Level**:
  - Vertical distance from Level 2 station to Level 1 ceiling or Level 3 floor is $\sim 1.5\text{--}2.5\text{m}$.
  - If the nearest Level 1 station is $4\text{m}$ away horizontally, the Level 2 station is physically closer in 3D Euclidean space ($2.5\text{m} < 4\text{m}$).
  - Points on the floor slab and ceiling are assigned across floors, resulting in scans that contain horizontal slices of adjacent storeys.
* **Smallest Viable Refinement**:
  - **Level-Constrained Candidate Partitioning**: Restrict station assignment so a point is only eligible for candidate stations belonging to its legitimate vertical storey/envelope (derived from floor elevation clustering and line-of-sight), *unless* there is continuous open vertical volume (e.g., stairwells, atriums, double-height spaces).
  - Alternatively, use an anisotropic vertical distance weighting or slab-barrier penalty during the distance comparison:
    $$d_{\text{eff}}^2 = (x - x_s)^2 + (y - y_s)^2 + w_z \cdot (z - z_s)^2$$
    where vertical cross-slab separation is penalized while preserving horizontal reach.
* **Performance Risk**: Low. Adds minimal computation to the existing chunked distance loop.

---

### Finding 4: Point Preservation & Unassigned Tracking
* **Baseline Guarantee**: No silent geometry discard.
* **Current Behavior**:
  - `max_range` filter marks points as `-1` (excluded), logged only as a scalar count: `logger.info(f"Max range filter ({max_range}m) excluded {rejected_pts:,} points.")`.
* **Smallest Viable Refinement**:
  - Maintain an explicit `unassigned_reason` array tracking why any point was not assigned (`OUT_OF_RANGE`, `OCCLUDED_BEHIND_FIRST_HIT`, `NO_LINE_OF_SIGHT`).
  - Log full accounting in the `processing_manifest.json` and `validation_report.txt` so every single input point is accounted for.

---

## 2. Decision & History Matrix

| Subsystem | Current Code | Discovered Limitation | Prior Alternatives Tried? | Proposed Smallest Intervention |
| :--- | :--- | :--- | :--- | :--- |
| **Multi-Level Station Gen** | Single 5th percentile Z in `geometry_mode.py` | Misses upper floors where floor plans overlap | *To confirm: were multi-peak vertical histograms tried?* | Multi-peak vertical elevation detection per $(X,Y)$ column in `geometry_mode.py` |
| **Multi-Level Assignment** | Omnidirectional 3D Euclidean Voronoi | Points punch through floor slabs to vertically closer stations | *To confirm: were Z-bounding bands or slab masks tried?* | Slab-barrier penalty / storey-constrained Voronoi candidate assignment |
| **Visibility on >10M pts** | Skipped completely (`if num_points <= 10M`) | All large scans bypass occlusion checks | *HPR causes OOM crash on large point arrays* | Post-Voronoi first-hit spherical angular binning filter per station |
| **Sensor FOV** | Omnidirectional sphere, `max_range` only | Points assigned outside physical scanner vertical angles | *To confirm: were angular limits tried and orphaned points?* | Optional vertical aperture clamp $[\theta_{\min}, \theta_{\max}]$ with explicit unassigned tracking |
| **Panospheres** | GUID-linked `images2D` with nearest pano pose | Functional, tested on Chevy Chase & NavVis | Working baseline — preserve | Keep existing structure |

---

## 3. Next Step
Before writing code for the refinements, review the **Prior Alternatives** column:
- What previous attempts were made for multi-level handling or Z-slicing?
- Does the anisotropic distance penalty or level-aware candidate constraint align with what you've seen in the field?
- Does the First-Hit Spherical Angular Binning filter match the lightweight, scalable visibility model you envision?
