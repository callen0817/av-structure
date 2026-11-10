# AV Structure: Discovery & Fine-Tuning Audit Specification

## Objective
Audit the existing, functioning AV Structure codebase and runtime behavior to fine-tune two critical production targets:
1. **Multi-level building handling** (eliminating cross-floor point contamination through slabs/ceilings while respecting atriums, double-height spaces, and stairwells).
2. **Physically plausible FOV / visibility** (preventing occlusion violations and points behind first-hit surfaces).

The audit follows a strict discovery-first methodology: establish **what AV Structure does today**, **why it exists**, **what alternatives were previously attempted**, and **measure actual severity**, leading to the **smallest viable intervention** without rewriting functioning portions of the system.

---

## 1. Non-Negotiable Baseline Guarantees
Every finding and proposed refinement must strictly uphold four baseline guarantees:
1. **Scale & Stability**: Flawless execution on $>100\text{M}$ points without out-of-memory crashes.
2. **Determinism**: Identical runs on identical inputs yield bit-wise identical structured outputs.
3. **Downstream Compatibility**: Correct ReCap-compatible structured E57 files with valid GUID-linked panospheres.
4. **No Silent Geometry Discard**: The system must never silently discard substantial geometry. Every unassigned or rejected point must be accounted for diagnostically (e.g. in validation reports) so an FOV/visibility improvement cannot appear successful merely by deleting difficult points.

---

## 2. Four Diagnostic Audits (Runtime Probes)

### Diagnostic 1: Vertical & Multi-Floor Stratification Audit (Cross-Floor Contamination)
- **Problem**: In 3D Euclidean Voronoi partitioning, points directly above/below a station on adjacent storeys may be closer than a horizontally distant station on their own storey.
- **Probe Methodology**:
  1. Discover candidate floor levels $\{Z_1, Z_2, \dots, Z_k\}$ using station $Z$-distributions and elevation histograms.
  2. Detect intervening solid surfaces (slabs/ceilings) between vertical station clusters.
  3. **Distinguish legitimate vertical volume from contamination**: Points $>2.5\text{m}$ above/below a station are checked for direct vertical line-of-sight vs. an intervening slab. Atriums, stairwells, and double-height spaces with open line-of-sight are recognized as valid; points separated by an opaque slab without line-of-sight are flagged as cross-floor leakage.
- **Target Metric**: Contaminated point ratio per level (%) and leakage distribution.

### Diagnostic 2: Visibility Bypass & Scalability Audit
- **Problem**: `simulation/scanner.py` sets an arbitrary threshold: `if num_points <= 10_000_000: HPR... else: Voronoi nearest-neighbor`.
- **Probe Methodology**:
  1. Profile memory footprint and runtime of HPR vs. cloud size ($1\text{M}, 5\text{M}, 10\text{M}, 25\text{M}, 100\text{M}+$ points).
  2. Document why the 10M cutoff was introduced (e.g., Open3D convex hull memory blow-up on large arrays).
  3. Measure how many points in $>10\text{M}$ production clouds fail true line-of-sight when pure Voronoi assignment is active.
- **Target Metric**: Visibility processing coverage (% of points evaluated for occlusion) and HPR resource scaling limits.

### Diagnostic 3: First-Hit Radial Consistency Audit (Spherical Angular Binning)
- **Problem**: A scanner cannot see through solid walls; assigning points behind a solid foreground surface violates physical visibility.
- **Probe Methodology**:
  1. For each virtual station, project assigned points into a spherical angular coordinate system $(\theta \in [-\pi/2, \pi/2], \phi \in [-\pi, \pi])$ discretized into angular bins (e.g. $0.25^\circ \times 0.25^\circ$).
  2. Within each populated angular bin, analyze the radial distance distribution of assigned points.
  3. Detect multi-modal radial clusters: if a bin has a dense surface at distance $r_1$ (e.g. a wall at $3.2\text{m}$) and assigned points at $r_2 \gg r_1$ (e.g. $9.7\text{m}$ in another room or outside), flag points $> r_1 + \delta r$ as first-hit occlusion violations.
- **Target Metric**: Percentage of assigned points residing behind a detected first-hit surface per scan.

### Diagnostic 4: Angular Aperture & Multi-Station Co-Visibility Audit
- **Problem**: Omnidirectional spherical assignment may assign points outside physical scanner vertical apertures; exclusive 1:1 assignment eliminates overlap.
- **Probe Methodology**:
  1. Measure assigned point distributions against realistic terrestrial scanner apertures (e.g. zenith/nadir limits).
  2. Measure candidate co-visibility: for every point, calculate how many virtual stations have clear line-of-sight to it.
  3. Quantify the difference between strict 1:1 exclusive partitioning and potential shared co-visibility without assuming an arbitrary overlap target.
- **Target Metric**: Aperture boundary violations count and co-visibility distribution histogram ($1\times, 2\times, 3\times+$ stations).

---

## 3. Audit Deliverable: Finding-to-Refinement Synthesis Report

Rather than jumping directly to major refactoring stages, the audit concludes with an evidence-based Synthesis Report structured as:

$$\boxed{\text{Finding}} \longrightarrow \boxed{\text{Current Code}} \longrightarrow \boxed{\text{Why It Exists}} \longrightarrow \boxed{\text{Prior Alternatives}} \longrightarrow \boxed{\text{Measured Severity}} \longrightarrow \boxed{\text{Smallest Viable Refinement}} \longrightarrow \boxed{\text{Expected Effect}} \longrightarrow \boxed{\text{Performance Risk}}$$

### Example Candidate Refinement Pathways (to be proven by audit):
- **For Multi-Level**: Instead of discarding Voronoi, apply level-aware candidate station bounds or vertical barrier constraints *before* Voronoi assignment.
- **For Visibility**: Instead of full 3D CAD raycasting, use the spherical angular binning from Diagnostic 3 as a fast, vectorized post-assignment rejection filter that discards or reassigns points behind first-hit surfaces.
- **For Unassigned Points**: Any points excluded by first-hit or range filters are explicitly routed to an unassigned diagnostic pool with reasons logged in the manifest.
