# AV Structure: Architectural Decisions & System Synthesis

This document records the core architectural decisions, design philosophies, and algorithmic innovations developed for **AV Structure** across its production evolution.

---

## 1. Paradigm Shift: Observation-Informed Reverse Engineering vs. Spatial Subdivision

### The Problem
Traditional naive point cloud structuring treats the problem as spatial subdivision—dividing an unstructured cloud into grid-sized geometric cells. This approach fails to reproduce Terrestrial Laser Scanning (TLS) behavior because real TLS scans represent:
$$\mathcal{S}_i = \{ P_j \mid P_j \text{ is visible from } T_i,\ P_j \in \text{FOV}_i,\ r_j < r_{\max} \}$$
A point cannot belong to a virtual scanner station simply because it is physically close in 3D Euclidean space; intervening walls, floor slabs, columns, and closed doors occlude line-of-sight.

### The Architectural Decision
AV Structure reverse-engineers the physical observations that generated the point cloud:
1. **Pose-Informed Stations**: Ingest NavVis trajectory poses and panoramas as candidate scanner viewpoints rather than relying on an arbitrary grid.
2. **Dataset Namespacing**: Rather than relying on original camera filenames (e.g., `000.jpg`, `001.jpg`), assign unique internal namespaces:
   ```
   Dataset (UUID, Source E57, Coordinate System)
   └── Captures[] (Capture UUID, Source Filename, Timestamp, Pose, Panorama)
   ```
3. **Multi-Input Quality Tiers**:
   - **Tier 1 (Unstructured E57 only)**: Estimate geometric candidate stations using multi-storey free-space discovery.
   - **Tier 2 (E57 + Trajectory/Poses)**: Sample along cumulative trajectory arc-length, weighting vertical transitions.
   - **Tier 3 (E57 + Panoramas + Poses)**: 1:1 observation-informed reconstruction linking physical camera photo-spheres into E57 `images2D` structures.

---

## 2. Multi-Level Building Stratification (Z-Histogram Peak Discovery)

### The Problem
In multi-storey buildings (e.g. 10-level structures spanning $>70\text{m}$ vertically), naive percentile slicing (`floor_z = np.percentile(col[:, 2], 5)`) collapsed stacked storeys, placing stations only on Level 1 and completely starving upper floors. Furthermore, 3D Euclidean distance caused points on upper slabs to be captured by lower stations ($2.5\text{m}$ vertical slab separation vs $4.0\text{m}$ horizontal station distance).

### The Implementation
1. **Storey Band Detection (`stations/geometry_mode.py`)**:
   - Compute building-wide vertical histograms with $0.25\text{m}$ bins.
   - Discover horizontal slab peaks separated by $\ge 2.2\text{m}$ with density exceeding the 65th percentile.
   - Segment vertical bounds into discrete storey envelopes: $[(Z_{0},\ Z_{1}), (Z_{1},\ Z_{2}), \dots]$.
2. **Anisotropic Slab Penalty (`simulation/scanner.py`)**:
   - In distance and Voronoi assignment, vertical distance $|\Delta z|$ is weighted non-linearly:
     $$d_{\text{eff}}^2 = \Delta x^2 + \Delta y^2 + w_z \cdot \Delta z^2$$
     where $w_z = 1.5$ within normal ceiling height ($|\Delta z| \le 2.2\text{m}$), and jumps to $w_z = 12.0$ across floor slabs ($|\Delta z| > 2.2\text{m}$). This prevents cross-floor slab bleed while allowing horizontal coverage.

---

## 3. Physical Visibility: Spherical First-Hit Depth Buffers vs. HPR Cutoff

### The Problem
Open3D's `hidden_point_removal` (HPR) calculates a spherical inversion followed by 3D convex hull computation in RAM. On point clouds exceeding 10M points, HPR causes out-of-memory crashes, leading earlier iterations of AV Structure to bypass visibility entirely on production datasets ($>10\text{M}$ points) and fall back to unoccluded Voronoi partitioning.

### The Implementation
- Replaced HPR with a vectorized **Spherical First-Hit Depth Buffer**:
  1. Map 3D relative point vectors $(\Delta x, \Delta y, \Delta z)$ to discrete spherical coordinates:
     $$u = \text{azimuth bin } \in [0, N_{\text{az}}-1], \quad v = \text{elevation bin } \in [0, N_{\text{el}}-1]$$
  2. Compute radial depth: $r = \sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$.
  3. Buffer first-hit surfaces via `np.minimum.at(depth_map, (u, v), r)`.
  4. Apply morphological gap closing via $3 \times 3$ `minimum_filter` to close small angular gaps on walls and slabs.
  5. Points behind first-hit surfaces by more than surface tolerance ($r > r_{\text{first}} + 0.40\text{m}$) are marked as occluded.
- Scale: Runs deterministically on $>150\text{M}$ points in $<1\text{s}$ per station using pure NumPy vectorization.

---

## 4. Sparse Inverted-Index Greedy Set Cover

### The Problem
When optimizing 18,731 candidate stations against 400,000 proxy points, allocating a dense boolean visibility matrix required:
$$18,731 \times 400,000 \text{ booleans} = 7.5\text{ GB}$$
Evaluating matrix bitwise operations on each greedy iteration allocated $\sim 19\text{ GB}$ of temporary arrays, causing virtual memory thrashing and stalling optimization for $>8$ hours.

### The Implementation (`simulation/visibility.py`)
- Built an **Inverted Index** mapping observable points directly to the candidate stations that observe them (`pt_to_candidates: List[List[int]]`).
- **Memory**: Decreased from $22\text{ GB}$ to $<50\text{ MB}$.
- **Performance**: Optimization time dropped from $8+$ hours to **$17$ seconds**, selecting the minimal set of stations required to achieve $99\%$ scene coverage with $\ge 40\%$ overlap continuity.
- **Progress Tracking**: Real-time logging emitted every 25 stations to guarantee UI visibility.

---

## 5. Synthetic 360° Equirectangular Panosphere Generation

### The Problem
E57 structured scan standards require valid `images2D` spherical projections for viewer compatibility (e.g., Autodesk ReCap, Leica Cyclone). When input clouds lack photographic panoramas (Geometry Mode or E57 without imagery), traditional tools omit `images2D`, breaking photorealistic panoramic inspection.

### The Implementation (`io_modules/exporter_e57.py`)
- Created `E57Exporter._generate_synthetic_pano_jpeg()`:
  1. Synthesize a $2048 \times 1024$ equirectangular panoramic image from the 3D point cloud coordinate and RGB/intensity values.
  2. Perform back-to-front painter's depth buffering to project points onto the spherical canvas.
  3. Fill sparse scan gaps using Euclidean Distance Transform inpainting (`scipy.ndimage.distance_transform_edt`).
  4. Encode compliant JPEG bytes and embed directly into the E57 `images2D` structure with matching pose coordinates.

---

## 6. Brand & UX Architecture (Artificial Vision Design System)

### The Implementation (`gui.py`)
- **Brand Tokens**: Converted the OKLCH design tokens from `prevdc.artificialviz.com` into CustomTkinter light/dark palettes:
  - Primary Brass/Gold: `#BDA469` (hover `#A68D52`, foreground text `#181818`)
  - Backgrounds: Light `#FAFAFA` / Dark `#0E0E0E`
  - Cards: Light `#FFFFFF` / Dark `#171717` (Border: `#E0E0E0` / `#292929`)
- **Adaptive Logo System**: Multi-layer transparent assets:
  - Light mode: Deep charcoal `#181818` `"AV"` monogram with `#BDA469` hexagonal lattice.
  - Dark mode: Crisp `#FAFAFA` `"AV"` monogram with `#BDA469` hexagonal lattice.
- **Native Taskbar & Window Management**: Restored standard Windows shell window handles, supporting minimization, Alt+Tab task switching, and dynamic theme toggling (`☾ / ☀`).
