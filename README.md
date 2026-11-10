# Artificial Vision — AV Structure

**AV Structure** is an automated spatial computing pipeline designed to convert massive unstructured mobile SLAM point clouds (NavVis VLX/M6, GeoSLAM, Leica BLK2GO, handheld lidar) into industry-standard **Structured E57** datasets featuring true spherical grid topologies, optimal virtual scanner station coverage, and embedded 360° panoramic imagery.

---

## Key Capabilities

* **Multi-Workflow Ingestion**:
  * **Auto-Detect / Heuristic Geometry**: Automatically slices building floor levels via vertical Z-histogram floor discovery and places candidates across navigable walking surfaces.
  * **E57 + Bundle / Reconstruction File**: Directly parses NavVis `.bundle` archives to extract embedded camera trajectory matrices and high-resolution panoramic frames.
  * **E57 + Pose CSV / Images**: Ingests custom CSV trajectory files with external panoramic camera directories.
* **Virtual Scanner Simulation**:
  * Ray-casts spherical depth buffers ($360^\circ \times 180^\circ$) to reconstruct true terrestrial scanner grid structures (`r, theta, phi`).
  * Generates synthetic equirectangular $360^\circ$ panoramic spheres when physical imagery is absent, projecting point cloud RGB and intensity data into spherical texture maps.
* **Visibility Optimization**:
  * Sparse inverted-index greedy set cover algorithm selects the minimal subset of candidate stations required to achieve target point cloud coverage (e.g., 85%–95%).
* **Modern Desktop GUI**:
  * CustomTkinter dark/light interface aligned with the Artificial Vision brand identity.

---

## Getting Started & Local Rebuild

For full instructions on setting up your local environment, rebuilding binary executables, and acquiring heavy datasets (e.g. NavVis bundles and point clouds), please read:

👉 **[EXCLUSIONS.md](EXCLUSIONS.md)** — **Exclusions & Local Reconstitution Guide**

### Quick Start:
```powershell
# 1. Setup virtual environment
python -m venv .venv_310
.\.venv_310\Scripts\activate
pip install -r requirements.txt

# 2. Launch GUI
.\run.bat
```

### Building the Standalone Executable:
```powershell
.\build.bat
```

---

## Architectural History & Design Records

Complete implementation plans, technical walkthroughs, large-scale dataset audit reports, and full conversation transcripts are preserved in:
* [`docs/antigravity-history/`](docs/antigravity-history/README.md)
