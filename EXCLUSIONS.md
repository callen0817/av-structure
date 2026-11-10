# Project Exclusions & Local Reconstitution Guide

This document provides a comprehensive inventory of all files, directories, datasets, and binaries excluded from the Git repository (via `.gitignore`), categorizing them into:
1. **Heavy Operational Requirements** (how to acquire or rebuild them locally for full functionality).
2. **Local Environment & Binary Compilation** (how to rebuild the Python virtual environment and the standalone PyInstaller executable).
3. **Transient Test & Runtime Artifacts** (temporary files, scratch directories, and output logs that are intentionally omitted).

---

## 1. Quick Summary of Ignored Items

| Path / Pattern | Classification | Required for Operation? | How to Rebuild / Source Locally |
| :--- | :--- | :--- | :--- |
| `.venv/`, `.venv_310/` | Environment | **Yes** (if running from source) | `pip install -r requirements.txt` (Python 3.10 x64) |
| `dist/`, `build/` | Binary Output | **Optional** (only for standalone .exe) | Run `build.bat` using PyInstaller and `converter.spec` |
| `*.bundle` (e.g., `260225_555_W_22_*.bundle` ~3.3 GB) | Operational Data | **Yes** (for NavVis Bundle workflow) | Sourced from NavVis VLX / M6 export drive |
| `*.e57`, `*.las`, `*.laz` | Operational Data | **Yes** (as processing inputs) | Sourced from SLAM capture drive or network storage |
| `output/`, `output/Job_*/` | Runtime Output | No (generated dynamically) | Automatically created when running conversions |
| `scratch/`, `golden_dataset/`, `expected_output/` | Test / Scratch | No (transient test data) | Created during unit/regression testing |
| `*.log`, `crash*.txt` | Logs & Dumps | No (transient diagnostics) | Generated at runtime when debugging |
| `__pycache__/`, `*.pyc` | Bytecode Cache | No | Generated automatically by Python |

---

## 2. Heavy Operational Requirements & Datasets

### A. NavVis Alignment Bundles (`*.bundle` / `*.zip`)
* **What it is**: NavVis VLX/M6 hardware datasets package camera calibration, trajectory telemetry, and high-resolution spherical camera frames into `.bundle` ZIP archives (e.g., `260225_555_W_22_2026-03-06.bundle`, ~3.3 GB).
* **Operational Role**:
  * Required when using the **"E57 + Bundle / Reconstruction File"** workflow in the GUI or passing `--alignment_file` in the CLI (`virtual_scanner.py`).
  * `io_modules/navvis_parser.py` extracts `pano-poses.csv` and reads embedded panoramic JPEG images (`pano_cam*.jpg`) directly from the bundle zip stream without unpacking to disk.
* **How to Supply Locally**:
  * Place the `.bundle` file in the project root or browse to its location on your external SSD / project network share in the GUI.
* **Alternative (if missing)**:
  * If a `.bundle` is not available, you can use:
    1. **"Auto-Detect / Heuristic Geometry"** workflow (derives candidate station locations directly from the point cloud floor geometry and generates synthetic panoramic imagery).
    2. **"E57 + Pose CSV"** workflow (uses standalone camera pose CSVs such as `pano-poses.csv` alongside an images folder).

### B. Input Point Clouds (`*.e57`, `*.las`, `*.laz`)
* **What it is**: Large unstructured or unified SLAM point clouds (e.g. `BuildingsMergeColor_subsampled.e57`, 159.6M points, ~5–50 GB).
* **Operational Role**: Primary input dataset passed to `io_modules/cloud_loader.py` via GUI or CLI (`--input_cloud`).
* **Storage Recommendation**: Store on a fast local NVMe SSD (e.g., `C:/Data/...` or high-throughput network share `V:/...`) to ensure maximum ray-casting and voxelization I/O performance.

---

## 3. Local Environment & Binary Rebuild Guide

### A. Rebuilding the Python Virtual Environment
The project relies on native C/C++ bindings (`open3d`, `pye57`, `pdal`, `scipy`). **Python 3.10 (64-bit)** is strongly recommended for Windows binary wheel compatibility.

To recreate the local environment from scratch:

```powershell
# 1. Open PowerShell in the project root directory
cd "c:\path\to\AV_structure"

# 2. Create the Python 3.10 virtual environment
py -3.10 -m venv .venv_310

# 3. Activate the environment
.\.venv_310\Scripts\activate

# 4. Upgrade pip and build tools
python -m pip install --upgrade pip setuptools wheel

# 5. Install all required dependencies
pip install -r requirements.txt
```

#### Required System Prerequisites (Windows)
* **Microsoft Visual C++ 2015–2022 Redistributable (x64)** (required by Open3D and pye57 C++ extensions).
* If installing `pdal` or `pye57` on a machine without prebuilt wheels, ensure the **Visual Studio C++ Build Tools** are available.

---

### B. Rebuilding the Standalone Executable (`dist/AV_Structure_Converter.exe`)
The standalone executable bundle allows users to run AV Structure without having Python installed.

To rebuild the binary bundle locally:

```powershell
# Ensure the virtual environment is activated and pyinstaller is installed
.\.venv_310\Scripts\activate
pip install pyinstaller

# Run the build script
.\build.bat
```

Or invoke PyInstaller directly with the tuned specification file:
```powershell
.\.venv_310\Scripts\pyinstaller --clean converter.spec
```

* **Build Output**: Generates `dist/AV_Structure_Converter/AV_Structure_Converter.exe` along with its `_internal` dependency tree.
* **Spec Configuration**: [`converter.spec`](file:///c:/Users/ChristopherAllen/OneDrive%20-%20Reisource/Desktop/AV_structure/converter.spec) automatically bundles CustomTkinter themes, Dark/Light brand logos from `assets/`, Open3D dynamic libraries, and hidden imports for `pye57` and `scipy`.

---

## 4. Launching the Application Locally

### Using the Batch Launcher:
Double-click [`run.bat`](file:///c:/Users/ChristopherAllen/OneDrive%20-%20Reisource/Desktop/AV_structure/run.bat) or run in PowerShell:
```powershell
.\run.bat
```

### Using the Python Environment:
```powershell
.\.venv_310\Scripts\python.exe gui.py
```

### Using the CLI for Headless Batch Processing:
```powershell
.\.venv_310\Scripts\python.exe virtual_scanner.py `
  --input_cloud "path/to/input.e57" `
  --output_file "path/to/output_structured.e57" `
  --mode "geometry" `
  --grid_spacing 2.0 `
  --min_coverage 0.85
```

---

## 5. Transient & Test Artifacts (Safely Ignored)

The following items are purely transient and **not** required for production or operation:
* `scratch/`: Contains temporary test scripts (e.g., `test_phase1.py`), dummy panorama folders, and one-off helper scripts.
* `golden_dataset/` and `expected_output/`: Empty local benchmark test stubs.
* `output/`: Holds job folders (`Job_001`, `Job_002`) and exported structured E57 files produced during runs.
* `crash.txt`, `crash_out.txt`, `output.log`, `error.log`, `*.log`: Local debug logs and stdout/stderr redirects.
* `__pycache__/`: Python compiled bytecode.
