import os
import logging
import numpy as np
import open3d as o3d
from config import LARGE_COORDINATE_THRESHOLD

logger = logging.getLogger(__name__)


class CloudLoader:
    def __init__(self, file_path: str):
        """
        Loads point cloud files relying on Open3D, pye57, laspy, or vectorized ASCII (.pts/.xyz) parsing.
        """
        self.file_path = file_path

    @staticmethod
    def _parse_ascii_cloud(file_path: str, ext: str):
        """
        Parses .pts, .xyz, .xyzrgb, .txt, .asc point cloud files.
        Supports optional leading integer point-count header line (standard Leica .pts)
        and 3 (XYZ), 4 (XYZI), 6 (XYZRGB), or 7+ (XYZIRGB) columns.
        """
        rows = []
        with open(file_path, "r", encoding="utf-8", errors="replace") as f:
            for line_idx, raw_line in enumerate(f):
                line = raw_line.strip()
                if not line or line.startswith("#") or line.startswith("//"):
                    continue
                # Replace commas or semicolons with spaces
                if "," in line or ";" in line:
                    line = line.replace(",", " ").replace(";", " ")
                parts = line.split()
                # Leica .pts files often have a single integer count on the first non-empty line
                if len(parts) == 1 and len(rows) == 0:
                    try:
                        int(parts[0])
                        continue
                    except ValueError:
                        pass
                if len(parts) < 3:
                    continue
                try:
                    vals = [float(x) for x in parts[:7]]
                    rows.append(vals)
                except ValueError:
                    # Skip non-numeric header lines on the first few lines
                    if line_idx < 5:
                        continue
                    raise ValueError(f"Malformed coordinate row at line {line_idx + 1} in {file_path}")

        if not rows:
            return np.empty((0, 3), dtype=np.float64), None, None

        # Normalize row lengths to the modal column count
        col_counts = [len(r) for r in rows]
        min_cols = min(col_counts)
        arr = np.array([r[:min_cols] for r in rows], dtype=np.float64)

        pts = arr[:, :3]
        colors = None
        intensity = None

        if min_cols == 4:
            intensity = arr[:, 3].astype(np.float32)
        elif min_cols == 6:
            rgb = arr[:, 3:6]
            if rgb.max() > 1.0:
                rgb = np.clip(rgb / 255.0, 0.0, 1.0)
            else:
                rgb = np.clip(rgb, 0.0, 1.0)
            colors = rgb
        elif min_cols >= 7:
            intensity = arr[:, 3].astype(np.float32)
            rgb = arr[:, 4:7]
            if rgb.max() > 1.0:
                rgb = np.clip(rgb / 255.0, 0.0, 1.0)
            else:
                rgb = np.clip(rgb, 0.0, 1.0)
            colors = rgb

        return pts, colors, intensity

    @staticmethod
    def _parse_flash_anchors(anchors_path: str):
        if not os.path.exists(anchors_path):
            return []
        try:
            import json
            with open(anchors_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            raw_list = data
            if isinstance(data, dict):
                raw_list = data.get("anchors") or data.get("positions") or data.get("points") or []
            anchors = []
            for item in raw_list:
                if isinstance(item, (list, tuple)) and len(item) >= 3:
                    anchors.append((float(item[0]), float(item[1]), float(item[2])))
                elif isinstance(item, dict):
                    x = item.get("x", item.get("X"))
                    y = item.get("y", item.get("Y"))
                    z = item.get("z", item.get("Z"))
                    if x is not None and y is not None and z is not None:
                        anchors.append((float(x), float(y), float(z)))
            return anchors
        except Exception as e:
            logger.warning(f"Could not parse flash anchors from {anchors_path}: {e}")
            return []

    @staticmethod
    def _parse_keyframes_manifest(manifest_path: str):
        if not os.path.exists(manifest_path):
            return []
        try:
            import json
            with open(manifest_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            raw_kfs = data if isinstance(data, list) else data.get("keyframes", [])
            base_dir = os.path.dirname(os.path.abspath(manifest_path))
            parsed = []
            for kf in raw_kfs:
                img_path = kf.get("image_path") or kf.get("file") or kf.get("path")
                if img_path and not os.path.isabs(img_path):
                    img_path = os.path.join(base_dir, img_path)
                item = dict(kf)
                item["image_path"] = img_path
                parsed.append(item)
            return parsed
        except Exception as e:
            logger.warning(f"Could not parse keyframes manifest from {manifest_path}: {e}")
            return []

    def _load_archive_or_session(self, root_path: str, is_zip: bool = False):
        import glob
        import zipfile
        import tempfile
        import json

        temp_dir = None
        work_dir = root_path
        if is_zip:
            temp_dir = tempfile.mkdtemp(prefix="av_session_")
            with zipfile.ZipFile(root_path, "r") as zf:
                zf.extractall(temp_dir)
            work_dir = temp_dir

        # Look for point clouds inside work_dir (priority order)
        cloud_patterns = [
            "pointcloud_fused_rgb.ply",
            "cleaned.ply",
            "pointcloud*.ply",
            "*.ply",
            "*.pcd",
            "*.xyz",
            "*.xyzrgb",
            "*.e57",
            "*.las",
            "*.laz",
            "*.pts",
        ]
        found_cloud = None
        for pat in cloud_patterns:
            matches = glob.glob(os.path.join(work_dir, "**", pat), recursive=True)
            if matches:
                # Prefer files not in hidden or temp folders
                matches = [m for m in matches if "/." not in m]
                if matches:
                    found_cloud = matches[0]
                    break

        if not found_cloud:
            raise FileNotFoundError(f"No supported 3D point cloud file found inside archive/session directory: {root_path}")

        logger.info(f"Discovered primary point cloud inside session: {found_cloud}")
        inner_loader = CloudLoader(found_cloud)
        pcd, extra_data = inner_loader.load()

        extra_data["session_dir"] = work_dir
        extra_data["is_session"] = True

        # Check for AV Scan / AV ScanAR flash anchors
        anchor_files = glob.glob(os.path.join(work_dir, "**", "flash_anchors.json"), recursive=True)
        if anchor_files:
            anchors = self._parse_flash_anchors(anchor_files[0])
            if anchors:
                extra_data["flash_anchors"] = anchors
                logger.info(f"Loaded {len(anchors)} user-pinned flash anchors from {anchor_files[0]}.")

        # Check for trajectory.csv
        traj_files = glob.glob(os.path.join(work_dir, "**", "trajectory.csv"), recursive=True)
        if traj_files:
            extra_data["trajectory_csv"] = traj_files[0]
            logger.info(f"Discovered session trajectory CSV: {traj_files[0]}")

        # Check for keyframes manifest
        kf_files = glob.glob(os.path.join(work_dir, "**", "keyframes_manifest.json"), recursive=True)
        if kf_files:
            kfs = self._parse_keyframes_manifest(kf_files[0])
            if kfs:
                extra_data["keyframes"] = kfs
                logger.info(f"Loaded {len(kfs)} posed narrow-FOV keyframes from {kf_files[0]}.")

        # Check for Matterport MatterPak manifest & floorplans
        manifest_files = glob.glob(os.path.join(work_dir, "**", "bundle_manifest.json"), recursive=True)
        if manifest_files:
            try:
                with open(manifest_files[0], "r", encoding="utf-8") as mf:
                    extra_data["bundle_manifest"] = json.load(mf)
                    extra_data["is_matterpak"] = True
                    logger.info(f"Identified Matterport MatterPak bundle manifest: {manifest_files[0]}")
            except Exception as me:
                logger.warning(f"Could not read bundle_manifest.json: {me}")

        floorplan_patterns = ["floorplan*.png", "floorplan*.jpg", "floor_*.png", "floor_*.jpg"]
        found_plans = {}
        plan_idx = 1
        for fpat in floorplan_patterns:
            for pmatch in glob.glob(os.path.join(work_dir, "**", fpat), recursive=True):
                found_plans[plan_idx] = pmatch
                plan_idx += 1
        if found_plans:
            extra_data["matterpak_floorplans"] = found_plans
            logger.info(f"Discovered {len(found_plans)} architectural floorplan images in session.")

        return pcd, extra_data

    def load(self):
        """
        Loads the unified point cloud and returns both the pcd and a metadata dict for scalar fields.
        Returns: (o3d.geometry.PointCloud, dict)
        """
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"Point cloud file not found: {self.file_path}")

        ext = os.path.splitext(self.file_path)[1].lower()

        # Handle directory or zip archive inputs (MatterPak, AV Scan session)
        if os.path.isdir(self.file_path):
            return self._load_archive_or_session(self.file_path, is_zip=False)
        elif ext == ".zip":
            return self._load_archive_or_session(self.file_path, is_zip=True)

        if os.path.getsize(self.file_path) == 0:
            raise ValueError(f"Point cloud file is empty (0 bytes): {self.file_path}")

        logger.info(f"Loading point cloud from {self.file_path}...")

        extra_data = {
            "classification": None,
            "intensity": None,
            "has_rgb": False,
            "has_intensity": False,
            "has_classification": False,
            "source_format": ext,
            "global_shift": np.zeros(3, dtype=np.float64),
            "large_coordinates": False,
            "invalid_points_removed": 0,
        }

        if ext in [".las", ".laz"]:
            try:
                import laspy
            except ImportError:
                raise ImportError("Please install laspy to support LAS/LAZ format.")

            try:
                las = laspy.read(self.file_path)
            except Exception as e:
                raise ValueError(f"Failed to read LAS/LAZ file {self.file_path}: {e}") from e

            pcd = o3d.geometry.PointCloud()
            pts = np.vstack((np.asarray(las.x, dtype=np.float64),
                             np.asarray(las.y, dtype=np.float64),
                             np.asarray(las.z, dtype=np.float64))).T
            pcd.points = o3d.utility.Vector3dVector(pts)

            dim_names = set(las.point_format.dimension_names) if hasattr(las, "point_format") else set()
            if {"red", "green", "blue"}.issubset(dim_names) or (
                hasattr(las, "red") and hasattr(las, "green") and hasattr(las, "blue")
            ):
                try:
                    colors = np.vstack((las.red, las.green, las.blue)).T.astype(np.float64)
                    if len(colors) > 0 and colors.max() > 0:
                        if colors.max() > 255:
                            colors = colors / 65535.0
                        else:
                            colors = colors / 255.0
                        pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
                        extra_data["has_rgb"] = True
                except Exception:
                    pass

            if "classification" in dim_names or hasattr(las, "classification"):
                try:
                    cls_arr = np.array(las.classification)
                    if len(cls_arr) == len(pts):
                        extra_data["classification"] = cls_arr
                        extra_data["has_classification"] = True
                except Exception:
                    pass

            if "intensity" in dim_names or hasattr(las, "intensity"):
                try:
                    int_arr = np.array(las.intensity, dtype=np.float32)
                    if len(int_arr) == len(pts):
                        extra_data["intensity"] = int_arr
                        extra_data["has_intensity"] = bool(np.any(int_arr != 0))
                except Exception:
                    pass

        elif ext == ".e57":
            try:
                import pye57
            except ImportError:
                raise ImportError("Please install pye57 to support E57 format.")

            try:
                e57 = pye57.E57(self.file_path)
            except Exception as e:
                raise ValueError(f"Corrupted or unreadable E57 file {self.file_path}: {e}") from e

            if hasattr(e57, "scan_count"):
                n_scans = (
                    e57.scan_count
                    if isinstance(e57.scan_count, int)
                    else (e57.scan_count() if callable(e57.scan_count) else 1)
                )
            else:
                n_scans = 1

            all_points = []
            all_colors = []
            all_intensity = []
            all_class = []
            has_rgb = True
            has_int = True
            has_cls = True

            for scan_idx in range(max(1, n_scans)):
                try:
                    data = e57.read_scan(
                        scan_idx, colors=True, intensity=True, transform=True, ignore_missing_fields=True
                    )
                except TypeError:
                    data = e57.read_scan(scan_idx, colors=True, ignore_missing_fields=True)
                except Exception as scan_err:
                    logger.warning(f"Could not read scan {scan_idx} via read_scan ({scan_err}), trying read_scan_raw...")
                    raw = e57.read_scan_raw(scan_idx, ignore_unsupported_fields=True)
                    hdr = e57.get_header(scan_idx)
                    pts_local = np.vstack((raw["cartesianX"], raw["cartesianY"], raw["cartesianZ"])).T
                    if hdr.has_pose():
                        pts_global = e57.to_global(pts_local, hdr.rotation, hdr.translation)
                    else:
                        pts_global = pts_local
                    data = {
                        "cartesianX": pts_global[:, 0],
                        "cartesianY": pts_global[:, 1],
                        "cartesianZ": pts_global[:, 2],
                    }
                    for k in ("colorRed", "colorGreen", "colorBlue", "intensity", "cartesianInvalidState"):
                        if k in raw:
                            data[k] = raw[k]

                x = data.get("cartesianX", [])
                if len(x) == 0:
                    continue
                pts = np.vstack((x, data.get("cartesianY", []), data.get("cartesianZ", []))).T.astype(np.float64)

                # Filter out points where cartesianInvalidState != 0 and exact (0,0,0) origin points
                invalid_mask = np.zeros(len(pts), dtype=bool)
                if "cartesianInvalidState" in data:
                    invalid_mask = invalid_mask | (np.asarray(data["cartesianInvalidState"]) != 0)
                if "is_invalid" in data:
                    invalid_mask = invalid_mask | (np.asarray(data["is_invalid"]) != 0)
                zero_mask = (pts[:, 0] == 0) & (pts[:, 1] == 0) & (pts[:, 2] == 0)
                invalid_mask = invalid_mask | zero_mask

                valid_mask = ~invalid_mask
                if np.any(invalid_mask):
                    extra_data["invalid_points_removed"] += int(np.sum(invalid_mask))
                    pts = pts[valid_mask]
                    if len(pts) == 0:
                        continue

                all_points.append(pts)

                if has_rgb and "colorRed" in data and "colorGreen" in data and "colorBlue" in data:
                    cols = np.vstack((data["colorRed"], data["colorGreen"], data["colorBlue"])).T.astype(np.float64)
                    if len(cols) == len(invalid_mask):
                        cols = cols[valid_mask]
                    if len(cols) > 0 and cols.max() > 1.0:
                        cols = cols / 255.0
                    all_colors.append(np.clip(cols, 0.0, 1.0))
                else:
                    has_rgb = False

                if has_int and "intensity" in data:
                    ints = np.asarray(data["intensity"], dtype=np.float32)
                    if len(ints) == len(invalid_mask):
                        ints = ints[valid_mask]
                    all_intensity.append(ints)
                else:
                    has_int = False

                cls_key = next((k for k in data.keys() if "class" in k.lower()), None)
                if has_cls and cls_key is not None:
                    clss = np.asarray(data[cls_key])
                    if len(clss) == len(invalid_mask):
                        clss = clss[valid_mask]
                    all_class.append(clss)
                else:
                    has_cls = False

            if hasattr(e57, "close"):
                e57.close()

            pcd = o3d.geometry.PointCloud()
            if all_points:
                points = np.vstack(all_points) if len(all_points) > 1 else all_points[0]
                pcd.points = o3d.utility.Vector3dVector(points)

                if has_rgb and len(all_colors) == len(all_points):
                    colors = np.vstack(all_colors) if len(all_colors) > 1 else all_colors[0]
                    pcd.colors = o3d.utility.Vector3dVector(colors)
                    extra_data["has_rgb"] = True

                if has_int and len(all_intensity) == len(all_points):
                    extra_data["intensity"] = (
                        np.concatenate(all_intensity) if len(all_intensity) > 1 else all_intensity[0]
                    )
                    extra_data["has_intensity"] = True

                if has_cls and len(all_class) == len(all_points):
                    extra_data["classification"] = (
                        np.concatenate(all_class) if len(all_class) > 1 else all_class[0]
                    )
                    extra_data["has_classification"] = True

        elif ext in [".pts", ".xyz", ".xyzrgb", ".txt", ".asc"]:
            pts, cols, ints = self._parse_ascii_cloud(self.file_path, ext)
            pcd = o3d.geometry.PointCloud()
            if len(pts) > 0:
                pcd.points = o3d.utility.Vector3dVector(pts)
                if cols is not None and len(cols) == len(pts):
                    pcd.colors = o3d.utility.Vector3dVector(cols)
                    extra_data["has_rgb"] = True
                if ints is not None and len(ints) == len(pts):
                    extra_data["intensity"] = ints
                    extra_data["has_intensity"] = True

        elif ext == ".bin":
            import struct
            pcd = o3d.geometry.PointCloud()
            with open(self.file_path, "rb") as f:
                header = f.read(4)
                if len(header) == 4:
                    num_pts = struct.unpack("<I", header)[0]
                    record_dtype = np.dtype([
                        ("x", "<f4"),
                        ("y", "<f4"),
                        ("z", "<f4"),
                        ("r", "u1"),
                        ("g", "u1"),
                        ("b", "u1"),
                    ])
                    raw_data = np.fromfile(f, dtype=record_dtype, count=num_pts)
                    pts = np.column_stack((raw_data["x"], raw_data["y"], raw_data["z"])).astype(np.float64)
                    cols = np.column_stack((raw_data["r"], raw_data["g"], raw_data["b"])).astype(np.float64) / 255.0
                    pcd.points = o3d.utility.Vector3dVector(pts)
                    pcd.colors = o3d.utility.Vector3dVector(cols)
                    extra_data["has_rgb"] = True

        else:
            # PLY, PCD, etc.
            pcd = o3d.io.read_point_cloud(self.file_path)
            if pcd.has_colors():
                extra_data["has_rgb"] = True

        # Filter out non-finite (NaN / Inf) coordinates across all attributes
        if not pcd.is_empty():
            pts_np = np.asarray(pcd.points)
            finite_mask = np.all(np.isfinite(pts_np), axis=1)
            if not np.all(finite_mask):
                removed = int(np.sum(~finite_mask))
                extra_data["invalid_points_removed"] = removed
                logger.warning(f"Filtered out {removed} points with non-finite (NaN/Inf) coordinates.")
                pts_np = pts_np[finite_mask]
                pcd.points = o3d.utility.Vector3dVector(pts_np)
                if pcd.has_colors():
                    cols_np = np.asarray(pcd.colors)[finite_mask]
                    pcd.colors = o3d.utility.Vector3dVector(cols_np)
                if extra_data["intensity"] is not None:
                    extra_data["intensity"] = extra_data["intensity"][finite_mask]
                if extra_data["classification"] is not None:
                    extra_data["classification"] = extra_data["classification"][finite_mask]

            if len(pts_np) > 0:
                max_abs_coord = float(np.max(np.abs(pts_np)))
                if max_abs_coord > LARGE_COORDINATE_THRESHOLD:
                    centroid_shift = np.median(pts_np, axis=0)
                    extra_data["global_shift"] = centroid_shift
                    extra_data["large_coordinates"] = True
                    logger.info(
                        f"Detected large global coordinates (max={max_abs_coord:,.2f}m). "
                        f"Using numerically stable local processing frame centered at ({centroid_shift[0]:.2f}, {centroid_shift[1]:.2f}, {centroid_shift[2]:.2f})."
                    )

        if pcd.is_empty():
            logger.warning("Loaded point cloud is empty!")
        else:
            logger.info(
                f"Successfully loaded unified point cloud with {len(pcd.points):,} points "
                f"(RGB={extra_data['has_rgb']}, Intensity={extra_data['has_intensity']})."
            )

        return pcd, extra_data


def load_point_cloud(file_path: str):
    """
    Convenience wrapper around CloudLoader(file_path).load() returning
    (xyz_numpy_float64, extra_data_dict).
    """
    pcd, extra_data = CloudLoader(file_path).load()
    pts = np.asarray(pcd.points, dtype=np.float64)
    if len(pts) == 0:
        raise ValueError(f"Point cloud file contains 0 valid points: {file_path}")
    if pcd.has_colors() and len(pcd.colors) == len(pts):
        extra_data["rgb"] = np.clip(np.round(np.asarray(pcd.colors) * 255.0), 0, 255).astype(np.uint8)
        extra_data["has_rgb"] = True
    return pts, extra_data



