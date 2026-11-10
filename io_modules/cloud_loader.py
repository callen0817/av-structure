import os
import logging
import numpy as np
import open3d as o3d

logger = logging.getLogger(__name__)

class CloudLoader:
    def __init__(self, file_path: str):
        """
        Loads point cloud files iteratively or in full relying on Open3D, pye57, or laspy.
        """
        self.file_path = file_path
        
    def load(self):
        """
        Loads the unified point cloud and returns both the pcd and a metadata dict for scalar fields.
        Returns: (o3d.geometry.PointCloud, dict)
        """
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"Point cloud file not found: {self.file_path}")
            
        ext = os.path.splitext(self.file_path)[1].lower()
        
        logger.info(f"Loading point cloud from {self.file_path}...")
        
        extra_data = {
            "classification": None,
            "intensity": None
        }
        
        if ext in ['.las', '.laz']:
            try:
                import laspy
            except ImportError:
                raise ImportError("Please install laspy to support LAS/LAZ format.")
                
            las = laspy.read(self.file_path)
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.vstack((las.x, las.y, las.z)).T)
            
            if hasattr(las, 'red') and hasattr(las, 'green') and hasattr(las, 'blue'):
                colors = np.vstack((las.red, las.green, las.blue)).T
                # LAS colors are often 16-bit
                if colors.max() > 255:
                    colors = colors / 65535.0
                else:
                    colors = colors / 255.0
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            if hasattr(las, 'classification'):
                extra_data["classification"] = np.array(las.classification)
            if hasattr(las, 'intensity'):
                extra_data["intensity"] = np.array(las.intensity)
                
        elif ext == '.e57':
            try:
                import pye57
            except ImportError:
                raise ImportError("Please install pye57 to support E57 format.")
                
            e57 = pye57.E57(self.file_path)
            if hasattr(e57, "scan_count"):
                n_scans = e57.scan_count if isinstance(e57.scan_count, int) else (e57.scan_count() if callable(e57.scan_count) else 1)
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
                    data = e57.read_scan(scan_idx, colors=True, intensity=True, transform=True, ignore_missing_fields=True)
                except TypeError:
                    data = e57.read_scan(scan_idx, colors=True, ignore_missing_fields=True)

                x = data.get("cartesianX", [])
                if len(x) == 0:
                    continue
                pts = np.vstack((x, data.get("cartesianY", []), data.get("cartesianZ", []))).T
                all_points.append(pts)

                if has_rgb and "colorRed" in data and "colorGreen" in data and "colorBlue" in data:
                    cols = np.vstack((data["colorRed"], data["colorGreen"], data["colorBlue"])).T
                    if cols.max() > 1.0:
                        cols = cols / 255.0
                    all_colors.append(cols)
                else:
                    has_rgb = False

                if has_int and "intensity" in data:
                    all_intensity.append(data["intensity"])
                else:
                    has_int = False

                cls_key = next((k for k in data.keys() if "class" in k.lower()), None)
                if has_cls and cls_key is not None:
                    all_class.append(data[cls_key])
                else:
                    has_cls = False

            pcd = o3d.geometry.PointCloud()
            if all_points:
                points = np.vstack(all_points) if len(all_points) > 1 else all_points[0]
                pcd.points = o3d.utility.Vector3dVector(points)

                if has_rgb and len(all_colors) == len(all_points):
                    colors = np.vstack(all_colors) if len(all_colors) > 1 else all_colors[0]
                    pcd.colors = o3d.utility.Vector3dVector(colors)

                if has_int and len(all_intensity) == len(all_points):
                    extra_data["intensity"] = np.concatenate(all_intensity) if len(all_intensity) > 1 else all_intensity[0]

                if has_cls and len(all_class) == len(all_points):
                    extra_data["classification"] = np.concatenate(all_class) if len(all_class) > 1 else all_class[0]
        else:
            # PLY, PCD, etc
            pcd = o3d.io.read_point_cloud(self.file_path)
            
        if pcd.is_empty():
            logger.warning("Loaded point cloud is empty!")
        else:
            logger.info(f"Successfully loaded unified point cloud with {len(pcd.points)} points.")
            
        return pcd, extra_data

