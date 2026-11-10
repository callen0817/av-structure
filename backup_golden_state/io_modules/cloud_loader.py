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
            # A unified SLAM point cloud typically occupies the first scan block
            data = e57.read_scan(0, colors=True, ignore_missing_fields=True)
            
            pcd = o3d.geometry.PointCloud()
            # Construct XYZ array
            points = np.vstack((data.get("cartesianX", []), 
                                data.get("cartesianY", []), 
                                data.get("cartesianZ", []))).T
            pcd.points = o3d.utility.Vector3dVector(points)
            
            # Construct RGB array if available
            if "colorRed" in data and "colorGreen" in data and "colorBlue" in data:
                colors = np.vstack((data["colorRed"], data["colorGreen"], data["colorBlue"])).T
                if colors.max() > 1.0:
                    colors = colors / 255.0
                pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # Handle potential classification/scalar fields in E57
            # Standard scalar fields often used for intensities or classifications
            if "intensity" in data:
                extra_data["intensity"] = data["intensity"]
            # Look for common custom classification fields
            for key in data.keys():
                if "class" in key.lower():
                    extra_data["classification"] = data[key]
                    break
        else:
            # PLY, PCD, etc
            pcd = o3d.io.read_point_cloud(self.file_path)
            
        if pcd.is_empty():
            logger.warning("Loaded point cloud is empty!")
        else:
            logger.info(f"Successfully loaded unified point cloud with {len(pcd.points)} points.")
            
        return pcd, extra_data

