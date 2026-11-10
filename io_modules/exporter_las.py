import laspy
import numpy as np
import open3d as o3d
import logging
from typing import List, Dict, Optional

logger = logging.getLogger(__name__)

class LASExporter:
    def __init__(self, output_path: str):
        self.output_path = output_path
        
    def export(self, pcd: o3d.geometry.PointCloud, scans: Dict[int, np.ndarray], 
               extra_data: Optional[Dict[str, np.ndarray]] = None):
        """
        Exports the point cloud to a LAS/LAZ file.
        In LAS format, we can keep the points unified or split by 'user_data'/fields if needed,
        but typically we export the whole thing with classification preserved.
        """
        logger.info(f"Exporting LAS cloud to: {self.output_path}")
        
        points = np.asarray(pcd.points)
        has_colors = pcd.has_colors()
        
        # Create a new LAS file
        # We use point format 3 or 7 to support colors
        header = laspy.LasHeader(point_format=3, version="1.2")
        las = laspy.LasData(header)
        
        las.x = points[:, 0]
        las.y = points[:, 1]
        las.z = points[:, 2]
        
        if has_colors:
            colors = (np.asarray(pcd.colors) * 65535.0).astype(np.uint16)
            las.red = colors[:, 0]
            las.green = colors[:, 1]
            las.blue = colors[:, 2]
            
        if extra_data:
            if extra_data.get("classification") is not None:
                las.classification = extra_data["classification"].astype(np.uint8)
            if extra_data.get("intensity") is not None:
                las.intensity = extra_data["intensity"].astype(np.uint16)
                
        las.write(self.output_path)
        logger.info(f"LAS Export successfully finalized: {len(points)} points.")
