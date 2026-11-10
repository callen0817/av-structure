import numpy as np
import open3d as o3d
import logging
from typing import List

logger = logging.getLogger(__name__)

class GeometryStationGenerator:
    def __init__(self, target_spacing: float = 3.0, tripod_height: float = 1.6):
        """
        Generates candidate stations unconditionally from raw point cloud geometry.
        """
        self.target_spacing = target_spacing
        self.tripod_height = tripod_height
        
    def generate(self, pcd: o3d.geometry.PointCloud) -> List[tuple]:
        """
        Computes free space and reasonable locations for scan placement
        based on local vertical geometric bounds.
        Returns a list of (X, Y, Z) tuples.
        """
        logger.info("Generating candidate stations purely from point cloud geometry...")
        
        voxel_size = 0.2
        downpcd = pcd.voxel_down_sample(voxel_size)
        points = np.asarray(downpcd.points)
        
        if len(points) == 0:
            return []
            
        min_bound = points.min(axis=0)
        max_bound = points.max(axis=0)
        
        x_steps = np.arange(min_bound[0], max_bound[0], self.target_spacing)
        y_steps = np.arange(min_bound[1], max_bound[1], self.target_spacing)
        
        candidates = []
        search_radius = self.target_spacing / 2.0
        
        # Grid loop over floor plan
        for x in x_steps:
            for y in y_steps:
                mask = np.logical_and(np.abs(points[:, 0] - x) < search_radius, np.abs(points[:, 1] - y) < search_radius)
                local_points = points[mask]
                
                if len(local_points) > 10:
                    floor_z = np.percentile(local_points[:, 2], 5)
                    ceil_z = np.percentile(local_points[:, 2], 95)
                    
                    if ceil_z - floor_z > 2.0:
                        station_pos = (x, y, floor_z + self.tripod_height)
                        candidates.append(station_pos)
                        
        logger.info(f"Generated {len(candidates)} geometric candidate stations.")
        return candidates
