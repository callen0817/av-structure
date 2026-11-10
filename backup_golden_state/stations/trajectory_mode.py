import numpy as np
import open3d as o3d
import logging
from typing import List
from io_modules.navvis_parser import PanoPose

logger = logging.getLogger(__name__)

class TrajectoryStationGenerator:
    def __init__(self, target_spacing: float = 3.0):
        """
        Initializes the generator with a specific target spacing in meters.
        """
        self.target_spacing = target_spacing
        
    def generate(self, poses: List[PanoPose]) -> List[PanoPose]:
        """
        Clusters dense trajectory poses into candidate scan stations
        using spatial voxel downsampling.
        """
        if not poses:
            logger.warning("No poses provided for trajectory scanning.")
            return []
            
        coords = np.array([p.position for p in poses])
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(coords)
        
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        
        logger.info(f"Voxel downsampling trajectory with spacing: {self.target_spacing}m")
        downpcd = pcd.voxel_down_sample(voxel_size=self.target_spacing)
        down_coords = np.asarray(downpcd.points)
        
        logger.info(f"Reduced {len(poses)} trajectory poses to {len(down_coords)} candidate stations.")
        
        selected_poses = []
        selected_indices = set()
        
        # Find original pose nearest to each voxel center to preserve attributes
        for p in down_coords:
            [k, idx, _] = pcd_tree.search_knn_vector_3d(p, 1)
            if k > 0:
                pos_idx = idx[0]
                if pos_idx not in selected_indices:
                    selected_poses.append(poses[pos_idx])
                    selected_indices.add(pos_idx)
                    
        return selected_poses
