import numpy as np
import open3d as o3d
import logging
from typing import List, Dict

logger = logging.getLogger(__name__)

class ScannerSimulator:
    def __init__(self, adaptive_multiplier: float = 1.0):
        self.adaptive_multiplier = adaptive_multiplier
        
    def estimate_angular_resolution(self, pcd: o3d.geometry.PointCloud, sample_size=10000) -> float:
        """
        Estimates the required angular density based on average local point spacing.
        """
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return 0.1
            
        # Sample points to compute average Nearest Neighbor spacing
        idx = np.random.choice(len(points), min(len(points), sample_size), replace=False)
        sub_pcd = pcd.select_by_index(idx)
        
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        distances = []
        for i in idx:
            [k, _, sq_dist] = pcd_tree.search_knn_vector_3d(points[i], 2)
            if k == 2:
                distances.append(np.sqrt(sq_dist[1]))
                
        avg_spacing = np.mean(distances) if distances else 0.05
        
        # Typical target distance to feature indoors is ~3m. 
        # So resolution (rad) = arc length / radius = spacing / 3.0
        ang_res_rad = avg_spacing / 3.0
        ang_res_deg = np.degrees(ang_res_rad) * self.adaptive_multiplier
        
        # Clamp to realistic terrestrial limits
        ang_res_deg = max(0.01, min(0.5, ang_res_deg))
        logger.info(f"Average point spacing: {avg_spacing:.4f}m. Adaptive ray angular resolution: {ang_res_deg:.4f} degrees.")
        return ang_res_deg

    def simulate_scans(self, pcd: o3d.geometry.PointCloud, stations: List[tuple]) -> Dict[int, np.ndarray]:
        """
        Partitions the original point cloud across the virtual terrestrial stations.
        Uses Hidden Point Removal to ensure realistic line-of-sight assignment, 
        but guarantees that ALL original points are retained without occlusion dropping.
        """
        if not stations:
            return {}
            
        points = np.asarray(pcd.points)
        num_points = len(points)
        scans = {i: [] for i in range(len(stations))}
        
        logger.info(f"Simulating realistic line-of-sight partitioning for {num_points} points...")
        
        diameter = np.linalg.norm(points.max(0) - points.min(0)) * 100
        
        # Memory-efficient distance tracking (Running minimums instead of NxM matrix)
        best_visible_station = np.full(num_points, -1, dtype=np.int16)
        best_visible_dist = np.full(num_points, np.inf, dtype=np.float32)
        
        best_absolute_station = np.full(num_points, -1, dtype=np.int16)
        best_absolute_dist = np.full(num_points, np.inf, dtype=np.float32)
        
        CHUNK_SIZE = 5_000_000
        
        for idx, pos in enumerate(stations):
            logger.info(f"Computing line-of-sight physics for station {idx+1}/{len(stations)}...")
            camera_loc = np.array(pos)
            
            # 1. Compute visibility
            # (Note: This still requires holding the point cloud in memory, but we avoid the 25 GB numpy matrix)
            try:
                # Qhull (used by HPR) has internal limits that cause integer overflows 
                # on very large clouds (typically > 10M - 20M points depending on density).
                if num_points > 10_000_000:
                    logger.warning(f"Point cloud ({num_points} pts) is too large for stable HPR visibility. "
                                   "Falling back to Euclidean partitioning for station {idx+1}.")
                    vis_indices_arr = np.array([], dtype=np.int64)
                else:
                    _, visible_indices = pcd.hidden_point_removal(camera_loc, diameter)
                    vis_indices_arr = np.asarray(visible_indices)
            except Exception as e:
                logger.error(f"Failed HPR visibility for station {idx+1}: {e}")
                vis_indices_arr = np.array([], dtype=np.int64)
            
            # 2. Compute distances in chunks to avoid large temporary matrices
            dists = np.empty(num_points, dtype=np.float32)
            for start_idx in range(0, num_points, CHUNK_SIZE):
                end_idx = min(start_idx + CHUNK_SIZE, num_points)
                chunk = points[start_idx:end_idx]
                dists[start_idx:end_idx] = np.sum((chunk - camera_loc) ** 2, axis=1)
                
            # 3. Update absolute best station
            abs_better_mask = dists < best_absolute_dist
            best_absolute_dist[abs_better_mask] = dists[abs_better_mask]
            best_absolute_station[abs_better_mask] = idx
            
            # 4. Update visible best station
            if len(vis_indices_arr) > 0:
                vis_dists = dists[vis_indices_arr]
                vis_better_mask = vis_dists < best_visible_dist[vis_indices_arr]
                
                # Get the actual original point indices where this station is the closest visible one
                update_indices = vis_indices_arr[vis_better_mask]
                best_visible_dist[update_indices] = vis_dists[vis_better_mask]
                best_visible_station[update_indices] = idx
        
        # 5. Determine optimal assignments
        logger.info("Resolving optimal physical assignments...")
        
        # For points where NO station is visible, fallback to absolute nearest
        no_visibility = (best_visible_station == -1)
        
        final_assignments = np.where(no_visibility, best_absolute_station, best_visible_station)
        
        # Split point indices by assigned station
        for station_idx in range(len(stations)):
            station_points = np.where(final_assignments == station_idx)[0]
            scans[station_idx] = station_points
            logger.info(f"Scan {station_idx+1} mapped {len(station_points)} points to its sector.")
            
        return scans
