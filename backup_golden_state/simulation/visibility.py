import logging
import numpy as np
import open3d as o3d
from typing import List, Tuple, Set

logger = logging.getLogger(__name__)

class VisibilityOptimizer:
    def __init__(self, min_overlap: float = 0.40, proxy_voxel_size: float = 0.5, target_coverage: float = 0.99):
        self.min_overlap = min_overlap
        self.proxy_voxel_size = proxy_voxel_size
        self.target_coverage = target_coverage
        
    def optimize(self, pcd: o3d.geometry.PointCloud, candidates: List[tuple]) -> List[tuple]:
        """
        Uses Open3D Hidden Point Removal and greedy set cover to select an optimal subset
        of candidate stations ensuring maximum coverage and minimum 40% overlap.
        """
        logger.info(f"Starting visibility optimization for {len(candidates)} candidates.")
        if not candidates:
            return []
            
        # 1. Create coarse geometric proxy for visibility testing
        logger.info(f"Creating coarse visibility proxy with voxel size {self.proxy_voxel_size}m...")
        proxy_pcd = pcd.voxel_down_sample(self.proxy_voxel_size)
        
        # Determine the radius for hidden point removal
        points = np.asarray(proxy_pcd.points)
        diameter = np.linalg.norm(points.max(0) - points.min(0)) * 100
        
        # 2. Compute visible sets for each candidate
        # Mapping: candidate_idx -> set of proxy point indices visible
        visible_sets: List[Set[int]] = []
        for i, pos in enumerate(candidates):
            camera_loc = np.array(pos)
            # radius should be sufficiently large
            _, visible_indices = proxy_pcd.hidden_point_removal(camera_loc, diameter)
            visible_sets.append(set(visible_indices))
            
        # 3. Greedy Set Cover with Overlap constraint
        selected_indices: List[int] = []
        covered_points: Set[int] = set()
        
        # Track remaining candidates and the points they uniquely add
        unselected: List[int] = list(range(len(candidates)))
        
        # Start with the station that sees the most points
        best_initial = max(unselected, key=lambda i: len(visible_sets[i]))
        selected_indices.append(best_initial)
        covered_points.update(visible_sets[best_initial])
        unselected.remove(best_initial)
        
        logger.info(f"Greedy optimization initialized with station ID {best_initial}.")

        while unselected:
            best_candidate = None
            best_new_coverage = -1
            
            for i in unselected:
                s_visible = visible_sets[i]
                new_points = s_visible - covered_points
                
                # Calculate overlap with currently selected stations
                # We need at least one existing station to share > min_overlap of THIS candidate's points
                # OR this candidate shares > min_overlap of the existing station's points.
                has_overlap = False
                for sel_i in selected_indices:
                    sel_visible = visible_sets[sel_i]
                    intersection = len(s_visible.intersection(sel_visible))
                    if len(s_visible) == 0 or len(sel_visible) == 0:
                        continue
                        
                    overlap_ratio1 = intersection / len(s_visible)
                    overlap_ratio2 = intersection / len(sel_visible)
                    
                    if overlap_ratio1 >= self.min_overlap or overlap_ratio2 >= self.min_overlap:
                        has_overlap = True
                        break
                        
                if has_overlap or len(selected_indices) == 0:
                    cov_size = len(new_points)
                    if cov_size > best_new_coverage:
                        best_new_coverage = cov_size
                        best_candidate = i
                        
            if best_candidate is None:
                # If no candidates satisfy the overlap constraint, we might have a disconnected component.
                # Just pick the one with the highest new coverage to bridge the gap.
                best_candidate = max(unselected, key=lambda idx: len(visible_sets[idx] - covered_points))
                new_points = visible_sets[best_candidate] - covered_points
                if len(new_points) == 0:
                    # All remaining candidates provide no new coverage
                    break
            
            assert best_candidate is not None
            selected_indices.append(best_candidate)
            covered_points.update(visible_sets[best_candidate])
            unselected.remove(best_candidate)
            
            # If we cover target % of total visible points, we can stop
            total_possible = len(set().union(*visible_sets))
            if len(covered_points) >= self.target_coverage * total_possible:
                break
                
        final_stations = [candidates[i] for i in selected_indices]
        logger.info(f"Optimization complete. Selected {len(final_stations)} stations out of {len(candidates)}.")
        return final_stations
