import numpy as np
import open3d as o3d
import logging
from typing import List
from io_modules.navvis_parser import PanoPose

logger = logging.getLogger(__name__)

class TrajectoryStationGenerator:
    def __init__(self, target_spacing: float = 3.0, scan_density_pct: float = 100.0):
        """
        Initializes the generator with target spacing (m) or scan density percentage (%).
        """
        self.target_spacing = target_spacing
        self.scan_density_pct = scan_density_pct
        
    def generate(self, poses: List[PanoPose]) -> List[PanoPose]:
        """
        Generates candidate scan stations from trajectory poses.
        By default (scan_density_pct=100.0), keeps all panorama poses 1:1.
        If scan_density_pct < 100.0, samples the exact target percentage of poses uniformly.
        """
        if not poses:
            logger.warning("No poses provided for trajectory scanning.")
            return []
            
        # Ensure poses are sorted chronologically by timestamp
        sorted_poses = sorted(poses, key=lambda p: p.timestamp)
        total_poses = len(sorted_poses)

        if self.scan_density_pct >= 100.0:
            logger.info(f"Scan density 100% (1:1 ratio). Preserving all {total_poses} panorama poses as scan stations.")
            return sorted_poses

        # Percentage-based reduction: select exact target count uniformly along trajectory
        target_count = max(1, int(round(total_poses * (self.scan_density_pct / 100.0))))
        if target_count >= total_poses:
            logger.info(f"Target count {target_count} >= total poses {total_poses}. Using all poses.")
            return sorted_poses

        logger.info(f"Reducing {total_poses} panoramas to {target_count} scan stations ({self.scan_density_pct}% density).")
        
        # Compute cumulative 3D arc-length (with vertical Z transitions weighted 2x to preserve multi-floor transitions)
        positions = np.array([p.position for p in sorted_poses], dtype=np.float64)
        diffs = np.diff(positions, axis=0)
        diffs[:, 2] *= 2.0
        step_dists = np.linalg.norm(diffs, axis=1)
        cum_dist = np.concatenate(([0.0], np.cumsum(step_dists)))
        total_dist = float(cum_dist[-1])

        if total_dist > 1e-3:
            target_dists = np.linspace(0.0, total_dist, target_count)
            chosen_indices = []
            used = set()
            for td in target_dists:
                idx = int(np.argmin(np.abs(cum_dist - td)))
                if idx in used:
                    # Find nearest unused index
                    rem = [j for j in range(total_poses) if j not in used]
                    idx = min(rem, key=lambda j: abs(cum_dist[j] - td))
                used.add(idx)
                chosen_indices.append(idx)
            chosen_indices.sort()
            selected_poses = [sorted_poses[i] for i in chosen_indices]
        else:
            indices = np.round(np.linspace(0, total_poses - 1, target_count)).astype(int)
            selected_poses = [sorted_poses[i] for i in indices]

        return selected_poses

