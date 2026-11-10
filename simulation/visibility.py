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
        Uses vectorized Spherical First-Hit Visibility and greedy set cover to select an optimal subset
        of candidate stations ensuring maximum coverage and minimum overlap across all building levels.
        """
        logger.info(f"Starting visibility optimization for {len(candidates)} candidates.")
        if not candidates:
            return []
            
        # 1. Create coarse geometric proxy for visibility testing
        logger.info(f"Creating coarse visibility proxy with voxel size {self.proxy_voxel_size}m...")
        proxy_pcd = pcd.voxel_down_sample(self.proxy_voxel_size)
        points = np.asarray(proxy_pcd.points, dtype=np.float32)
        n_pts = len(points)
        if n_pts == 0:
            return candidates[:1]

        from scipy.ndimage import minimum_filter
        import time

        N_AZ, N_EL = 90, 45
        MAX_VIS_RANGE = 25.0
        TOLERANCE = 0.60
        n_cand = len(candidates)

        # 2. Compute sparse visibility sets for each candidate using fast spherical first-hit
        # Storing 1D int32 index arrays per candidate uses < 50MB instead of a multi-gigabyte dense matrix
        logger.info(f"Computing spherical first-hit visibility fields across {n_cand} candidates...")
        cand_visible_pts: List[Optional[np.ndarray]] = [None] * n_cand
        total_vis_counts = np.zeros(n_cand, dtype=np.int32)
        pt_visible_flags = np.zeros(n_pts, dtype=bool)

        t_vis_start = time.time()
        for i, pos in enumerate(candidates):
            if (i + 1) % 2500 == 0 or (i + 1) == n_cand:
                elapsed = time.time() - t_vis_start
                logger.info(f"Visibility field evaluated for {i+1}/{n_cand} candidates ({elapsed:.1f}s)...")

            cam = np.array(pos, dtype=np.float32)
            rel = points - cam
            r = np.linalg.norm(rel, axis=1)
            in_range = (r > 0.25) & (r <= MAX_VIS_RANGE) & (np.abs(rel[:, 2]) <= 3.0)
            if not np.any(in_range):
                continue

            idx_in = np.where(in_range)[0]
            rel_in = rel[idx_in]
            r_in = r[idx_in]

            az = np.arctan2(rel_in[:, 1], rel_in[:, 0])
            el = np.arcsin(np.clip(rel_in[:, 2] / np.maximum(r_in, 1e-6), -1.0, 1.0))
            u = np.clip(((az + np.pi) * (N_AZ / (2.0 * np.pi))).astype(np.int32), 0, N_AZ - 1)
            v = np.clip(((el + (0.5 * np.pi)) * (N_EL / np.pi)).astype(np.int32), 0, N_EL - 1)

            depth_map = np.full((N_AZ, N_EL), np.inf, dtype=np.float32)
            np.minimum.at(depth_map, (u, v), r_in)
            min_3x3 = minimum_filter(depth_map, size=3, mode=('wrap', 'nearest'))
            close_occ = (min_3x3 < 8.0) & (depth_map > (min_3x3 + 0.8))
            depth_map = np.where(close_occ, min_3x3, depth_map)

            vis_mask = r_in <= (depth_map[u, v] + TOLERANCE)
            vis_pts_idx = idx_in[vis_mask].astype(np.int32)
            if len(vis_pts_idx) > 0:
                cand_visible_pts[i] = vis_pts_idx
                total_vis_counts[i] = len(vis_pts_idx)
                pt_visible_flags[vis_pts_idx] = True

        total_possible = int(np.sum(pt_visible_flags))
        if total_possible == 0:
            logger.warning("No visible proxy points found for any candidate station.")
            return candidates[:1]

        # 3. Build sparse Inverted Index: proxy point -> observing candidate stations
        logger.info(f"Building sparse visibility index ({total_possible:,} observable proxy elements)...")
        pt_to_candidates: List[List[int]] = [[] for _ in range(n_pts)]
        for c_idx, pts in enumerate(cand_visible_pts):
            if pts is not None:
                for p in pts:
                    pt_to_candidates[p].append(c_idx)

        # 4. Ultra-fast Greedy Set Cover with Overlap continuity constraint
        selected_indices: List[int] = []
        covered_mask = np.zeros(n_pts, dtype=bool)
        is_selected = np.zeros(n_cand, dtype=bool)
        gains = np.copy(total_vis_counts)

        best_initial = int(np.argmax(gains))
        selected_indices.append(best_initial)
        is_selected[best_initial] = True
        gains[best_initial] = -1

        init_pts = cand_visible_pts[best_initial]
        covered_mask[init_pts] = True
        for p in init_pts:
            for c in pt_to_candidates[p]:
                if not is_selected[c]:
                    gains[c] -= 1

        covered_count = len(init_pts)
        target_pts = int(self.target_coverage * total_possible)
        logger.info(f"Greedy optimization initialized with station {best_initial} ({covered_count:,}/{total_possible:,} points covered).")

        t_opt_start = time.time()
        last_log_time = t_opt_start

        while covered_count < target_pts and len(selected_indices) < n_cand:
            # Overlap with already covered volume: overlap_pts = total_vis - current_gain
            # Overlap ratio = overlap_pts / total_vis
            overlap_ratios = (total_vis_counts - gains) / np.maximum(1, total_vis_counts)
            valid_mask = (~is_selected) & (gains > 0) & (overlap_ratios >= self.min_overlap)

            if np.any(valid_mask):
                masked_gains = np.where(valid_mask, gains, -1)
                best = int(np.argmax(masked_gains))
            else:
                masked_gains = np.where(~is_selected, gains, -1)
                best = int(np.argmax(masked_gains))
                if gains[best] <= 0:
                    break

            selected_indices.append(best)
            is_selected[best] = True
            gains[best] = -1

            pts = cand_visible_pts[best]
            new_pts = pts[~covered_mask[pts]]
            covered_mask[new_pts] = True

            for p in new_pts:
                for c in pt_to_candidates[p]:
                    if not is_selected[c]:
                        gains[c] -= 1

            covered_count += len(new_pts)

            now = time.time()
            if len(selected_indices) % 25 == 0 or (now - last_log_time) >= 10.0:
                pct = (covered_count / total_possible) * 100.0
                logger.info(f"Greedy optimization: selected {len(selected_indices)} stations ({pct:.1f}% coverage, {now - t_opt_start:.1f}s)...")
                last_log_time = now

        final_coverage_pct = (covered_count / total_possible) * 100.0
        final_stations = [candidates[i] for i in selected_indices]
        logger.info(f"Visibility optimization complete: selected {len(final_stations)} optimal stations ({final_coverage_pct:.1f}% coverage in {time.time() - t_opt_start:.1f}s).")
        return final_stations

