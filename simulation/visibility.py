import logging
import numpy as np
import open3d as o3d
from typing import List, Tuple, Optional

logger = logging.getLogger(__name__)


class VisibilityOptimizer:
    def __init__(
        self,
        min_overlap: float = 0.40,
        proxy_voxel_size: float = 0.5,
        target_coverage: float = 0.99,
        optimization_priority: str = "balanced",
        min_stations: Optional[int] = None,
        max_stations: Optional[int] = None,
    ):
        self.min_overlap = float(min_overlap)
        self.proxy_voxel_size = float(proxy_voxel_size)
        self.target_coverage = float(target_coverage)
        self.optimization_priority = (optimization_priority or "balanced").lower()
        self.min_stations = int(min_stations) if min_stations is not None and min_stations > 0 else None
        self.max_stations = int(max_stations) if max_stations is not None and max_stations > 0 else None

    def optimize(
        self,
        pcd: o3d.geometry.PointCloud,
        candidates: List[tuple],
        pinned_stations: Optional[List[tuple]] = None,
    ) -> List[tuple]:
        """
        Uses vectorized Spherical First-Hit Visibility and greedy set cover to select an optimal subset
        of candidate stations ensuring maximum coverage and minimum overlap across all building levels.
        Supports optimization_priority ('balanced', 'geometry', 'panorama', 'efficiency'),
        min_stations / max_stations bounds, and manual pinned_stations.
        """
        logger.info(
            f"Starting visibility optimization for {len(candidates)} candidates "
            f"(priority='{self.optimization_priority}', min_stations={self.min_stations}, max_stations={self.max_stations})."
        )
        if not candidates and not pinned_stations:
            return []

        # Merge pinned (manual) stations at the front of candidates if not already present
        merged_candidates = list(candidates)
        pinned_indices = []
        if pinned_stations:
            for m_pos in pinned_stations:
                m_t = (float(m_pos[0]), float(m_pos[1]), float(m_pos[2]))
                # Check if already close to an existing candidate
                found_idx = None
                for idx, c_pos in enumerate(merged_candidates):
                    if (
                        abs(c_pos[0] - m_t[0]) < 0.05
                        and abs(c_pos[1] - m_t[1]) < 0.05
                        and abs(c_pos[2] - m_t[2]) < 0.05
                    ):
                        found_idx = idx
                        break
                if found_idx is None:
                    found_idx = len(merged_candidates)
                    merged_candidates.append(m_t)
                if found_idx not in pinned_indices:
                    pinned_indices.append(found_idx)

        candidates = merged_candidates
        if not candidates:
            return []

        # Adjust proxy resolution & coverage target according to optimization_priority
        eff_voxel = self.proxy_voxel_size
        eff_target_coverage = self.target_coverage
        if self.optimization_priority == "efficiency":
            eff_voxel = max(self.proxy_voxel_size, 0.75)
            eff_target_coverage = min(self.target_coverage, 0.90)
        elif self.optimization_priority == "geometry":
            eff_voxel = min(self.proxy_voxel_size, 0.40)
            eff_target_coverage = max(self.target_coverage, 0.99)

        # 1. Create coarse geometric proxy for visibility testing (in local shifted frame for float32 accuracy)
        raw_pts = np.asarray(pcd.points)
        if len(raw_pts) == 0:
            return candidates[:1]

        origin_shift = np.median(raw_pts, axis=0)
        shifted_pcd = o3d.geometry.PointCloud()
        shifted_pcd.points = o3d.utility.Vector3dVector(raw_pts - origin_shift)
        if pcd.has_colors():
            shifted_pcd.colors = pcd.colors

        actual_voxel = eff_voxel if len(raw_pts) > 5000 else min(eff_voxel, 0.15)
        logger.info(f"Creating coarse visibility proxy with voxel size {actual_voxel}m...")
        proxy_pcd = shifted_pcd.voxel_down_sample(actual_voxel)
        points = np.asarray(proxy_pcd.points, dtype=np.float32)
        n_pts = len(points)
        if n_pts == 0:
            return candidates[:1]

        from scipy.ndimage import minimum_filter
        import time

        N_AZ, N_EL = 180, 90
        MAX_VIS_RANGE = 25.0
        n_cand = len(candidates)

        # 2. Compute sparse visibility sets and angular completeness scores for each candidate
        logger.info(f"Computing spherical first-hit visibility fields across {n_cand} candidates...")
        cand_visible_pts: List[Optional[np.ndarray]] = [None] * n_cand
        total_vis_counts = np.zeros(n_cand, dtype=np.int32)
        angular_completeness = np.zeros(n_cand, dtype=np.float32)
        pt_visible_flags = np.zeros(n_pts, dtype=bool)

        t_vis_start = time.time()
        for i, pos in enumerate(candidates):
            if (i + 1) % 2500 == 0 or (i + 1) == n_cand:
                elapsed = time.time() - t_vis_start
                logger.info(f"Visibility field evaluated for {i+1}/{n_cand} candidates ({elapsed:.1f}s)...")

            cam = (np.array(pos, dtype=np.float64) - origin_shift).astype(np.float32)
            rel = points - cam
            r = np.linalg.norm(rel, axis=1)
            in_range = (r > 0.25) & (r <= MAX_VIS_RANGE) & (np.abs(rel[:, 2]) <= 3.5)
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
            min_3x3 = minimum_filter(depth_map, size=3, mode=("wrap", "nearest"))
            close_occ = (min_3x3 < 12.0) & (depth_map > (min_3x3 + 0.30))
            depth_map = np.where(close_occ, min_3x3, depth_map)

            # Fraction of 360 spherical bins with observed geometry (measures panoramic completeness)
            occupied_bins = int(np.sum(np.isfinite(depth_map)))
            angular_completeness[i] = float(occupied_bins) / float(N_AZ * N_EL)

            # Adaptive tolerance to prevent false through-wall visibility
            occ_tol = np.clip(0.12 + 0.006 * r_in, 0.12, 0.25)
            vis_mask = r_in <= (depth_map[u, v] + occ_tol)
            vis_pts_idx = idx_in[vis_mask].astype(np.int32)
            if len(vis_pts_idx) > 0:
                cand_visible_pts[i] = vis_pts_idx
                total_vis_counts[i] = len(vis_pts_idx)
                pt_visible_flags[vis_pts_idx] = True

        total_possible = int(np.sum(pt_visible_flags))
        if total_possible == 0:
            logger.warning("No visible proxy points found for any candidate station.")
            return candidates[: max(1, self.min_stations or 1)]

        # 3. Build sparse Inverted Index: proxy point -> observing candidate stations
        logger.info(f"Building sparse visibility index ({total_possible:,} observable proxy elements)...")
        pt_to_candidates: List[List[int]] = [[] for _ in range(n_pts)]
        for c_idx, pts in enumerate(cand_visible_pts):
            if pts is not None:
                for p in pts:
                    pt_to_candidates[p].append(c_idx)

        # Priority weight multiplier per candidate based on angular panoramic completeness
        if self.optimization_priority == "panorama":
            priority_mult = 0.4 + 1.6 * angular_completeness
        elif self.optimization_priority == "balanced":
            priority_mult = 0.75 + 0.5 * angular_completeness
        else:
            priority_mult = np.ones(n_cand, dtype=np.float32)

        # 4. Ultra-fast Greedy Set Cover with Overlap continuity constraint
        selected_indices: List[int] = []
        covered_mask = np.zeros(n_pts, dtype=bool)
        is_selected = np.zeros(n_cand, dtype=bool)
        gains = np.copy(total_vis_counts)
        covered_count = 0

        def _select_candidate(c_idx: int):
            nonlocal covered_count
            selected_indices.append(c_idx)
            is_selected[c_idx] = True
            gains[c_idx] = -1
            pts = cand_visible_pts[c_idx]
            if pts is not None and len(pts) > 0:
                new_pts = pts[~covered_mask[pts]]
                covered_mask[new_pts] = True
                for p in new_pts:
                    for c in pt_to_candidates[p]:
                        if not is_selected[c]:
                            gains[c] -= 1
                covered_count += len(new_pts)

        # Always select pinned (manual) stations first
        for p_idx in pinned_indices:
            if not is_selected[p_idx]:
                _select_candidate(p_idx)
                if self.max_stations is not None and len(selected_indices) >= self.max_stations:
                    break

        # If no pinned stations were provided, pick the highest-scoring initial candidate
        if not selected_indices:
            weighted_initial = gains.astype(np.float32) * priority_mult
            best_initial = int(np.argmax(weighted_initial))
            _select_candidate(best_initial)

        target_pts = int(eff_target_coverage * total_possible)
        logger.info(
            f"Greedy optimization initialized with {len(selected_indices)} station(s) "
            f"({covered_count:,}/{total_possible:,} points covered)."
        )

        t_opt_start = time.time()
        last_log_time = t_opt_start

        while len(selected_indices) < n_cand:
            if self.max_stations is not None and len(selected_indices) >= self.max_stations:
                break
            min_satisfied = (self.min_stations is None) or (len(selected_indices) >= self.min_stations)
            if covered_count >= target_pts and min_satisfied:
                break

            overlap_ratios = (total_vis_counts - gains) / np.maximum(1, total_vis_counts)
            valid_mask = (~is_selected) & (gains > 0) & (overlap_ratios >= self.min_overlap)

            weighted_gains = gains.astype(np.float32) * priority_mult

            if np.any(valid_mask):
                masked_scores = np.where(valid_mask, weighted_gains, -1.0)
                best = int(np.argmax(masked_scores))
            else:
                masked_scores = np.where((~is_selected) & (gains > 0), weighted_gains, -1.0)
                best = int(np.argmax(masked_scores))
                if gains[best] <= 0:
                    # If min_stations is still not reached, select remaining candidates with highest total visibility
                    if not min_satisfied:
                        rem_scores = np.where(~is_selected, total_vis_counts.astype(np.float32) * priority_mult, -1.0)
                        best = int(np.argmax(rem_scores))
                        if is_selected[best]:
                            break
                    else:
                        break

            _select_candidate(best)

            now = time.time()
            if len(selected_indices) % 25 == 0 or (now - last_log_time) >= 10.0:
                pct = (covered_count / total_possible) * 100.0
                logger.info(
                    f"Greedy optimization: selected {len(selected_indices)} stations ({pct:.1f}% coverage, {now - t_opt_start:.1f}s)..."
                )
                last_log_time = now

        final_coverage_pct = (covered_count / total_possible) * 100.0
        final_stations = [candidates[i] for i in selected_indices]
        logger.info(
            f"Visibility optimization complete: selected {len(final_stations)} optimal stations "
            f"({final_coverage_pct:.1f}% coverage in {time.time() - t_opt_start:.1f}s)."
        )
        return final_stations


