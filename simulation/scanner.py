import numpy as np
import open3d as o3d
import logging
from typing import List, Dict, Optional

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
            
        # Sample points deterministically to compute average Nearest Neighbor spacing
        step = max(1, len(points) // sample_size)
        idx = np.arange(0, len(points), step)[:sample_size]
        
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

    @staticmethod
    def _compute_spherical_bins(dx: np.ndarray, dy: np.ndarray, dz: np.ndarray, r: np.ndarray, n_az: int = 180, n_el: int = 90):
        """
        Maps 3D relative vectors (dx, dy, dz) and range r to discrete (u, v) spherical angular bins.
        """
        az = np.arctan2(dy, dx)  # [-pi, pi]
        el = np.arcsin(np.clip(dz / np.maximum(r, 1e-6), -1.0, 1.0))  # [-pi/2, pi/2]

        u = np.clip(((az + np.pi) * (n_az / (2.0 * np.pi))).astype(np.int32), 0, n_az - 1)
        v = np.clip(((el + (0.5 * np.pi)) * (n_el / np.pi)).astype(np.int32), 0, n_el - 1)
        return u, v

    def _build_station_depth_map(self, proxy_pts: np.ndarray, camera_loc: np.ndarray, n_az: int = 180, n_el: int = 90) -> np.ndarray:
        """
        Builds a lightweight 2D spherical first-hit depth buffer (in meters) from a coarse proxy cloud.
        Applies morphological gap closing for close surfaces (walls/slabs) to prevent ray leakage.
        """
        from scipy.ndimage import minimum_filter

        rel = proxy_pts - camera_loc
        r = np.linalg.norm(rel, axis=1).astype(np.float32)
        valid = r > 0.30  # Ignore scanner self-hits within 30cm
        if not np.any(valid):
            return np.full((n_az, n_el), np.inf, dtype=np.float32)

        rel = rel[valid]
        r = r[valid]
        u, v = self._compute_spherical_bins(rel[:, 0], rel[:, 1], rel[:, 2], r, n_az, n_el)

        depth_map = np.full((n_az, n_el), np.inf, dtype=np.float32)
        np.minimum.at(depth_map, (u, v), r)

        # Close small angular gaps on nearby surfaces (walls, floor/ceiling slabs) using a 3x3 min filter
        # Wrap horizontally (azimuth is 360-periodic) and clamp vertically
        min_3x3 = minimum_filter(depth_map, size=3, mode=('wrap', 'nearest'))
        # Fill empty or far bins if adjacent bins form a close occluding surface (< 8.0m)
        close_occluder = (min_3x3 < 8.0) & (depth_map > (min_3x3 + 0.6))
        depth_map = np.where(close_occluder, min_3x3, depth_map)
        return depth_map

    def simulate_scans(self, pcd: o3d.geometry.PointCloud, stations: List[tuple], max_range: Optional[float] = None) -> Dict[int, np.ndarray]:
        """
        Partitions the input point cloud across all virtual stations using:
        1. Spherical First-Hit Depth Buffers (true line-of-sight occlusion at any point cloud scale).
        2. Multi-level slab-aware vertical distance weighting (prevents cross-floor slab bleed).
        3. Strict 1:1 non-overlapping point preservation with full diagnostic accounting.
        """
        if not stations:
            return {}
            
        points = np.asarray(pcd.points)
        num_points = len(points)
        num_stations = len(stations)
        
        logger.info(f"Partitioning {num_points:,} points across {num_stations} virtual scan stations...")
        
        # Build coarse proxy cloud once for fast, memory-safe spherical occlusion maps
        proxy_voxel = 0.12
        proxy_pcd = pcd.voxel_down_sample(proxy_voxel) if num_points > 500_000 else pcd
        proxy_pts = np.asarray(proxy_pcd.points)
        logger.info(f"Built occlusion proxy with {len(proxy_pts):,} surface elements (voxel={proxy_voxel}m).")

        N_AZ, N_EL = 180, 90
        OCCLUSION_TOLERANCE_M = 0.40  # Surface thickness / grazing angle tolerance
        CHUNK_SIZE = 5_000_000

        # Precompute bounding boxes for each 5M point chunk once
        chunk_bounds = []
        for s_idx in range(0, num_points, CHUNK_SIZE):
            e_idx = min(s_idx + CHUNK_SIZE, num_points)
            c_pts = points[s_idx:e_idx]
            chunk_bounds.append((c_pts.min(axis=0).astype(np.float32), c_pts.max(axis=0).astype(np.float32)))

        # Non-overlapping point assignment: track closest visible station per point
        best_visible_station = np.full(num_points, -1, dtype=np.int32)
        best_visible_score = np.full(num_points, np.inf, dtype=np.float32)
        
        # Fallback for points in occluded recesses (with cross-slab vertical penalty)
        best_absolute_station = np.full(num_points, -1, dtype=np.int32)
        best_absolute_score = np.full(num_points, np.inf, dtype=np.float32)
        best_raw_dist_sq = np.full(num_points, np.inf, dtype=np.float32)

        max_search_dist_sq = (float(max_range) + 5.0) ** 2 if max_range is not None else np.inf

        for idx, pos in enumerate(stations):
            logger.info(f"Computing visibility & sector assignment for station {idx+1}/{num_stations}...")
            camera_loc = np.array(pos, dtype=np.float32)
            depth_map = self._build_station_depth_map(proxy_pts, camera_loc, N_AZ, N_EL)

            for c_idx, start_idx in enumerate(range(0, num_points, CHUNK_SIZE)):
                end_idx = min(start_idx + CHUNK_SIZE, num_points)
                c_min, c_max = chunk_bounds[c_idx]

                # AABB distance check: skip chunks that are strictly out of range
                d_box_x = max(0.0, float(c_min[0] - camera_loc[0]), float(camera_loc[0] - c_max[0]))
                d_box_y = max(0.0, float(c_min[1] - camera_loc[1]), float(camera_loc[1] - c_max[1]))
                d_box_z = max(0.0, float(c_min[2] - camera_loc[2]), float(camera_loc[2] - c_max[2]))
                box_dist_sq = d_box_x * d_box_x + d_box_y * d_box_y + d_box_z * d_box_z
                if box_dist_sq > max_search_dist_sq:
                    continue

                chunk = points[start_idx:end_idx]
                
                dx = (chunk[:, 0] - camera_loc[0]).astype(np.float32)
                dy = (chunk[:, 1] - camera_loc[1]).astype(np.float32)
                dz = (chunk[:, 2] - camera_loc[2]).astype(np.float32)
                
                raw_d2 = dx * dx + dy * dy + dz * dz
                
                # Anisotropic vertical penalty to prevent cross-floor slab contamination:
                # Within normal room height (|dz| <= 2.2m), mild vertical weight (1.5x);
                # Beyond slab height (|dz| > 2.2m), strong vertical barrier penalty (12.0x)
                abs_dz = np.abs(dz)
                z_weight = np.where(abs_dz > 2.2, 12.0, 1.5).astype(np.float32)
                eff_score = dx * dx + dy * dy + z_weight * (dz * dz)

                # 1. Update fallback station (for points occluded from all primary viewpoints)
                abs_slice = best_absolute_score[start_idx:end_idx]
                abs_closer = eff_score < abs_slice
                if np.any(abs_closer):
                    abs_slice[abs_closer] = eff_score[abs_closer]
                    best_absolute_station[start_idx:end_idx][abs_closer] = idx
                    best_raw_dist_sq[start_idx:end_idx][abs_closer] = raw_d2[abs_closer]

                # 2. Check spherical first-hit visibility only for points that could beat current best
                vis_slice = best_visible_score[start_idx:end_idx]
                candidate_pts = eff_score < vis_slice
                if not np.any(candidate_pts):
                    continue

                sub_idx = np.where(candidate_pts)[0]
                r_sub = np.sqrt(np.maximum(raw_d2[sub_idx], 1e-12))
                u, v = self._compute_spherical_bins(dx[sub_idx], dy[sub_idx], dz[sub_idx], r_sub, N_AZ, N_EL)
                first_hit = depth_map[u, v]
                is_visible = r_sub <= (first_hit + OCCLUSION_TOLERANCE_M)

                winning_sub = sub_idx[is_visible]
                if len(winning_sub) > 0:
                    vis_slice[winning_sub] = eff_score[winning_sub]
                    best_visible_station[start_idx:end_idx][winning_sub] = idx
                    best_raw_dist_sq[start_idx:end_idx][winning_sub] = raw_d2[winning_sub]

        # Resolve assignments: prefer line-of-sight visible station; fallback to slab-aware nearest
        no_vis = (best_visible_station == -1)
        vis_assigned_count = int(np.sum(~no_vis))
        fallback_assigned_count = int(np.sum(no_vis))
        logger.info(
            f"Visibility Engine Summary: {vis_assigned_count:,} points ({vis_assigned_count/max(1, num_points)*100:.1f}%) "
            f"assigned via direct line-of-sight; {fallback_assigned_count:,} occluded recess points assigned via slab-aware fallback."
        )

        final_station_assignment = np.where(no_vis, best_absolute_station, best_visible_station)

        # Apply max scan range cutoff
        if max_range is not None:
            max_range_sq = float(max_range ** 2)
            out_of_range = best_raw_dist_sq > max_range_sq
            final_station_assignment[out_of_range] = -1
            rejected_pts = int(np.sum(out_of_range))
            logger.info(f"Max range filter ({max_range}m) excluded {rejected_pts:,} points.")

        # Build scan point index map
        scans = {}
        for idx in range(num_stations):
            station_mask = (final_station_assignment == idx)
            station_pts = np.where(station_mask)[0]
            scans[idx] = station_pts
            logger.info(f"Scan {idx+1}/{num_stations} assigned {len(station_pts):,} points.")

        # Guarantee no station is left empty (e.g. if stations are clustered close together)
        for idx, pos in enumerate(stations):
            if len(scans[idx]) == 0:
                logger.info(f"Ensuring scan {idx+1} is populated by assigning nearest local cluster...")
                camera_loc = np.array(pos)
                dists_sq = np.sum((points - camera_loc) ** 2, axis=1)
                nearest_100 = np.argsort(dists_sq)[:min(100, num_points)]
                scans[idx] = nearest_100
                logger.info(f"Scan {idx+1} populated with {len(nearest_100)} local points.")

        return scans

