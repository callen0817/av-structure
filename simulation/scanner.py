import numpy as np
import open3d as o3d
import logging
from typing import List, Dict, Optional, Tuple, Any
from config import (
    DEFAULT_ANGULAR_RES_DEG,
    MIN_ANGULAR_RES_DEG,
    MAX_ANGULAR_RES_DEG,
    DEFAULT_GRID_COLLISION_POLICY,
    DEFAULT_SIMULATION_MODE,
    WALL_GRID_RESOLUTION_M,
    WALL_CLEARANCE_STATION_M,
    WALL_CLEARANCE_TARGET_M,
    OCCLUSION_TOLERANCE_NEAR_M,
    OCCLUSION_TOLERANCE_FAR_M,
    ENABLE_NORMAL_BACKFACE_CULLING,
)

logger = logging.getLogger(__name__)


class ScannerSimulator:
    def __init__(self, adaptive_multiplier: float = 1.0):
        self.adaptive_multiplier = adaptive_multiplier
        self.last_simulation_stats: Dict[str, Any] = {}
        self.station_depth_maps: Dict[int, np.ndarray] = {}

    def estimate_angular_resolution(self, pcd: o3d.geometry.PointCloud, sample_size=10000) -> float:
        """
        Estimates the required angular density based on average local point spacing.
        """
        points = np.asarray(pcd.points)
        if len(points) < 2:
            return DEFAULT_ANGULAR_RES_DEG

        # Shift to local frame before building KDTree for large UTM coordinate stability
        origin = points[0]
        shifted_pcd = o3d.geometry.PointCloud()
        shifted_pts = points - origin
        shifted_pcd.points = o3d.utility.Vector3dVector(shifted_pts)

        # Sample points deterministically to compute average Nearest Neighbor spacing
        step = max(1, len(shifted_pts) // sample_size)
        idx = np.arange(0, len(shifted_pts), step)[:sample_size]

        pcd_tree = o3d.geometry.KDTreeFlann(shifted_pcd)
        distances = []
        for i in idx:
            [k, _, sq_dist] = pcd_tree.search_knn_vector_3d(shifted_pts[i], 2)
            if k == 2:
                distances.append(np.sqrt(sq_dist[1]))

        avg_spacing = np.mean(distances) if distances else 0.05

        # Typical target distance to feature indoors is ~3m.
        # So resolution (rad) = arc length / radius = spacing / 3.0
        ang_res_rad = avg_spacing / 3.0
        ang_res_deg = float(np.degrees(ang_res_rad) * self.adaptive_multiplier)

        # Clamp to realistic terrestrial limits
        ang_res_deg = max(MIN_ANGULAR_RES_DEG, min(0.5, ang_res_deg))
        logger.info(
            f"Average point spacing: {avg_spacing:.4f}m. Adaptive ray angular resolution: {ang_res_deg:.4f} degrees."
        )
        return ang_res_deg

    @staticmethod
    def _compute_spherical_bins(
        dx: np.ndarray, dy: np.ndarray, dz: np.ndarray, r: np.ndarray, n_az: int = 180, n_el: int = 90
    ):
        """
        Maps 3D relative vectors (dx, dy, dz) and range r to discrete (u, v) spherical angular bins.
        """
        az = np.arctan2(dy, dx)  # [-pi, pi]
        el = np.arcsin(np.clip(dz / np.maximum(r, 1e-6), -1.0, 1.0))  # [-pi/2, pi/2]

        u = np.clip(((az + np.pi) * (n_az / (2.0 * np.pi))).astype(np.int32), 0, n_az - 1)
        v = np.clip(((el + (0.5 * np.pi)) * (n_el / np.pi)).astype(np.int32), 0, n_el - 1)
        return u, v

    @staticmethod
    def organize_spherical_grid(
        rel_points: np.ndarray,
        angular_res_deg: float = DEFAULT_ANGULAR_RES_DEG,
        collision_policy: str = DEFAULT_GRID_COLLISION_POLICY,
    ) -> Dict[str, Any]:
        """
        Projects station-local 3D points (N, 3) into a structured 2D spherical scan grid
        with E57-compliant rowIndex, columnIndex, cartesianInvalidState, and line-grouping metadata.

        Conventions (matching ASTM E2807 / E57 and equirectangular sphericalRepresentation):
        - Azimuth theta = atan2(y, x) in [-pi, pi]
        - Elevation phi = arcsin(z / r) in [-pi/2, pi/2]
        - columnIndex in [0, n_cols - 1] maps azimuth from +pi to -pi
        - rowIndex in [0, n_rows - 1] maps elevation from +pi/2 to -pi/2
        - Points are sorted by (columnIndex, rowIndex, r) so each column forms a contiguous scan line.
        """
        eff_res_deg = float(np.clip(angular_res_deg, MIN_ANGULAR_RES_DEG, MAX_ANGULAR_RES_DEG))
        n_cols = int(np.clip(round(360.0 / eff_res_deg), 36, 65534))
        n_rows = int(np.clip(round(180.0 / eff_res_deg), 18, 32767))

        n_pts = len(rel_points)
        if n_pts == 0:
            return {
                "order": np.empty(0, dtype=np.int64),
                "row_index": np.empty(0, dtype=np.uint16),
                "col_index": np.empty(0, dtype=np.uint16),
                "invalid_state": np.empty(0, dtype=np.int8),
                "grid_rows": n_rows,
                "grid_cols": n_cols,
                "angular_res_deg": eff_res_deg,
                "unique_cells": 0,
                "collided_points": 0,
                "omitted_collision_points": 0,
                "spherical_bounds": {
                    "rangeMinimum": 0.0,
                    "rangeMaximum": 0.0,
                    "elevationMinimum": -float(0.5 * np.pi),
                    "elevationMaximum": float(0.5 * np.pi),
                    "azimuthStart": -float(np.pi),
                    "azimuthEnd": float(np.pi),
                },
                "line_groups": {
                    "id_values": np.empty(0, dtype=np.int64),
                    "start_indices": np.empty(0, dtype=np.int64),
                    "point_counts": np.empty(0, dtype=np.int64),
                },
            }

        pts = np.asarray(rel_points, dtype=np.float64)
        r = np.linalg.norm(pts, axis=1)
        finite_valid = np.all(np.isfinite(pts), axis=1) & (r > 1e-6)

        az = np.arctan2(pts[:, 1], pts[:, 0])
        el = np.arcsin(np.clip(pts[:, 2] / np.maximum(r, 1e-9), -1.0, 1.0))

        col_idx = np.clip(((np.pi - az) * (n_cols / (2.0 * np.pi))).astype(np.int32), 0, n_cols - 1).astype(np.uint16)
        row_idx = np.clip((((0.5 * np.pi) - el) * (n_rows / np.pi)).astype(np.int32), 0, n_rows - 1).astype(np.uint16)
        invalid_state = np.where(finite_valid, 0, 2).astype(np.int8)

        # Sort deterministically by (col_index, row_index, range) so closest hit in each angular bin comes first
        order = np.lexsort((r, row_idx, col_idx))
        col_sorted = col_idx[order]
        row_sorted = row_idx[order]
        inv_sorted = invalid_state[order]
        r_sorted = r[order]
        el_sorted = el[order]
        az_sorted = az[order]

        cell_keys = col_sorted.astype(np.int64) * int(n_rows) + row_sorted.astype(np.int64)
        unique_keys, first_hit_positions = np.unique(cell_keys, return_index=True)
        unique_cells = int(len(unique_keys))
        collided_points = int(n_pts - unique_cells)
        omitted_points = 0

        if collision_policy == "first_hit_cell" and collided_points > 0:
            # Keep strictly one point (closest first-hit) per (rowIndex, columnIndex) cell
            keep_pos = np.sort(first_hit_positions)
            omitted_points = int(n_pts - len(keep_pos))
            order = order[keep_pos]
            col_sorted = col_sorted[keep_pos]
            row_sorted = row_sorted[keep_pos]
            inv_sorted = inv_sorted[keep_pos]
            r_sorted = r_sorted[keep_pos]
            el_sorted = el_sorted[keep_pos]
            az_sorted = az_sorted[keep_pos]

        # Build column line-group descriptors for E57 pointGroupingSchemes.groupingByLine
        if len(col_sorted) > 0:
            u_cols, col_starts, col_counts = np.unique(col_sorted, return_index=True, return_counts=True)
            line_groups = {
                "id_values": u_cols.astype(np.int64),
                "start_indices": col_starts.astype(np.int64),
                "point_counts": col_counts.astype(np.int64),
            }
            valid_r = r_sorted[inv_sorted == 0] if np.any(inv_sorted == 0) else r_sorted
            valid_el = el_sorted[inv_sorted == 0] if np.any(inv_sorted == 0) else el_sorted
            valid_az = az_sorted[inv_sorted == 0] if np.any(inv_sorted == 0) else az_sorted
            spherical_bounds = {
                "rangeMinimum": float(np.min(valid_r)) if len(valid_r) > 0 else 0.0,
                "rangeMaximum": float(np.max(valid_r)) if len(valid_r) > 0 else 0.0,
                "elevationMinimum": float(np.min(valid_el)) if len(valid_el) > 0 else -float(0.5 * np.pi),
                "elevationMaximum": float(np.max(valid_el)) if len(valid_el) > 0 else float(0.5 * np.pi),
                "azimuthStart": float(np.min(valid_az)) if len(valid_az) > 0 else -float(np.pi),
                "azimuthEnd": float(np.max(valid_az)) if len(valid_az) > 0 else float(np.pi),
            }
        else:
            line_groups = {
                "id_values": np.empty(0, dtype=np.int64),
                "start_indices": np.empty(0, dtype=np.int64),
                "point_counts": np.empty(0, dtype=np.int64),
            }
            spherical_bounds = {
                "rangeMinimum": 0.0,
                "rangeMaximum": 0.0,
                "elevationMinimum": -float(0.5 * np.pi),
                "elevationMaximum": float(0.5 * np.pi),
                "azimuthStart": -float(np.pi),
                "azimuthEnd": float(np.pi),
            }

        return {
            "order": order,
            "row_index": row_sorted,
            "col_index": col_sorted,
            "invalid_state": inv_sorted,
            "grid_rows": n_rows,
            "grid_cols": n_cols,
            "angular_res_deg": eff_res_deg,
            "unique_cells": unique_cells,
            "collided_points": collided_points,
            "omitted_collision_points": omitted_points,
            "spherical_bounds": spherical_bounds,
            "line_groups": line_groups,
        }

    @staticmethod
    def _build_floor_wall_grids(
        points: np.ndarray,
        stations: List[tuple],
        voxel_size: float = WALL_GRID_RESOLUTION_M,
    ) -> Dict[int, Dict[str, Any]]:
        """
        Builds 2D architectural wall occupancy grids per floor level at eye/scanner level.
        Detects solid walls and structures while leaving doorways, hallways, and portals open.
        """
        from scipy.ndimage import binary_dilation

        if len(points) == 0 or not stations:
            return {}

        min_x = float(np.min(points[:, 0]))
        min_y = float(np.min(points[:, 1]))
        max_x = float(np.max(points[:, 0]))
        max_y = float(np.max(points[:, 1]))

        w_px = max(10, int(np.ceil((max_x - min_x) / voxel_size)) + 1)
        h_px = max(10, int(np.ceil((max_y - min_y) / voxel_size)) + 1)

        # Cluster stations into elevation floors separated by >= 2.0m
        z_stations = [float(s[2]) for s in stations]
        sorted_z = sorted(set(z_stations))
        floor_elevs = [sorted_z[0]]
        for z in sorted_z[1:]:
            if z - floor_elevs[-1] >= 2.0:
                floor_elevs.append(z)

        floor_grids: Dict[int, Dict[str, Any]] = {}
        for f_idx, f_z in enumerate(floor_elevs):
            # Eye/scanner level wall band [-0.7m, +0.7m]
            band_mask = (points[:, 2] >= (f_z - 0.7)) & (points[:, 2] <= (f_z + 0.7))
            band_pts = points[band_mask]
            grid = np.zeros((h_px, w_px), dtype=bool)

            if len(band_pts) > 0:
                gx = np.clip(np.floor((band_pts[:, 0] - min_x) / voxel_size).astype(np.int32), 0, w_px - 1)
                gy = np.clip(np.floor((band_pts[:, 1] - min_y) / voxel_size).astype(np.int32), 0, h_px - 1)
                cell_idx = gy * w_px + gx
                counts = np.bincount(cell_idx, minlength=h_px * w_px)
                # Require at least 2 points per 5cm cell to ignore isolated airborne noise
                solid_mask = counts >= 2
                grid.flat[solid_mask] = True
                # Dilate slightly so 10-15cm walls form a solid barrier without holes
                grid = binary_dilation(grid, iterations=1)

            floor_grids[f_idx] = {
                "grid": grid,
                "ref_z": f_z,
                "origin_x": min_x,
                "origin_y": min_y,
                "res": voxel_size,
                "w": w_px,
                "h": h_px,
            }

        return floor_grids

    @staticmethod
    def _check_wall_line_of_sight_vectorized(
        camera_loc: np.ndarray,
        target_pts: np.ndarray,
        wall_grid_info: Optional[Dict[str, Any]],
        clearance_station: float = WALL_CLEARANCE_STATION_M,
        clearance_target: float = WALL_CLEARANCE_TARGET_M,
        n_steps: int = 40,
    ) -> np.ndarray:
        """
        Fast vectorized 2D line-of-sight test between scanner camera_loc and target_pts.
        Returns a boolean array of length len(target_pts) where True means BLOCKED by an intervening wall.
        """
        n_pts = len(target_pts)
        if n_pts == 0 or wall_grid_info is None:
            return np.zeros(n_pts, dtype=bool)

        grid = wall_grid_info["grid"]
        res = wall_grid_info["res"]
        origin_x = wall_grid_info["origin_x"]
        origin_y = wall_grid_info["origin_y"]
        w_px = wall_grid_info["w"]
        h_px = wall_grid_info["h"]

        dx = target_pts[:, 0] - camera_loc[0]
        dy = target_pts[:, 1] - camera_loc[1]
        dist = np.hypot(dx, dy)

        # Points within immediate scanner clearance (< 0.40m) are never blocked by an intermediate wall
        close_mask = dist <= (clearance_station + 0.10)
        eval_mask = (~close_mask) & (dist > (clearance_station + clearance_target + 0.05))
        if not np.any(eval_mask):
            return np.zeros(n_pts, dtype=bool)

        eval_indices = np.where(eval_mask)[0]
        e_dist = dist[eval_indices]
        e_dx = dx[eval_indices]
        e_dy = dy[eval_indices]

        # Normalized t along ray from station clearance to target clearance
        t_min = np.clip(clearance_station / np.maximum(e_dist, 1e-4), 0.0, 0.95)[:, None]
        t_max = np.clip((e_dist - clearance_target) / np.maximum(e_dist, 1e-4), 0.05, 1.0)[:, None]
        alpha = np.linspace(0.0, 1.0, n_steps)[None, :]
        t = t_min + alpha * (t_max - t_min)

        xs = camera_loc[0] + t * e_dx[:, None]
        ys = camera_loc[1] + t * e_dy[:, None]

        cgx = np.clip(np.floor((xs - origin_x) / res).astype(np.int32), 0, w_px - 1)
        cgy = np.clip(np.floor((ys - origin_y) / res).astype(np.int32), 0, h_px - 1)

        hit = np.any(grid[cgy, cgx], axis=1)

        blocked = np.zeros(n_pts, dtype=bool)
        blocked[eval_indices] = hit
        return blocked

    def _build_station_depth_map(
        self, proxy_pts: np.ndarray, camera_loc: np.ndarray, n_az: int = 360, n_el: int = 180
    ) -> np.ndarray:
        """
        Builds a high-resolution 2D spherical first-hit depth buffer (in meters) from a coarse proxy cloud.
        Applies morphological gap closing with depth-jump protection to preserve doorways and openings.
        """
        from scipy.ndimage import minimum_filter

        rel = (proxy_pts - camera_loc).astype(np.float32)
        r = np.linalg.norm(rel, axis=1).astype(np.float32)
        valid = (r > 0.30) & (r < 60.0)  # Ignore scanner self-hits within 30cm
        if not np.any(valid):
            return np.full((n_az, n_el), np.inf, dtype=np.float32)

        rel = rel[valid]
        r = r[valid]
        u, v = self._compute_spherical_bins(rel[:, 0], rel[:, 1], rel[:, 2], r, n_az, n_el)

        depth_map = np.full((n_az, n_el), np.inf, dtype=np.float32)
        np.minimum.at(depth_map, (u, v), r)

        # Close small angular gaps on solid walls while protecting large depth edges (doorways, window portals):
        # Only close bins where neighboring foreground is close (< 15.0m) and depth difference is small (< 0.25m)
        min_3x3 = minimum_filter(depth_map, size=3, mode=("wrap", "nearest"))
        close_wall = (min_3x3 < 15.0) & (depth_map > (min_3x3 + 0.25))
        depth_map = np.where(close_wall, min_3x3, depth_map)
        return depth_map

    def simulate_scans(
        self,
        pcd: o3d.geometry.PointCloud,
        stations: List[tuple],
        max_range: Optional[float] = None,
        simulation_mode: str = DEFAULT_SIMULATION_MODE,
        allow_overlap: bool = False,
    ) -> Dict[int, np.ndarray]:
        """
        Partitions or simulates physical terrestrial LiDAR scans across virtual stations:
        1. Room & Solid Obstacle Awareness: 2D architectural wall slice raycasting prevents x-ray vision
           through solid dividing walls, while naturally allowing rays to pass through doorways and openings.
        2. High-Resolution Spherical First-Hit Depth Buffers: 3D line-of-sight occlusion with adaptive
           sub-wall thickness tolerance (0.08m - 0.18m).
        3. Surface Normal Orientation: Rejects back-facing surfaces (cos > 0.15) reflecting away from the scanner.
        4. Multi-level slab-aware vertical distance weighting (prevents cross-floor slab bleed).
        5. In 'physical' mode (default), points on the opposite side of a wall without an opening are strictly
           excluded from that virtual scan station.
        """
        if not stations:
            return {}

        points = np.asarray(pcd.points)
        num_points = len(points)
        num_stations = len(stations)
        self.station_depth_maps = {}

        if num_points == 0:
            return {idx: np.empty(0, dtype=np.int64) for idx in range(num_stations)}

        logger.info(
            f"Simulating scans across {num_stations} virtual scan stations "
            f"(mode='{simulation_mode}', num_points={num_points:,})..."
        )

        # Estimate surface normals if not present for realistic back-face culling
        has_normals = pcd.has_normals()
        if not has_normals and ENABLE_NORMAL_BACKFACE_CULLING and num_points <= 2_000_000:
            try:
                pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=20))
                has_normals = pcd.has_normals()
            except Exception as norm_err:
                logger.warning(f"Normal estimation skipped: {norm_err}")
                has_normals = False

        normals = np.asarray(pcd.normals, dtype=np.float32) if has_normals else None

        # Build 2D architectural wall slices for each building floor
        floor_wall_grids = self._build_floor_wall_grids(points, stations)

        # Shift to local frame for voxel proxy so large UTM coordinates retain sub-millimeter precision
        origin_shift = np.median(points, axis=0)
        shifted_pcd = o3d.geometry.PointCloud()
        shifted_pcd.points = o3d.utility.Vector3dVector(points - origin_shift)

        # Build coarse proxy cloud once for fast, memory-safe spherical occlusion maps
        proxy_voxel = 0.08 if num_points < 1_000_000 else 0.12
        proxy_pcd = shifted_pcd.voxel_down_sample(proxy_voxel) if num_points > 200_000 else shifted_pcd
        proxy_pts = np.asarray(proxy_pcd.points, dtype=np.float64) + origin_shift
        logger.info(f"Built occlusion proxy with {len(proxy_pts):,} surface elements (voxel={proxy_voxel}m).")

        N_AZ, N_EL = 360, 180
        CHUNK_SIZE = 5_000_000

        # Precompute bounding boxes in float64 for each 5M point chunk once
        chunk_bounds = []
        for s_idx in range(0, num_points, CHUNK_SIZE):
            e_idx = min(s_idx + CHUNK_SIZE, num_points)
            c_pts = points[s_idx:e_idx]
            chunk_bounds.append((c_pts.min(axis=0).astype(np.float64), c_pts.max(axis=0).astype(np.float64)))

        # Non-overlapping point assignment: track closest visible station per point
        best_visible_station = np.full(num_points, -1, dtype=np.int32)
        best_visible_score = np.full(num_points, np.inf, dtype=np.float32)

        # Fallback for legacy partitioning (with cross-slab vertical penalty)
        best_absolute_station = np.full(num_points, -1, dtype=np.int32)
        best_absolute_score = np.full(num_points, np.inf, dtype=np.float32)
        best_raw_dist_sq = np.full(num_points, np.inf, dtype=np.float32)

        max_search_dist_sq = (float(max_range) + 5.0) ** 2 if max_range is not None else np.inf

        # For overlapping physical scans
        station_visible_indices: Dict[int, List[np.ndarray]] = {idx: [] for idx in range(num_stations)}
        wall_blocked_total = 0
        doorway_transmitted_total = 0
        backface_culled_total = 0

        for idx, pos in enumerate(stations):
            logger.info(f"Evaluating physical room & line-of-sight visibility for station {idx+1}/{num_stations}...")
            camera_loc = np.array(pos, dtype=np.float64)
            depth_map = self._build_station_depth_map(proxy_pts, camera_loc, N_AZ, N_EL)
            self.station_depth_maps[idx] = depth_map

            # Find matching floor wall grid
            best_f_grid = None
            if floor_wall_grids:
                min_f_dist = float("inf")
                for f_info in floor_wall_grids.values():
                    fd = abs(pos[2] - f_info["ref_z"])
                    if fd < min_f_dist:
                        min_f_dist = fd
                        best_f_grid = f_info

            for c_idx, start_idx in enumerate(range(0, num_points, CHUNK_SIZE)):
                end_idx = min(start_idx + CHUNK_SIZE, num_points)
                c_min, c_max = chunk_bounds[c_idx]

                # AABB distance check in float64
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
                abs_dz = np.abs(dz)
                z_weight = np.where(abs_dz > 2.0, 50.0, 1.2).astype(np.float32)
                eff_score = dx * dx + dy * dy + z_weight * (dz * dz)

                # 1. Update fallback station (only used in partition_all mode)
                abs_slice = best_absolute_score[start_idx:end_idx]
                abs_closer = eff_score < abs_slice
                if np.any(abs_closer):
                    abs_slice[abs_closer] = eff_score[abs_closer]
                    best_absolute_station[start_idx:end_idx][abs_closer] = idx
                    best_raw_dist_sq[start_idx:end_idx][abs_closer] = raw_d2[abs_closer]

                # 2. Check candidates within plausible range and same storey band
                vis_slice = best_visible_score[start_idx:end_idx]
                if allow_overlap:
                    candidate_mask = (abs_dz <= 2.2) & (raw_d2 <= max_search_dist_sq)
                else:
                    candidate_mask = (eff_score < vis_slice) & (abs_dz <= 2.2) & (raw_d2 <= max_search_dist_sq)

                if not np.any(candidate_mask):
                    continue

                sub_idx = np.where(candidate_mask)[0]
                r_sub = np.sqrt(np.maximum(raw_d2[sub_idx], 1e-12))

                # Spherical first-hit depth test with adaptive wall-tight tolerance
                u, v = self._compute_spherical_bins(dx[sub_idx], dy[sub_idx], dz[sub_idx], r_sub, N_AZ, N_EL)
                first_hit = depth_map[u, v]
                occ_tol = np.clip(OCCLUSION_TOLERANCE_NEAR_M + 0.005 * r_sub, OCCLUSION_TOLERANCE_NEAR_M, OCCLUSION_TOLERANCE_FAR_M)
                depth_vis = r_sub <= (first_hit + occ_tol)

                depth_vis_sub = sub_idx[depth_vis]
                if len(depth_vis_sub) == 0:
                    continue

                # 2D wall line-of-sight test: prevents rays from penetrating solid dividing walls
                sub_pts = chunk[depth_vis_sub]
                wall_blocked = self._check_wall_line_of_sight_vectorized(
                    camera_loc=camera_loc,
                    target_pts=sub_pts,
                    wall_grid_info=best_f_grid,
                    clearance_station=WALL_CLEARANCE_STATION_M,
                    clearance_target=WALL_CLEARANCE_TARGET_M,
                )
                wall_blocked_total += int(np.sum(wall_blocked))

                passed_wall_sub = depth_vis_sub[~wall_blocked]
                if len(passed_wall_sub) == 0:
                    continue

                # Surface normal back-face check (reject surfaces facing away from scanner by > 81 deg)
                if normals is not None and ENABLE_NORMAL_BACKFACE_CULLING:
                    c_norm = normals[start_idx:end_idx][passed_wall_sub]
                    r_p = r_sub[np.isin(sub_idx, passed_wall_sub)]
                    dot_n = (
                        dx[passed_wall_sub] * c_norm[:, 0]
                        + dy[passed_wall_sub] * c_norm[:, 1]
                        + dz[passed_wall_sub] * c_norm[:, 2]
                    ) / np.maximum(r_p, 1e-6)
                    # When cos > 0.15, normal points away from scanner (back face of wall/furniture)
                    backface = dot_n > 0.15
                    backface_culled_total += int(np.sum(backface))
                    winning_sub = passed_wall_sub[~backface]
                else:
                    winning_sub = passed_wall_sub

                if len(winning_sub) > 0:
                    if allow_overlap:
                        global_sub = start_idx + winning_sub
                        station_visible_indices[idx].append(global_sub)
                    else:
                        vis_slice[winning_sub] = eff_score[winning_sub]
                        best_visible_station[start_idx:end_idx][winning_sub] = idx
                        best_raw_dist_sq[start_idx:end_idx][winning_sub] = raw_d2[winning_sub]

        # Resolve assignments based on simulation_mode
        no_vis = best_visible_station == -1
        vis_assigned_count = int(np.sum(~no_vis))
        unassigned_count = int(np.sum(no_vis))

        logger.info(
            f"Physical Line-of-Sight Engine: {vis_assigned_count:,} points ({vis_assigned_count/max(1, num_points)*100:.1f}%) "
            f"directly observable in free space / doorways. "
            f"Blocked by solid dividing walls: {wall_blocked_total:,} ray intersections; "
            f"Back-face culled: {backface_culled_total:,} points."
        )

        if simulation_mode == "physical":
            # Strict terrestrial LiDAR reality: unobserved points behind solid walls are NEVER assigned
            final_station_assignment = np.copy(best_visible_station)
        else:
            # Legacy fallback partitioning: assign all points to nearest station
            final_station_assignment = np.where(no_vis, best_absolute_station, best_visible_station)
            logger.info(f"Legacy Partitioning: assigned {unassigned_count:,} occluded points via nearest fallback.")

        # Apply max scan range cutoff
        rejected_pts = 0
        if max_range is not None:
            max_range_sq = float(max_range ** 2)
            out_of_range = best_raw_dist_sq > max_range_sq
            final_station_assignment[out_of_range] = -1
            rejected_pts = int(np.sum(out_of_range))
            logger.info(f"Max range filter ({max_range}m) excluded {rejected_pts:,} points.")

        self.last_simulation_stats = {
            "total_input_points": int(num_points),
            "line_of_sight_points": int(vis_assigned_count),
            "unassigned_occluded_points": int(unassigned_count) if simulation_mode == "physical" else 0,
            "wall_blocked_rays": int(wall_blocked_total),
            "backface_culled_points": int(backface_culled_total),
            "out_of_range_rejected_points": int(rejected_pts),
            "simulation_mode": simulation_mode,
            "duplication_policy": "overlapping_physical" if allow_overlap else "partitioned_1_to_1",
        }

        # Build scan point index map
        scans = {}
        for idx in range(num_stations):
            if allow_overlap and len(station_visible_indices[idx]) > 0:
                station_pts = np.unique(np.concatenate(station_visible_indices[idx]))
            else:
                station_mask = final_station_assignment == idx
                station_pts = np.where(station_mask)[0]
            scans[idx] = station_pts
            logger.info(f"Scan {idx+1}/{num_stations} assigned {len(station_pts):,} points (LOS verified).")

        # Guarantee no station is left completely empty if points exist in same room
        for idx, pos in enumerate(stations):
            if len(scans[idx]) == 0 and num_points > 0:
                camera_loc = np.array(pos, dtype=np.float64)
                dists_sq = np.sum((points - camera_loc) ** 2, axis=1)
                # Only populate with close local points in immediate free space (< 3.0m)
                local_mask = dists_sq <= (3.0 ** 2)
                within_r = np.where(local_mask)[0]
                if len(within_r) > 0:
                    sub_order = np.argsort(dists_sq[within_r])[: min(100, len(within_r))]
                    nearest_100 = within_r[sub_order]
                    scans[idx] = nearest_100
                    logger.info(f"Scan {idx+1} populated with {len(nearest_100)} immediate local points.")
                else:
                    logger.warning(
                        f"Virtual scan station {idx+1} has 0 line-of-sight visible points "
                        f"(no unoccluded geometry within range)."
                    )

        return scans


