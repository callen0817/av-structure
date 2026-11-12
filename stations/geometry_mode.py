import numpy as np
import open3d as o3d
import logging
from typing import List, Dict, Tuple

logger = logging.getLogger(__name__)


class GeometryStationGenerator:
    def __init__(self, target_spacing: float = 6.0, tripod_height: float = 1.6):
        """
        Generates candidate stations unconditionally from raw point cloud geometry.
        """
        self.target_spacing = max(0.5, float(target_spacing))
        self.tripod_height = float(tripod_height)
        self.candidate_confidences: Dict[Tuple[float, float, float], float] = {}
        self.placement_uncertain: bool = False

    @staticmethod
    def _evaluate_station_clearance_and_confidence(
        band_pts: np.ndarray, x: float, y: float, z: float
    ) -> Tuple[bool, float, float]:
        """
        Evaluates whether a candidate position (x, y, z) is in free space (not inside walls/columns)
        and computes a free-space confidence score in [0.0, 1.0].
        Returns: (is_valid_free_space, min_clearance_m, confidence_score)
        """
        dx = band_pts[:, 0] - x
        dy = band_pts[:, 1] - y
        dz = band_pts[:, 2] - z
        dist_sq = dx * dx + dy * dy + dz * dz

        # Reject if inside solid geometry at scanner head height (within 0.30m sphere)
        collision_pts = int(np.sum(dist_sq < (0.30 ** 2)))
        if collision_pts > 2:
            return False, 0.0, 0.0

        # Clearance to nearest surface near scanner head height (|dz| <= 0.6m)
        head_band = np.abs(dz) <= 0.60
        if np.any(head_band):
            horiz_d = np.sqrt(dx[head_band] ** 2 + dy[head_band] ** 2)
            min_clearance = float(np.min(horiz_d)) if len(horiz_d) > 0 else 1.5
        else:
            min_clearance = float(np.sqrt(np.min(dist_sq))) if len(dist_sq) > 0 else 1.5

        if min_clearance < 0.28:
            return False, min_clearance, 0.0

        # Check angular quadrant coverage (interior vs boundary confidence)
        az = np.arctan2(dy, dx)
        q1 = np.any((az >= 0) & (az < 0.5 * np.pi))
        q2 = np.any(az >= 0.5 * np.pi)
        q3 = np.any(az < -0.5 * np.pi)
        q4 = np.any((az >= -0.5 * np.pi) & (az < 0))
        quadrant_ratio = (int(q1) + int(q2) + int(q3) + int(q4)) / 4.0

        clearance_score = min(1.0, max(0.2, (min_clearance - 0.25) / 1.0))
        confidence = float(np.clip(0.55 * clearance_score + 0.45 * quadrant_ratio, 0.15, 1.0))
        return True, min_clearance, confidence

    def generate(self, pcd: o3d.geometry.PointCloud) -> List[tuple]:
        """
        Computes free space and reasonable locations for scan placement
        based on local vertical geometric bounds across single or multiple building levels.
        Returns a list of (X, Y, Z) tuples.
        """
        logger.info("Generating candidate stations purely from point cloud geometry...")
        self.candidate_confidences = {}
        self.placement_uncertain = False

        if pcd is None or pcd.is_empty():
            return []

        raw_points = np.asarray(pcd.points)
        if len(raw_points) == 0:
            return []

        # Shift to local frame for voxelization if coordinates are very large (e.g., UTM)
        origin_shift = np.median(raw_points, axis=0)
        shifted_pcd = o3d.geometry.PointCloud()
        shifted_pcd.points = o3d.utility.Vector3dVector(raw_points - origin_shift)

        voxel_size = 0.2 if len(raw_points) > 5000 else 0.05
        downpcd = shifted_pcd.voxel_down_sample(voxel_size)
        points = np.asarray(downpcd.points) + origin_shift

        if len(points) == 0:
            points = raw_points

        min_bound = points.min(axis=0)
        max_bound = points.max(axis=0)

        # 1. Discover vertical storey bands from Z-density peaks (handles multi-level buildings)
        z_all = points[:, 2]
        z_min, z_max = float(min_bound[2]), float(max_bound[2])
        z_span = z_max - z_min

        storey_bands = []
        if z_span > 4.2:
            bin_h = 0.25
            bins = np.arange(z_min, z_max + bin_h, bin_h)
            if len(bins) > 4:
                hist, edges = np.histogram(z_all, bins=bins)
                # Find horizontal slab peaks (bins with high point count) separated by >= 2.2m (9 bins)
                slab_elevations = [z_min]
                threshold = np.percentile(hist, 65)
                for b_idx in range(1, len(hist) - 1):
                    if (
                        hist[b_idx] >= threshold
                        and hist[b_idx] >= hist[b_idx - 1]
                        and hist[b_idx] >= hist[b_idx + 1]
                    ):
                        elev = float(0.5 * (edges[b_idx] + edges[b_idx + 1]))
                        if elev - slab_elevations[-1] >= 2.2:
                            slab_elevations.append(elev)
                if z_max - slab_elevations[-1] >= 2.0:
                    slab_elevations.append(z_max)
                else:
                    slab_elevations[-1] = z_max

                if len(slab_elevations) >= 3:
                    for i in range(len(slab_elevations) - 1):
                        storey_bands.append((slab_elevations[i] - 0.2, slab_elevations[i + 1] + 0.2))

        if not storey_bands:
            storey_bands = [(z_min - 0.1, z_max + 0.1)]
        else:
            logger.info(
                f"Detected {len(storey_bands)} vertical building levels: {[(round(a, 2), round(b, 2)) for a, b in storey_bands]}"
            )

        # 2. Vectorized 2D (X, Y) grid cell grouping
        spacing = float(self.target_spacing)
        ix = np.floor((points[:, 0] - min_bound[0]) / spacing).astype(np.int32)
        iy = np.floor((points[:, 1] - min_bound[1]) / spacing).astype(np.int32)

        nx = int(ix.max()) + 1
        ny = int(iy.max()) + 1
        cell_ids = ix * ny + iy

        order = np.argsort(cell_ids, kind="mergesort")
        sorted_cells = cell_ids[order]
        sorted_pts = points[order]

        unique_cells, split_indices = np.unique(sorted_cells, return_index=True)
        cell_point_groups = np.split(sorted_pts, split_indices[1:])

        candidates = []
        sub_offsets = [
            (0.0, 0.0),
            (-0.22 * spacing, 0.0),
            (0.22 * spacing, 0.0),
            (0.0, -0.22 * spacing),
            (0.0, 0.22 * spacing),
        ]

        for cid, cell_pts in zip(unique_cells, cell_point_groups):
            if len(cell_pts) <= 10:
                continue
            cx_idx = cid // ny
            cy_idx = cid % ny
            base_x = float(min_bound[0] + (cx_idx + 0.5) * spacing)
            base_y = float(min_bound[1] + (cy_idx + 0.5) * spacing)

            cell_z = cell_pts[:, 2]
            added_in_cell = []

            for b_low, b_high in storey_bands:
                band_mask = (cell_z >= b_low) & (cell_z <= b_high)
                band_pts = cell_pts[band_mask]
                if len(band_pts) > 8:
                    floor_z = float(np.percentile(band_pts[:, 2], 5))
                    ceil_z = float(np.percentile(band_pts[:, 2], 95))

                    if ceil_z - floor_z > 2.0:
                        st_z = floor_z + min(self.tripod_height, 0.65 * (ceil_z - floor_z))
                        if not all(abs(st_z - prev_z) > 1.8 for prev_z in added_in_cell):
                            continue

                        # Try cell center first, then local free-space offsets if center collides with wall/column
                        best_pos = None
                        best_clearance = -1.0
                        best_conf = 0.0

                        for ox, oy in sub_offsets:
                            cand_x = base_x + ox
                            cand_y = base_y + oy
                            ok, clearance, conf = self._evaluate_station_clearance_and_confidence(
                                band_pts, cand_x, cand_y, st_z
                            )
                            if ok and clearance > best_clearance:
                                best_pos = (cand_x, cand_y, st_z)
                                best_clearance = clearance
                                best_conf = conf
                                if ox == 0.0 and oy == 0.0 and clearance >= 0.60:
                                    break

                        if best_pos is not None:
                            candidates.append(best_pos)
                            added_in_cell.append(st_z)
                            key = (round(best_pos[0], 3), round(best_pos[1], 3), round(best_pos[2], 3))
                            self.candidate_confidences[key] = best_conf

        # 3. Conservative fallback when free space cannot be established via standard indoor floor-ceiling slabs
        # (e.g., sparse scans, exterior scenes, low-ceiling spaces, or small test fixtures)
        if not candidates and len(points) > 0:
            self.placement_uncertain = True
            logger.warning(
                "Placement Uncertainty Warning: Standard indoor floor-to-ceiling headroom (>2.0m) was not detected. "
                "Using conservative free-space candidate placement outside solid geometry."
            )
            center = np.mean(points, axis=0)
            span = max_bound - min_bound
            eff_spacing = min(spacing, max(1.0, float(np.max(span[:2])) / 2.5))

            xs = np.arange(min_bound[0] + 0.25 * span[0], max_bound[0] - 0.1 * span[0] + 1e-5, eff_spacing)
            ys = np.arange(min_bound[1] + 0.25 * span[1], max_bound[1] - 0.1 * span[1] + 1e-5, eff_spacing)
            if len(xs) == 0:
                xs = np.array([float(center[0])])
            if len(ys) == 0:
                ys = np.array([float(center[1])])

            floor_est = float(np.percentile(points[:, 2], 10))
            ceil_est = float(np.percentile(points[:, 2], 90))
            st_z = float(center[2]) if (ceil_est - floor_est) < 1.2 else (floor_est + min(self.tripod_height, 0.5 * (ceil_est - floor_est)))

            for x in xs:
                for y in ys:
                    cand = np.array([float(x), float(y), st_z], dtype=np.float64)
                    dists = np.linalg.norm(points - cand, axis=1)
                    if np.min(dists) < 0.30:
                        # Shift slightly toward local centroid away from nearest obstacle
                        nearest_pt = points[int(np.argmin(dists))]
                        away = cand - nearest_pt
                        norm_away = float(np.linalg.norm(away))
                        if norm_away > 1e-6:
                            cand = nearest_pt + (away / norm_away) * 0.60
                        else:
                            cand[2] += 0.60
                    pos_t = (float(cand[0]), float(cand[1]), float(cand[2]))
                    candidates.append(pos_t)
                    key = (round(pos_t[0], 3), round(pos_t[1], 3), round(pos_t[2], 3))
                    self.candidate_confidences[key] = 0.55

        # Sort deterministically
        candidates.sort(key=lambda p: (round(p[2], 2), round(p[0], 2), round(p[1], 2)))
        logger.info(
            f"Generated {len(candidates)} geometric candidate stations across {len(storey_bands)} level(s) "
            f"(placement_uncertain={self.placement_uncertain})."
        )
        return candidates


