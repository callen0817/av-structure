import numpy as np
import open3d as o3d
import logging
from typing import List

logger = logging.getLogger(__name__)

class GeometryStationGenerator:
    def __init__(self, target_spacing: float = 6.0, tripod_height: float = 1.6):
        """
        Generates candidate stations unconditionally from raw point cloud geometry.
        """
        self.target_spacing = target_spacing
        self.tripod_height = tripod_height
        
    def generate(self, pcd: o3d.geometry.PointCloud) -> List[tuple]:
        """
        Computes free space and reasonable locations for scan placement
        based on local vertical geometric bounds across single or multiple building levels.
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
                    if hist[b_idx] >= threshold and hist[b_idx] >= hist[b_idx - 1] and hist[b_idx] >= hist[b_idx + 1]:
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
            logger.info(f"Detected {len(storey_bands)} vertical building levels: {[(round(a, 2), round(b, 2)) for a, b in storey_bands]}")

        # 2. Vectorized 2D (X, Y) grid cell grouping
        spacing = float(self.target_spacing)
        ix = np.floor((points[:, 0] - min_bound[0]) / spacing).astype(np.int32)
        iy = np.floor((points[:, 1] - min_bound[1]) / spacing).astype(np.int32)
        
        nx = int(ix.max()) + 1
        ny = int(iy.max()) + 1
        cell_ids = ix * ny + iy
        
        order = np.argsort(cell_ids, kind='mergesort')
        sorted_cells = cell_ids[order]
        sorted_pts = points[order]
        
        unique_cells, split_indices = np.unique(sorted_cells, return_index=True)
        cell_Point_groups = np.split(sorted_pts, split_indices[1:])
        
        candidates = []
        for cid, cell_pts in zip(unique_cells, cell_Point_groups):
            if len(cell_pts) <= 10:
                continue
            cx_idx = cid // ny
            cy_idx = cid % ny
            x = float(min_bound[0] + (cx_idx + 0.5) * spacing)
            y = float(min_bound[1] + (cy_idx + 0.5) * spacing)
            
            cell_z = cell_pts[:, 2]
            added_in_cell = []
            
            for b_low, b_high in storey_bands:
                band_mask = (cell_z >= b_low) & (cell_z <= b_high)
                band_pts = cell_pts[band_mask]
                if len(band_pts) > 8:
                    floor_z = float(np.percentile(band_pts[:, 2], 5))
                    ceil_z = float(np.percentile(band_pts[:, 2], 95))
                    
                    if ceil_z - floor_z > 2.0:
                        st_z = floor_z + self.tripod_height
                        # Ensure station is not colliding with a dense wall/column at scanner head height
                        head_dx = band_pts[:, 0] - x
                        head_dy = band_pts[:, 1] - y
                        head_dz = band_pts[:, 2] - st_z
                        collision_pts = np.sum((head_dx**2 + head_dy**2 + head_dz**2) < (0.30 ** 2))
                        if collision_pts <= 3:
                            if all(abs(st_z - prev_z) > 1.8 for prev_z in added_in_cell):
                                candidates.append((x, y, st_z))
                                added_in_cell.append(st_z)

        logger.info(f"Generated {len(candidates)} geometric candidate stations across {len(storey_bands)} level(s).")
        return candidates

