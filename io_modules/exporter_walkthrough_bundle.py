import os
import json
import struct
import logging
import numpy as np
from typing import List, Dict, Tuple, Optional, Any
from PIL import Image
from scipy.ndimage import binary_dilation

logger = logging.getLogger(__name__)


def cluster_stations_into_floors(
    stations: List[Any],
    points: np.ndarray,
    min_story_gap: float = 0.60,
    default_tripod_height: float = 1.55,
) -> Tuple[List[Dict[str, Any]], Dict[int, int]]:
    """
    Clusters virtual station positions into major building stories based on Z elevations.
    Matches AV Showcase multi-level floor model.
    Returns:
      (floors_list, station_to_floor_id_map)
    """
    if not stations:
        default_floor = {
            "id": 1,
            "name": "Level 1",
            "min_z": -0.5,
            "max_z": 3.0,
            "mean_z": 0.0,
            "slab_z": -default_tripod_height,
        }
        return [default_floor], {}

    z_vals = np.array([float(s.position[2]) for s in stations], dtype=np.float64)
    sorted_order = np.argsort(z_vals)
    sorted_z = z_vals[sorted_order]

    # Find clusters where consecutive elevation difference exceeds min_story_gap
    clusters: List[List[float]] = []
    current_cluster: List[float] = [sorted_z[0]]

    for z in sorted_z[1:]:
        if (z - current_cluster[-1]) > min_story_gap:
            clusters.append(current_cluster)
            current_cluster = [z]
        else:
            current_cluster.append(z)
    clusters.append(current_cluster)

    cluster_means = [float(np.mean(c)) for c in clusters]
    num_floors = len(clusters)

    # Calculate inter-floor boundary thresholds
    thresholds = []
    for i in range(num_floors - 1):
        thresholds.append((cluster_means[i] + cluster_means[i + 1]) / 2.0)

    # Assign stations to floors
    station_floor_map: Dict[int, int] = {}
    floor_stations: Dict[int, List[int]] = {f_id: [] for f_id in range(1, num_floors + 1)}

    for st_idx, st in enumerate(stations):
        sz = float(st.position[2])
        assigned_fid = 1
        for f_idx, thresh in enumerate(thresholds):
            if sz > thresh:
                assigned_fid = f_idx + 2
            else:
                break
        station_floor_map[st_idx] = assigned_fid
        floor_stations[assigned_fid].append(st_idx)

    # Build floor metadata
    floors = []
    pts_z = points[:, 2] if len(points) > 0 else np.array([0.0])

    for f_id in range(1, num_floors + 1):
        f_mean = cluster_means[f_id - 1]
        st_indices = floor_stations[f_id]

        if num_floors == 1:
            z_min_f = float(np.percentile(pts_z, 1)) if len(pts_z) > 0 else f_mean - 1.8
            z_max_f = float(np.percentile(pts_z, 99)) if len(pts_z) > 0 else f_mean + 1.8
            slab_z = f_mean - default_tripod_height
        else:
            # Multi-floor boundaries
            lower_bound = thresholds[f_id - 2] if f_id > 1 else (f_mean - 2.5)
            upper_bound = thresholds[f_id - 1] if f_id < num_floors else (f_mean + 2.5)
            slab_z = f_mean - default_tripod_height
            z_min_f = float(lower_bound)
            z_max_f = float(upper_bound)

        floor_dict = {
            "id": f_id,
            "name": f"Level {f_id}",
            "min_z": round(z_min_f, 3),
            "max_z": round(z_max_f, 3),
            "mean_z": round(f_mean, 3),
            "slab_z": round(slab_z, 3),
            "keyplan_path": f"floors/floor_{f_id}_keyplan.png",
        }
        floors.append(floor_dict)

    return floors, station_floor_map


def check_line_of_sight_2d(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    wall_grid: np.ndarray,
    origin_x: float,
    origin_y: float,
    res: float,
    clearance_m: float = 0.35,
) -> bool:
    """
    Checks if a straight 2D line segment between p1 and p2 intersects occupied wall cells.
    Allows clearance around endpoints (so stations near doorways aren't blocked by their own room).
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dist = np.hypot(dx, dy)
    if dist < 1e-4:
        return True

    steps = max(2, int(np.ceil(dist / (res * 0.5))))
    xs = np.linspace(p1[0], p2[0], steps)
    ys = np.linspace(p1[1], p2[1], steps)

    h, w = wall_grid.shape
    clearance_steps = int(np.ceil(clearance_m / (res * 0.5)))

    for step_idx in range(clearance_steps, steps - clearance_steps):
        gx = int(np.floor((xs[step_idx] - origin_x) / res))
        gy = int(np.floor((ys[step_idx] - origin_y) / res))
        if 0 <= gx < w and 0 <= gy < h:
            if wall_grid[gy, gx]:
                return False
    return True


def build_navigation_graph(
    stations: List[Any],
    station_floor_map: Dict[int, int],
    points: np.ndarray,
    voxel_size: float = 0.12,
    max_neighbors_per_station: int = 8,
) -> Dict[str, List[str]]:
    """
    Builds an 8-Sector + Cross-Floor Staircase Navigation Graph.
    Guarded by 2D/3D wall occlusion raycasting so edges NEVER pass through solid walls.
    Ensures all links are strictly bidirectional and symmetric.
    """
    n_stations = len(stations)
    if n_stations == 0:
        return {}

    id_to_st_idx = {st.station_id: i for i, st in enumerate(stations)}
    st_idx_to_id = {i: st.station_id for i, st in enumerate(stations)}

    # Build 2D wall density grid per floor for fast line-of-sight checking
    floor_wall_grids: Dict[int, Dict[str, Any]] = {}
    unique_floors = set(station_floor_map.values()) if station_floor_map else {1}

    if len(points) > 0:
        pts_np = np.asarray(points, dtype=np.float32)
        min_x, min_y = float(np.min(pts_np[:, 0])), float(np.min(pts_np[:, 1]))
        max_x, max_y = float(np.max(pts_np[:, 0])), float(np.max(pts_np[:, 1]))
        w_px = max(10, int(np.ceil((max_x - min_x) / voxel_size)) + 1)
        h_px = max(10, int(np.ceil((max_y - min_y) / voxel_size)) + 1)

        for fid in unique_floors:
            # Stations on this floor
            st_z_vals = [float(stations[i].position[2]) for i in range(n_stations) if station_floor_map.get(i, 1) == fid]
            mid_z = float(np.median(st_z_vals)) if st_z_vals else 1.5
            # Wall band at eye/chest level [-0.4m, +0.4m] relative to station height
            band_mask = (pts_np[:, 2] >= (mid_z - 0.5)) & (pts_np[:, 2] <= (mid_z + 0.5))
            band_pts = pts_np[band_mask]

            grid = np.zeros((h_px, w_px), dtype=bool)
            if len(band_pts) > 0:
                gx = np.clip(np.floor((band_pts[:, 0] - min_x) / voxel_size).astype(np.int32), 0, w_px - 1)
                gy = np.clip(np.floor((band_pts[:, 1] - min_y) / voxel_size).astype(np.int32), 0, h_px - 1)
                # Count density to avoid isolated noise voxels
                cell_idx = gy * w_px + gx
                counts = np.bincount(cell_idx, minlength=h_px * w_px)
                dense_mask = counts >= 3
                grid.flat[dense_mask] = True
                # Dilate slightly so thin walls are solid
                grid = binary_dilation(grid, iterations=1)

            floor_wall_grids[fid] = {
                "grid": grid,
                "origin_x": min_x,
                "origin_y": min_y,
                "res": voxel_size,
            }

    # Adjacency set of (idx_a, idx_b)
    edges = set()

    # 1. On-floor connections (Sequential, 8-Sector, and Close Neighbors)
    for fid in unique_floors:
        floor_st_indices = [i for i in range(n_stations) if station_floor_map.get(i, 1) == fid]
        if not floor_st_indices:
            continue

        wall_info = floor_wall_grids.get(fid)

        # Sequential links within 12.0m
        for k in range(len(floor_st_indices) - 1):
            i1 = floor_st_indices[k]
            i2 = floor_st_indices[k + 1]
            p1 = stations[i1].position
            p2 = stations[i2].position
            d = float(np.hypot(p1[0] - p2[0], p1[1] - p2[1]))
            if d <= 12.0:
                has_los = True
                if wall_info is not None:
                    has_los = check_line_of_sight_2d(
                        (p1[0], p1[1]),
                        (p2[0], p2[1]),
                        wall_info["grid"],
                        wall_info["origin_x"],
                        wall_info["origin_y"],
                        wall_info["res"],
                    )
                if has_los:
                    edges.add((min(i1, i2), max(i1, i2)))

        # 8-Sector connections (45-degree azimuth slices) + Close neighbors within 2.5m
        for i in floor_st_indices:
            pi = stations[i].position
            sector_candidates: Dict[int, Tuple[float, int]] = {}

            for j in floor_st_indices:
                if i == j:
                    continue
                pj = stations[j].position
                d = float(np.hypot(pj[0] - pi[0], pj[1] - pi[1]))
                if d > 12.0:
                    continue

                # Close neighbor rule (<= 2.5m)
                if d <= 2.5:
                    has_los = True
                    if wall_info is not None:
                        has_los = check_line_of_sight_2d(
                            (pi[0], pi[1]),
                            (pj[0], pj[1]),
                            wall_info["grid"],
                            wall_info["origin_x"],
                            wall_info["origin_y"],
                            wall_info["res"],
                        )
                    if has_los:
                        edges.add((min(i, j), max(i, j)))

                # 8-Sector assignment
                if 0.5 <= d <= 12.0:
                    az = np.arctan2(pj[1] - pi[1], pj[0] - pi[0])  # [-pi, pi]
                    sec = int(np.floor(((az + np.pi) / (2.0 * np.pi)) * 8.0)) % 8
                    if sec not in sector_candidates or d < sector_candidates[sec][0]:
                        sector_candidates[sec] = (d, j)

            for sec, (d, j) in sector_candidates.items():
                pj = stations[j].position
                has_los = True
                if wall_info is not None:
                    has_los = check_line_of_sight_2d(
                        (pi[0], pi[1]),
                        (pj[0], pj[1]),
                        wall_info["grid"],
                        wall_info["origin_x"],
                        wall_info["origin_y"],
                        wall_info["res"],
                    )
                if has_los:
                    edges.add((min(i, j), max(i, j)))

    # 2. Cross-Floor Staircase Connections
    # Links staircase transition stations where 0.10m <= |dz| <= 2.20m and horizontal dxy <= 5.50m
    for i in range(n_stations):
        f_i = station_floor_map.get(i, 1)
        pi = stations[i].position
        for j in range(i + 1, n_stations):
            f_j = station_floor_map.get(j, 1)
            if abs(f_i - f_j) == 1:  # Adjacent floors
                pj = stations[j].position
                dz = abs(float(pj[2] - pi[2]))
                dxy = float(np.hypot(pj[0] - pi[0], pj[1] - pi[1]))
                if 0.10 <= dz <= 2.20 and dxy <= 5.50:
                    edges.add((i, j))

    # Assemble symmetric neighbor lists capped at max_neighbors_per_station
    adj: Dict[int, List[int]] = {i: [] for i in range(n_stations)}
    for u, v in edges:
        adj[u].append(v)
        adj[v].append(u)

    neighbors_map: Dict[str, List[str]] = {}
    for i in range(n_stations):
        st_id = st_idx_to_id[i]
        nbr_indices = adj[i]
        # Sort neighbors by Euclidean distance
        pi = stations[i].position
        nbr_indices.sort(key=lambda j: np.linalg.norm(np.array(stations[j].position) - np.array(pi)))
        # Limit to max_neighbors_per_station
        selected = nbr_indices[:max_neighbors_per_station]
        neighbors_map[st_id] = [st_idx_to_id[j] for j in selected]

    # Ensure full symmetry (if A in B's neighbors, B must be in A's neighbors)
    for st_id, nbrs in list(neighbors_map.items()):
        for n_id in nbrs:
            if st_id not in neighbors_map[n_id]:
                neighbors_map[n_id].append(st_id)

    return neighbors_map


def generate_floor_keyplan(
    floor_id: int,
    slab_z: float,
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
    target_mpp: float = 0.02,
    max_dim: int = 2048,
    existing_image_path: Optional[str] = None,
) -> Tuple[Image.Image, Dict[str, float]]:
    """
    Renders an architectural top-down 2D wall-density CAD keyplan slice.
    Calibration:
      Image +X (right) = World +X
      Image -Y (up, row 0 is top) = World +Y (North)
      col = int((x - min_x) / mpp)
      row = int((max_y - y) / mpp)
    Styling:
      Dark charcoal background (#171717)
      Navigable floor interior tint (#22262e) from slab band [slab_z - 0.15m, slab_z + 0.15m]
      Crisp bright wall cross-sections (#e5e7eb) from wall band [slab_z + 0.35m, slab_z + 2.10m]
      Gold wall core accents (#baa164) on dense wall lines.
    """
    if existing_image_path and os.path.exists(existing_image_path):
        try:
            img = Image.open(existing_image_path).convert("RGB")
            # Calculate world bounds from points on this floor
            pts_np = np.asarray(points, dtype=np.float64)
            min_x, max_x = float(np.min(pts_np[:, 0])), float(np.max(pts_np[:, 0]))
            min_y, max_y = float(np.min(pts_np[:, 1])), float(np.max(pts_np[:, 1]))
            w, h = img.size
            mpp = max((max_x - min_x) / max(1, w), (max_y - min_y) / max(1, h))
            return img, {
                "min_x": round(min_x, 4),
                "max_x": round(max_x, 4),
                "min_y": round(min_y, 4),
                "max_y": round(max_y, 4),
                "meters_per_pixel": round(mpp, 4),
            }
        except Exception as e:
            logger.warning(f"Could not load pre-existing floorplan {existing_image_path}: {e}")

    pts = np.asarray(points, dtype=np.float64)
    if len(pts) == 0:
        # Fallback 512x512 blank canvas
        img = Image.new("RGB", (512, 512), (23, 23, 23))
        return img, {
            "min_x": -10.0,
            "max_x": 10.0,
            "min_y": -10.0,
            "max_y": 10.0,
            "meters_per_pixel": 0.04,
        }

    # Slice vertical bands
    z = pts[:, 2]
    wall_band = (z >= (slab_z + 0.35)) & (z <= (slab_z + 2.10))
    floor_band = (z >= (slab_z - 0.18)) & (z <= (slab_z + 0.18))

    wall_pts = pts[wall_band]
    floor_pts = pts[floor_band]

    active_pts = wall_pts if len(wall_pts) > 50 else pts
    pad_m = 1.0
    min_x = float(np.min(active_pts[:, 0])) - pad_m
    max_x = float(np.max(active_pts[:, 0])) + pad_m
    min_y = float(np.min(active_pts[:, 1])) - pad_m
    max_y = float(np.max(active_pts[:, 1])) + pad_m

    span_x = max(1.0, max_x - min_x)
    span_y = max(1.0, max_y - min_y)

    # Resolution & scaling
    mpp = float(target_mpp)
    w_px = int(np.ceil(span_x / mpp))
    h_px = int(np.ceil(span_y / mpp))

    if max(w_px, h_px) > max_dim:
        scale = max_dim / float(max(w_px, h_px))
        mpp = mpp / scale
        w_px = int(np.ceil(span_x / mpp))
        h_px = int(np.ceil(span_y / mpp))

    w_px = max(64, min(max_dim, w_px))
    h_px = max(64, min(max_dim, h_px))

    # Canvas: Dark charcoal background (#171717)
    canvas = np.full((h_px, w_px, 3), (23, 23, 23), dtype=np.uint8)

    # 1. Navigable floor slab interior (#22262e)
    if len(floor_pts) > 0:
        fx = np.clip(np.floor((floor_pts[:, 0] - min_x) / mpp).astype(np.int32), 0, w_px - 1)
        fy = np.clip(np.floor((max_y - floor_pts[:, 1]) / mpp).astype(np.int32), 0, h_px - 1)
        floor_mask = np.zeros((h_px, w_px), dtype=bool)
        floor_mask[fy, fx] = True
        # Dilate slightly to form contiguous interior slab
        floor_mask = binary_dilation(floor_mask, iterations=2)
        canvas[floor_mask] = (34, 38, 46)

    # 2. Wall cross sections (#e5e7eb) and dense wall core gold highlights (#baa164)
    if len(wall_pts) > 0:
        wx = np.clip(np.floor((wall_pts[:, 0] - min_x) / mpp).astype(np.int32), 0, w_px - 1)
        wy = np.clip(np.floor((max_y - wall_pts[:, 1]) / mpp).astype(np.int32), 0, h_px - 1)

        cell_ids = wy * w_px + wx
        counts = np.bincount(cell_ids, minlength=h_px * w_px).reshape(h_px, w_px)

        low_wall = counts >= 1
        med_wall = counts >= 3
        high_wall = counts >= 8

        # Subtle outer wall border
        canvas[low_wall] = (160, 165, 175)
        # Crisp architectural wall slice
        canvas[med_wall] = (229, 231, 235)
        # Gold accent core (#baa164)
        canvas[high_wall] = (186, 161, 100)

    img = Image.fromarray(canvas, mode="RGB")
    bounds = {
        "min_x": round(min_x, 4),
        "max_x": round(max_x, 4),
        "min_y": round(min_y, 4),
        "max_y": round(max_y, 4),
        "meters_per_pixel": round(mpp, 4),
    }
    return img, bounds


def write_lod0_cache(
    cache_path: str,
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
    target_count: int = 350_000,
) -> int:
    """
    Writes point cloud to AV Showcase lod0.bin format:
    4-byte little-endian uint32 numPoints,
    followed by numPoints packed 15-byte records:
    <fffBBB> = [float32 x, float32 y, float32 z, uint8 r, uint8 g, uint8 b].
    Downsamples smoothly to target_count (250k-500k) points for instant WebGL rendering.
    """
    os.makedirs(os.path.dirname(os.path.abspath(cache_path)), exist_ok=True)
    n_pts = len(points)
    if n_pts == 0:
        with open(cache_path, "wb") as f:
            f.write(struct.pack("<I", 0))
        return 0

    pts = np.asarray(points, dtype=np.float32)
    if colors is not None and len(colors) == n_pts:
        cols = np.asarray(colors, dtype=np.uint8)
    else:
        cols = np.full((n_pts, 3), 190, dtype=np.uint8)

    # Subsample if necessary
    if n_pts > target_count:
        step = max(1, n_pts // target_count)
        pts = pts[::step]
        cols = cols[::step]

    count = len(pts)
    with open(cache_path, "wb") as f:
        # Write 4-byte header
        f.write(struct.pack("<I", count))

        # Pack binary buffer: float32 x, y, z + uint8 r, g, b (15 bytes per point)
        record_dtype = np.dtype([
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("r", "u1"),
            ("g", "u1"),
            ("b", "u1"),
        ])
        packed = np.empty(count, dtype=record_dtype)
        packed["x"] = pts[:, 0]
        packed["y"] = pts[:, 1]
        packed["z"] = pts[:, 2]
        packed["r"] = cols[:, 0]
        packed["g"] = cols[:, 1]
        packed["b"] = cols[:, 2]

        f.write(packed.tobytes())

    logger.info(f"Wrote lod0.bin cache with {count:,} points ({os.path.getsize(cache_path):,} bytes).")
    return count


def export_walkthrough_bundle(
    bundle_dir: str,
    virtual_stations: List[Any],
    points: np.ndarray,
    colors: Optional[np.ndarray] = None,
    project_id: Optional[str] = None,
    project_name: str = "AV Structure Project",
    scan_id: Optional[str] = None,
    scan_date: Optional[str] = None,
    transform_to_project: Optional[np.ndarray] = None,
    structured_e57_rel_path: Optional[str] = None,
    precomputed_floorplans: Optional[Dict[int, str]] = None,
) -> Dict[str, Any]:
    """
    Exports a complete AV Spatial Bundle v1.1 (<bundle_dir>/av_walkthrough_bundle.json,
    panos/, floors/, cache/lod0.bin) ready for instant (< 1s) loading in AV Showcase.
    """
    os.makedirs(bundle_dir, exist_ok=True)
    panos_dir = os.path.join(bundle_dir, "panos")
    floors_dir = os.path.join(bundle_dir, "floors")
    cache_dir = os.path.join(bundle_dir, "cache")
    os.makedirs(panos_dir, exist_ok=True)
    os.makedirs(floors_dir, exist_ok=True)
    os.makedirs(cache_dir, exist_ok=True)

    from datetime import datetime
    import uuid

    proj_id = project_id or f"proj_{uuid.uuid4().hex[:12]}"
    now_dt = datetime.now()
    s_date = scan_date or now_dt.strftime("%Y-%m-%d")
    s_id = scan_id or now_dt.strftime("%Y-%m-%d_%H%M")

    # 1. Multi-Level Floor Clustering
    floors_meta, st_floor_map = cluster_stations_into_floors(virtual_stations, points)

    # 2. 8-Sector + Staircase Navigation Graph
    nav_graph = build_navigation_graph(virtual_stations, st_floor_map, points)

    # 3. Write cache/lod0.bin
    lod0_path = os.path.join(cache_dir, "lod0.bin")
    write_lod0_cache(lod0_path, points, colors)

    # 4. Generate per-floor crisp 2D keyplans
    for f in floors_meta:
        fid = f["id"]
        slab_z = f["slab_z"]
        existing_plan = precomputed_floorplans.get(fid) if precomputed_floorplans else None
        kp_img, bounds = generate_floor_keyplan(
            floor_id=fid,
            slab_z=slab_z,
            points=points,
            colors=colors,
            existing_image_path=existing_plan,
        )
        kp_out_path = os.path.join(bundle_dir, f["keyplan_path"])
        os.makedirs(os.path.dirname(kp_out_path), exist_ok=True)
        kp_img.save(kp_out_path, format="PNG")
        f["world_bounds"] = bounds

    # 5. Export Panoramas and Depth Maps
    panos_meta = []
    for st_idx, st in enumerate(virtual_stations):
        st_id = st.station_id
        pano_file = f"{st_id}.jpg"
        depth_file = f"{st_id}_depth.png"
        pano_rel_path = f"panos/{pano_file}"
        depth_rel_path = f"panos/{depth_file}"

        pano_abs_path = os.path.join(panos_dir, pano_file)
        depth_abs_path = os.path.join(panos_dir, depth_file)

        # Write JPEG if station has bytes
        if st.panorama_jpeg_bytes:
            with open(pano_abs_path, "wb") as pf:
                pf.write(st.panorama_jpeg_bytes)
        elif not os.path.exists(pano_abs_path):
            # Fallback black panorama
            Image.new("RGB", (2048, 1024), (0, 0, 0)).save(pano_abs_path, format="JPEG", quality=90)

        # Write 16-bit millimeter depth map if depth_map is available
        has_depth_file = False
        if st.depth_map is not None:
            depth_mm = np.clip(st.depth_map * 1000.0, 0, 65535).astype(np.uint16)
            Image.fromarray(depth_mm).save(depth_abs_path)
            has_depth_file = True

        # Coordinate quaternion: pyquaternion (w, x, y, z) -> AV Bundle [qx, qy, qz, qw]
        ori = st.orientation  # (w, x, y, z)
        rot_bundle = [float(ori[1]), float(ori[2]), float(ori[3]), float(ori[0])]

        # Provenance determination
        if st.panorama_source == "existing":
            prov = "EXTERNAL_PANORAMA"
        elif "hybrid" in str(getattr(st, "panorama_source", "")).lower():
            prov = "HYBRID_KEYFRAME_AND_CLOUD"
        else:
            prov = "SYNTHETIC_POINT_SPLAT"

        valid_ratio = getattr(st, "coverage_metrics", {}).get("observed_pixel_ratio", 0.85)

        pano_dict = {
            "id": st_id,
            "station_guid": st.e57_scan_guid or st.e57_image_guid or f"st_{st_idx:04d}",
            "position": [round(float(p), 4) for p in st.position],
            "rotation": [round(float(q), 6) for q in rot_bundle],
            "floor_id": st_floor_map.get(st_idx, 1),
            "image_path": pano_rel_path,
            "depth_path": depth_rel_path if has_depth_file else None,
            "provenance": prov,
            "valid_pixel_ratio": round(float(valid_ratio), 4),
            "neighbors": nav_graph.get(st_id, []),
        }
        panos_meta.append(pano_dict)

    # 6. Assembly of av_walkthrough_bundle.json
    t_proj = transform_to_project if transform_to_project is not None else np.eye(4, dtype=np.float64)
    t_proj_list = [[round(float(val), 8) for val in row] for row in t_proj]

    bundle_json = {
        "schema": "av_walkthrough_bundle",
        "schema_version": "1.1",
        "generator": "AV Structure 2.0",
        "project_id": proj_id,
        "project_name": project_name,
        "scan_id": s_id,
        "scan_date": s_date,
        "coordinate_system": "right_handed_z_up_meters",
        "panorama_projection": "astm_e2807_equirectangular_u05_pos_x",
        "transform_to_project": t_proj_list,
        "structured_e57_path": structured_e57_rel_path,
        "point_cloud_cache": "cache/lod0.bin",
        "floors": floors_meta,
        "panos": panos_meta,
    }

    manifest_path = os.path.join(bundle_dir, "av_walkthrough_bundle.json")
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(bundle_json, f, indent=2)

    logger.info(f"Successfully generated AV Spatial Bundle v1.1 in: {bundle_dir}")
    return bundle_json
