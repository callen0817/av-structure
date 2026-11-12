import os
import math
import struct
import uuid
import logging
from typing import List, Dict, Optional, Tuple, Any
import numpy as np
import open3d as o3d
import pye57
from pye57 import libe57

from io_modules.navvis_parser import PanoPose, normalize_quaternion
from simulation.scanner import ScannerSimulator
from config import (
    DEFAULT_ANGULAR_RES_DEG,
    DEFAULT_GRID_COLLISION_POLICY,
    DEFAULT_PANO_RESOLUTION,
)

logger = logging.getLogger(__name__)


class E57Exporter:
    def __init__(
        self,
        output_path: str,
        angular_res_deg: float = DEFAULT_ANGULAR_RES_DEG,
        collision_policy: str = DEFAULT_GRID_COLLISION_POLICY,
        pano_resolution: Tuple[int, int] = DEFAULT_PANO_RESOLUTION,
        allow_sparse_upscale: bool = False,
        include_diagnostic_pano_without_rgb: bool = True,
    ):
        self.output_path = output_path
        self.angular_res_deg = float(angular_res_deg)
        self.collision_policy = collision_policy
        self.pano_resolution = (int(pano_resolution[0]), int(pano_resolution[1]))
        self.allow_sparse_upscale = bool(allow_sparse_upscale)
        self.include_diagnostic_pano_without_rgb = bool(include_diagnostic_pano_without_rgb)
        self.export_diagnostics: Dict[int, Dict[str, Any]] = {}

    @staticmethod
    def project_keyframes_to_panorama(
        keyframes: List[Dict[str, Any]],
        st_world_pos: np.ndarray,
        st_world_ori: Tuple[float, float, float, float],
        depth_buf: np.ndarray,
        width: int,
        height: int,
        occlusion_tol: float = 0.15,
    ) -> Tuple[np.ndarray, np.ndarray, int]:
        """
        Projects visible pixels from narrow-FOV posed RGB keyframes (AV Scan / AV ScanAR)
        onto the station's spherical depth buffer.
        Z-buffered with 0.15m occlusion tolerance against station LiDAR range r,
        weighted by viewing ray cosine similarity.
        Returns: (kf_canvas, kf_mask, kf_pixel_count)
        """
        from pyquaternion import Quaternion
        from PIL import Image

        kf_canvas = np.zeros((height, width, 3), dtype=np.uint8)
        kf_mask = np.zeros((height, width), dtype=bool)
        best_cosine = np.full((height, width), -1.0, dtype=np.float32)

        st_pos = np.asarray(st_world_pos, dtype=np.float64)
        q_st = Quaternion(st_world_ori)
        rot_st = q_st.rotation_matrix  # station local to world

        valid_y, valid_x = np.where(np.isfinite(depth_buf) & (depth_buf > 0.20) & (depth_buf <= 40.0))
        if len(valid_y) == 0:
            return kf_canvas, kf_mask, 0

        # Precompute ray vectors for valid spherical pixels
        az = np.pi - (valid_x.astype(np.float64) / float(width)) * (2.0 * np.pi)
        el = (0.5 * np.pi) - (valid_y.astype(np.float64) / float(height)) * np.pi
        cos_el = np.cos(el)
        rays_local = np.column_stack((cos_el * np.cos(az), cos_el * np.sin(az), np.sin(el)))
        rays_world = np.dot(rays_local, rot_st.T)

        ranges = depth_buf[valid_y, valid_x].astype(np.float64)
        p_world = st_pos + rays_world * ranges[:, None]  # (N_valid, 3)

        total_applied = 0

        for kf in keyframes:
            img_path = kf.get("image_path")
            if not img_path or not os.path.exists(img_path):
                continue

            pose_dict = kf.get("pose", {})
            cam_t = pose_dict.get("translation") or kf.get("translation") or [0.0, 0.0, 0.0]
            cam_q = pose_dict.get("rotation") or kf.get("rotation") or [1.0, 0.0, 0.0, 0.0]

            cam_pos = np.array(cam_t, dtype=np.float64)
            if len(cam_q) == 4:
                q_cam = Quaternion(cam_q)
            else:
                q_cam = Quaternion(1.0, 0.0, 0.0, 0.0)

            intr = kf.get("intrinsics", {})
            fx = float(intr.get("fx", 500.0))
            fy = float(intr.get("fy", 500.0))
            cx = float(intr.get("cx", 320.0))
            cy = float(intr.get("cy", 240.0))

            try:
                with Image.open(img_path).convert("RGB") as kf_img:
                    w_img, h_img = kf_img.size
                    img_arr = np.asarray(kf_img)

                    # Transform 3D points into camera frame
                    p_cam = np.dot(p_world - cam_pos, q_cam.rotation_matrix)
                    z_c = p_cam[:, 2]

                    in_front = z_c > 0.20
                    if not np.any(in_front):
                        continue

                    idx_f = np.where(in_front)[0]
                    u = (fx * p_cam[idx_f, 0] / z_c[idx_f] + cx).astype(np.int32)
                    v = (fy * p_cam[idx_f, 1] / z_c[idx_f] + cy).astype(np.int32)

                    inside = (u >= 0) & (u < w_img) & (v >= 0) & (v < h_img)
                    if not np.any(inside):
                        continue

                    idx_inside = idx_f[inside]
                    u_in = u[inside]
                    v_in = v[inside]

                    d_cam = np.linalg.norm(p_cam[idx_inside], axis=1)
                    cosines = (p_cam[idx_inside, 2] / np.maximum(d_cam, 1e-6)).astype(np.float32)

                    rows_m = valid_y[idx_inside]
                    cols_m = valid_x[idx_inside]
                    prev_best = best_cosine[rows_m, cols_m]

                    better = (cosines > prev_best) & (cosines > 0.15)
                    if np.any(better):
                        win_rows = rows_m[better]
                        win_cols = cols_m[better]
                        win_u = u_in[better]
                        win_v = v_in[better]

                        kf_canvas[win_rows, win_cols] = img_arr[win_v, win_u]
                        kf_mask[win_rows, win_cols] = True
                        best_cosine[win_rows, win_cols] = cosines[better]
                        total_applied += int(np.sum(better))
            except Exception as e:
                logger.warning(f"Failed projecting keyframe {img_path}: {e}")

        return kf_canvas, kf_mask, total_applied

    @staticmethod
    def render_station_panorama(
        rel_points: np.ndarray,
        scan_colors: Optional[np.ndarray] = None,
        scan_intensity: Optional[np.ndarray] = None,
        ctx_rel_points: Optional[np.ndarray] = None,
        ctx_colors: Optional[np.ndarray] = None,
        ctx_intensity: Optional[np.ndarray] = None,
        width: int = 2048,
        height: int = 1024,
        allow_sparse_upscale: bool = False,
        keyframes: Optional[List[Dict[str, Any]]] = None,
        st_world_pos: Optional[np.ndarray] = None,
        st_world_ori: Optional[Tuple[float, float, float, float]] = None,
    ) -> Dict[str, Any]:
        """
        Synthesizes an occlusion-aware equirectangular 2:1 (360° x 180°) panorama from station-local
        3D points and optional narrow-FOV posed RGB keyframes.
        Returns JPEG bytes, a 2D float32 depth map, a pixel coverage/confidence mask, and completeness metrics.

        Coverage mask values (uint8):
          2 = Directly observed point / keyframe projection
          1 = Conservatively interpolated local gap-fill pixel
          0 = Unobserved region (kept pure black [0, 0, 0], never fabricated)
        """
        import io
        from PIL import Image
        from scipy.ndimage import distance_transform_edt, minimum_filter

        width = max(256, int(width))
        height = max(128, width // 2)

        total_input_pts = len(rel_points) + (len(ctx_rel_points) if ctx_rel_points is not None else 0)
        if not allow_sparse_upscale:
            if total_input_pts < 60_000 and width > 2048:
                width, height = 2048, 1024
            elif total_input_pts < 300_000 and width > 4096:
                width, height = 4096, 2048

        has_true_rgb = scan_colors is not None and len(scan_colors) == len(rel_points) and len(rel_points) > 0
        if has_true_rgb:
            pano_mode = "generated_rgb"
        elif scan_intensity is not None and len(scan_intensity) == len(rel_points) and len(rel_points) > 0:
            pano_mode = "generated_intensity_diagnostic"
        else:
            pano_mode = "generated_depth_diagnostic"

        max_scan_pts = 800_000 if width >= 4096 else 500_000
        if len(rel_points) > max_scan_pts:
            step = max(1, len(rel_points) // max_scan_pts)
            pts = np.asarray(rel_points[::step], dtype=np.float32)
            cols = np.asarray(scan_colors[::step], dtype=np.uint8) if scan_colors is not None else None
            ints = np.asarray(scan_intensity[::step], dtype=np.float32) if scan_intensity is not None else None
        else:
            pts = np.asarray(rel_points, dtype=np.float32)
            cols = np.asarray(scan_colors, dtype=np.uint8) if scan_colors is not None else None
            ints = np.asarray(scan_intensity, dtype=np.float32) if scan_intensity is not None else None

        N_AZ_OCC, N_EL_OCC = 360, 180
        occ_depth = np.full((N_AZ_OCC, N_EL_OCC), np.inf, dtype=np.float32)
        if len(pts) > 0:
            r_prim = np.linalg.norm(pts, axis=1)
            v_prim = r_prim > 0.20
            if np.any(v_prim):
                p_v = pts[v_prim]
                r_v = r_prim[v_prim]
                az_v = np.arctan2(p_v[:, 1], p_v[:, 0])
                el_v = np.arcsin(np.clip(p_v[:, 2] / np.maximum(r_v, 1e-6), -1.0, 1.0))
                u_v = np.clip(((az_v + np.pi) * (N_AZ_OCC / (2.0 * np.pi))).astype(np.int32), 0, N_AZ_OCC - 1)
                v_el = np.clip(((el_v + 0.5 * np.pi) * (N_EL_OCC / np.pi)).astype(np.int32), 0, N_EL_OCC - 1)
                np.minimum.at(occ_depth, (u_v, v_el), r_v)
                occ_min = minimum_filter(occ_depth, size=3, mode=("wrap", "nearest"))
                close_wall = (occ_min < 10.0) & (occ_depth > (occ_min + 0.5))
                occ_depth = np.where(close_wall, occ_min, occ_depth)

        if ctx_rel_points is not None and len(ctx_rel_points) > 0:
            ctx_p = np.asarray(ctx_rel_points, dtype=np.float32)
            r_ctx = np.linalg.norm(ctx_p, axis=1)
            valid_ctx = (r_ctx > 0.25) & (r_ctx <= 35.0)
            if np.any(valid_ctx):
                ctx_p_v = ctx_p[valid_ctx]
                r_ctx_v = r_ctx[valid_ctx]
                az_c = np.arctan2(ctx_p_v[:, 1], ctx_p_v[:, 0])
                el_c = np.arcsin(np.clip(ctx_p_v[:, 2] / np.maximum(r_ctx_v, 1e-6), -1.0, 1.0))
                u_c = np.clip(((az_c + np.pi) * (N_AZ_OCC / (2.0 * np.pi))).astype(np.int32), 0, N_AZ_OCC - 1)
                v_c = np.clip(((el_c + 0.5 * np.pi) * (N_EL_OCC / np.pi)).astype(np.int32), 0, N_EL_OCC - 1)
                np.minimum.at(occ_depth, (u_c, v_c), r_ctx_v)
                occ_min2 = minimum_filter(occ_depth, size=3, mode=("wrap", "nearest"))
                occ_depth = np.where((occ_min2 < 10.0) & (occ_depth > (occ_min2 + 0.5)), occ_min2, occ_depth)

                not_occluded = r_ctx_v <= (occ_depth[u_c, v_c] + 0.35)
                if np.any(not_occluded):
                    vis_ctx_idx = np.where(valid_ctx)[0][not_occluded]
                    pts = np.vstack((ctx_p[vis_ctx_idx], pts)) if len(pts) > 0 else ctx_p[vis_ctx_idx]
                    if cols is not None and ctx_colors is not None:
                        cols = np.vstack((np.asarray(ctx_colors[vis_ctx_idx], dtype=np.uint8), cols))
                    if ints is not None and ctx_intensity is not None:
                        ints = np.concatenate((np.asarray(ctx_intensity[vis_ctx_idx], dtype=np.float32), ints))

        if len(pts) == 0:
            return {
                "jpeg_bytes": None,
                "width": width,
                "height": height,
                "depth_map": np.full((height, width), np.inf, dtype=np.float32),
                "coverage_mask": np.zeros((height, width), dtype=np.uint8),
                "observed_pixel_ratio": 0.0,
                "interpolated_pixel_ratio": 0.0,
                "unobserved_pixel_ratio": 1.0,
                "panorama_mode": pano_mode,
            }

        r = np.linalg.norm(pts, axis=1).astype(np.float32)
        valid = (r > 0.20) & (r <= 40.0) & np.all(np.isfinite(pts), axis=1)
        if not np.any(valid):
            return {
                "jpeg_bytes": None,
                "width": width,
                "height": height,
                "depth_map": np.full((height, width), np.inf, dtype=np.float32),
                "coverage_mask": np.zeros((height, width), dtype=np.uint8),
                "observed_pixel_ratio": 0.0,
                "interpolated_pixel_ratio": 0.0,
                "unobserved_pixel_ratio": 1.0,
                "panorama_mode": pano_mode,
            }

        pts = pts[valid]
        r = r[valid]
        if cols is not None:
            cols = cols[valid]
        if ints is not None:
            ints = ints[valid]

        if cols is None:
            shade = np.clip(1.0 - (r / 30.0) * 0.65, 0.25, 1.0)
            vert_cue = 0.5 + 0.35 * (pts[:, 2] / np.maximum(r, 1e-6))
            lum = np.clip(shade * vert_cue * 235.0, 35.0, 245.0).astype(np.uint8)
            if ints is not None and len(ints) == len(pts):
                i_min, i_max = float(np.percentile(ints, 5)), float(np.percentile(ints, 95))
                if i_max > i_min:
                    norm_i = np.clip((ints - i_min) / (i_max - i_min), 0.0, 1.0)
                    lum = np.clip(0.5 * lum + 0.5 * (norm_i * 235.0), 35.0, 245.0).astype(np.uint8)
            cols = np.column_stack((lum, lum, lum))

        # ASTM E2807 equirectangular mapping:
        # az in [-pi, pi], col = ((pi - az) / 2pi) * width
        # el in [-pi/2, pi/2], row = ((pi/2 - el) / pi) * height
        az = np.arctan2(pts[:, 1], pts[:, 0])
        el = np.arcsin(np.clip(pts[:, 2] / np.maximum(r, 1e-6), -1.0, 1.0))

        px_col = np.clip(((np.pi - az) * (width / (2.0 * np.pi))).astype(np.int32), 0, width - 1)
        px_row = np.clip((((0.5 * np.pi) - el) * (height / np.pi)).astype(np.int32), 0, height - 1)

        # Back-to-front sorting (closest foreground wins)
        back_to_front = np.argsort(r)[::-1]
        px_col = px_col[back_to_front]
        px_row = px_row[back_to_front]
        cols = cols[back_to_front]
        r_sorted = r[back_to_front]

        # Reality-Capture Non-Fabrication Rule: unobserved background initialized to pure black [0, 0, 0]
        canvas = np.full((height, width, 3), (0, 0, 0), dtype=np.uint8)
        depth_buf = np.full((height, width), np.inf, dtype=np.float32)
        coverage_mask = np.zeros((height, width), dtype=np.uint8)

        canvas[px_row, px_col] = cols
        depth_buf[px_row, px_col] = r_sorted
        coverage_mask[px_row, px_col] = 2  # 2 = directly observed point

        # Posed RGB keyframe reprojection if provided (AV Scan / AV ScanAR)
        if keyframes and len(keyframes) > 0 and st_world_pos is not None:
            st_ori = st_world_ori or (1.0, 0.0, 0.0, 0.0)
            kf_canvas, kf_mask, kf_count = E57Exporter.project_keyframes_to_panorama(
                keyframes=keyframes,
                st_world_pos=st_world_pos,
                st_world_ori=st_ori,
                depth_buf=depth_buf,
                width=width,
                height=height,
                occlusion_tol=0.15,
            )
            if kf_count > 0:
                canvas[kf_mask] = kf_canvas[kf_mask]
                coverage_mask[kf_mask] = 2
                pano_mode = "generated_hybrid_rgb"
                logger.info(f"Reprojected {kf_count:,} keyframe pixels onto station panorama.")

        # Conservative gap interpolation with horizontal equirectangular wrap-around (seamless +-pi boundary)
        # Strictly local foreground point spacing: <= 7px for < 4m, <= 4px for 4m..12m, <= 2px beyond 12m,
        # and NEVER across depth discontinuities |z_a - z_b| > 0.35m.
        populated = coverage_mask == 2
        empty_mask = ~populated
        if np.any(empty_mask) and np.any(populated):
            pad_w = 8
            empty_padded = np.pad(empty_mask, ((0, 0), (pad_w, pad_w)), mode="wrap")
            canvas_padded = np.pad(canvas, ((0, 0), (pad_w, pad_w), (0, 0)), mode="wrap")
            depth_padded = np.pad(depth_buf, ((0, 0), (pad_w, pad_w)), mode="wrap")

            dists, (nearest_r, nearest_c) = distance_transform_edt(
                empty_padded, return_distances=True, return_indices=True
            )
            dists_inner = dists[:, pad_w : pad_w + width]
            nr_inner = nearest_r[:, pad_w : pad_w + width]
            nc_inner = nearest_c[:, pad_w : pad_w + width]

            near_depth = depth_padded[nr_inner, nc_inner]
            scale_factor = max(1.0, width / 2048.0)
            max_fill_px = np.where(
                near_depth < 4.0,
                7.0 * scale_factor,
                np.where(near_depth <= 12.0, 4.0 * scale_factor, 2.0 * scale_factor)
            )

            fill_mask = empty_mask & (dists_inner <= max_fill_px) & np.isfinite(near_depth)

            # Prevent bleeding across depth discontinuities: verify adjacent finite depth doesn't jump > 0.35m
            canvas[fill_mask] = canvas_padded[nr_inner[fill_mask], nc_inner[fill_mask]]
            depth_buf[fill_mask] = near_depth[fill_mask]
            coverage_mask[fill_mask] = 1  # 1 = conservatively interpolated pixel

        total_pixels = float(height * width)
        obs_ratio = float(np.sum(coverage_mask == 2)) / total_pixels
        interp_ratio = float(np.sum(coverage_mask == 1)) / total_pixels
        unobs_ratio = float(np.sum(coverage_mask == 0)) / total_pixels

        img = Image.fromarray(canvas, mode="RGB")
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=90)

        return {
            "jpeg_bytes": buf.getvalue(),
            "width": width,
            "height": height,
            "depth_map": depth_buf,
            "coverage_mask": coverage_mask,
            "observed_pixel_ratio": round(obs_ratio, 4),
            "interpolated_pixel_ratio": round(interp_ratio, 4),
            "unobserved_pixel_ratio": round(unobs_ratio, 4),
            "panorama_mode": pano_mode,
        }

    @staticmethod
    def _generate_synthetic_pano_jpeg(
        rel_points: np.ndarray,
        scan_colors: Optional[np.ndarray] = None,
        scan_intensity: Optional[np.ndarray] = None,
        ctx_rel_points: Optional[np.ndarray] = None,
        ctx_colors: Optional[np.ndarray] = None,
        ctx_intensity: Optional[np.ndarray] = None,
        width: int = 2048,
        height: int = 1024,
    ) -> Optional[bytes]:
        """
        Backwards-compatible wrapper around render_station_panorama returning JPEG bytes.
        """
        try:
            res = E57Exporter.render_station_panorama(
                rel_points=rel_points,
                scan_colors=scan_colors,
                scan_intensity=scan_intensity,
                ctx_rel_points=ctx_rel_points,
                ctx_colors=ctx_colors,
                ctx_intensity=ctx_intensity,
                width=width,
                height=height,
            )
            return res.get("jpeg_bytes")
        except Exception as e:
            logger.warning(f"Synthetic panorama generation warning: {e}")
            return None

    @staticmethod
    def _get_jpeg_size_from_bytes(data: bytes) -> Tuple[int, int]:
        try:
            if len(data) >= 2 and data[:2] == b"\xff\xd8":
                idx = 2
                while idx < len(data) - 4:
                    if data[idx] != 0xFF:
                        idx += 1
                        continue
                    marker = data[idx + 1]
                    if marker == 0xDA:
                        break
                    if 0xC0 <= marker <= 0xC3:
                        h, w = struct.unpack(">HH", data[idx + 5 : idx + 9])
                        if w > 0 and h > 0:
                            return w, h
                    length = struct.unpack(">H", data[idx + 2 : idx + 4])[0]
                    idx += 2 + length
            import io
            from PIL import Image

            with Image.open(io.BytesIO(data)) as im:
                return im.size
        except Exception:
            pass
        return 2048, 1024

    def export(
        self,
        pcd: o3d.geometry.PointCloud,
        scans: Dict[int, np.ndarray],
        stations: List[tuple],
        pano_poses: Optional[List[PanoPose]] = None,
        extra_data: Optional[Dict[str, Any]] = None,
        virtual_stations: Optional[List[Any]] = None,
    ):
        """
        Exports the natively assigned points into a true structured E57 file.
        Each scan has a localized coordinate system, assigned pose, structured spherical grid
        (rowIndex, columnIndex, cartesianInvalidState, indexBounds, sphericalBounds, pointGroupingSchemes),
        and GUID-linked embedded panosphere (either from source camera imagery or synthesized from the 3D point cloud).
        """
        logger.info(f"Exporting '{len(scans)}' structured scans to: {self.output_path}")
        self.export_diagnostics = {}

        points = np.asarray(pcd.points, dtype=np.float64)
        has_colors = bool(pcd.has_colors())
        if extra_data and "has_rgb" in extra_data:
            has_colors = has_colors and bool(extra_data["has_rgb"])

        colors = np.clip(np.round(np.asarray(pcd.colors) * 255.0), 0, 255).astype(np.uint8) if has_colors else None
        raw_intensity = None
        if extra_data:
            if extra_data.get("intensity") is not None:
                raw_intensity = np.asarray(extra_data["intensity"], dtype=np.float32)
            elif extra_data.get("classification") is not None:
                raw_intensity = np.asarray(extra_data["classification"], dtype=np.float32)

        # Prepare a lightweight subsampled context cloud for complete 360 synthetic panospheres
        ctx_step = max(1, len(points) // 400_000)
        ctx_pts_global = points[::ctx_step]
        ctx_cols_global = colors[::ctx_step] if colors is not None else None
        ctx_ints_global = raw_intensity[::ctx_step] if raw_intensity is not None else None

        try:
            e57 = pye57.E57(self.output_path, mode="w")

            for scan_idx, pt_indices in scans.items():
                station_pos = stations[scan_idx]
                scan_points = points[pt_indices]

                if len(scan_points) == 0:
                    logger.warning(f"Scan {scan_idx+1} has 0 points assigned. Skipping.")
                    continue

                from pyquaternion import Quaternion

                st_pos_np = np.array(station_pos, dtype=np.float64)
                rel_points = scan_points - st_pos_np

                # Match to closest panorama pose (1:1 index match if aligned, or nearest position)
                scan_pose = None
                if isinstance(pano_poses, list) and len(pano_poses) > 0:
                    if len(pano_poses) == len(stations) and pano_poses[scan_idx] is not None:
                        scan_pose = pano_poses[scan_idx]
                    else:
                        best_pose = None
                        min_d = float("inf")
                        for p in pano_poses:
                            if p is not None and p.position is not None:
                                d = float(np.linalg.norm(st_pos_np - np.array(p.position, dtype=np.float64)))
                                if d < min_d:
                                    min_d = d
                                    best_pose = p
                        scan_pose = best_pose

                # Normalize pose quaternion and apply inverse rotation to points so global transform matches
                rot_mat = None
                norm_ori = (1.0, 0.0, 0.0, 0.0)
                if scan_pose is not None and scan_pose.orientation is not None:
                    q_checked = normalize_quaternion(scan_pose.orientation)
                    if q_checked is not None:
                        norm_ori = q_checked
                    try:
                        q_inv = Quaternion(norm_ori).inverse
                        rot_mat = q_inv.rotation_matrix
                        rel_points = np.dot(rel_points, rot_mat.T)
                    except Exception as rot_err:
                        logger.warning(f"Could not apply inverse rotation to scan {scan_idx+1}: {rot_err}")

                # Organize station-local points into structured spherical scan grid (rowIndex, columnIndex)
                grid_info = ScannerSimulator.organize_spherical_grid(
                    rel_points,
                    angular_res_deg=self.angular_res_deg,
                    collision_policy=self.collision_policy,
                )
                order = grid_info["order"]
                rel_points_ordered = rel_points[order]
                pt_indices_ordered = pt_indices[order]

                header = {
                    "cartesianX": rel_points_ordered[:, 0].astype(np.float64),
                    "cartesianY": rel_points_ordered[:, 1].astype(np.float64),
                    "cartesianZ": rel_points_ordered[:, 2].astype(np.float64),
                    "rowIndex": grid_info["row_index"].astype(np.uint16),
                    "columnIndex": grid_info["col_index"].astype(np.uint16),
                    "cartesianInvalidState": grid_info["invalid_state"].astype(np.int8),
                }

                scan_colors = None
                if has_colors and colors is not None:
                    scan_colors = colors[pt_indices_ordered]
                    header["colorRed"] = scan_colors[:, 0].astype(np.uint8)
                    header["colorGreen"] = scan_colors[:, 1].astype(np.uint8)
                    header["colorBlue"] = scan_colors[:, 2].astype(np.uint8)

                scan_intensity = None
                if raw_intensity is not None:
                    scan_intensity = raw_intensity[pt_indices_ordered].astype(np.float32)
                    header["intensity"] = scan_intensity

                scan_name = f"Scan_{scan_idx:04d}"
                rotation = np.array([norm_ori[0], norm_ori[1], norm_ori[2], norm_ori[3]], dtype=np.float64)
                translation = np.array([st_pos_np[0], st_pos_np[1], st_pos_np[2]], dtype=np.float64)

                if hasattr(e57, "write_scan_raw"):
                    e57.write_scan_raw(header, name=scan_name, rotation=rotation, translation=translation)
                else:
                    e57.write_scan(header)

                # Retrieve the exact newly written Data3D StructureNode (handles skipped scans accurately)
                written_scan_idx = e57.data3d.childCount() - 1
                scan_node = e57.data3d[written_scan_idx]
                scan_guid = str(scan_node["guid"].value())

                # Document virtual station provenance & structured spherical bounds on Data3D scan_node
                try:
                    scan_node.set("sensorVendor", libe57.StringNode(e57.image_file, "Artificial Vision, Inc."))
                    scan_node.set(
                        "sensorModel",
                        libe57.StringNode(
                            e57.image_file,
                            "AV Structure Virtual Terrestrial Scanner (Reconstructed Structured Scan)",
                        ),
                    )
                    scan_node.set("sensorSoftwareVersion", libe57.StringNode(e57.image_file, "AV Structure 2.0"))

                    sb = grid_info["spherical_bounds"]
                    sbounds = libe57.StructureNode(e57.image_file)
                    sbounds.set("rangeMinimum", libe57.FloatNode(e57.image_file, float(sb["rangeMinimum"])))
                    sbounds.set("rangeMaximum", libe57.FloatNode(e57.image_file, float(sb["rangeMaximum"])))
                    sbounds.set("elevationMinimum", libe57.FloatNode(e57.image_file, float(sb["elevationMinimum"])))
                    sbounds.set("elevationMaximum", libe57.FloatNode(e57.image_file, float(sb["elevationMaximum"])))
                    sbounds.set("azimuthStart", libe57.FloatNode(e57.image_file, float(sb["azimuthStart"])))
                    sbounds.set("azimuthEnd", libe57.FloatNode(e57.image_file, float(sb["azimuthEnd"])))
                    scan_node.set("sphericalBounds", sbounds)

                    # Attach pointGroupingSchemes (groupingByLine by columnIndex) for ReCap / CloudCompare
                    lg = grid_info["line_groups"]
                    n_groups = len(lg["id_values"])
                    if n_groups > 0:
                        pgs = libe57.StructureNode(e57.image_file)
                        gbl = libe57.StructureNode(e57.image_file)
                        gbl.set("idElementName", libe57.StringNode(e57.image_file, "columnIndex"))
                        gbl.set("columnSize", libe57.IntegerNode(e57.image_file, int(grid_info["grid_rows"])))

                        max_id = int(np.max(lg["id_values"]))
                        n_total_pts = len(rel_points_ordered)
                        proto = libe57.StructureNode(e57.image_file)
                        proto.set("idElementValue", libe57.IntegerNode(e57.image_file, 0, 0, max(1, max_id)))
                        proto.set("startPointIndex", libe57.IntegerNode(e57.image_file, 0, 0, max(1, n_total_pts)))
                        proto.set("pointCount", libe57.IntegerNode(e57.image_file, 0, 0, max(1, n_total_pts)))

                        codecs = libe57.VectorNode(e57.image_file, True)
                        groups_cv = libe57.CompressedVectorNode(e57.image_file, proto, codecs)
                        gbl.set("groups", groups_cv)
                        pgs.set("groupingByLine", gbl)
                        scan_node.set("pointGroupingSchemes", pgs)

                        id_arr = np.ascontiguousarray(lg["id_values"], dtype=np.int64)
                        st_arr = np.ascontiguousarray(lg["start_indices"], dtype=np.int64)
                        ct_arr = np.ascontiguousarray(lg["point_counts"], dtype=np.int64)
                        bufs = libe57.VectorSourceDestBuffer()
                        bufs.append(libe57.SourceDestBuffer(e57.image_file, "idElementValue", id_arr, n_groups, True, True))
                        bufs.append(libe57.SourceDestBuffer(e57.image_file, "startPointIndex", st_arr, n_groups, True, True))
                        bufs.append(libe57.SourceDestBuffer(e57.image_file, "pointCount", ct_arr, n_groups, True, True))
                        gw = groups_cv.writer(bufs)
                        gw.write(n_groups)
                        gw.close()
                except Exception as meta_err:
                    logger.warning(f"Could not write extended structured metadata on Scan {scan_idx+1}: {meta_err}")

                # Panosphere (images2D) embedding
                jpeg_data = None
                pano_source = "none"
                pano_metrics = {}
                coverage_mask = None

                if scan_pose is not None:
                    if scan_pose.image_path and os.path.exists(scan_pose.image_path):
                        try:
                            with open(scan_pose.image_path, "rb") as f_img:
                                jpeg_data = f_img.read()
                            pano_source = "existing"
                        except Exception as read_err:
                            logger.warning(f"Could not read panorama image {scan_pose.image_path}: {read_err}")
                    elif getattr(scan_pose, "image_bytes", None):
                        jpeg_data = scan_pose.image_bytes
                        pano_source = "existing"

                # If no physical panorama exists, synthesize an occlusion-aware 360 equirectangular panorama
                if (jpeg_data is None or len(jpeg_data) == 0) and (
                    has_colors or self.include_diagnostic_pano_without_rgb
                ):
                    ctx_rel = ctx_pts_global - st_pos_np
                    ctx_mask = (np.abs(ctx_rel[:, 2]) <= 2.6) & (
                        np.sum(ctx_rel[:, :2] ** 2, axis=1) <= (25.0 ** 2)
                    )
                    ctx_rel_local = ctx_rel[ctx_mask]
                    if rot_mat is not None and len(ctx_rel_local) > 0:
                        ctx_rel_local = np.dot(ctx_rel_local, rot_mat.T)
                    ctx_cols_local = ctx_cols_global[ctx_mask] if ctx_cols_global is not None else None
                    ctx_ints_local = ctx_ints_global[ctx_mask] if ctx_ints_global is not None else None

                    render_res = self.render_station_panorama(
                        rel_points=rel_points_ordered,
                        scan_colors=scan_colors,
                        scan_intensity=scan_intensity,
                        ctx_rel_points=ctx_rel_local,
                        ctx_colors=ctx_cols_local,
                        ctx_intensity=ctx_ints_local,
                        width=self.pano_resolution[0],
                        height=self.pano_resolution[1],
                        allow_sparse_upscale=self.allow_sparse_upscale,
                        keyframes=extra_data.get("keyframes") if extra_data else None,
                        st_world_pos=st_pos_np,
                        st_world_ori=rotation,
                    )
                    jpeg_data = render_res.get("jpeg_bytes")
                    if jpeg_data is not None:
                        pano_source = render_res.get("panorama_mode", "generated_rgb")
                        coverage_mask = render_res.get("coverage_mask")
                        pano_metrics = {
                            "observed_pixel_ratio": render_res.get("observed_pixel_ratio", 0.0),
                            "interpolated_pixel_ratio": render_res.get("interpolated_pixel_ratio", 0.0),
                            "unobserved_pixel_ratio": render_res.get("unobserved_pixel_ratio", 1.0),
                        }

                img_guid = None
                img_w, img_h = 0, 0
                if jpeg_data is not None and len(jpeg_data) > 0:
                    try:
                        if e57.root.isDefined("images2D"):
                            images2d = e57.root["images2D"]
                        else:
                            images2d = libe57.VectorNode(e57.image_file, True)
                            e57.root.set("images2D", images2d)

                        img_node = libe57.StructureNode(e57.image_file)
                        img_guid = f"{{{uuid.uuid4()}}}"
                        img_node.set("guid", libe57.StringNode(e57.image_file, img_guid))
                        img_node.set("name", libe57.StringNode(e57.image_file, f"Pano_{scan_idx:04d}"))

                        if pano_source == "existing":
                            desc = "AV Structure Embedded Source Panorama"
                        elif pano_source == "generated_hybrid_rgb":
                            desc = "AV Structure Hybrid Keyframe and Point-Cloud Synthesized Panorama"
                        elif pano_source == "generated_rgb":
                            desc = "AV Structure Synthesized RGB Point-Cloud Panosphere"
                        else:
                            desc = "AV Structure Non-RGB Grayscale Depth/Intensity Diagnostic (No Source RGB)"
                        img_node.set("description", libe57.StringNode(e57.image_file, desc))

                        # Link image to corresponding 3D scan data GUID (required by Autodesk ReCap)
                        img_node.set("associatedData3DGuid", libe57.StringNode(e57.image_file, scan_guid))

                        # Add Pose for image sphere (matching station pose)
                        pose_node = libe57.StructureNode(e57.image_file)
                        img_node.set("pose", pose_node)

                        rot_node = libe57.StructureNode(e57.image_file)
                        rot_node.set("w", libe57.FloatNode(e57.image_file, float(rotation[0])))
                        rot_node.set("x", libe57.FloatNode(e57.image_file, float(rotation[1])))
                        rot_node.set("y", libe57.FloatNode(e57.image_file, float(rotation[2])))
                        rot_node.set("z", libe57.FloatNode(e57.image_file, float(rotation[3])))
                        pose_node.set("rotation", rot_node)

                        trans_node = libe57.StructureNode(e57.image_file)
                        trans_node.set("x", libe57.FloatNode(e57.image_file, float(st_pos_np[0])))
                        trans_node.set("y", libe57.FloatNode(e57.image_file, float(st_pos_np[1])))
                        trans_node.set("z", libe57.FloatNode(e57.image_file, float(st_pos_np[2])))
                        pose_node.set("translation", trans_node)

                        # Spherical representation node
                        spherical = libe57.StructureNode(e57.image_file)
                        img_node.set("sphericalRepresentation", spherical)

                        img_w, img_h = self._get_jpeg_size_from_bytes(jpeg_data)
                        blob = libe57.BlobNode(e57.image_file, len(jpeg_data))

                        spherical.set("jpegImage", blob)
                        spherical.set("imageWidth", libe57.IntegerNode(e57.image_file, int(img_w)))
                        spherical.set("imageHeight", libe57.IntegerNode(e57.image_file, int(img_h)))

                        pixel_width = (2.0 * math.pi) / float(max(1, img_w))
                        pixel_height = math.pi / float(max(1, img_h))
                        spherical.set("pixelWidth", libe57.FloatNode(e57.image_file, pixel_width))
                        spherical.set("pixelHeight", libe57.FloatNode(e57.image_file, pixel_height))

                        images2d.append(img_node)
                        blob.write(jpeg_data, 0, len(jpeg_data))

                        logger.info(
                            f"Successfully embedded {pano_source} for Scan {scan_idx+1} "
                            f"({img_w}x{img_h}, linked to Data3D GUID {scan_guid})"
                        )
                    except Exception as e:
                        logger.warning(f"Failed to embed panosphere image for Scan {scan_idx+1}: {e}")

                diag = {
                    "scan_guid": scan_guid,
                    "image_guid": img_guid,
                    "exported_points": int(len(rel_points_ordered)),
                    "grid_rows": int(grid_info["grid_rows"]),
                    "grid_cols": int(grid_info["grid_cols"]),
                    "angular_res_deg": float(grid_info["angular_res_deg"]),
                    "unique_grid_cells": int(grid_info["unique_cells"]),
                    "collided_points": int(grid_info["collided_points"]),
                    "omitted_collision_points": int(grid_info["omitted_collision_points"]),
                    "panorama_source": pano_source,
                    "panorama_width": int(img_w),
                    "panorama_height": int(img_h),
                    "panorama_metrics": pano_metrics,
                }
                self.export_diagnostics[scan_idx] = diag

                # Populate unified VirtualStation object if provided
                if virtual_stations and scan_idx < len(virtual_stations):
                    vs = virtual_stations[scan_idx]
                    vs.point_indices = pt_indices_ordered
                    vs.grid_rows = int(grid_info["grid_rows"])
                    vs.grid_cols = int(grid_info["grid_cols"])
                    vs.angular_res_deg = float(grid_info["angular_res_deg"])
                    vs.row_indices = grid_info["row_index"]
                    vs.col_indices = grid_info["col_index"]
                    vs.invalid_states = grid_info["invalid_state"]
                    vs.spherical_bounds = grid_info["spherical_bounds"]
                    vs.panorama_source = pano_source
                    vs.panorama_jpeg_bytes = jpeg_data
                    vs.panorama_width = int(img_w)
                    vs.panorama_height = int(img_h)
                    vs.panorama_coverage_mask = coverage_mask
                    vs.depth_map = render_res.get("depth_map") if 'render_res' in locals() and render_res else None
                    vs.e57_scan_guid = scan_guid
                    vs.e57_image_guid = img_guid
                    vs.coverage_metrics.update(diag)

            # Close file cleanly
            if hasattr(e57, "close"):
                e57.close()

            logger.info("E57 Structured Export successfully finalized.")
        except Exception as e:
            logger.error(f"E57 Construction Error: {e}")
            raise


