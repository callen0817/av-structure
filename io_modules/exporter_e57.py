import os
import pye57
import numpy as np
import open3d as o3d
import logging
from typing import List, Dict, Optional
from io_modules.navvis_parser import PanoPose

logger = logging.getLogger(__name__)

class E57Exporter:
    def __init__(self, output_path: str):
        self.output_path = output_path

    @staticmethod
    def _generate_synthetic_pano_jpeg(
        rel_points: np.ndarray,
        scan_colors: Optional[np.ndarray] = None,
        scan_intensity: Optional[np.ndarray] = None,
        ctx_rel_points: Optional[np.ndarray] = None,
        ctx_colors: Optional[np.ndarray] = None,
        ctx_intensity: Optional[np.ndarray] = None,
        width: int = 2048,
        height: int = 1024
    ) -> Optional[bytes]:
        """
        Synthesizes an equirectangular 360x180 JPEG panorama directly from the 3D point cloud
        when no physical camera panorama exists, enabling 3D Panospheres / RealView in Autodesk ReCap.
        """
        try:
            import io
            from PIL import Image
            from scipy.ndimage import distance_transform_edt

            # Subsample scan points if very large for fast rendering
            max_scan_pts = 500_000
            if len(rel_points) > max_scan_pts:
                step = max(1, len(rel_points) // max_scan_pts)
                pts = rel_points[::step]
                cols = scan_colors[::step] if scan_colors is not None else None
                ints = scan_intensity[::step] if scan_intensity is not None else None
            else:
                pts = rel_points
                cols = scan_colors
                ints = scan_intensity

            # Merge with surrounding 360 context points (so station has full 360 view even if Voronoi-partitioned)
            if ctx_rel_points is not None and len(ctx_rel_points) > 0:
                pts = np.vstack((ctx_rel_points, pts))
                if cols is not None and ctx_colors is not None:
                    cols = np.vstack((ctx_colors, cols))
                if ints is not None and ctx_intensity is not None:
                    ints = np.concatenate((ctx_intensity, ints))

            r = np.linalg.norm(pts, axis=1).astype(np.float32)
            valid = (r > 0.25) & (r <= 35.0)
            if not np.any(valid):
                return None

            pts = pts[valid]
            r = r[valid]
            if cols is not None:
                cols = cols[valid]
            if ints is not None:
                ints = ints[valid]

            # Compute RGB if not available (architectural ambient + depth + intensity shading)
            if cols is None:
                shade = np.clip(1.0 - (r / 30.0) * 0.65, 0.25, 1.0)
                vert_cue = 0.5 + 0.35 * (pts[:, 2] / np.maximum(r, 1e-6))
                lum = np.clip(shade * vert_cue * 235.0, 30.0, 245.0).astype(np.uint8)
                if ints is not None and len(ints) == len(pts):
                    i_min, i_max = float(np.percentile(ints, 5)), float(np.percentile(ints, 95))
                    if i_max > i_min:
                        norm_i = np.clip((ints - i_min) / (i_max - i_min), 0.0, 1.0)
                        lum = np.clip(0.5 * lum + 0.5 * (norm_i * 235.0), 25.0, 250.0).astype(np.uint8)
                cols = np.column_stack((lum, lum, np.clip(lum.astype(np.int32) + 8, 0, 255).astype(np.uint8)))

            # E57 standard spherical projection:
            # Azimuth theta = atan2(y, x) in [-pi, pi], col = ((pi - theta) / 2pi) * width
            # Elevation phi = arcsin(z / r) in [-pi/2, pi/2], row = ((pi/2 - phi) / pi) * height
            az = np.arctan2(pts[:, 1], pts[:, 0])
            el = np.arcsin(np.clip(pts[:, 2] / np.maximum(r, 1e-6), -1.0, 1.0))

            px_col = np.clip(((np.pi - az) * (width / (2.0 * np.pi))).astype(np.int32), 0, width - 1)
            px_row = np.clip((((0.5 * np.pi) - el) * (height / np.pi)).astype(np.int32), 0, height - 1)

            # Sort back-to-front (farthest to nearest) so closest foreground surface wins (painter's Z-buffer)
            back_to_front = np.argsort(r)[::-1]
            px_col = px_col[back_to_front]
            px_row = px_row[back_to_front]
            cols = cols[back_to_front]
            r_sorted = r[back_to_front]

            canvas = np.full((height, width, 3), 24, dtype=np.uint8)
            depth_buf = np.full((height, width), np.inf, dtype=np.float32)
            populated = np.zeros((height, width), dtype=bool)

            canvas[px_row, px_col] = cols
            depth_buf[px_row, px_col] = r_sorted
            populated[px_row, px_col] = True

            # Fill sparse gaps between projected points using nearest-neighbor Euclidean distance transform
            empty_mask = ~populated
            if np.any(empty_mask) and np.any(populated):
                dists, (nearest_r, nearest_c) = distance_transform_edt(
                    empty_mask, return_distances=True, return_indices=True
                )
                near_depth = depth_buf[nearest_r, nearest_c]
                # Adaptive splat radius: closer surfaces get wider hole filling (up to 7px), far surfaces up to 4px
                max_fill_px = np.where(near_depth < 6.0, 7.0, 4.0)
                fill_mask = empty_mask & (dists <= max_fill_px)
                canvas[fill_mask] = canvas[nearest_r[fill_mask], nearest_c[fill_mask]]

            img = Image.fromarray(canvas, mode="RGB")
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=88)
            return buf.getvalue()
        except Exception as e:
            logger.warning(f"Synthetic panorama generation warning: {e}")
            return None
        
    def export(self, pcd: o3d.geometry.PointCloud, scans: Dict[int, np.ndarray], 
               stations: List[tuple], pano_poses: Optional[List[PanoPose]] = None,
               extra_data: Optional[Dict[str, np.ndarray]] = None):
        """
        Exports the natively assigned points into a structured E57 file.
        Each scan has a localized coordinate system, assigned pose, and embedded panosphere
        (either from source camera imagery or synthesized from the 3D point cloud).
        """
        logger.info(f"Exporting '{len(scans)}' structured scans to: {self.output_path}")
        
        points = np.asarray(pcd.points)
        has_colors = pcd.has_colors()
        colors = (np.asarray(pcd.colors) * 255.0).astype(np.uint8) if has_colors else None
        raw_intensity = None
        if extra_data:
            if extra_data.get("intensity") is not None:
                raw_intensity = extra_data["intensity"]
            elif extra_data.get("classification") is not None:
                raw_intensity = extra_data["classification"]

        # Prepare a lightweight subsampled context cloud for complete 360 synthetic panospheres
        ctx_step = max(1, len(points) // 400_000)
        ctx_pts_global = points[::ctx_step]
        ctx_cols_global = colors[::ctx_step] if colors is not None else None
        ctx_ints_global = raw_intensity[::ctx_step] if raw_intensity is not None else None
            
        try:
            e57 = pye57.E57(self.output_path, mode='w')
            
            for scan_idx, pt_indices in scans.items():
                station_pos = stations[scan_idx]
                scan_points = points[pt_indices]
                
                if len(scan_points) == 0:
                    logger.warning(f"Scan {scan_idx+1} has 0 points assigned. Skipping.")
                    continue
                
                from pyquaternion import Quaternion
                rel_points = scan_points - station_pos
                
                # Match to closest panorama pose
                scan_pose = None
                if isinstance(pano_poses, list) and len(pano_poses) > 0:
                    st_np = np.array(station_pos)
                    best_pose = None
                    min_d = float('inf')
                    for p in pano_poses:
                        if p is not None and p.position is not None:
                            d = float(np.linalg.norm(st_np - np.array(p.position)))
                            if d < min_d:
                                min_d = d
                                best_pose = p
                    scan_pose = best_pose

                # If pose is available, apply inverse rotation to points so global transform matches
                rot_mat = None
                if scan_pose is not None:
                    try:
                        q_inv = Quaternion(scan_pose.orientation).inverse
                        rot_mat = q_inv.rotation_matrix
                        rel_points = np.dot(rel_points, rot_mat.T)
                    except Exception as rot_err:
                        logger.warning(f"Could not apply inverse rotation to scan {scan_idx+1}: {rot_err}")
                
                header = {
                    "cartesianX": rel_points[:, 0],
                    "cartesianY": rel_points[:, 1],
                    "cartesianZ": rel_points[:, 2],
                }
                
                scan_colors = None
                if has_colors and colors is not None:
                    scan_colors = colors[pt_indices]
                    header["colorRed"] = scan_colors[:, 0]
                    header["colorGreen"] = scan_colors[:, 1]
                    header["colorBlue"] = scan_colors[:, 2]
                
                scan_intensity = None
                if extra_data:
                    if extra_data.get("classification") is not None:
                        scan_intensity = extra_data["classification"][pt_indices].astype(np.float32)
                        header["intensity"] = scan_intensity
                    elif extra_data.get("intensity") is not None:
                        scan_intensity = extra_data["intensity"][pt_indices].astype(np.float32)
                        header["intensity"] = scan_intensity
                    
                scan_name = f"Scan_{scan_idx:04d}"
                rotation = np.array([
                    scan_pose.orientation[0] if scan_pose is not None else 1.0,
                    scan_pose.orientation[1] if scan_pose is not None else 0.0,
                    scan_pose.orientation[2] if scan_pose is not None else 0.0,
                    scan_pose.orientation[3] if scan_pose is not None else 0.0,
                ])
                translation = np.array([station_pos[0], station_pos[1], station_pos[2]])
                
                if hasattr(e57, 'write_scan_raw'):
                    e57.write_scan_raw(header, name=scan_name, rotation=rotation, translation=translation)
                else:
                    e57.write_scan(header)
                
                # Panosphere (images2D) embedding for Autodesk ReCap
                jpeg_data = None
                is_synthetic_pano = False
                if scan_pose is not None:
                    if scan_pose.image_path and os.path.exists(scan_pose.image_path):
                        try:
                            with open(scan_pose.image_path, "rb") as f_img:
                                jpeg_data = f_img.read()
                        except Exception as read_err:
                            logger.warning(f"Could not read panorama image {scan_pose.image_path}: {read_err}")
                    elif getattr(scan_pose, "image_bytes", None):
                        jpeg_data = scan_pose.image_bytes

                # If no physical panorama exists, synthesize a 360 equirectangular panosphere from the point cloud
                if jpeg_data is None or len(jpeg_data) == 0:
                    st_arr = np.array(station_pos)
                    ctx_rel = ctx_pts_global - st_arr
                    # Keep local storey context within 25m horizontally and 2.6m vertically
                    ctx_mask = (np.abs(ctx_rel[:, 2]) <= 2.6) & (np.sum(ctx_rel[:, :2] ** 2, axis=1) <= (25.0 ** 2))
                    ctx_rel_local = ctx_rel[ctx_mask]
                    if rot_mat is not None and len(ctx_rel_local) > 0:
                        ctx_rel_local = np.dot(ctx_rel_local, rot_mat.T)
                    ctx_cols_local = ctx_cols_global[ctx_mask] if ctx_cols_global is not None else None
                    ctx_ints_local = ctx_ints_global[ctx_mask] if ctx_ints_global is not None else None

                    jpeg_data = self._generate_synthetic_pano_jpeg(
                        rel_points=rel_points,
                        scan_colors=scan_colors,
                        scan_intensity=scan_intensity,
                        ctx_rel_points=ctx_rel_local,
                        ctx_colors=ctx_cols_local,
                        ctx_intensity=ctx_ints_local
                    )
                    if jpeg_data is not None:
                        is_synthetic_pano = True

                if jpeg_data is not None and len(jpeg_data) > 0:
                    try:
                        import struct
                        import math
                        import uuid
                        from pye57 import libe57
                        
                        def _get_jpeg_size_from_bytes(data):
                            try:
                                if len(data) >= 2 and data[:2] == b'\xff\xd8':
                                    idx = 2
                                    while idx < len(data) - 4:
                                        if data[idx] != 0xff:
                                            idx += 1
                                            continue
                                        marker = data[idx+1]
                                        if marker == 0xda:
                                            break
                                        if 0xc0 <= marker <= 0xc3:
                                            h, w = struct.unpack('>HH', data[idx+4:idx+8])
                                            return w, h
                                        length = struct.unpack('>H', data[idx+2:idx+4])[0]
                                        idx += 2 + length
                            except Exception:
                                pass
                            return 2048, 1024

                        if e57.root.isDefined("images2D"):
                            images2d = e57.root["images2D"]
                        else:
                            images2d = libe57.VectorNode(e57.image_file, True)
                            e57.root.set("images2D", images2d)
                            
                        img_node = libe57.StructureNode(e57.image_file)
                        img_node.set("guid", libe57.StringNode(e57.image_file, f"{{{uuid.uuid4()}}}"))
                        img_node.set("name", libe57.StringNode(e57.image_file, f"Pano_{scan_idx:04d}"))
                        desc = "AV Structure Synthetic Panosphere" if is_synthetic_pano else "AV Structure Panorama"
                        img_node.set("description", libe57.StringNode(e57.image_file, desc))
                        
                        # Link image to corresponding 3D scan data (required by Autodesk ReCap)
                        try:
                            scan_header = e57.data3d[scan_idx]
                            if hasattr(scan_header, "get"):
                                scan_guid_node = scan_header.get("guid")
                                if hasattr(scan_guid_node, "value"):
                                    scan_guid = scan_guid_node.value()
                                else:
                                    scan_guid = str(scan_guid_node)
                            else:
                                scan_guid = str(scan_header.get("guid", ""))
                            
                            img_node.set("associatedData3DGuid", libe57.StringNode(e57.image_file, scan_guid))
                        except Exception as e:
                            logger.warning(f"Could not link image to 3D scan data GUID: {e}")
                        
                        # Add Pose for image sphere
                        pose_node = libe57.StructureNode(e57.image_file)
                        img_node.set("pose", pose_node)
                        
                        rot_node = libe57.StructureNode(e57.image_file)
                        rot_node.set("w", libe57.FloatNode(e57.image_file, float(rotation[0])))
                        rot_node.set("x", libe57.FloatNode(e57.image_file, float(rotation[1])))
                        rot_node.set("y", libe57.FloatNode(e57.image_file, float(rotation[2])))
                        rot_node.set("z", libe57.FloatNode(e57.image_file, float(rotation[3])))
                        pose_node.set("rotation", rot_node)
                        
                        trans_node = libe57.StructureNode(e57.image_file)
                        trans_node.set("x", libe57.FloatNode(e57.image_file, float(station_pos[0])))
                        trans_node.set("y", libe57.FloatNode(e57.image_file, float(station_pos[1])))
                        trans_node.set("z", libe57.FloatNode(e57.image_file, float(station_pos[2])))
                        pose_node.set("translation", trans_node)
                        
                        # Spherical representation node
                        spherical = libe57.StructureNode(e57.image_file)
                        img_node.set("sphericalRepresentation", spherical)
                        
                        img_w, img_h = _get_jpeg_size_from_bytes(jpeg_data)
                            
                        blob = libe57.BlobNode(e57.image_file, len(jpeg_data))
                        
                        spherical.set("jpegImage", blob)
                        spherical.set("imageWidth", libe57.IntegerNode(e57.image_file, img_w)) 
                        spherical.set("imageHeight", libe57.IntegerNode(e57.image_file, img_h))
                        
                        pixel_width = (2.0 * math.pi) / float(img_w)
                        pixel_height = math.pi / float(img_h)
                        spherical.set("pixelWidth", libe57.FloatNode(e57.image_file, pixel_width))
                        spherical.set("pixelHeight", libe57.FloatNode(e57.image_file, pixel_height))
                        
                        images2d.append(img_node)
                        blob.write(jpeg_data, 0, len(jpeg_data))
                        
                        pano_type = "synthetic panosphere" if is_synthetic_pano else "panosphere"
                        logger.info(f"Successfully embedded {pano_type} for Scan {scan_idx+1} ({img_w}x{img_h})")
                    except Exception as e:
                        logger.warning(f"Failed to embed panosphere image for Scan {scan_idx+1}: {e}")
                        
            # Close file cleanly
            if hasattr(e57, 'close'):
                e57.close()
                
            logger.info("E57 Structured Export successfully finalized.")
        except Exception as e:
            logger.error(f"E57 Construction Error: {e}")
            raise

