import csv
import logging
from dataclasses import dataclass
from typing import List, Optional

logger = logging.getLogger(__name__)

@dataclass
class PanoPose:
    pano_id: int
    filename: str
    timestamp: float
    position: tuple # (x, y, z)
    orientation: tuple # (w, x, y, z) quaternion
    image_path: Optional[str] = None
    image_bytes: Optional[bytes] = None

class NavVisParser:
    def __init__(self, pose_csv_path: Optional[str] = None, images_dir: Optional[str] = None, alignment_file: Optional[str] = None):
        """
        Initializes the parser for NavVis trajectory CSV or .bundle archive.
        """
        self.pose_csv_path = pose_csv_path
        self.images_dir = images_dir
        self.alignment_file = alignment_file
        self.R, self.T = self._load_alignment()
        
    def _load_alignment(self):
        if not self.alignment_file:
            return None, None
            
        import os
        if not os.path.exists(self.alignment_file):
            logger.warning(f"Alignment file not found: {self.alignment_file}")
            return None, None
            
        try:
            import json
            import zipfile
            import numpy as np
            
            ext = os.path.splitext(self.alignment_file)[1].lower()
            data = None
            
            if ext in ['.bundle', '.zip']:
                with zipfile.ZipFile(self.alignment_file, 'r') as z:
                    for filename in z.namelist():
                        if filename.endswith('alignment.json'):
                            data = json.loads(z.read(filename).decode('utf-8'))
                            break
            elif ext == '.json':
                with open(self.alignment_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    
            if data:
                # Prefer site_transform (aligned to BIM/Site), fallback to basic transform
                transform = data.get('site_transform') or data.get('transform')
                if transform:
                    R = np.array([
                        [transform.get('r11', 1), transform.get('r12', 0), transform.get('r13', 0)],
                        [transform.get('r21', 0), transform.get('r22', 1), transform.get('r23', 0)],
                        [transform.get('r31', 0), transform.get('r32', 0), transform.get('r33', 1)]
                    ])
                    T = np.array([transform.get('tx', 0), transform.get('ty', 0), transform.get('tz', 0)])
                    logger.info(f"Loaded NavVis site alignment matrix from {self.alignment_file}")
                    return R, T
            
            logger.warning(f"No valid alignment transform found inside {self.alignment_file}")
        except Exception as e:
            logger.error(f"Failed to load alignment file {self.alignment_file}: {e}")
            
        return None, None
        
    def parse(self) -> List[PanoPose]:
        """
        Parses the NavVis pose CSV (either from disk or embedded inside a .bundle archive).
        Returns a list of PanoPose objects, transformed if an alignment file is provided.
        """
        import os
        import zipfile
        import numpy as np
        from pyquaternion import Quaternion

        poses = []
        lines = []
        bundle_zip = None
        bundle_jpg_map = {}

        if self.pose_csv_path and os.path.exists(self.pose_csv_path):
            with open(self.pose_csv_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
        elif self.alignment_file and os.path.exists(self.alignment_file):
            ext = os.path.splitext(self.alignment_file)[1].lower()
            if ext in ['.bundle', '.zip']:
                try:
                    bundle_zip = zipfile.ZipFile(self.alignment_file, 'r')
                    for name in bundle_zip.namelist():
                        if name.endswith('pano-poses.csv'):
                            lines = bundle_zip.read(name).decode('utf-8', errors='replace').splitlines()
                        elif name.lower().endswith('.jpg'):
                            bundle_jpg_map[os.path.basename(name)] = name
                except Exception as e:
                    logger.warning(f"Could not read poses from bundle {self.alignment_file}: {e}")

        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            parts = [p.strip() for p in line.split(';')]
            if len(parts) >= 10:
                try:
                    pano_id = int(parts[0])
                    filename = parts[1]
                    timestamp = float(parts[2])
                    pos = (float(parts[3]), float(parts[4]), float(parts[5]))
                    ori = (float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9]))
                    
                    # Apply Coordinate Transformation
                    if self.R is not None and self.T is not None:
                        pos_np = np.array(pos)
                        pos_np = np.dot(self.R, pos_np) + self.T
                        pos = (float(pos_np[0]), float(pos_np[1]), float(pos_np[2]))
                        
                        # Apply rotation to the orientation quaternion
                        q_site = Quaternion(matrix=self.R)
                        q_pose = Quaternion(ori)
                        q_new = q_site * q_pose
                        ori = (float(q_new.w), float(q_new.x), float(q_new.y), float(q_new.z))
                    
                    image_path = None
                    img_bytes = None
                    if self.images_dir is not None:
                        potential_path = os.path.join(self.images_dir, filename)
                        if os.path.exists(potential_path):
                            image_path = potential_path
                    if image_path is None and bundle_zip is not None:
                        base_fn = os.path.basename(filename)
                        if base_fn in bundle_jpg_map:
                            try:
                                img_bytes = bundle_zip.read(bundle_jpg_map[base_fn])
                            except Exception:
                                pass
                            
                    poses.append(PanoPose(
                        pano_id=pano_id,
                        filename=filename,
                        timestamp=timestamp,
                        position=pos,
                        orientation=ori,
                        image_path=image_path,
                        image_bytes=img_bytes
                    ))
                except ValueError as e:
                    logger.warning(f"Failed to parse line: {line}. Error: {e}")

        if bundle_zip is not None:
            try:
                bundle_zip.close()
            except Exception:
                pass
        
        logger.info(f"Parsed {len(poses)} NavVis poses.")
        return poses


def verify_and_align_poses(poses: List[PanoPose], pcd, alignment_file: Optional[str] = None) -> List[PanoPose]:
    """
    Verifies that the extracted camera poses overlap with the loaded point cloud's bounding box.
    If poses fall outside the cloud due to a coordinate system mismatch (e.g., unaligned vs site-aligned),
    automatically checks if applying or inverting the alignment_file transform resolves the mismatch.
    """
    if not poses or pcd is None or pcd.is_empty():
        return poses

    import numpy as np
    from pyquaternion import Quaternion

    pts = np.asarray(pcd.points)
    min_b = pts.min(axis=0) - 5.0
    max_b = pts.max(axis=0) + 5.0

    def _in_bounds_ratio(pos_array):
        inside = np.all((pos_array >= min_b) & (pos_array <= max_b), axis=1)
        return float(np.mean(inside))

    pose_xyz = np.array([p.position for p in poses], dtype=np.float64)
    base_ratio = _in_bounds_ratio(pose_xyz)
    if base_ratio >= 0.5:
        return poses

    if alignment_file:
        temp_parser = NavVisParser(None, None, alignment_file)
        R, T = temp_parser.R, temp_parser.T
        if R is not None and T is not None:
            # Test forward transform: R @ p + T
            fwd_xyz = (pose_xyz @ R.T) + T
            fwd_ratio = _in_bounds_ratio(fwd_xyz)

            # Test inverse transform: R.T @ (p - T)
            inv_xyz = (pose_xyz - T) @ R
            inv_ratio = _in_bounds_ratio(inv_xyz)

            if fwd_ratio > base_ratio and fwd_ratio >= inv_ratio and fwd_ratio > 0.2:
                logger.info(f"Auto-aligning poses to point cloud using forward site transform ({fwd_ratio*100:.1f}% inside bounds).")
                q_site = Quaternion(matrix=R)
                for i, p in enumerate(poses):
                    p.position = (float(fwd_xyz[i, 0]), float(fwd_xyz[i, 1]), float(fwd_xyz[i, 2]))
                    q_new = q_site * Quaternion(p.orientation)
                    p.orientation = (float(q_new.w), float(q_new.x), float(q_new.y), float(q_new.z))
                return poses
            elif inv_ratio > base_ratio and inv_ratio > 0.2:
                logger.info(f"Auto-aligning poses to point cloud using inverse site transform ({inv_ratio*100:.1f}% inside bounds).")
                q_inv = Quaternion(matrix=R).inverse
                for i, p in enumerate(poses):
                    p.position = (float(inv_xyz[i, 0]), float(inv_xyz[i, 1]), float(inv_xyz[i, 2]))
                    q_new = q_inv * Quaternion(p.orientation)
                    p.orientation = (float(q_new.w), float(q_new.x), float(q_new.y), float(q_new.z))
                return poses

    if base_ratio < 0.1:
        logger.warning(
            f"Coordinate Warning: Only {base_ratio*100:.1f}% of camera poses fall within the point cloud bounding box! "
            "Check if an Alignment Bundle is needed or if cloud and poses use different coordinate systems."
        )
    return poses


def extract_e57_poses(e57_path: str) -> List[PanoPose]:
    """
    Extracts embedded scan/camera poses directly from an E57 file.
    Supports multi-scan E57 files and embedded image structures.
    """
    import os
    if not e57_path or not os.path.exists(e57_path):
        return []
    ext = os.path.splitext(e57_path)[1].lower()
    if ext != '.e57':
        return []

    poses = []
    try:
        import pye57
        import numpy as np
        from pyquaternion import Quaternion

        e57 = pye57.E57(e57_path)
        
        if hasattr(e57, "scan_count"):
            n_scans = e57.scan_count if isinstance(e57.scan_count, int) else (e57.scan_count() if callable(e57.scan_count) else 0)
        else:
            n_scans = len(e57.data3d) if hasattr(e57, "data3d") else 0

        if n_scans > 1:
            logger.info(f"Found {n_scans} embedded scan headers in E57 file: {e57_path}")
            for i in range(n_scans):
                header = e57.get_header(i)
                trans = header.get("translation", [0.0, 0.0, 0.0])
                tx = float(trans[0]) if len(trans) > 0 else 0.0
                ty = float(trans[1]) if len(trans) > 1 else 0.0
                tz = float(trans[2]) if len(trans) > 2 else 0.0
                pos = (tx, ty, tz)

                rot = header.get("rotation", np.eye(3))
                ori = (1.0, 0.0, 0.0, 0.0)
                if isinstance(rot, np.ndarray):
                    if rot.shape == (3, 3):
                        try:
                            q = Quaternion(matrix=rot)
                            ori = (q.w, q.x, q.y, q.z)
                        except Exception:
                            pass
                    elif rot.shape == (4,) or len(rot) == 4:
                        ori = (float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))

                poses.append(PanoPose(
                    pano_id=i,
                    filename=f"embedded_pano_{i:04d}.jpg",
                    timestamp=float(i),
                    position=pos,
                    orientation=ori
                ))
            logger.info(f"Extracted {len(poses)} embedded poses from E57 scan headers.")
            return poses

        # Fallback: check images2D for embedded panorama image poses
        try:
            if hasattr(e57, "root") and e57.root.isDefined("images2D"):
                images2d = e57.root["images2D"]
                n_images = images2d.childCount()
                if n_images > 0:
                    logger.info(f"Found {n_images} embedded images in E57 images2D node.")
                    for i in range(n_images):
                        img_node = images2d[i]
                        pos = (0.0, 0.0, 0.0)
                        ori = (1.0, 0.0, 0.0, 0.0)
                        
                        if img_node.isDefined("pose"):
                            pose_node = img_node["pose"]
                            if pose_node.isDefined("translation"):
                                tr = pose_node["translation"]
                                pos = (float(tr["x"].value()), float(tr["y"].value()), float(tr["z"].value()))
                            if pose_node.isDefined("rotation"):
                                rot = pose_node["rotation"]
                                ori = (float(rot["w"].value()), float(rot["x"].value()), float(rot["y"].value()), float(rot["z"].value()))

                        img_bytes = None
                        for rep_name in ("sphericalRepresentation", "pinholeRepresentation", "cylindricalRepresentation", "visualReferenceRepresentation"):
                            if img_node.isDefined(rep_name):
                                rep = img_node[rep_name]
                                if rep.isDefined("jpegImage"):
                                    try:
                                        blob = rep["jpegImage"]
                                        buf = bytearray(blob.byteCount())
                                        blob.read(buf, 0, len(buf))
                                        img_bytes = bytes(buf)
                                        break
                                    except Exception as b_err:
                                        logger.debug(f"Could not read jpegImage blob for image {i}: {b_err}")

                        filename = f"embedded_pano_{i:04d}.jpg"
                        if img_node.isDefined("name"):
                            try:
                                filename = str(img_node["name"].value())
                            except Exception:
                                pass

                        image_path = None
                        if img_bytes:
                            import tempfile
                            temp_dir = os.path.join(tempfile.gettempdir(), "av_structure_panos")
                            os.makedirs(temp_dir, exist_ok=True)
                            out_img_path = os.path.join(temp_dir, f"pano_{i:04d}.jpg")
                            try:
                                with open(out_img_path, "wb") as f_img:
                                    f_img.write(img_bytes)
                                image_path = out_img_path
                            except Exception:
                                pass

                        poses.append(PanoPose(
                            pano_id=i,
                            filename=filename,
                            timestamp=float(i),
                            position=pos,
                            orientation=ori,
                            image_path=image_path,
                            image_bytes=img_bytes
                        ))
                    logger.info(f"Extracted {len(poses)} embedded poses from E57 images2D node.")
                    return poses
        except Exception as img_err:
            logger.debug(f"Error checking images2D: {img_err}")

    except Exception as e:
        logger.warning(f"Could not extract embedded poses from E57 file {e57_path}: {e}")

    return poses

