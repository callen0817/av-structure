import csv
import logging
import math
from dataclasses import dataclass
from typing import List, Optional

logger = logging.getLogger(__name__)


@dataclass
class PanoPose:
    pano_id: int
    filename: str
    timestamp: float
    position: tuple  # (x, y, z)
    orientation: tuple  # (w, x, y, z) quaternion
    image_path: Optional[str] = None
    image_bytes: Optional[bytes] = None
    orientation_resolved: bool = False
    associated_guid: Optional[str] = None


def normalize_quaternion(ori: Optional[tuple]) -> Optional[tuple]:
    """
    Returns a normalized (w, x, y, z) unit quaternion if valid, or None if degenerate/missing.
    """
    if ori is None or len(ori) != 4:
        return None
    try:
        w, x, y, z = float(ori[0]), float(ori[1]), float(ori[2]), float(ori[3])
        if not (math.isfinite(w) and math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return None
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm < 1e-6:
            return None
        return (w / norm, x / norm, y / norm, z / norm)
    except Exception:
        return None


def resolve_missing_orientations(poses: List[PanoPose]) -> List[PanoPose]:
    """
    Validates and normalizes pose quaternions. When a pose lacks a valid orientation
    (e.g., 6-column position-only CSV or zero quaternion), resolves a calibrated
    level-horizon orientation from the trajectory tangent vector (or principal axis)
    and marks `pose.orientation_resolved = True`.
    """
    if not poses:
        return poses

    from pyquaternion import Quaternion
    import numpy as np

    resolved_count = 0
    n = len(poses)
    for i, p in enumerate(poses):
        norm_q = normalize_quaternion(p.orientation)
        if norm_q is not None and not getattr(p, "orientation_resolved", False):
            p.orientation = norm_q
            continue

        # Compute horizontal tangent along trajectory if multiple poses exist
        yaw = 0.0
        if n > 1:
            curr_pos = np.array(p.position, dtype=np.float64)
            if i < n - 1:
                next_pos = np.array(poses[i + 1].position, dtype=np.float64)
                delta = next_pos - curr_pos
            else:
                prev_pos = np.array(poses[i - 1].position, dtype=np.float64)
                delta = curr_pos - prev_pos
            if np.linalg.norm(delta[:2]) > 1e-4:
                yaw = float(math.atan2(delta[1], delta[0]))

        q = Quaternion(axis=[0.0, 0.0, 1.0], angle=yaw)
        p.orientation = (float(q.w), float(q.x), float(q.y), float(q.z))
        p.orientation_resolved = True
        resolved_count += 1

    if resolved_count > 0:
        logger.info(
            f"Orientation Resolution: Calibrated level-horizon orientations from trajectory tangents "
            f"for {resolved_count}/{n} pose(s) that lacked valid source quaternions."
        )
    return poses


class NavVisParser:
    def __init__(
        self,
        pose_csv_path: Optional[str] = None,
        images_dir: Optional[str] = None,
        alignment_file: Optional[str] = None,
    ):
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

            if ext in [".bundle", ".zip"]:
                with zipfile.ZipFile(self.alignment_file, "r") as z:
                    for filename in z.namelist():
                        if filename.endswith("alignment.json"):
                            data = json.loads(z.read(filename).decode("utf-8"))
                            break
            elif ext == ".json":
                with open(self.alignment_file, "r", encoding="utf-8") as f:
                    data = json.load(f)

            if data:
                # Prefer site_transform (aligned to BIM/Site), fallback to basic transform
                transform = data.get("site_transform") or data.get("transform")
                if transform:
                    R = np.array(
                        [
                            [transform.get("r11", 1), transform.get("r12", 0), transform.get("r13", 0)],
                            [transform.get("r21", 0), transform.get("r22", 1), transform.get("r23", 0)],
                            [transform.get("r31", 0), transform.get("r32", 0), transform.get("r33", 1)],
                        ],
                        dtype=np.float64,
                    )
                    T = np.array(
                        [transform.get("tx", 0), transform.get("ty", 0), transform.get("tz", 0)],
                        dtype=np.float64,
                    )
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
            with open(self.pose_csv_path, "r", encoding="utf-8") as f:
                lines = f.readlines()
        elif self.alignment_file and os.path.exists(self.alignment_file):
            ext = os.path.splitext(self.alignment_file)[1].lower()
            if ext in [".bundle", ".zip"]:
                try:
                    bundle_zip = zipfile.ZipFile(self.alignment_file, "r")
                    for name in bundle_zip.namelist():
                        if name.endswith("pano-poses.csv"):
                            lines = bundle_zip.read(name).decode("utf-8", errors="replace").splitlines()
                        elif name.lower().endswith((".jpg", ".jpeg", ".png")):
                            bundle_jpg_map[os.path.basename(name)] = name
                except Exception as e:
                    logger.warning(f"Could not read poses from bundle {self.alignment_file}: {e}")

        for line in lines:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            delimiter = ";" if ";" in line else ","
            parts = [p.strip() for p in line.split(delimiter)]
            # Skip header rows if non-numeric ID
            if len(parts) >= 6:
                try:
                    pano_id = int(parts[0])
                except ValueError:
                    continue

                try:
                    filename = parts[1]
                    timestamp = float(parts[2])
                    pos = (float(parts[3]), float(parts[4]), float(parts[5]))

                    ori_resolved = False
                    if len(parts) >= 10:
                        raw_ori = (float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9]))
                        norm_ori = normalize_quaternion(raw_ori)
                        if norm_ori is None:
                            ori = (0.0, 0.0, 0.0, 0.0)
                            ori_resolved = True
                        else:
                            ori = norm_ori
                    else:
                        ori = (0.0, 0.0, 0.0, 0.0)
                        ori_resolved = True

                    # Apply Coordinate Transformation
                    if self.R is not None and self.T is not None:
                        pos_np = np.array(pos, dtype=np.float64)
                        pos_np = np.dot(self.R, pos_np) + self.T
                        pos = (float(pos_np[0]), float(pos_np[1]), float(pos_np[2]))

                        if not ori_resolved:
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
                        else:
                            base_potential = os.path.join(self.images_dir, os.path.basename(filename))
                            if os.path.exists(base_potential):
                                image_path = base_potential
                    if image_path is None and bundle_zip is not None:
                        base_fn = os.path.basename(filename)
                        if base_fn in bundle_jpg_map:
                            try:
                                img_bytes = bundle_zip.read(bundle_jpg_map[base_fn])
                            except Exception:
                                pass

                    poses.append(
                        PanoPose(
                            pano_id=pano_id,
                            filename=filename,
                            timestamp=timestamp,
                            position=pos,
                            orientation=ori,
                            image_path=image_path,
                            image_bytes=img_bytes,
                            orientation_resolved=ori_resolved,
                        )
                    )
                except ValueError as e:
                    logger.warning(f"Failed to parse line: {line}. Error: {e}")

        if bundle_zip is not None:
            try:
                bundle_zip.close()
            except Exception:
                pass

        poses = resolve_missing_orientations(poses)
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
                logger.info(
                    f"Auto-aligning poses to point cloud using forward site transform ({fwd_ratio*100:.1f}% inside bounds)."
                )
                q_site = Quaternion(matrix=R)
                for i, p in enumerate(poses):
                    p.position = (float(fwd_xyz[i, 0]), float(fwd_xyz[i, 1]), float(fwd_xyz[i, 2]))
                    q_new = q_site * Quaternion(p.orientation)
                    p.orientation = (float(q_new.w), float(q_new.x), float(q_new.y), float(q_new.z))
                return poses
            elif inv_ratio > base_ratio and inv_ratio > 0.2:
                logger.info(
                    f"Auto-aligning poses to point cloud using inverse site transform ({inv_ratio*100:.1f}% inside bounds)."
                )
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
    Extracts embedded scan/camera poses and panoramic imagery directly from an E57 file.
    Prioritizes embedded images2D panoramas (linking via associatedData3DGuid or pose),
    and falls back to Data3D scan headers when no embedded images2D exist.
    """
    import os

    if not e57_path or not os.path.exists(e57_path):
        return []
    ext = os.path.splitext(e57_path)[1].lower()
    if ext != ".e57":
        return []

    poses = []
    try:
        import pye57
        import numpy as np
        from pyquaternion import Quaternion

        e57 = pye57.E57(e57_path)

        if hasattr(e57, "scan_count"):
            n_scans = (
                e57.scan_count
                if isinstance(e57.scan_count, int)
                else (e57.scan_count() if callable(e57.scan_count) else 0)
            )
        else:
            n_scans = len(e57.data3d) if hasattr(e57, "data3d") else 0

        # Build lookup of Data3D scan poses by GUID and by scan index
        scan_poses_by_guid = {}
        scan_poses_by_idx = {}
        for i in range(n_scans):
            try:
                header = e57.get_header(i)
                trans = header.translation if hasattr(header, "translation") else [0.0, 0.0, 0.0]
                tx = float(trans[0]) if len(trans) > 0 else 0.0
                ty = float(trans[1]) if len(trans) > 1 else 0.0
                tz = float(trans[2]) if len(trans) > 2 else 0.0
                pos = (tx, ty, tz)

                rot = header.rotation if hasattr(header, "rotation") else np.array([1.0, 0.0, 0.0, 0.0])
                ori = (1.0, 0.0, 0.0, 0.0)
                if isinstance(rot, np.ndarray):
                    if rot.shape == (3, 3):
                        try:
                            q = Quaternion(matrix=rot)
                            ori = (float(q.w), float(q.x), float(q.y), float(q.z))
                        except Exception:
                            pass
                    elif rot.shape == (4,) or len(rot) == 4:
                        norm_q = normalize_quaternion((rot[0], rot[1], rot[2], rot[3]))
                        if norm_q is not None:
                            ori = norm_q
                scan_guid = None
                try:
                    scan_guid = str(header.guid)
                except Exception:
                    pass
                scan_poses_by_idx[i] = (pos, ori, scan_guid)
                if scan_guid:
                    scan_poses_by_guid[scan_guid] = (pos, ori)
            except Exception as hdr_err:
                logger.debug(f"Could not inspect Data3D header {i}: {hdr_err}")

        # 1. Always inspect images2D first so embedded panoramas are preserved even on multi-scan E57s
        try:
            if hasattr(e57, "root") and e57.root.isDefined("images2D"):
                images2d = e57.root["images2D"]
                n_images = images2d.childCount()
                if n_images > 0:
                    logger.info(f"Found {n_images} embedded image(s) in E57 images2D node.")
                    for i in range(n_images):
                        img_node = images2d[i]
                        assoc_guid = None
                        if img_node.isDefined("associatedData3DGuid"):
                            try:
                                assoc_guid = str(img_node["associatedData3DGuid"].value())
                            except Exception:
                                pass

                        pos = None
                        ori = None
                        ori_resolved = False

                        if img_node.isDefined("pose"):
                            pose_node = img_node["pose"]
                            if pose_node.isDefined("translation"):
                                tr = pose_node["translation"]
                                pos = (
                                    float(tr["x"].value()),
                                    float(tr["y"].value()),
                                    float(tr["z"].value()),
                                )
                            if pose_node.isDefined("rotation"):
                                rot = pose_node["rotation"]
                                raw_ori = (
                                    float(rot["w"].value()),
                                    float(rot["x"].value()),
                                    float(rot["y"].value()),
                                    float(rot["z"].value()),
                                )
                                ori = normalize_quaternion(raw_ori)

                        # Fallback to associated Data3D scan pose if image node lacks pose
                        if (pos is None or ori is None) and assoc_guid and assoc_guid in scan_poses_by_guid:
                            s_pos, s_ori = scan_poses_by_guid[assoc_guid]
                            if pos is None:
                                pos = s_pos
                            if ori is None:
                                ori = s_ori
                        elif (pos is None or ori is None) and i in scan_poses_by_idx:
                            s_pos, s_ori, _ = scan_poses_by_idx[i]
                            if pos is None:
                                pos = s_pos
                            if ori is None:
                                ori = s_ori

                        if pos is None:
                            pos = (0.0, 0.0, 0.0)
                        if ori is None:
                            ori = (0.0, 0.0, 0.0, 0.0)
                            ori_resolved = True

                        img_bytes = None
                        for rep_name in (
                            "sphericalRepresentation",
                            "pinholeRepresentation",
                            "cylindricalRepresentation",
                            "visualReferenceRepresentation",
                        ):
                            if img_node.isDefined(rep_name):
                                rep = img_node[rep_name]
                                for blob_key in ("jpegImage", "pngImage"):
                                    if rep.isDefined(blob_key):
                                        try:
                                            blob = rep[blob_key]
                                            buf = bytearray(blob.byteCount())
                                            blob.read(buf, 0, len(buf))
                                            img_bytes = bytes(buf)
                                            break
                                        except Exception as b_err:
                                            logger.debug(f"Could not read {blob_key} blob for image {i}: {b_err}")
                                if img_bytes is not None:
                                    break

                        filename = f"embedded_pano_{i:04d}.jpg"
                        if img_node.isDefined("name"):
                            try:
                                name_val = str(img_node["name"].value()).strip()
                                if name_val:
                                    filename = name_val if name_val.lower().endswith(".jpg") else f"{name_val}.jpg"
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

                        poses.append(
                            PanoPose(
                                pano_id=i,
                                filename=filename,
                                timestamp=float(i),
                                position=pos,
                                orientation=ori,
                                image_path=image_path,
                                image_bytes=img_bytes,
                                orientation_resolved=ori_resolved,
                                associated_guid=assoc_guid,
                            )
                        )
                    if hasattr(e57, "close"):
                        e57.close()
                    poses = resolve_missing_orientations(poses)
                    logger.info(f"Extracted {len(poses)} embedded panorama pose(s) from E57 images2D node.")
                    return poses
        except Exception as img_err:
            logger.debug(f"Error checking images2D: {img_err}")

        # 2. Fallback: extract scan poses from multi-scan Data3D headers if no images2D were present
        if n_scans > 1 and scan_poses_by_idx:
            logger.info(f"Found {n_scans} embedded scan headers in E57 file (no images2D): {e57_path}")
            for i in range(n_scans):
                if i not in scan_poses_by_idx:
                    continue
                pos, ori, s_guid = scan_poses_by_idx[i]
                poses.append(
                    PanoPose(
                        pano_id=i,
                        filename=f"embedded_pano_{i:04d}.jpg",
                        timestamp=float(i),
                        position=pos,
                        orientation=ori,
                        associated_guid=s_guid,
                    )
                )
            if hasattr(e57, "close"):
                e57.close()
            poses = resolve_missing_orientations(poses)
            logger.info(f"Extracted {len(poses)} embedded poses from E57 scan headers.")
            return poses

        if hasattr(e57, "close"):
            e57.close()
    except Exception as e:
        logger.warning(f"Could not extract embedded poses from E57 file {e57_path}: {e}")

    return poses


