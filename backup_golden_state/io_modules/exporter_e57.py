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
        
    def export(self, pcd: o3d.geometry.PointCloud, scans: Dict[int, np.ndarray], 
               stations: List[tuple], pano_poses: Optional[List[PanoPose]] = None,
               extra_data: Optional[Dict[str, np.ndarray]] = None):
        """
        Exports the natively assigned points into a structured E57 file.
        Each scan has a localized coordinate system and assigned pose.
        """
        logger.info(f"Exporting '{len(scans)}' structured scans to: {self.output_path}")
        
        points = np.asarray(pcd.points)
        has_colors = pcd.has_colors()
        if has_colors:
            colors = (np.asarray(pcd.colors) * 255.0).astype(np.uint8)
            
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
                
                # If trajectory mode is used with a pose, the points must be actively inverse rotated 
                # so that when readers apply `to_global` forward rotation, it matches original geometry.
                if isinstance(pano_poses, list) and scan_idx < len(pano_poses):
                    pose = pano_poses[scan_idx]
                    if pose is not None:
                        q_inv = Quaternion(pose.orientation).inverse
                        rot_mat = q_inv.rotation_matrix
                        rel_points = np.dot(rel_points, rot_mat.T)
                
                header = {
                    "cartesianX": rel_points[:, 0],
                    "cartesianY": rel_points[:, 1],
                    "cartesianZ": rel_points[:, 2],
                }
                
                if has_colors:
                    scan_colors = colors[pt_indices]
                    header["colorRed"] = scan_colors[:, 0]
                    header["colorGreen"] = scan_colors[:, 1]
                    header["colorBlue"] = scan_colors[:, 2]
                
                if extra_data:
                    # In E57, "classification" is not a standard field. 
                    # Many readers (Recap etc) use the "intensity" field for scalar data if needed.
                    # We map classification to "intensity" if intensity is missing, 
                    # otherwise we try to use it as a custom field (though pye57 may be strict).
                    if extra_data.get("classification") is not None:
                        # Map to intensity to avoid E57 field error if possible
                        header["intensity"] = extra_data["classification"][pt_indices].astype(np.float32)
                    elif extra_data.get("intensity") is not None:
                        header["intensity"] = extra_data["intensity"][pt_indices].astype(np.float32)
                    
                is_pano = False
                scan_pose = None
                
                # Match to panorama if trajectory sequence is provided
                if isinstance(pano_poses, list):
                    if scan_idx < len(pano_poses):
                        scan_pose = pano_poses[scan_idx]
                        is_pano = True
                    
                # Use raw pye57 write_scan if possible. pye57's write_scan supports a 'scan_header' 
                # or native property injection for Pose.
                scan_name = f"Scan_{scan_idx:04d}"
                
                # We attempt to write using standard write_scan_raw if exposed, otherwise write_scan
                if hasattr(e57, 'write_scan_raw'):
                    rotation = np.array([
                        scan_pose.orientation[0] if scan_pose is not None else 1.0,
                        scan_pose.orientation[1] if scan_pose is not None else 0.0,
                        scan_pose.orientation[2] if scan_pose is not None else 0.0,
                        scan_pose.orientation[3] if scan_pose is not None else 0.0,
                    ])
                    translation = np.array([station_pos[0], station_pos[1], station_pos[2]])
                    e57.write_scan_raw(header, name=scan_name, rotation=rotation, translation=translation)
                else:
                    e57.write_scan(header)
                
                # Panorama image embedding
                if scan_pose is not None and scan_pose.image_path is not None:
                    try:
                        import os
                        import struct
                        import math
                        import uuid
                        from pye57 import libe57
                        
                        def _get_jpeg_size(fname):
                            with open(fname, 'rb') as f:
                                # Start of Image (SOI) check
                                if f.read(2) != b'\xff\xd8':
                                    return 8192, 4096
                                b = f.read(1)
                                while b and b != b'\xda':
                                    while b != b'\xff': b = f.read(1)
                                    while b == b'\xff': b = f.read(1)
                                    if b'\xc0' <= b <= b'\xc3':
                                        f.read(3)
                                        h, w = struct.unpack('>HH', f.read(4))
                                        return w, h
                                    else:
                                        length = struct.unpack('>H', f.read(2))[0]
                                        f.read(length - 2)
                                    b = f.read(1)
                            return 8192, 4096

                        if os.path.exists(scan_pose.image_path):
                            # Get the images2D vector node
                            try:
                                images2d = e57.root["images2D"]
                            except KeyError:
                                images2d = libe57.VectorNode(e57.image_file, True)
                                e57.root.set("images2D", images2d)
                                
                            img_node = libe57.StructureNode(e57.image_file)
                            img_node.set("guid", libe57.StringNode(e57.image_file, f"{{{uuid.uuid4()}}}"))
                            img_node.set("name", libe57.StringNode(e57.image_file, f"Pano_{scan_idx:04d}"))
                            img_node.set("description", libe57.StringNode(e57.image_file, "NavVis Panorama"))
                            
                            # Link to the corresponding 3D scan data (required by Autodesk Recap)
                            try:
                                # In pye57/libe57, we can access the scan GUID from the data3d list
                                scan_header = e57.data3d[scan_idx]
                                # Many versions of pye57 return a dict-like object for the header.
                                # If it's a Node, we try to get the value safely.
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
                            
                            # Add Pose explicitly for the image sphere
                            pose_node = libe57.StructureNode(e57.image_file)
                            img_node.set("pose", pose_node)
                            
                            # ASTM E57 states that images2D poses are global, exactly like data3D poses.
                            rot_node = libe57.StructureNode(e57.image_file)
                            rot_node.set("w", libe57.FloatNode(e57.image_file, scan_pose.orientation[0]))
                            rot_node.set("x", libe57.FloatNode(e57.image_file, scan_pose.orientation[1]))
                            rot_node.set("y", libe57.FloatNode(e57.image_file, scan_pose.orientation[2]))
                            rot_node.set("z", libe57.FloatNode(e57.image_file, scan_pose.orientation[3]))
                            pose_node.set("rotation", rot_node)
                            
                            trans_node = libe57.StructureNode(e57.image_file)
                            trans_node.set("x", libe57.FloatNode(e57.image_file, station_pos[0]))
                            trans_node.set("y", libe57.FloatNode(e57.image_file, station_pos[1]))
                            trans_node.set("z", libe57.FloatNode(e57.image_file, station_pos[2]))
                            pose_node.set("translation", trans_node)
                            
                            # Spherical representation node
                            spherical = libe57.StructureNode(e57.image_file)
                            img_node.set("sphericalRepresentation", spherical)
                            
                            img_w, img_h = _get_jpeg_size(scan_pose.image_path)
                            
                            # Read JPEG and write to blob
                            with open(scan_pose.image_path, "rb") as f:
                                 jpeg_data = f.read()
                                
                            blob = libe57.BlobNode(e57.image_file, len(jpeg_data))
                            
                            # MUST ATTACH BLOB BEFORE WRITING
                            spherical.set("jpegImage", blob)
                            spherical.set("imageWidth", libe57.IntegerNode(e57.image_file, img_w)) 
                            spherical.set("imageHeight", libe57.IntegerNode(e57.image_file, img_h))
                            
                            # Recap requires pixelWidth and pixelHeight (in radians) to define the spherical mapping
                            pixel_width = (2.0 * math.pi) / float(img_w)
                            pixel_height = math.pi / float(img_h)
                            spherical.set("pixelWidth", libe57.FloatNode(e57.image_file, pixel_width))
                            spherical.set("pixelHeight", libe57.FloatNode(e57.image_file, pixel_height))
                            
                            # Add image node to the images2D vector BEFORE writing binary blob
                            images2d.append(img_node)
                            
                            # Write exact shape to the blob using the write function.
                            blob.write(jpeg_data, 0, len(jpeg_data))
                            
                            logger.info(f"Successfully embedded panorama {scan_pose.image_path} ({img_w}x{img_h})")
                    except Exception as e:
                        logger.warning(f"Failed to embed NavVis image {scan_pose.image_path}: {e}")
                        
            # Close file cleanly
            if hasattr(e57, 'close'):
                e57.close()
                
            logger.info("E57 Structured Export successfully finalized.")
        except Exception as e:
            logger.error(f"E57 Construction Error: {e}")
            raise
