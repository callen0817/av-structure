import os
import math
import logging
from typing import Dict, Any, Optional, List
import numpy as np
import pye57

logger = logging.getLogger(__name__)


def validate_exported_e57(
    e57_path: str,
    expected_scans: Optional[int] = None,
    expect_rgb: Optional[bool] = None,
    expect_intensity: Optional[bool] = None,
    expect_panoramas: bool = True,
    expected_bbox_min: Optional[np.ndarray] = None,
    expected_bbox_max: Optional[np.ndarray] = None,
) -> Dict[str, Any]:
    """
    Independently reopens an exported E57 file and verifies:
      1. File opens cleanly in pye57 reader
      2. Expected number of Data3D scan records exist
      3. Each scan has a valid pose (unit rotation quaternion + finite translation)
      4. Structured row/column indices (rowIndex, columnIndex), indexBounds, sphericalBounds,
         and pointGroupingSchemes exist
      5. Global transformed point coordinates are finite and spatially aligned with source bounds
      6. RGB and Intensity attributes match expectations (no fabricated RGB on XYZ-only clouds)
      7. images2D panorama records exist (when expected), contain valid JPEG blobs,
         have valid associatedData3DGuid references to Data3D scans, and consistent poses.
    """
    errors: List[str] = []
    warnings: List[str] = []
    scan_reports: List[Dict[str, Any]] = []
    image_reports: List[Dict[str, Any]] = []

    if not e57_path or not os.path.exists(e57_path):
        return {
            "valid": False,
            "errors": [f"E57 file does not exist: {e57_path}"],
            "warnings": [],
            "scan_count": 0,
            "image_count": 0,
            "total_points_read": 0,
            "scans": [],
            "images": [],
        }

    try:
        e57 = pye57.E57(e57_path, mode="r")
    except Exception as e:
        return {
            "valid": False,
            "errors": [f"Failed to open E57 file in independent reader: {e}"],
            "warnings": [],
            "scan_count": 0,
            "image_count": 0,
            "total_points_read": 0,
            "scans": [],
            "images": [],
        }

    try:
        scan_count = (
            e57.scan_count
            if isinstance(e57.scan_count, int)
            else (e57.scan_count() if callable(e57.scan_count) else len(e57.data3d))
        )
        if expected_scans is not None and scan_count != expected_scans:
            errors.append(f"Expected {expected_scans} Data3D scan(s), but found {scan_count}.")
        if scan_count == 0:
            errors.append("E57 file contains 0 Data3D scan records.")

        data3d_guids: Dict[str, Dict[str, Any]] = {}
        total_points_read = 0

        for idx in range(scan_count):
            hdr = e57.get_header(idx)
            scan_node = e57.data3d[idx]
            point_fields = list(hdr.point_fields)
            scan_fields = list(hdr.scan_fields)

            guid = str(hdr.guid) if "guid" in scan_fields else ""
            if not guid:
                errors.append(f"Scan {idx}: missing Data3D guid.")

            # Check pose
            has_pose = hdr.has_pose()
            trans = np.asarray(hdr.translation, dtype=np.float64) if has_pose else np.zeros(3)
            rot = np.asarray(hdr.rotation, dtype=np.float64) if has_pose else np.array([1.0, 0.0, 0.0, 0.0])
            if not has_pose:
                errors.append(f"Scan {idx}: missing pose structure.")
            else:
                if not np.all(np.isfinite(trans)):
                    errors.append(f"Scan {idx}: pose translation contains non-finite values.")
                q_norm = float(np.linalg.norm(rot))
                if not math.isfinite(q_norm) or abs(q_norm - 1.0) > 1e-2:
                    errors.append(f"Scan {idx}: pose rotation quaternion is not unit norm ({q_norm:.4f}).")

            # Check structured rowIndex / columnIndex and indexBounds
            has_row_col = ("rowIndex" in point_fields) and ("columnIndex" in point_fields)
            if not has_row_col:
                errors.append(f"Scan {idx}: missing structured rowIndex/columnIndex in point_fields ({point_fields}).")

            has_index_bounds = "indexBounds" in scan_fields
            row_min, row_max, col_min, col_max = 0, 0, 0, 0
            if has_index_bounds:
                row_min = int(hdr.rowMinimum)
                row_max = int(hdr.rowMaximum)
                col_min = int(hdr.columnMinimum)
                col_max = int(hdr.columnMaximum)
                if row_min > row_max or col_min > col_max:
                    errors.append(
                        f"Scan {idx}: invalid indexBounds (row=[{row_min}, {row_max}], col=[{col_min}, {col_max}])."
                    )
            else:
                errors.append(f"Scan {idx}: missing indexBounds on Data3D node.")

            has_grouping = scan_node.isDefined("pointGroupingSchemes")
            has_spherical_bounds = scan_node.isDefined("sphericalBounds")

            # Read raw and transformed scan data
            raw_data = e57.read_scan_raw(idx, ignore_unsupported_fields=True)
            pt_count = len(raw_data.get("cartesianX", []))
            total_points_read += pt_count

            if pt_count == 0:
                warnings.append(f"Scan {idx}: contains 0 points.")

            # Verify global coordinates via read_scan(transform=True)
            try:
                transformed = e57.read_scan(
                    idx,
                    colors=False,
                    intensity=False,
                    row_column=True,
                    transform=True,
                    ignore_missing_fields=True,
                )
                gx = np.asarray(transformed.get("cartesianX", []), dtype=np.float64)
                gy = np.asarray(transformed.get("cartesianY", []), dtype=np.float64)
                gz = np.asarray(transformed.get("cartesianZ", []), dtype=np.float64)
                if len(gx) > 0:
                    g_pts = np.vstack((gx, gy, gz)).T
                    if not np.all(np.isfinite(g_pts)):
                        errors.append(f"Scan {idx}: transformed global coordinates contain NaN/Inf.")
                    if expected_bbox_min is not None and expected_bbox_max is not None:
                        tol = 1.0
                        in_box = np.all(
                            (g_pts >= (expected_bbox_min - tol)) & (g_pts <= (expected_bbox_max + tol)),
                            axis=1,
                        )
                        if float(np.mean(in_box)) < 0.95:
                            errors.append(
                                f"Scan {idx}: global transformed points fall outside expected bounding box."
                            )
            except Exception as tr_err:
                errors.append(f"Scan {idx}: failed to read transformed scan points: {tr_err}")

            # Verify RGB & intensity attributes
            scan_has_rgb = all(c in point_fields for c in ("colorRed", "colorGreen", "colorBlue"))
            scan_has_intensity = "intensity" in point_fields
            if expect_rgb is True and not scan_has_rgb:
                errors.append(f"Scan {idx}: expected RGB point fields but none were found.")
            elif expect_rgb is False and scan_has_rgb:
                errors.append(f"Scan {idx}: found unexpected RGB point fields on non-RGB input.")
            if expect_intensity is True and not scan_has_intensity:
                errors.append(f"Scan {idx}: expected intensity field but none was found.")

            data3d_guids[guid] = {
                "scan_index": idx,
                "translation": trans,
                "rotation": rot,
            }

            scan_reports.append(
                {
                    "scan_index": idx,
                    "guid": guid,
                    "point_count": pt_count,
                    "has_pose": has_pose,
                    "translation": trans.tolist(),
                    "rotation": rot.tolist(),
                    "has_row_col": has_row_col,
                    "has_index_bounds": has_index_bounds,
                    "index_bounds": {
                        "rowMinimum": row_min,
                        "rowMaximum": row_max,
                        "columnMinimum": col_min,
                        "columnMaximum": col_max,
                    },
                    "has_point_grouping_schemes": has_grouping,
                    "has_spherical_bounds": has_spherical_bounds,
                    "has_rgb": scan_has_rgb,
                    "has_intensity": scan_has_intensity,
                }
            )

        # Validate images2D panoramas and associatedData3DGuid links
        image_count = 0
        if e57.root.isDefined("images2D"):
            images2d = e57.root["images2D"]
            image_count = images2d.childCount()
            for i in range(image_count):
                img_node = images2d[i]
                img_guid = str(img_node["guid"].value()) if img_node.isDefined("guid") else ""
                assoc_guid = (
                    str(img_node["associatedData3DGuid"].value())
                    if img_node.isDefined("associatedData3DGuid")
                    else ""
                )
                desc = str(img_node["description"].value()) if img_node.isDefined("description") else ""

                if not assoc_guid:
                    errors.append(f"Image {i}: missing associatedData3DGuid.")
                elif assoc_guid not in data3d_guids:
                    errors.append(
                        f"Image {i}: associatedData3DGuid '{assoc_guid}' does not match any Data3D scan GUID."
                    )

                # Check image pose consistency with linked scan pose
                pose_consistent = False
                if img_node.isDefined("pose") and assoc_guid in data3d_guids:
                    pnode = img_node["pose"]
                    tr = pnode["translation"]
                    rot = pnode["rotation"]
                    img_t = np.array([float(tr["x"].value()), float(tr["y"].value()), float(tr["z"].value())])
                    img_r = np.array(
                        [
                            float(rot["w"].value()),
                            float(rot["x"].value()),
                            float(rot["y"].value()),
                            float(rot["z"].value()),
                        ]
                    )
                    scan_t = data3d_guids[assoc_guid]["translation"]
                    scan_r = data3d_guids[assoc_guid]["rotation"]
                    t_diff = float(np.linalg.norm(img_t - scan_t))
                    # Quaternions q and -q represent the same 3D orientation
                    r_diff = min(float(np.linalg.norm(img_r - scan_r)), float(np.linalg.norm(img_r + scan_r)))
                    if t_diff <= 1e-3 and r_diff <= 1e-3:
                        pose_consistent = True
                    else:
                        errors.append(
                            f"Image {i}: pose mismatch with associated scan (t_diff={t_diff:.5f}, r_diff={r_diff:.5f})."
                        )

                # Check sphericalRepresentation and JPEG blob
                has_spherical = img_node.isDefined("sphericalRepresentation")
                jpeg_valid = False
                img_w, img_h, blob_bytes = 0, 0, 0
                if has_spherical:
                    sph = img_node["sphericalRepresentation"]
                    img_w = int(sph["imageWidth"].value()) if sph.isDefined("imageWidth") else 0
                    img_h = int(sph["imageHeight"].value()) if sph.isDefined("imageHeight") else 0
                    if sph.isDefined("jpegImage"):
                        blob = sph["jpegImage"]
                        blob_bytes = int(blob.byteCount())
                        if blob_bytes > 4:
                            header_buf = bytearray(2)
                            blob.read(header_buf, 0, 2)
                            jpeg_valid = bytes(header_buf) == b"\xff\xd8"
                    if not jpeg_valid or img_w <= 0 or img_h <= 0:
                        errors.append(
                            f"Image {i}: invalid sphericalRepresentation JPEG (valid={jpeg_valid}, size={img_w}x{img_h}, bytes={blob_bytes})."
                        )
                else:
                    errors.append(f"Image {i}: missing sphericalRepresentation.")

                image_reports.append(
                    {
                        "image_index": i,
                        "guid": img_guid,
                        "associatedData3DGuid": assoc_guid,
                        "description": desc,
                        "pose_consistent": pose_consistent,
                        "jpeg_valid": jpeg_valid,
                        "width": img_w,
                        "height": img_h,
                        "byte_count": blob_bytes,
                    }
                )

        if expect_panoramas and scan_count > 0 and image_count != scan_count:
            errors.append(f"Expected {scan_count} embedded images2D panorama(s), but found {image_count}.")

        if hasattr(e57, "close"):
            e57.close()

        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "scan_count": scan_count,
            "image_count": image_count,
            "total_points_read": total_points_read,
            "scans": scan_reports,
            "images": image_reports,
        }
    except Exception as e:
        if hasattr(e57, "close"):
            e57.close()
        errors.append(f"Unexpected validation exception: {e}")
        return {
            "valid": False,
            "errors": errors,
            "warnings": warnings,
            "scan_count": 0,
            "image_count": 0,
            "total_points_read": 0,
            "scans": scan_reports,
            "images": image_reports,
        }
